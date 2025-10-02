#include "jutta_connection.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "esphome/core/log.h"
#include "esphome/core/time.h"

namespace jutta_proto {
namespace {
static const char *const TAG = "jutta_connection";

constexpr uint32_t JUTTA_SERIAL_GAP_MS = 35;
constexpr std::array<uint8_t, 8> DB_TRAILER = {0xDF, 0xFF, 0xDB, 0xDB, 0xFB, 0xFB, 0xDB, 0xDB};
constexpr uint8_t DB_ESC = 0xDB;

bool time_reached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

std::string printable_snippet(const std::string &value, size_t limit = 80) {
  std::string out;
  out.reserve(std::min(limit, value.size() * 2));
  for (unsigned char c : value) {
    if (out.size() >= limit) {
      out.append("...");
      break;
    }
    switch (c) {
      case '\r':
        out.append("\\r");
        break;
      case '\n':
        out.append("\\n");
        break;
      case '\t':
        out.append("\\t");
        break;
      default:
        if (std::isprint(c) != 0) {
          out.push_back(static_cast<char>(c));
        } else {
          char buffer[5];
          snprintf(buffer, sizeof(buffer), "\\x%02X", static_cast<int>(c));
          out.append(buffer);
        }
        break;
    }
  }
  return out;
}

bool equals_ignore_case(const std::string &lhs, const std::string &rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
        std::tolower(static_cast<unsigned char>(rhs[i]))) {
      return false;
    }
  }
  return true;
}

}  // namespace

JuttaConnection::JuttaConnection(esphome::uart::UARTComponent *parent) : serial_(parent) {}

void JuttaConnection::init() {
  serial_.init();
  line_rx_buffer_.clear();
  pending_lines_.clear();
  db_rx_buffer_.clear();
  ok_wait_context_ = {};
}

void JuttaConnection::flush_serial_input() {
  line_rx_buffer_.clear();
  pending_lines_.clear();
  db_rx_buffer_.clear();
  ok_wait_context_ = {};

  auto *self = const_cast<serial::SerialConnection *>(&serial_);
  while (self->available() > 0) {
    uint8_t byte;
    if (!self->read_serial_byte(&byte)) {
      break;
    }
  }
}

void JuttaConnection::flush_and_gap() {
  flush_serial_input();
  if (JUTTA_SERIAL_GAP_MS > 0) {
    esphome::delay(JUTTA_SERIAL_GAP_MS);
  }
}

std::string JuttaConnection::trim_command(const std::string &command) {
  std::string trimmed = command;
  while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n')) {
    trimmed.pop_back();
  }
  return trimmed;
}

bool JuttaConnection::command_requires_ok(const std::string &command) {
  std::string upper = command;
  std::transform(upper.begin(), upper.end(), upper.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return upper.rfind("TY:", 0) == 0 || upper.rfind("FN:", 0) == 0 || upper.rfind("AN:", 0) == 0 ||
         upper.rfind("FA:", 0) == 0;
}

void JuttaConnection::send_line_cmd(const std::string &line) {
  ESP_LOGD(TAG, "TX_LINE \"%s\"", printable_snippet(line).c_str());
  std::vector<uint8_t> buffer(line.begin(), line.end());
  buffer.push_back('\r');
  buffer.push_back('\n');
  serial_.write_serial_buffer(buffer);
  serial_.flush();
}

void JuttaConnection::send_db_cmd(const std::string &command) {
  std::vector<uint8_t> encoded;
  encoded.reserve(command.size() * 2 + DB_TRAILER.size());
  for (uint8_t byte : command) {
    if (byte == DB_ESC || byte == 0xDF || byte == 0xFB || byte == 0xFF) {
      encoded.push_back(DB_ESC);
      encoded.push_back(static_cast<uint8_t>(byte ^ 0x20));
    } else {
      encoded.push_back(byte);
    }
  }
  encoded.insert(encoded.end(), DB_TRAILER.begin(), DB_TRAILER.end());
  ESP_LOGD(TAG, "TX_DB len=%zu", encoded.size());
  serial_.write_serial_buffer(encoded);
  serial_.flush();
}

void JuttaConnection::poll_serial_lines(uint32_t timeout_ms) {
  uint32_t start = esphome::millis();
  while (true) {
    auto *self = const_cast<serial::SerialConnection *>(&serial_);
    while (self->available() > 0) {
      uint8_t byte;
      if (!self->read_serial_byte(&byte)) {
        break;
      }
      line_rx_buffer_.push_back(static_cast<char>(byte));
      if (line_rx_buffer_.size() > 512) {
        line_rx_buffer_.erase(0, line_rx_buffer_.size() - 512);
      }
      while (true) {
        auto terminator = line_rx_buffer_.find("\r\n");
        if (terminator == std::string::npos) {
          break;
        }
        std::string line = line_rx_buffer_.substr(0, terminator);
        line_rx_buffer_.erase(0, terminator + 2);
        ESP_LOGD(TAG, "RX_LINE \"%s\"", printable_snippet(line).c_str());
        pending_lines_.push_back(line);
      }
    }

    if (!pending_lines_.empty()) {
      return;
    }

    if (timeout_ms == 0) {
      return;
    }

    uint32_t now = esphome::millis();
    if (time_reached(now, start + timeout_ms)) {
      return;
    }

    esphome::delay(1);
  }
}

bool JuttaConnection::read_line_until(std::string &out, uint32_t timeout_ms) {
  poll_serial_lines(timeout_ms);
  if (pending_lines_.empty()) {
    return false;
  }
  out = pending_lines_.front();
  pending_lines_.pop_front();
  return true;
}

bool JuttaConnection::read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms) {
  uint32_t start = esphome::millis();
  std::vector<uint8_t> raw;
  while (true) {
    auto *self = const_cast<serial::SerialConnection *>(&serial_);
    while (self->available() > 0) {
      uint8_t byte;
      if (!self->read_serial_byte(&byte)) {
        break;
      }
      db_rx_buffer_.push_back(byte);
      if (db_rx_buffer_.size() >= DB_TRAILER.size()) {
        auto frame_start = db_rx_buffer_.end() - DB_TRAILER.size();
        if (std::equal(DB_TRAILER.begin(), DB_TRAILER.end(), frame_start)) {
          raw.assign(db_rx_buffer_.begin(), db_rx_buffer_.end() - DB_TRAILER.size());
          db_rx_buffer_.clear();

          decoded.clear();
          decoded.reserve(raw.size());
          for (size_t i = 0; i < raw.size(); ++i) {
            uint8_t byte_raw = raw[i];
            if (byte_raw == DB_ESC) {
              if (i + 1 >= raw.size()) {
                ESP_LOGW(TAG, "Discarding DB frame with dangling escape (raw_len=%zu).", raw.size());
                decoded.clear();
                return false;
              }
              ++i;
              decoded.push_back(static_cast<uint8_t>(raw[i] ^ 0x20));
            } else {
              decoded.push_back(byte_raw);
            }
          }
          ESP_LOGD(TAG, "RX_DB raw_len=%zu decoded_len=%zu trailer_ok=1", raw.size(), decoded.size());
          return true;
        }
      }
    }

    if (timeout_ms == 0) {
      return false;
    }
    uint32_t now = esphome::millis();
    if (time_reached(now, start + timeout_ms)) {
      return false;
    }
    esphome::delay(1);
  }
}

bool JuttaConnection::wait_for_line(const std::string &expected, const std::chrono::milliseconds &timeout) {
  uint32_t start = esphome::millis();
  while (true) {
    poll_serial_lines(0);
    for (auto it = pending_lines_.begin(); it != pending_lines_.end(); ++it) {
      if (*it == expected) {
        pending_lines_.erase(it);
        return true;
      }
    }
    if (timeout.count() == 0) {
      return false;
    }
    uint32_t now = esphome::millis();
    if (time_reached(now, start + static_cast<uint32_t>(timeout.count()))) {
      return false;
    }
    esphome::delay(1);
  }
}

std::shared_ptr<std::string> JuttaConnection::transact_db_internal(const std::string &command,
                                                                   const std::chrono::milliseconds &timeout, bool *ok) {
  flush_and_gap();
  send_db_cmd(command);
  std::vector<uint8_t> decoded;
  if (!read_db_frame(decoded, static_cast<uint32_t>(timeout.count()))) {
    if (ok != nullptr) {
      *ok = false;
    }
    return nullptr;
  }
  if (ok != nullptr) {
    *ok = true;
  }
  return std::make_shared<std::string>(decoded.begin(), decoded.end());
}

bool JuttaConnection::write_decoded(const std::string &command) {
  std::string trimmed = trim_command(command);
  if (trimmed.empty()) {
    return false;
  }
  bool had_newline = command.find('\n') != std::string::npos || command.find('\r') != std::string::npos;
  if (trimmed.front() == '@' && !had_newline) {
    flush_and_gap();
    send_db_cmd(trimmed);
    return true;
  }
  flush_and_gap();
  send_line_cmd(trimmed);
  if (command_requires_ok(trimmed)) {
    ok_wait_context_.active = false;
  }
  return true;
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(
    const std::string &command, const std::string &expected_response, const std::chrono::milliseconds &timeout) {
  std::string trimmed_command = trim_command(command);
  std::string trimmed_expected = trim_command(expected_response);
  if (trimmed_command.empty()) {
    return WaitResult::Error;
  }
  bool had_newline = command.find('\n') != std::string::npos || command.find('\r') != std::string::npos;
  if (trimmed_command.front() == '@' && !had_newline) {
    bool ok = false;
    auto response = transact_db_internal(trimmed_command, timeout, &ok);
    if (!ok) {
      return WaitResult::Error;
    }
    if (response == nullptr) {
      return WaitResult::Timeout;
    }
    if (!trimmed_expected.empty() && response->compare(trimmed_expected) != 0) {
      return WaitResult::Error;
    }
    return WaitResult::Success;
  }

  flush_and_gap();
  send_line_cmd(trimmed_command);
  if (trimmed_expected.empty()) {
    return WaitResult::Success;
  }
  if (wait_for_line(trimmed_expected, timeout)) {
    return WaitResult::Success;
  }
  return WaitResult::Timeout;
}

std::shared_ptr<std::string> JuttaConnection::transact_db(const std::string &command,
                                                          const std::chrono::milliseconds &timeout) {
  std::string trimmed = trim_command(command);
  if (trimmed.empty()) {
    return nullptr;
  }
  bool ok = false;
  return transact_db_internal(trimmed, timeout, &ok);
}

JuttaConnection::WaitResult JuttaConnection::wait_for_ok(const std::chrono::milliseconds &timeout) {
  if (!ok_wait_context_.active) {
    ok_wait_context_.active = true;
    ok_wait_context_.timeout = timeout;
    ok_wait_context_.start_time = esphome::millis();
  }

  poll_serial_lines(0);
  for (auto it = pending_lines_.begin(); it != pending_lines_.end(); ++it) {
    if (equals_ignore_case(*it, "ok:")) {
      pending_lines_.erase(it);
      ok_wait_context_.active = false;
      ESP_LOGD(TAG, "OK=1");
      return WaitResult::Success;
    }
  }

  if (timeout.count() == 0) {
    return WaitResult::Pending;
  }
  uint32_t now = esphome::millis();
  if (time_reached(now, ok_wait_context_.start_time + static_cast<uint32_t>(timeout.count()))) {
    ok_wait_context_.active = false;
    ESP_LOGD(TAG, "OK=0");
    return WaitResult::Timeout;
  }
  return WaitResult::Pending;
}

bool JuttaConnection::poll_response_line(std::string &line) {
  poll_serial_lines(0);
  if (pending_lines_.empty()) {
    return false;
  }
  line = pending_lines_.front();
  pending_lines_.pop_front();
  return true;
}

void JuttaConnection::reset_response_line_buffer() {
  line_rx_buffer_.clear();
  pending_lines_.clear();
}

}  // namespace jutta_proto
