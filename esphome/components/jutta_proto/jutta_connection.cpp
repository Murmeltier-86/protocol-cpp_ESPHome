#include "jutta_connection.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <utility>
#include <vector>

#include "esphome/core/log.h"
#include "esphome/core/time.h"

namespace jutta_proto {

namespace {
constexpr uint8_t ESCAPE_BYTE = 0xDB;
constexpr std::array<uint8_t, 4> RESERVED_BYTES{0xDB, 0xDF, 0xFB, 0xFF};
constexpr std::array<uint8_t, 8> DB_TRAILER{0xDF, 0xFF, 0xDB, 0xDB, 0xFB, 0xFB, 0xDB, 0xDB};

bool needs_escape(uint8_t byte) {
  return std::find(RESERVED_BYTES.begin(), RESERVED_BYTES.end(), byte) != RESERVED_BYTES.end();
}

std::string trim_crlf(const std::string &value) {
  std::string result = value;
  while (!result.empty() && (result.back() == '\r' || result.back() == '\n')) {
    result.pop_back();
  }
  return result;
}

bool time_expired(uint32_t start, uint32_t timeout_ms) {
  return timeout_ms != 0 && esphome::millis() - start >= timeout_ms;
}

}  // namespace

static const char *const TAG = "jutta_connection";

JuttaConnection::JuttaConnection(esphome::uart::UARTComponent *parent) : serial_(parent) {}

void JuttaConnection::init() { serial_.init(); }

std::string JuttaConnection::sanitize_line_for_log(const std::string &line) {
  std::string sanitized;
  sanitized.reserve(line.size());
  for (unsigned char c : line) {
    if (std::isprint(c) != 0) {
      sanitized.push_back(static_cast<char>(c));
    } else {
      sanitized.push_back('.');
    }
  }
  return sanitized;
}

size_t JuttaConnection::available() const {
  auto *self = const_cast<serial::SerialConnection *>(&serial_);
  return self->available();
}

bool JuttaConnection::read_byte(uint8_t *byte) {
  auto *self = const_cast<serial::SerialConnection *>(&serial_);
  return self->read_byte(byte);
}

bool JuttaConnection::write_bytes(const uint8_t *data, size_t length) {
  return serial_.write_serial_buffer(data, length);
}

void JuttaConnection::drain_uart() {
  size_t drained = 0;
  uint8_t byte = 0;
  while (available() > 0) {
    if (!read_byte(&byte)) {
      break;
    }
    ++drained;
  }
  if (drained > 0) {
    ESP_LOGV(TAG, "Drained %zu pending byte%s from UART before transmit.", drained, drained == 1 ? "" : "s");
  }
  line_buffer_.clear();
}

void JuttaConnection::flush_and_gap() {
  drain_uart();
  serial_.flush();
  if (PRE_SEND_GAP_MS > 0) {
    esphome::delay(PRE_SEND_GAP_MS);
  }
}

bool JuttaConnection::send_line_cmd(const std::string &command) {
  std::string trimmed = trim_crlf(command);
  std::string frame = trimmed;
  frame.append("\r\n");

  ESP_LOGD(TAG, "TX_LINE \"%s\"", sanitize_line_for_log(trimmed).c_str());

  flush_and_gap();
  if (!write_bytes(reinterpret_cast<const uint8_t *>(frame.data()), frame.size())) {
    ESP_LOGE(TAG, "Failed to transmit line command.");
    return false;
  }
  serial_.flush();
  return true;
}

bool JuttaConnection::send_db_cmd(const std::string &command) {
  std::vector<uint8_t> encoded;
  encoded.reserve(command.size() * 2 + DB_TRAILER.size());
  for (unsigned char byte : command) {
    if (needs_escape(byte)) {
      encoded.push_back(ESCAPE_BYTE);
      encoded.push_back(static_cast<uint8_t>(byte ^ 0x20));
    } else {
      encoded.push_back(static_cast<uint8_t>(byte));
    }
  }
  encoded.insert(encoded.end(), DB_TRAILER.begin(), DB_TRAILER.end());

  ESP_LOGD(TAG, "TX_DB len=%zu", encoded.size());

  flush_and_gap();
  if (!write_bytes(encoded.data(), encoded.size())) {
    ESP_LOGE(TAG, "Failed to transmit DB command.");
    return false;
  }
  serial_.flush();
  return true;
}

bool JuttaConnection::extract_line_from_buffer(std::string &line) {
  auto terminator = line_buffer_.find("\r\n");
  if (terminator == std::string::npos) {
    return false;
  }
  line = line_buffer_.substr(0, terminator);
  line_buffer_.erase(0, terminator + 2);
  return true;
}

bool JuttaConnection::read_line_until(std::string &line, uint32_t timeout_ms) {
  if (extract_line_from_buffer(line)) {
    return true;
  }

  uint32_t start = esphome::millis();
  while (!time_expired(start, timeout_ms)) {
    bool any = false;
    while (available() > 0) {
      uint8_t byte = 0;
      if (!read_byte(&byte)) {
        break;
      }
      any = true;
      line_buffer_.push_back(static_cast<char>(byte));
      if (extract_line_from_buffer(line)) {
        return true;
      }
    }
    if (!any) {
      esphome::delay(1);
    }
  }
  return extract_line_from_buffer(line);
}

bool JuttaConnection::read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms) {
  std::vector<uint8_t> raw;
  raw.reserve(256);
  uint32_t last_activity = esphome::millis();

  while (!time_expired(last_activity, timeout_ms)) {
    bool any = false;
    while (available() > 0) {
      uint8_t byte = 0;
      if (!read_byte(&byte)) {
        break;
      }
      any = true;
      raw.push_back(byte);
      last_activity = esphome::millis();
      if (raw.size() >= DB_TRAILER.size() &&
          std::equal(DB_TRAILER.begin(), DB_TRAILER.end(), raw.end() - DB_TRAILER.size())) {
        raw.resize(raw.size() - DB_TRAILER.size());
        decoded.clear();
        decoded.reserve(raw.size());
        for (size_t i = 0; i < raw.size();) {
          uint8_t value = raw[i++];
          if (value == ESCAPE_BYTE) {
            if (i >= raw.size()) {
              ESP_LOGW(TAG, "Dangling escape in DB frame (%zu bytes).", raw.size());
              return false;
            }
            value = static_cast<uint8_t>(raw[i++] ^ 0x20);
          }
          decoded.push_back(value);
        }
        ESP_LOGD(TAG, "RX_DB raw_len=%zu decoded_len=%zu trailer_ok=1", raw.size(), decoded.size());
        return true;
      }
    }
    if (!any) {
      esphome::delay(1);
    }
  }

  ESP_LOGW(TAG, "Timeout waiting for DB frame (raw_len=%zu).", raw.size());
  return false;
}

bool JuttaConnection::transact_line(const std::string &command, std::string *response_line, bool need_ok,
                                    uint32_t timeout_ms) {
  if (!send_line_cmd(command)) {
    return false;
  }

  std::string line;
  if (response_line != nullptr) {
    if (!read_line_until(line, timeout_ms)) {
      ESP_LOGW(TAG, "Timeout waiting for line response to '%s'.", sanitize_line_for_log(command).c_str());
      return false;
    }
    ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_line_for_log(line).c_str());
    *response_line = line;
  }

  if (need_ok) {
    if (!read_line_until(line, timeout_ms)) {
      ESP_LOGW(TAG, "Timeout waiting for ok after '%s'.", sanitize_line_for_log(command).c_str());
      return false;
    }
    ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_line_for_log(line).c_str());
    bool ok = line == "ok:";
    ESP_LOGD(TAG, "OK=%d", ok ? 1 : 0);
    return ok;
  }

  return true;
}

bool JuttaConnection::transact_db(const std::string &command, std::vector<uint8_t> *decoded,
                                  uint32_t timeout_ms) {
  if (!send_db_cmd(command)) {
    return false;
  }

  std::vector<uint8_t> payload;
  if (!read_db_frame(payload, timeout_ms)) {
    return false;
  }

  if (decoded != nullptr) {
    *decoded = payload;
  }
  return true;
}

bool JuttaConnection::write_decoded(const std::string &command) {
  if (!command.empty() && command.front() == '@') {
    return send_db_cmd(command);
  }
  return send_line_cmd(command);
}

JuttaConnection::WaitResult JuttaConnection::wait_for_ok(const std::chrono::milliseconds &timeout) {
  uint32_t timeout_ms = timeout.count() > 0 ? static_cast<uint32_t>(timeout.count()) : 0;
  uint32_t start = esphome::millis();
  std::string line;
  while (true) {
    uint32_t elapsed = esphome::millis() - start;
    if (timeout_ms != 0 && elapsed >= timeout_ms) {
      ESP_LOGW(TAG, "Timeout waiting for ok response.");
      return WaitResult::Timeout;
    }
    uint32_t remaining = timeout_ms == 0 ? LINE_TIMEOUT_MS : std::min<uint32_t>(LINE_TIMEOUT_MS, timeout_ms - elapsed);
    if (!read_line_until(line, remaining)) {
      continue;
    }
    ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_line_for_log(line).c_str());
    if (line == "ok:") {
      ESP_LOGD(TAG, "OK=1");
      return WaitResult::Success;
    }
  }
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(
    const std::string &command, const std::string &expected_response, const std::chrono::milliseconds &timeout) {
  bool is_db = !command.empty() && command.front() == '@';
  std::string trimmed_expected = trim_crlf(expected_response);
  uint32_t timeout_ms = timeout.count() > 0 ? static_cast<uint32_t>(timeout.count()) : 0;

  if (is_db) {
    if (!send_db_cmd(command)) {
      return WaitResult::Error;
    }
    uint32_t effective_timeout = timeout_ms != 0 ? timeout_ms : DB_TIMEOUT_MS;
    uint32_t start = esphome::millis();
    while (true) {
      std::vector<uint8_t> decoded;
      uint32_t elapsed = esphome::millis() - start;
      if (timeout_ms != 0 && elapsed >= effective_timeout) {
        ESP_LOGW(TAG, "Timeout waiting for DB response to '%s'.", sanitize_line_for_log(command).c_str());
        return WaitResult::Timeout;
      }
      uint32_t remaining = timeout_ms != 0 ? effective_timeout - elapsed : DB_TIMEOUT_MS;
      if (!read_db_frame(decoded, remaining)) {
        if (timeout_ms != 0 && esphome::millis() - start >= effective_timeout) {
          return WaitResult::Timeout;
        }
        continue;
      }
      std::string response(decoded.begin(), decoded.end());
      if (response == trimmed_expected) {
        return WaitResult::Success;
      }
    }
  }

  if (!send_line_cmd(command)) {
    return WaitResult::Error;
  }

  uint32_t effective_timeout = timeout_ms != 0 ? timeout_ms : LINE_TIMEOUT_MS;
  uint32_t start = esphome::millis();
  std::string line;
  while (true) {
    uint32_t elapsed = esphome::millis() - start;
    if (timeout_ms != 0 && elapsed >= effective_timeout) {
      ESP_LOGW(TAG, "Timeout waiting for '%s' after '%s'.", sanitize_line_for_log(trimmed_expected).c_str(),
               sanitize_line_for_log(command).c_str());
      return WaitResult::Timeout;
    }
    uint32_t remaining = timeout_ms != 0 ? effective_timeout - elapsed : LINE_TIMEOUT_MS;
    if (!read_line_until(line, remaining)) {
      continue;
    }
    ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_line_for_log(line).c_str());
    if (line == trimmed_expected) {
      return WaitResult::Success;
    }
  }
}

std::shared_ptr<std::string> JuttaConnection::write_xml_with_response(
    const std::string &command, const std::chrono::milliseconds &timeout) {
  std::vector<uint8_t> decoded;
  if (!transact_db(command, &decoded, timeout.count() > 0 ? static_cast<uint32_t>(timeout.count()) : DB_TIMEOUT_MS)) {
    return nullptr;
  }
  return std::make_shared<std::string>(decoded.begin(), decoded.end());
}

bool JuttaConnection::poll_response_line(std::string &line) {
  if (extract_line_from_buffer(line)) {
    ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_line_for_log(line).c_str());
    return true;
  }

  bool any = false;
  while (available() > 0) {
    uint8_t byte = 0;
    if (!read_byte(&byte)) {
      break;
    }
    any = true;
    line_buffer_.push_back(static_cast<char>(byte));
  }

  if (any && extract_line_from_buffer(line)) {
    ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_line_for_log(line).c_str());
    return true;
  }
  return false;
}

void JuttaConnection::reset_response_line_buffer() { line_buffer_.clear(); }

void JuttaConnection::flush_serial_input() { drain_uart(); }

}  // namespace jutta_proto
