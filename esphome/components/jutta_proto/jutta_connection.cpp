#include "jutta_connection.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/time.h"

namespace jutta_proto {

std::atomic<bool> uart_busy{false};

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

uint8_t unswap_nibbles(uint8_t value) {
  return static_cast<uint8_t>(((value & 0x0F) << 4) | ((value & 0xF0) >> 4));
}

uint8_t decode_quartet(const std::array<uint8_t, 4> &group) {
  uint8_t result = 0;
  for (size_t index = 0; index < group.size(); ++index) {
    uint8_t byte = group[index];
    uint8_t low_bit = static_cast<uint8_t>((byte >> 2) & 0x01);
    uint8_t high_bit = static_cast<uint8_t>((byte >> 5) & 0x01);
    result |= static_cast<uint8_t>(low_bit << (index * 2));
    result |= static_cast<uint8_t>(high_bit << (index * 2 + 1));
  }
  return result;
}

}  // namespace

JuttaConnection::JuttaConnection(esphome::uart::UARTComponent *parent) : serial_(parent) {}

void JuttaConnection::init() {
  serial_.init();
  encoded_rx_buffer_.clear();
  plain_rx_buffer_.clear();
  pending_lines_.clear();
  pending_xml_frames_.clear();
  ok_wait_context_ = {};
  active_pipeline_ = ActivePipeline::Idle;
  line_read_buffer_.clear();
  device_probe_ = {};
}

void JuttaConnection::flush_serial_input() {
  encoded_rx_buffer_.clear();
  plain_rx_buffer_.clear();
  pending_lines_.clear();
  pending_xml_frames_.clear();
  ok_wait_context_ = {};
  active_pipeline_ = ActivePipeline::Idle;
  line_read_buffer_.clear();
  device_probe_ = {};

  auto *self = const_cast<serial::SerialConnection *>(&serial_);
  while (self->available() > 0) {
    uint8_t byte;
    if (!self->read_serial_byte(&byte)) {
      break;
    }
  }
}

void JuttaConnection::drain_uart_for_ms(uint32_t duration_ms) {
  encoded_rx_buffer_.clear();
  plain_rx_buffer_.clear();
  pending_lines_.clear();
  pending_xml_frames_.clear();
  line_read_buffer_.clear();
  ok_wait_context_ = {};
  active_pipeline_ = ActivePipeline::Idle;
  device_probe_ = {};

  auto *self = const_cast<serial::SerialConnection *>(&serial_);
  if (duration_ms == 0) {
    while (self->available() > 0) {
      uint8_t byte;
      if (!self->read_serial_byte(&byte)) {
        break;
      }
    }
    return;
  }

  uint32_t start = esphome::millis();
  while (!time_reached(esphome::millis(), start + duration_ms)) {
    while (self->available() > 0) {
      uint8_t byte;
      if (!self->read_serial_byte(&byte)) {
        break;
      }
    }
    esphome::delay(1);
  }
}

void JuttaConnection::send_plain_line(const std::string &line) {
  send_line_cmd(line);
}

void JuttaConnection::flush_and_gap() {
  flush_serial_input();
  if (JUTTA_SERIAL_GAP_MS > 0) {
    esphome::delay(JUTTA_SERIAL_GAP_MS);
  }
}

void JuttaConnection::drain_encoded(const std::chrono::milliseconds &duration) {
  uint32_t wait_ms = static_cast<uint32_t>(std::max<long long>(0, duration.count()));
  if (wait_ms == 0) {
    pump_serial(0);
    return;
  }
  uint32_t start = esphome::millis();
  while (!time_reached(esphome::millis(), start + wait_ms)) {
    pump_serial(5);
    esphome::delay(1);
  }
}

bool JuttaConnection::pump_serial(uint32_t timeout_ms) {
  uint32_t start = esphome::millis();
  bool changed = false;
  while (true) {
    auto *self = const_cast<serial::SerialConnection *>(&serial_);
    bool read_any = false;
    while (self->available() > 0) {
      uint8_t byte;
      if (!self->read_serial_byte(&byte)) {
        break;
      }
      encoded_rx_buffer_.push_back(byte);
      read_any = true;
    }
    if (read_any) {
      changed = true;
    }

    if (process_encoded_frames()) {
      changed = true;
    }

    if (!pending_lines_.empty() || !pending_xml_frames_.empty()) {
      break;
    }

    if (timeout_ms == 0) {
      break;
    }
    uint32_t now = esphome::millis();
    if (time_reached(now, start + timeout_ms)) {
      break;
    }
    esphome::delay(1);
  }
  return changed;
}

bool JuttaConnection::process_encoded_frames() {
  bool any = false;
  while (encoded_rx_buffer_.size() >= DB_TRAILER.size()) {
    auto it = std::search(encoded_rx_buffer_.begin(), encoded_rx_buffer_.end(), DB_TRAILER.begin(), DB_TRAILER.end());
    if (it == encoded_rx_buffer_.end()) {
      break;
    }

    std::vector<uint8_t> encoded(encoded_rx_buffer_.begin(), it);
    encoded_rx_buffer_.erase(encoded_rx_buffer_.begin(), it + DB_TRAILER.size());

    if (!uart_busy.load()) {
      std::string encoded_hex = format_hex(encoded);
      std::string trailer_hex = format_hex(std::vector<uint8_t>(DB_TRAILER.begin(), DB_TRAILER.end()), DB_TRAILER.size());
      ESP_LOGV(TAG, "RX_ENC len=%zu hex=%s terminator=%s", encoded.size(), encoded_hex.c_str(),
               trailer_hex.c_str());
    }

    std::vector<uint8_t> decoded;
    if (!decode_encoded_frame(encoded, decoded)) {
      any = true;
      continue;
    }

    route_decoded_frame(std::move(decoded), encoded.size());
    any = true;
  }
  return any;
}

bool JuttaConnection::decode_encoded_frame(const std::vector<uint8_t> &encoded,
                                           std::vector<uint8_t> &decoded) const {
  std::vector<uint8_t> unescaped;
  unescaped.reserve(encoded.size());
  for (size_t i = 0; i < encoded.size(); ++i) {
    uint8_t value = encoded[i];
    if (value == DB_ESC) {
      if (i + 1 >= encoded.size()) {
        ESP_LOGW(TAG, "Discarding encoded frame with dangling escape (encoded_len=%zu, hex=%s).",
                 encoded.size(), format_hex(encoded).c_str());
        return false;
      }
      ++i;
      unescaped.push_back(static_cast<uint8_t>(encoded[i] ^ 0x20));
    } else {
      unescaped.push_back(value);
    }
  }

  if (unescaped.empty()) {
    ESP_LOGW(TAG, "Discarding empty encoded frame (encoded_hex=%s).", format_hex(encoded).c_str());
    return false;
  }

  if (unescaped.size() % 4 != 0) {
    ESP_LOGW(TAG,
             "Discarding encoded frame with unexpected length (encoded_len=%zu, unescaped_len=%zu, hex=%s).",
             encoded.size(), unescaped.size(), format_hex(encoded).c_str());
    return false;
  }

  decoded.clear();
  decoded.reserve(unescaped.size() / 4);
  for (size_t i = 0; i < unescaped.size(); i += 4) {
    std::array<uint8_t, 4> quartet = {unescaped[i], unescaped[i + 1], unescaped[i + 2], unescaped[i + 3]};
    for (auto &byte : quartet) {
      byte = unswap_nibbles(byte);
    }
    uint8_t decoded_byte = decode_quartet(quartet);
    decoded.push_back(decoded_byte);
  }

  return true;
}

bool JuttaConnection::is_plain_text_frame(const std::vector<uint8_t> &decoded) const {
  if (decoded.empty()) {
    return false;
  }

  bool has_line_ending = false;
  for (size_t i = 0; i < decoded.size(); ++i) {
    uint8_t byte = decoded[i];
    if (byte == '\r') {
      if (i + 1 < decoded.size() && decoded[i + 1] == '\n') {
        has_line_ending = true;
      }
      continue;
    }
    if (byte == '\n') {
      continue;
    }
    if (byte == '\t') {
      continue;
    }
    if (byte >= 0x20 && byte <= 0x7E) {
      continue;
    }
    return false;
  }

  return has_line_ending;
}

void JuttaConnection::route_decoded_frame(std::vector<uint8_t> decoded, size_t encoded_length) {
  if (is_plain_text_frame(decoded)) {
    plain_rx_buffer_.append(reinterpret_cast<const char *>(decoded.data()), decoded.size());
    size_t terminator_pos = 0;
    while ((terminator_pos = plain_rx_buffer_.find("\r\n")) != std::string::npos) {
      std::string line = plain_rx_buffer_.substr(0, terminator_pos);
      plain_rx_buffer_.erase(0, terminator_pos + 2);
      if (!uart_busy.load()) {
        ESP_LOGD(TAG, "RX_LINE \"%s\"", printable_snippet(line).c_str());
      }
      pending_lines_.push_back(std::move(line));
    }
    return;
  }

  XmlFrame frame;
  frame.payload = std::move(decoded);
  frame.encoded_size = encoded_length;
  if (!uart_busy.load()) {
    ESP_LOGD(TAG, "RX_XML encoded_len=%zu decoded_len=%zu", frame.encoded_size, frame.payload.size());
  }
  pending_xml_frames_.push_back(std::move(frame));
}

std::string JuttaConnection::format_hex(const std::vector<uint8_t> &buffer, size_t max_bytes) {
  std::ostringstream stream;
  size_t limit = std::min(buffer.size(), max_bytes);
  stream << std::uppercase << std::hex << std::setfill('0');
  for (size_t i = 0; i < limit; ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<int>(buffer[i]);
  }
  if (buffer.size() > limit) {
    stream << " …";
  }
  return stream.str();
}

void JuttaConnection::set_active_pipeline(ActivePipeline pipeline) {
  active_pipeline_ = pipeline;
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

bool JuttaConnection::starts_with_lower(const std::string &s, const char *prefix) {
  size_t prefix_len = std::strlen(prefix);
  if (s.size() < prefix_len) {
    return false;
  }
  for (size_t i = 0; i < prefix_len; ++i) {
    if (std::tolower(static_cast<unsigned char>(s[i])) !=
        std::tolower(static_cast<unsigned char>(prefix[i]))) {
      return false;
    }
  }
  return true;
}

void JuttaConnection::send_line_cmd(const std::string &line) {
  ESP_LOGD(TAG, "TX_LINE \"%s\"", printable_snippet(line).c_str());
  std::vector<uint8_t> buffer(line.begin(), line.end());
  buffer.push_back('\r');
  buffer.push_back('\n');
  if (!serial_.write_serial_buffer(buffer)) {
    ESP_LOGW(TAG, "Failed to send line command over serial");
  }
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
  if (!uart_busy.load()) {
    ESP_LOGD(TAG, "TX_DB len=%zu", encoded.size());
  }
  if (!serial_.write_serial_buffer(encoded)) {
    ESP_LOGW(TAG, "Failed to send DB command over serial (len=%zu)", encoded.size());
  }
}

void JuttaConnection::poll_serial_lines(uint32_t timeout_ms) {
  if (uart_busy.load()) {
    return;
  }
  pump_serial(timeout_ms);
}

bool JuttaConnection::read_line_until(std::string &out, uint32_t timeout_ms, uint32_t *bytes_seen) {
  out.clear();
  line_buffer_.clear();
  uint32_t seen = 0;
  uint32_t start = esphome::millis();
  auto *self = const_cast<serial::SerialConnection *>(&serial_);
  while (!time_reached(esphome::millis(), start + timeout_ms)) {
    while (self->available() > 0) {
      uint8_t byte;
      if (!self->read_serial_byte(&byte)) {
        break;
      }
      ++seen;
      char c = static_cast<char>(byte);
      line_buffer_.push_back(c);
      if (line_buffer_.size() >= 2 && line_buffer_[line_buffer_.size() - 2] == '\r' &&
          line_buffer_.back() == '\n') {
        out.assign(line_buffer_.begin(), line_buffer_.end() - 2);
        line_buffer_.clear();
        if (bytes_seen != nullptr) {
          *bytes_seen = seen;
        }
        return true;
      }
    }
    esphome::delay(1);
  }
  if (bytes_seen != nullptr) {
    *bytes_seen = seen;
  }
  return false;
}

JuttaConnection::WaitResult JuttaConnection::await_device_type(std::string &out_ty) {
  constexpr uint32_t TY_TIMEOUT_MS = 2500;
  constexpr uint32_t OK_TIMEOUT_MS = 1200;

  if (!device_probe_.active) {
    {
      UartGuard guard(uart_busy);
      flush_serial_input();
      send_line_cmd("ty:");
    }
    device_probe_ = {};
    device_probe_.active = true;
    device_probe_.command_sent = true;
    device_probe_.ok_received = false;
    device_probe_.ty_line.clear();
    device_probe_.start_time = esphome::millis();
    set_active_pipeline(ActivePipeline::Plain);
    ESP_LOGD(TAG, "HELLO: starting asynchronous device probe");
    return WaitResult::Pending;
  }

  poll_serial_lines(0);

  std::vector<std::string> leftovers;
  while (!pending_lines_.empty()) {
    std::string line = std::move(pending_lines_.front());
    pending_lines_.pop_front();
    if (starts_with_lower(line, "ty:") && device_probe_.ty_line.empty()) {
      device_probe_.ty_line = line;
      ESP_LOGD(TAG, "HELLO: received %s", printable_snippet(line).c_str());
    } else if (equals_ignore_case(line, "ok:")) {
      device_probe_.ok_received = true;
    } else {
      leftovers.push_back(std::move(line));
    }
  }

  for (auto it = leftovers.rbegin(); it != leftovers.rend(); ++it) {
    pending_lines_.push_front(std::move(*it));
  }

  uint32_t now = esphome::millis();
  uint32_t ty_deadline = device_probe_.start_time + TY_TIMEOUT_MS;
  uint32_t ok_deadline = ty_deadline + OK_TIMEOUT_MS;

  if (!device_probe_.ty_line.empty() && device_probe_.ok_received) {
    out_ty = device_probe_.ty_line;
    cancel_device_probe();
    return WaitResult::Success;
  }

  if ((device_probe_.ty_line.empty() && time_reached(now, ty_deadline)) ||
      (!device_probe_.ok_received && time_reached(now, ok_deadline))) {
    ESP_LOGW(TAG, "HELLO timeout; ty_received=%s ok_received=%s",
             YESNO(!device_probe_.ty_line.empty()), YESNO(device_probe_.ok_received));
    cancel_device_probe();
    return WaitResult::Timeout;
  }

  return WaitResult::Pending;
}

void JuttaConnection::cancel_device_probe() {
  if (!device_probe_.active) {
    return;
  }
  device_probe_ = {};
  set_active_pipeline(ActivePipeline::Idle);
}

bool JuttaConnection::prime_initial_db() {
  UartGuard lock(uart_busy);
  const std::array<const char *, 3> commands = {"@TR:32", "@TG:43", "@TG:C0"};
  for (const char *cmd : commands) {
    pending_xml_frames_.clear();
    drain_encoded(std::chrono::milliseconds{40});
    send_db_cmd(cmd);
    std::vector<uint8_t> decoded;
    if (!read_db_frame(decoded, 1500)) {
      return false;
    }
  }
  return true;
}

bool JuttaConnection::read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms) {
  uint32_t start = esphome::millis();
  while (true) {
    pump_serial(5);

    if (!pending_xml_frames_.empty()) {
      XmlFrame frame = std::move(pending_xml_frames_.front());
      pending_xml_frames_.pop_front();
      decoded = std::move(frame.payload);
      if (!uart_busy.load()) {
        ESP_LOGD(TAG, "XML frame ready encoded_len=%zu decoded_len=%zu", frame.encoded_size, decoded.size());
      }
      return true;
    }

    if (timeout_ms == 0) {
      return false;
    }

    uint32_t now = esphome::millis();
    if (time_reached(now, start + timeout_ms)) {
      if (!encoded_rx_buffer_.empty()) {
        if (!uart_busy.load()) {
          ESP_LOGW(TAG, "Timeout waiting for XML frame. buffered_hex=%s",
                   format_hex(encoded_rx_buffer_).c_str());
        }
      }
      return false;
    }
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
  if (uart_busy.load()) {
    if (ok != nullptr) {
      *ok = false;
    }
    return nullptr;
  }
  drain_encoded(std::chrono::milliseconds{40});
  pending_xml_frames_.clear();
  set_active_pipeline(ActivePipeline::Xml);
  send_db_cmd(command);
  std::vector<uint8_t> decoded;
  if (!read_db_frame(decoded, static_cast<uint32_t>(timeout.count()))) {
    if (ok != nullptr) {
      *ok = false;
    }
    set_active_pipeline(ActivePipeline::Idle);
    return nullptr;
  }
  if (ok != nullptr) {
    *ok = true;
  }
  set_active_pipeline(ActivePipeline::Idle);
  return std::make_shared<std::string>(decoded.begin(), decoded.end());
}

bool JuttaConnection::write_decoded(const std::string &command) {
  std::string trimmed = trim_command(command);
  if (trimmed.empty()) {
    return false;
  }
  bool had_newline = command.find('\n') != std::string::npos || command.find('\r') != std::string::npos;
  if (trimmed.front() == '@' && !had_newline) {
    if (uart_busy.load()) {
      return false;
    }
    drain_encoded(std::chrono::milliseconds{40});
    pending_xml_frames_.clear();
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
    if (uart_busy.load()) {
      return WaitResult::Error;
    }
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
  set_active_pipeline(ActivePipeline::Plain);
  send_line_cmd(trimmed_command);
  if (trimmed_expected.empty()) {
    set_active_pipeline(ActivePipeline::Idle);
    return WaitResult::Success;
  }
  if (wait_for_line(trimmed_expected, timeout)) {
    set_active_pipeline(ActivePipeline::Idle);
    return WaitResult::Success;
  }
  set_active_pipeline(ActivePipeline::Idle);
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
    set_active_pipeline(ActivePipeline::Plain);
  }

  poll_serial_lines(0);
  for (auto it = pending_lines_.begin(); it != pending_lines_.end(); ++it) {
    if (equals_ignore_case(*it, "ok:")) {
      pending_lines_.erase(it);
      ok_wait_context_.active = false;
      set_active_pipeline(ActivePipeline::Idle);
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
    set_active_pipeline(ActivePipeline::Idle);
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
  plain_rx_buffer_.clear();
  pending_lines_.clear();
  device_probe_ = {};
}

}  // namespace jutta_proto
