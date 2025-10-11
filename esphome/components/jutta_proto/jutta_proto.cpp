#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <utility>

#include "esphome/core/time.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 2000;
const char *const MACHINE_DATA_COMMAND = "&STAT?\r\n";
constexpr uint32_t XML_POLL_REQUEST_TIMEOUT_MS = 2000;

std::string format_printable_char(uint8_t byte) {
  switch (byte) {
    case '\r':
      return "\\r";
    case '\n':
      return "\\n";
    case '\t':
      return "\\t";
    default:
      break;
  }
  if (std::isprint(static_cast<int>(byte)) != 0) {
    return std::string(1, static_cast<char>(byte));
  }
  std::ostringstream stream;
  stream << "\\x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
         << static_cast<int>(byte);
  return stream.str();
}

std::string format_printable_string(const std::string &value) {
  std::ostringstream stream;
  for (unsigned char c : value) {
    stream << format_printable_char(c);
  }
  return stream.str();
}

std::string format_hex_string(const std::string &value) {
  if (value.empty()) {
    return "[]";
  }
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < value.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
           << static_cast<int>(static_cast<unsigned char>(value[i]));
  }
  stream << "]";
  return stream.str();
}

std::string format_buffer_preview(const std::string &value) {
  if (value.size() <= HANDSHAKE_LOG_PREVIEW_LIMIT) {
    return format_printable_string(value);
  }
  std::string suffix = value.substr(value.size() - HANDSHAKE_LOG_PREVIEW_LIMIT);
  return std::string("...") + format_printable_string(suffix);
}

std::string format_buffer_hex_preview(const std::string &value) {
  if (value.size() <= HANDSHAKE_LOG_PREVIEW_LIMIT) {
    return format_hex_string(value);
  }
  std::string suffix = value.substr(value.size() - HANDSHAKE_LOG_PREVIEW_LIMIT);
  std::string formatted_suffix = format_hex_string(suffix);
  if (formatted_suffix.size() > 1) {
    return std::string("...") + formatted_suffix;
  }
  return formatted_suffix;
}

std::string sanitize_response(const std::string &value) {
  std::string sanitized = value;
  sanitized.erase(std::remove(sanitized.begin(), sanitized.end(), '\r'), sanitized.end());
  sanitized.erase(std::remove(sanitized.begin(), sanitized.end(), '\n'), sanitized.end());
  return sanitized;
}

std::string trim_copy(const std::string &value) {
  size_t start = 0;
  size_t end = value.size();
  while (start < end && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
    ++start;
  }
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    --end;
  }
  return value.substr(start, end - start);
}

bool parse_numeric_value(const std::string &text, double &out) {
  if (text.empty()) {
    return false;
  }
  char *end_ptr = nullptr;
  double value = std::strtod(text.c_str(), &end_ptr);
  if (end_ptr == text.c_str() || trim_copy(std::string(end_ptr)).size() != 0) {
    end_ptr = nullptr;
    long fallback = std::strtol(text.c_str(), &end_ptr, 16);
    if (end_ptr == text.c_str() || trim_copy(std::string(end_ptr)).size() != 0) {
      return false;
    }
    value = static_cast<double>(fallback);
  }
  out = value;
  return true;
}

std::map<std::string, double> parse_key_value_response(const std::string &command, const std::string &response) {
  std::map<std::string, double> values;
  if (response.empty()) {
    return values;
  }

  std::string normalized = response;
  if (!normalized.empty() && normalized[0] == '@') {
    normalized.erase(0, 1);
  }

  std::string expected = command;
  if (!expected.empty() && expected[0] == '@') {
    expected.erase(0, 1);
  }

  // Strip prefix (e.g. "TR:32")
  if (!expected.empty()) {
    if (normalized.rfind(expected, 0) == 0) {
      normalized.erase(0, expected.size());
    }
  }

  if (!normalized.empty() && (normalized[0] == ':' || normalized[0] == ',')) {
    normalized.erase(0, 1);
  }

  size_t pos = 0;
  while (pos <= normalized.size()) {
    size_t next = normalized.find_first_of(",;", pos);
    std::string token = normalized.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
    token = trim_copy(token);
    if (!token.empty()) {
      size_t eq = token.find('=');
      if (eq == std::string::npos) {
        eq = token.find(':');
      }
      if (eq != std::string::npos) {
        std::string key = trim_copy(token.substr(0, eq));
        std::string value_str = trim_copy(token.substr(eq + 1));
        if (!key.empty() && !value_str.empty()) {
          std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
          double parsed_value = 0.0;
          if (parse_numeric_value(value_str, parsed_value)) {
            values[key] = parsed_value;
          }
        }
      }
    }
    if (next == std::string::npos) {
      break;
    }
    pos = next + 1;
  }

  return values;
}

}  // namespace

std::shared_ptr<std::string> JuraComponent::wait_for_response_(::jutta_proto::JuttaConnection *connection,
                                                               const std::string &command, uint32_t timeout_ms) {
  if (connection == nullptr) {
    return nullptr;
  }
  auto timeout = std::chrono::milliseconds{timeout_ms};
  uint32_t start = esphome::millis();
  auto response = connection->write_decoded_with_response(command, timeout);
  while (response == nullptr) {
    if (timeout_ms > 0 && JuraComponent::time_reached(esphome::millis(), start + timeout_ms)) {
      break;
    }
    esphome::delay(10);
    response = connection->write_decoded_with_response(command, timeout);
  }
  return response;
}

bool JuraComponent::ensure_transaction_ready_(const char *operation) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "%s nicht möglich - Handshake läuft noch.", operation != nullptr ? operation : "Aktion");
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    ESP_LOGW(TAG, "%s nicht möglich - Verbindung zur Maschine fehlt.", operation != nullptr ? operation : "Aktion");
    return false;
  }
  if (this->is_busy()) {
    ESP_LOGW(TAG, "%s nicht möglich - Maschine ist beschäftigt.", operation != nullptr ? operation : "Aktion");
    return false;
  }
  return true;
}

void JuraComponent::publish_machine_settings_(const std::string &payload) {
  this->last_machine_settings_xml_ = payload;
  if (this->machine_settings_sensor_ != nullptr) {
    this->machine_settings_sensor_->publish_state(payload);
  }
}

void JuraComponent::run_legacy_probe_handshake_() {
  if (this->connection_ == nullptr) {
    return;
  }
  this->legacy_probe_command_.clear();
  this->legacy_probe_response_.clear();
  this->legacy_codec_mode_ = LegacyCodecMode::Auto;

  auto store_result = [&](const std::string &command, const std::shared_ptr<std::string> &response) {
    std::string trimmed_command = command;
    trimmed_command.erase(std::remove(trimmed_command.begin(), trimmed_command.end(), '\r'), trimmed_command.end());
    trimmed_command.erase(std::remove(trimmed_command.begin(), trimmed_command.end(), '\n'), trimmed_command.end());
    this->legacy_probe_command_ = trimmed_command;
    if (response != nullptr) {
      this->legacy_probe_response_ = sanitize_response(*response);
    } else {
      this->legacy_probe_response_.clear();
    }
  };

  auto response = this->wait_for_response_(this->connection_.get(), "&WHO\r\n", 500);
  if (response != nullptr) {
    this->legacy_codec_mode_ = LegacyCodecMode::Plain;
    store_result("&WHO", response);
    ESP_LOGI(TAG, "Legacy-Probe '&WHO' beantwortet mit '%s'.", this->legacy_probe_response_.c_str());
    this->connection_->reset_response_line_buffer();
    return;
  }

  static const char *const PROBES[] = {"@TR:37\r\n", "@TR:32\r\n", "@t2:8188\r\n", "@TS:00\r\n"};
  for (const char *probe : PROBES) {
    response = this->wait_for_response_(this->connection_.get(), probe, 800);
    if (response != nullptr) {
      this->legacy_codec_mode_ = LegacyCodecMode::Auto;
      store_result(probe, response);
      ESP_LOGI(TAG, "Legacy-Probe '%s' beantwortet mit '%s'.", format_printable_string(probe).c_str(),
               this->legacy_probe_response_.c_str());
      this->connection_->reset_response_line_buffer();
      return;
    }
  }

  this->legacy_codec_mode_ = LegacyCodecMode::Escaped;
  ESP_LOGW(TAG, "Legacy-Probe erhielt keine Antwort, gehe von ESC-Mode aus.");
  this->connection_->reset_response_line_buffer();
}

const char *JuraComponent::handshake_stage_name(JuraComponent::HandshakeStage stage) {
  switch (stage) {
    case JuraComponent::HandshakeStage::IDLE:
      return "idle";
    case JuraComponent::HandshakeStage::HELLO:
      return "hello";
    case JuraComponent::HandshakeStage::SEND_T1:
      return "send_t1";
    case JuraComponent::HandshakeStage::WAIT_T2:
      return "wait_t2";
    case JuraComponent::HandshakeStage::SEND_T2:
      return "send_t2";
    case JuraComponent::HandshakeStage::WAIT_T3:
      return "wait_t3";
    case JuraComponent::HandshakeStage::SEND_T3:
      return "send_t3";
    case JuraComponent::HandshakeStage::DONE:
      return "done";
    case JuraComponent::HandshakeStage::FAILED:
      return "failed";
  }
  return "unknown";
}

void JuraComponent::setup() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not configured for JUTTA Proto component.");
    this->mark_failed();
    return;
  }

  this->connection_ = std::make_unique<::jutta_proto::JuttaConnection>(this->parent_);
  this->connection_->init();

  this->run_legacy_probe_handshake_();

  this->handshake_stage_ = HandshakeStage::HELLO;
  ESP_LOGI(TAG, "Starting handshake with coffee maker...");
}

void JuraComponent::loop() {
  if (this->handshake_stage_ != this->last_logged_stage_) {
    ESP_LOGI(TAG, "Handshake stage changed: %s -> %s (buffer size=%zu, preview='%s', hex %s)",
             JuraComponent::handshake_stage_name(this->last_logged_stage_),
             JuraComponent::handshake_stage_name(this->handshake_stage_), this->handshake_buffer_.size(),
             format_buffer_preview(this->handshake_buffer_).c_str(),
             format_buffer_hex_preview(this->handshake_buffer_).c_str());
    this->last_logged_stage_ = this->handshake_stage_;
  }

  if (this->connection_ != nullptr && this->handshake_stage_ != HandshakeStage::DONE &&
      this->handshake_stage_ != HandshakeStage::FAILED) {
    this->process_handshake();
  }

  if (this->coffee_maker_ != nullptr) {
    this->coffee_maker_->loop();
    if (!this->coffee_maker_->is_locked()) {
      this->custom_cancel_flag_ = false;
    }
  }

  this->process_machine_data_query();
  this->process_xml_poll();
}

void JuraComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "JUTTA Proto");
  if (!this->device_type_.empty()) {
    ESP_LOGCONFIG(TAG, "  Detected device: %s", this->device_type_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Detected device: (pending)");
  }

  const char *state = "unknown";
  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      state = "idle";
      break;
    case HandshakeStage::HELLO:
      state = "awaiting type";
      break;
    case HandshakeStage::SEND_T1:
      state = "waiting for @t1";
      break;
    case HandshakeStage::WAIT_T2:
      state = "waiting for @T2";
      break;
    case HandshakeStage::SEND_T2:
      state = "sending @t2";
      break;
    case HandshakeStage::WAIT_T3:
      state = "waiting for @T3";
      break;
    case HandshakeStage::SEND_T3:
      state = "sending @t3";
      break;
    case HandshakeStage::DONE:
      state = "ready";
      break;
    case HandshakeStage::FAILED:
      state = "failed";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Handshake state: %s", state);

  if (!this->handshake_t2_response_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last key exchange T2: %s", this->handshake_t2_response_.c_str());
  }
  if (!this->handshake_t3_response_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last key exchange T3: %s", this->handshake_t3_response_.c_str());
  }

  const char *legacy_mode = "unbekannt";
  switch (this->legacy_codec_mode_) {
    case LegacyCodecMode::Unknown:
      legacy_mode = "unbekannt";
      break;
    case LegacyCodecMode::Plain:
      legacy_mode = "kein Codec";
      break;
    case LegacyCodecMode::Auto:
      legacy_mode = "automatisch";
      break;
    case LegacyCodecMode::Escaped:
      legacy_mode = "ESC";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Legacy-Kompatibilitätsprobe: %s", legacy_mode);
  if (!this->legacy_probe_command_.empty()) {
    ESP_LOGCONFIG(TAG, "    Letzte Anfrage: %s -> %s", this->legacy_probe_command_.c_str(),
                  this->legacy_probe_response_.empty() ? "(keine Antwort)" : this->legacy_probe_response_.c_str());
  }

  if (this->coffee_maker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(true));
  } else {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(false));
  }

  if (this->machine_settings_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Sensor für Maschineneinstellungen aktiv: %s", YESNO(true));
    if (!this->last_machine_settings_xml_.empty()) {
      ESP_LOGCONFIG(TAG, "    Letzte gelesene XML-Länge: %zu Zeichen", this->last_machine_settings_xml_.size());
    }
    if (this->machine_settings_write_attempted_) {
      ESP_LOGCONFIG(TAG, "    Letzter Schreibversuch: %s", YESNO(this->last_machine_settings_write_ok_));
    }
  }

  if (!this->xml_sensors_.empty()) {
    if (this->xml_poll_enabled_) {
      ESP_LOGCONFIG(TAG, "  XML polling interval: %u ms", this->xml_poll_interval_ms_);
    } else {
      ESP_LOGCONFIG(TAG, "  XML polling disabled (configured sensors: %zu)", this->xml_sensors_.size());
    }
    for (const auto &sensor : this->xml_sensors_) {
      ESP_LOGCONFIG(TAG, "    XML field '%s' via %s -> key %s (multiplier=%.3f offset=%.3f)", sensor.field.c_str(),
                    sensor.command.c_str(), sensor.key.c_str(), sensor.multiplier, sensor.offset);
    }
  }
}

void JuraComponent::process_handshake() {
  using ::jutta_proto::JuttaConnection;
  using ::jutta_proto::JUTTA_GET_TYPE;

  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      break;
    case HandshakeStage::HELLO: {
      if (!this->handshake_hello_request_sent_) {
        ESP_LOGD(TAG, "HELLO: requesting device type with payload '%s' (hex %s).",
                 format_printable_string(JUTTA_GET_TYPE).c_str(),
                 format_hex_string(JUTTA_GET_TYPE).c_str());
        if (this->connection_->write_decoded(JUTTA_GET_TYPE)) {
          this->connection_->reset_response_line_buffer();
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = esphome::millis() + 2000;
          this->handshake_hello_request_sent_ = true;
          ESP_LOGD(TAG, "HELLO: device type request sent, waiting for response (deadline in 2000 ms).");
        } else {
          this->restart_handshake("failed to request device type");
          break;
        }
      }

      if (this->read_handshake_bytes()) {
        bool handled = false;
        while (!handled) {
          auto newline = this->handshake_buffer_.find("\r\n");
          if (newline == std::string::npos) {
            break;
          }

          std::string line = this->handshake_buffer_.substr(0, newline);
          this->handshake_buffer_.erase(0, newline + 2);

          std::string lowercase_line = line;
          std::transform(lowercase_line.begin(), lowercase_line.end(), lowercase_line.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

          if (lowercase_line.rfind("ty:", 0) == 0) {
            if (line.size() <= 3) {
              ESP_LOGW(TAG,
                       "HELLO: device type response line '%s' has no payload, proceeding with unknown type.",
                       format_printable_string(line).c_str());
              this->device_type_ = "TY:unknown";
            } else {
              this->device_type_ = line;
              ESP_LOGI(TAG, "Detected coffee maker response: %s", this->device_type_.c_str());
            }
            this->handshake_buffer_.clear();
            this->handshake_deadline_ = 0;
            this->handshake_stage_ = HandshakeStage::SEND_T1;
            this->handshake_hello_request_sent_ = false;
            handled = true;
          } else {
            ESP_LOGD(TAG, "HELLO: ignoring unexpected response line: '%s'",
                     format_printable_string(line).c_str());
          }
        }
      }

      if (this->handshake_stage_ == HandshakeStage::HELLO && this->handshake_deadline_ != 0 &&
          time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for device type");
      }
      break;
    }
    case HandshakeStage::SEND_T1: {
      ESP_LOGD(TAG, "SEND_T1: writing '@T1\\r\\n' and waiting for '@t1\\r\\n' (timeout=1000 ms).");
      auto wait_result = this->connection_->write_decoded_wait_for("@T1\r\n", "@t1\r\n", std::chrono::milliseconds{1000});
      if (wait_result == JuttaConnection::WaitResult::Success) {
        ESP_LOGD(TAG, "Received @t1 acknowledgment.");
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
        this->handshake_stage_ = HandshakeStage::WAIT_T2;
      } else if (wait_result == JuttaConnection::WaitResult::Timeout) {
        this->restart_handshake("timeout waiting for @t1");
      } else if (wait_result == JuttaConnection::WaitResult::Error) {
        this->restart_handshake("failed to send @T1");
      }
      break;
    }
    case HandshakeStage::WAIT_T2: {
      if (this->handshake_deadline_ == 0) {
        this->handshake_deadline_ = esphome::millis() + 5000;
        ESP_LOGD(TAG, "WAIT_T2: started response timer (deadline in 5000 ms).");
      }
      bool any = this->read_handshake_bytes();
      if (any) {
        auto pos = this->handshake_buffer_.find("@T2");
        if (pos != std::string::npos) {
          auto end = this->handshake_buffer_.find("\r\n", pos);
          if (end != std::string::npos) {
            this->handshake_t2_response_ = this->handshake_buffer_.substr(pos, end - pos);
          } else {
            this->handshake_t2_response_ = this->handshake_buffer_.substr(pos);
          }
          ESP_LOGD(TAG, "Received %s", this->handshake_t2_response_.c_str());
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = 0;
          this->handshake_stage_ = HandshakeStage::SEND_T2;
        }
      }
      if (this->handshake_deadline_ != 0 && time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for @T2");
      }
      break;
    }
    case HandshakeStage::SEND_T2: {
      ESP_LOGD(TAG, "SEND_T2: sending '@t2:8120000000\\r\\n'.");
      if (this->connection_->write_decoded("@t2:8120000000\r\n")) {
        ESP_LOGD(TAG, "Sent @t2 response.");
        this->handshake_stage_ = HandshakeStage::WAIT_T3;
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
      } else {
        this->restart_handshake("failed to send @t2");
      }
      break;
    }
    case HandshakeStage::WAIT_T3: {
      if (this->handshake_deadline_ == 0) {
        this->handshake_deadline_ = esphome::millis() + 5000;
        ESP_LOGD(TAG, "WAIT_T3: started response timer (deadline in 5000 ms).");
      }
      bool any = this->read_handshake_bytes();
      if (any) {
        auto pos = this->handshake_buffer_.find("@T3");
        if (pos != std::string::npos) {
          auto end = this->handshake_buffer_.find("\r\n", pos);
          if (end != std::string::npos) {
            this->handshake_t3_response_ = this->handshake_buffer_.substr(pos, end - pos);
          } else {
            this->handshake_t3_response_ = this->handshake_buffer_.substr(pos);
          }
          ESP_LOGD(TAG, "Received %s", this->handshake_t3_response_.c_str());
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = 0;
          this->handshake_stage_ = HandshakeStage::SEND_T3;
        }
      }
      if (this->handshake_deadline_ != 0 && time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for @T3");
      }
      break;
    }
    case HandshakeStage::SEND_T3: {
      ESP_LOGD(TAG, "SEND_T3: sending '@t3\\r\\n' to finish handshake.");
      if (this->connection_->write_decoded("@t3\r\n")) {
        ESP_LOGI(TAG, "Handshake finished successfully.");
        this->handshake_stage_ = HandshakeStage::DONE;
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
      } else {
        this->restart_handshake("failed to send @t3");
      }
      break;
    }
    case HandshakeStage::DONE:
    case HandshakeStage::FAILED:
      break;
  }

  if (this->handshake_stage_ == HandshakeStage::DONE && this->connection_ != nullptr &&
      this->coffee_maker_ == nullptr) {
    auto connection = std::move(this->connection_);
    this->coffee_maker_ = std::make_unique<::jutta_proto::CoffeeMaker>(std::move(connection));
    ESP_LOGI(TAG, "Coffee maker controller initialized.");
  }
}

void JuraComponent::restart_handshake(const char *reason) {
  if (reason != nullptr) {
    ESP_LOGW(TAG, "Restarting handshake: %s", reason);
  }
  this->handshake_buffer_.clear();
  this->handshake_deadline_ = 0;
  this->handshake_hello_request_sent_ = false;
  this->handshake_stage_ = HandshakeStage::HELLO;
  this->last_logged_stage_ = HandshakeStage::FAILED;
  if (this->connection_ != nullptr) {
    this->connection_->reset_response_line_buffer();
  }
}

bool JuraComponent::read_handshake_bytes() {
  if (this->connection_ == nullptr) {
    return false;
  }
  bool read_any = false;
  std::string line;
  while (this->connection_->poll_response_line(line)) {
    read_any = true;
    this->handshake_buffer_.append(line);
    this->handshake_buffer_.append("\r\n");
    if (this->handshake_buffer_.size() > 128) {
      this->handshake_buffer_.erase(0, this->handshake_buffer_.size() - 128);
    }
    ESP_LOGV(TAG,
             "Handshake buffered line: '%s'; buffer size=%zu; buffer now '%s' (hex %s)",
             format_printable_string(line).c_str(), this->handshake_buffer_.size(),
             format_buffer_preview(this->handshake_buffer_).c_str(),
             format_buffer_hex_preview(this->handshake_buffer_).c_str());
  }
  return read_any;
}

bool JuraComponent::time_reached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

void JuraComponent::process_machine_data_query() {
  if (this->machine_data_sensor_ == nullptr) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  if (this->is_busy()) {
    return;
  }

  uint32_t now = esphome::millis();

  auto handle_response = [&](const std::shared_ptr<std::string> &response) {
    if (response != nullptr) {
      this->publish_machine_data_(*response);
      this->machine_data_query_next_ = now + MACHINE_DATA_QUERY_INTERVAL_MS;
      this->machine_data_request_pending_ = false;
      return true;
    }
    return false;
  };

  if (this->machine_data_request_pending_) {
    auto response = this->coffee_maker_->connection->write_decoded_with_response(
        MACHINE_DATA_COMMAND, std::chrono::milliseconds{MACHINE_DATA_REQUEST_TIMEOUT_MS});
    if (handle_response(response)) {
      return;
    }
    if (time_reached(now, this->machine_data_request_start_ + MACHINE_DATA_REQUEST_TIMEOUT_MS)) {
      ESP_LOGW(TAG, "Timeout while waiting for machine data response.");
      this->machine_data_request_pending_ = false;
      this->machine_data_query_next_ = now + MACHINE_DATA_QUERY_INTERVAL_MS;
    }
    return;
  }

  if (this->machine_data_query_next_ != 0 &&
      !time_reached(now, this->machine_data_query_next_)) {
    return;
  }

  this->machine_data_request_start_ = now;
  auto response = this->coffee_maker_->connection->write_decoded_with_response(
      MACHINE_DATA_COMMAND, std::chrono::milliseconds{MACHINE_DATA_REQUEST_TIMEOUT_MS});
  if (handle_response(response)) {
    return;
  }

  this->machine_data_request_pending_ = true;
}

void JuraComponent::publish_machine_data_(const std::string &response) {
  std::string sanitized = response;
  sanitized.erase(std::remove_if(sanitized.begin(), sanitized.end(),
                                 [](unsigned char c) { return c == '\r' || c == '\n'; }),
                  sanitized.end());
  ESP_LOGD(TAG, "Machine data response: %s", sanitized.c_str());
  if (this->machine_data_sensor_ != nullptr) {
    this->machine_data_sensor_->publish_state(sanitized);
  }
}

void JuraComponent::register_xml_sensor(const std::string &field, const std::string &command, const std::string &key,
                                        float multiplier, float offset, sensor::Sensor *sensor) {
  XmlSensorEntry entry;
  entry.field = field;
  entry.command = command;
  entry.key = key;
  std::transform(entry.command.begin(), entry.command.end(), entry.command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  std::transform(entry.key.begin(), entry.key.end(), entry.key.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  entry.multiplier = multiplier;
  entry.offset = offset;
  entry.sensor = sensor;
  this->xml_sensors_.push_back(entry);
}

void JuraComponent::process_xml_poll() {
  if (!this->xml_poll_enabled_) {
    return;
  }
  if (this->xml_sensors_.empty()) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  if (this->is_busy()) {
    return;
  }

  uint32_t now = esphome::millis();
  if (this->xml_next_poll_ != 0 && !time_reached(now, this->xml_next_poll_)) {
    return;
  }

  std::map<std::string, std::vector<XmlSensorEntry *>> sensors_by_command;
  for (auto &entry : this->xml_sensors_) {
    sensors_by_command[entry.command].push_back(&entry);
  }

  bool any_success = false;

  for (auto &item : sensors_by_command) {
    const std::string &command = item.first;
    std::string request = std::string("@") + command + "\r\n";
    ESP_LOGV(TAG, "Requesting XML bank %s", command.c_str());
    auto response = this->coffee_maker_->connection->write_decoded_with_response(
        request, std::chrono::milliseconds{XML_POLL_REQUEST_TIMEOUT_MS});
    if (response == nullptr) {
      ESP_LOGW(TAG, "Keine Antwort auf %s erhalten", command.c_str());
      continue;
    }
    std::string sanitized = sanitize_response(*response);
    if (sanitized.empty()) {
      ESP_LOGW(TAG, "Leere Antwort auf %s erhalten", command.c_str());
      continue;
    }
    ESP_LOGD(TAG, "XML-Antwort %s: %s", command.c_str(), sanitized.c_str());
    auto values = parse_key_value_response(command, sanitized);
    if (values.empty()) {
      ESP_LOGW(TAG, "Antwort für %s konnte nicht interpretiert werden: %s", command.c_str(), sanitized.c_str());
      continue;
    }
    any_success = true;
    for (auto *sensor_entry : item.second) {
      auto key_it = values.find(sensor_entry->key);
      if (key_it == values.end()) {
        ESP_LOGV(TAG, "Feld %s (%s) nicht in Antwort von %s gefunden", sensor_entry->field.c_str(),
                 sensor_entry->key.c_str(), command.c_str());
        continue;
      }
      double value = key_it->second;
      value = value * sensor_entry->multiplier + sensor_entry->offset;
      if (sensor_entry->sensor != nullptr) {
        sensor_entry->sensor->publish_state(value);
      }
    }
  }

  if (!any_success) {
    ESP_LOGW(TAG, "XML-Abfrage lieferte keine verwertbaren Daten");
  }
  this->xml_next_poll_ = now + this->xml_poll_interval_ms_;
}

void JuraComponent::request_machine_settings() {
  if (!this->ensure_transaction_ready_("Maschineneinstellungen lesen")) {
    return;
  }
  auto *connection = this->coffee_maker_->connection.get();
  auto response = this->wait_for_response_(connection, "@hr:00\r\n", 1500);
  std::string payload;
  if (response != nullptr) {
    payload = *response;
  }
  if (payload.size() < 32) {
    response = this->wait_for_response_(connection, "@hr:05\r\n", 1500);
    if (response != nullptr) {
      payload = *response;
    }
  }
  if (payload.empty()) {
    ESP_LOGW(TAG, "Maschineneinstellungen konnten nicht gelesen werden.");
    return;
  }
  while (!payload.empty() && (payload.back() == '\r' || payload.back() == '\n')) {
    payload.pop_back();
  }
  ESP_LOGI(TAG, "Maschineneinstellungen gelesen (%zu Zeichen).", payload.size());
  this->publish_machine_settings_(payload);
}

void JuraComponent::write_machine_settings(const std::string &xml) {
  if (xml.empty()) {
    ESP_LOGW(TAG, "Leere XML-Daten werden nicht zur Maschine gesendet.");
    return;
  }
  if (!this->ensure_transaction_ready_("Maschineneinstellungen schreiben")) {
    return;
  }
  auto *connection = this->coffee_maker_->connection.get();
  if (connection == nullptr) {
    ESP_LOGW(TAG, "Keine Verbindung für das Schreiben der Maschineneinstellungen verfügbar.");
    return;
  }

  auto expect_response = [&](const std::string &command, uint32_t timeout_ms, const char *label) -> bool {
    auto reply = this->wait_for_response_(connection, command, timeout_ms);
    if (reply == nullptr) {
      ESP_LOGW(TAG, "%s: keine Antwort auf '%s' erhalten.", label, format_printable_string(command).c_str());
      return false;
    }
    ESP_LOGD(TAG, "%s: Antwort '%s'", label, sanitize_response(*reply).c_str());
    return true;
  };

  bool ok = true;
  if (!expect_response("@ha:00\r\n", 1000, "Schreibstart")) {
    ok = false;
  }
  if (ok && !expect_response("@HD:000000000040\r\n", 1000, "Header")) {
    ok = false;
  }
  if (ok) {
    if (!connection->write_decoded(xml)) {
      ESP_LOGW(TAG, "Übertragung der XML-Daten fehlgeschlagen.");
      ok = false;
    }
  }
  if (ok && !expect_response("@hu:ok\r\n", 1000, "Abschluss")) {
    ok = false;
  }

  this->machine_settings_write_attempted_ = true;
  this->last_machine_settings_write_ok_ = ok;
  if (ok) {
    ESP_LOGI(TAG, "Maschineneinstellungen erfolgreich geschrieben (%zu Zeichen).", xml.size());
    this->publish_machine_settings_(xml);
  } else {
    ESP_LOGW(TAG, "Maschineneinstellungen konnten nicht vollständig geschrieben werden.");
  }
}

void JuraComponent::start_brew(::jutta_proto::CoffeeMaker::coffee_t coffee) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot start brew - component not ready.");
    return;
  }
  this->coffee_maker_->brew_coffee(coffee);
}

void JuraComponent::start_custom_brew(uint32_t grind_duration_ms, uint32_t water_duration_ms) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot brew custom coffee - component not ready.");
    return;
  }
  this->custom_cancel_flag_ = false;
  this->coffee_maker_->brew_custom_coffee(&this->custom_cancel_flag_, std::chrono::milliseconds{grind_duration_ms},
                                          std::chrono::milliseconds{water_duration_ms});
}

void JuraComponent::cancel_custom_brew() {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot cancel custom brew - component not ready.");
    return;
  }
  if (!this->custom_cancel_flag_) {
    ESP_LOGI(TAG, "Cancelling custom brew.");
  }
  this->custom_cancel_flag_ = true;
}

void JuraComponent::switch_page(uint32_t page) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot switch page - component not ready.");
    return;
  }
  this->coffee_maker_->switch_page(page);
}

void JuraComponent::run_sequence(const std::vector<::jutta_proto::CoffeeMaker::SequenceStep> &steps) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot run sequence - component not ready.");
    return;
  }
  if (this->coffee_maker_->is_locked()) {
    ESP_LOGW(TAG, "Cannot run sequence - coffee maker busy.");
    return;
  }
  this->coffee_maker_->run_sequence(steps);
}

bool JuraComponent::is_busy() const {
  if (this->coffee_maker_ == nullptr) {
    return false;
  }
  return this->coffee_maker_->is_locked();
}

}  // namespace jutta_component
}  // namespace esphome

