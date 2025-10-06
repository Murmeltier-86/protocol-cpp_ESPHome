#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "esphome/core/application.h"
#include "esphome/core/time.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 2000;
const char *const MACHINE_DATA_COMMAND = "&STAT?\r\n";
constexpr uint32_t kXmlRxTimeoutMs = 900;
constexpr uint32_t kInterCmdGapMs = 120;
constexpr uint32_t kCycleSleepMs = 2000;

constexpr double XML_COUNTER_MIN = 0.0;
constexpr double XML_COUNTER_MAX = 1'000'000.0;
constexpr double XML_MEASUREMENT_MIN = 0.0;
constexpr double XML_MEASUREMENT_MAX = 250.0;
constexpr float XML_COUNTER_TOLERANCE = 0.5f;
constexpr float XML_MEASUREMENT_TOLERANCE = 0.1f;
constexpr double TGC0_DEFAULT_DIVISOR = 256.0;
constexpr bool TGC0_TRY_LITTLE_ENDIAN_FIRST = true;
constexpr std::size_t TGC0_MIN_FRAME_LENGTH = 13;

std::string format_hex_head(const std::vector<uint8_t> &data, std::size_t max_bytes) {
  if (data.empty() || max_bytes == 0) {
    return {};
  }
  std::size_t count = std::min<std::size_t>(data.size(), max_bytes);
  std::string out;
  out.reserve(count * 3U);
  char buf[4];
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      out.push_back(' ');
    }
    std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(data[i]));
    out.append(buf);
  }
  return out;
}

std::string format_ascii_string(const std::vector<uint8_t> &data) {
  if (data.empty()) {
    return {};
  }
  std::string out;
  out.reserve(data.size());
  for (uint8_t byte : data) {
    unsigned char c = static_cast<unsigned char>(byte);
    if (c >= 0x20 && c <= 0x7E) {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('.');
    }
  }
  return out;
}

int determine_accuracy(XmlSensorKind kind, double scale) {
  if (kind == XmlSensorKind::Counter) {
    return 0;
  }
  double abs_scale = std::fabs(scale);
  constexpr double EPSILON = 1e-6;
  if (abs_scale <= 0.01 + EPSILON) {
    return 2;
  }
  if (abs_scale < 1.0 - EPSILON) {
    return 1;
  }
  return 0;
}

template<typename AppT>
auto try_register_sensor(AppT &app, sensor::Sensor *sensor, long)
    -> decltype(app.register_sensor(sensor), void()) {
  app.register_sensor(sensor);
}

template<typename AppT>
auto try_register_sensor(AppT &app, sensor::Sensor *sensor, double)
    -> decltype(app.register_entity(sensor), void()) {
  app.register_entity(sensor);
}

template<typename SensorT>
auto try_set_name_impl(SensorT *sensor, const std::string &value, int)
    -> decltype(sensor->set_name(value.c_str()), void()) {
  sensor->set_name(value.c_str());
}

inline void try_set_name_impl(...) {}

template<typename SensorT>
void try_set_name(SensorT *sensor, const std::string &value) {
  try_set_name_impl(sensor, value, 0);
}

template<typename SensorT>
auto try_set_unique_id_impl(SensorT *sensor, const std::string &value, int)
    -> decltype(sensor->set_unique_id(value.c_str()), void()) {
  sensor->set_unique_id(value.c_str());
}

inline void try_set_unique_id_impl(...) {}

template<typename SensorT>
void try_set_unique_id(SensorT *sensor, const std::string &value) {
  try_set_unique_id_impl(sensor, value, 0);
}

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

std::string format_hex_string(const std::vector<uint8_t> &value) {
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
           << static_cast<int>(value[i]);
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

template<typename ArrayT>
std::string format_numeric_array(const ArrayT &values) {
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

}  // namespace

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

JuraComponent::~JuraComponent() { this->xml_sensors_.clear(); }

void JuraComponent::setup() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not configured for JUTTA Proto component.");
    this->mark_failed();
    return;
  }

  this->connection_ = std::make_unique<::jutta_proto::JuttaConnection>(this->parent_);
  this->connection_->init();

  this->handshake_stage_ = HandshakeStage::HELLO;
  ESP_LOGI(TAG, "Starting handshake with coffee maker...");

  this->reset_xml_poll_state_();
  this->xml_next_poll_ = esphome::millis();
  if (this->enable_xml_poll_) {
    this->ensure_xml_mapping_loaded_();
    this->ensure_xml_sensors_created_();
  }
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
  this->process_xml_polling();
}

void JuraComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "JUTTA Proto");
  if (!this->device_type_.empty()) {
    ESP_LOGCONFIG(TAG, "  Detected device: %s", this->device_type_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Detected device: (pending)");
  }

  if (this->enable_xml_poll_) {
    // Stelle sicher, dass der aktuelle Status des XML-Mappings bereits zur
    // Konfigurationsausgabe geladen und die Sensoren angelegt wurden.
    this->ensure_xml_mapping_loaded_();
    this->ensure_xml_sensors_created_();
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

  if (this->coffee_maker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(true));
  } else {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(false));
  }

  ESP_LOGCONFIG(TAG, "  XML polling: %s", this->enable_xml_poll_ ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  XML mapping Quelle: %s", this->xml_mapping_path_.c_str());
  ESP_LOGCONFIG(TAG, "  XML poll interval: %u ms", static_cast<unsigned>(this->xml_poll_interval_ms_));
  if (this->enable_xml_poll_) {
    this->log_xml_mapping_status_(true);
    ESP_LOGCONFIG(TAG, "  XML mapping loaded: %s", YESNO(this->xml_mapping_loaded_));
    ESP_LOGCONFIG(TAG, "  XML mapping valid: %s", YESNO(this->xml_mapping_.valid));
    ESP_LOGCONFIG(TAG, "  XML mapping commands: TR32=%s (%u Felder), TG43=%s (%u Felder), TGC0=%s (%u Felder)",
                  YESNO(!this->xml_mapping_.tr32.empty()),
                  static_cast<unsigned>(this->xml_mapping_.tr32.fields.size()),
                  YESNO(!this->xml_mapping_.tg43.empty()),
                  static_cast<unsigned>(this->xml_mapping_.tg43.fields.size()),
                  YESNO(!this->xml_mapping_.tgc0.empty()),
                  static_cast<unsigned>(this->xml_mapping_.tgc0.fields.size()));
  } else {
    ESP_LOGCONFIG(TAG, "  XML mapping loaded: %s", YESNO(false));
  }
  ESP_LOGCONFIG(TAG, "  XML sensors: %u", static_cast<unsigned>(this->xml_sensors_.size()));
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
        if (this->enable_xml_poll_) {
          this->ensure_xml_mapping_loaded_();
        }
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
    this->reset_xml_poll_state_();
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
  this->reset_xml_poll_state_();
}

bool JuraComponent::read_handshake_bytes() {
  if (this->connection_ == nullptr) {
    return false;
  }
  bool read_any = false;
  std::string line;
  while (this->connection_->read_line_until(line)) {
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

bool JuraComponent::decode_field_value_(const std::vector<uint8_t> &decoded, const XmlField &field,
                                        bool little_endian, std::uint64_t &out) const {
  if (field.offset + field.size > decoded.size()) {
    ESP_LOGW(TAG, "XML @TG:C0 Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", field.name.c_str(),
             static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
             static_cast<unsigned>(decoded.size()));
    return false;
  }
  out = 0;
  if (little_endian) {
    for (std::size_t i = 0; i < field.size; ++i) {
      out |= static_cast<std::uint64_t>(decoded[field.offset + i]) << (8U * i);
    }
  } else {
    for (std::size_t i = 0; i < field.size; ++i) {
      out = (out << 8U) | static_cast<std::uint64_t>(decoded[field.offset + i]);
    }
  }
  return true;
}

bool JuraComponent::stage_tgc0_value_(const std::string &name, const std::string &label, float raw_percent,
                                      uint16_t header_value, uint16_t encoded_value, uint16_t raw_value) {
  auto &state = this->tgc0_filters_[name];
  if (std::isnan(raw_percent)) {
    state.window.clear();
    state.consecutive_valid = 0;
    return false;
  }
  state.window.push_back(raw_percent);
  if (state.window.size() > 3) {
    state.window.pop_front();
  }
  if (state.consecutive_valid < std::numeric_limits<uint8_t>::max()) {
    state.consecutive_valid += 1;
  }
  if (state.consecutive_valid < 2) {
    return false;
  }
  float sum = 0.0f;
  for (float value : state.window) {
    sum += value;
  }
  float filtered = sum / static_cast<float>(state.window.size());
  this->xml_stats_.set_value(name, filtered, label);
  if (!state.logged_once) {
    std::string log_label = label.empty() ? name : label;
    ESP_LOGD(TAG, "TGC0 %s Frame: header=0x%02X encoded=0x%02X raw=%u percent=%.1f", log_label.c_str(),
             static_cast<unsigned>(header_value), static_cast<unsigned>(encoded_value),
             static_cast<unsigned>(raw_value), static_cast<double>(filtered));
    state.logged_once = true;
  }
  return true;
}

bool JuraComponent::process_tgc0_response_(const std::vector<uint8_t> &decoded) {
  const auto &mapping = this->xml_mapping_.tgc0;
  if (mapping.empty()) {
    ESP_LOGW(TAG, "XML TGC0: kein Mapping aktiv");
    return false;
  }
  bool first_frame_debug = !this->tgc0_first_frame_logged_;
  std::size_t raw_len = decoded.size();
  bool trimmed = false;
  bool had_crlf = false;
  std::vector<std::string> first_frame_logs;
  int start_index = -1;
  for (std::size_t i = 0; i < decoded.size(); ++i) {
    if (decoded[i] == 0x26) {
      start_index = static_cast<int>(i);
      break;
    }
  }
  if (start_index < 0) {
    ESP_LOGW(TAG, "XML @TG:C0: Startbyte 0x26 nicht gefunden (decoded_len=%u)", static_cast<unsigned>(decoded.size()));
    return false;
  }
  trimmed = start_index > 0;
  for (std::size_t i = 0; i + 1 < decoded.size(); ++i) {
    if (decoded[i] == '\r' && decoded[i + 1] == '\n') {
      had_crlf = true;
      break;
    }
  }
  std::size_t avail = raw_len - static_cast<std::size_t>(start_index);
  std::size_t expected_len = 0;
  for (const auto &field : mapping.fields) {
    expected_len = std::max(expected_len, field.offset + field.size);
  }
  std::size_t expected_min_len = std::max<std::size_t>(expected_len, TGC0_MIN_FRAME_LENGTH);
  if (avail < expected_min_len) {
    ESP_LOGW(TAG, "XML @TG:C0: decoded_len ab Start (%u) < expected_min_len (%u)", static_cast<unsigned>(avail),
             static_cast<unsigned>(expected_min_len));
    return false;
  }

  std::vector<uint8_t> frame(decoded.begin() + start_index, decoded.end());
  std::size_t payload_len = frame.size();

  ESP_LOGD(TAG, "XML frame: cmd=@TG:C0 decoded_len=%u start=%u expected_min_len=%u", static_cast<unsigned>(raw_len),
           static_cast<unsigned>(start_index), static_cast<unsigned>(expected_min_len));
  std::string hex_head = format_hex_head(frame, 32);
  if (!hex_head.empty()) {
    ESP_LOGD(TAG, "XML @TG:C0 hex head: %s", hex_head.c_str());
  }
  std::string payload_hex = format_hex_string(frame);
  if (!payload_hex.empty()) {
    ESP_LOGD(TAG, "XML @TG:C0 payload HEX: %s", payload_hex.c_str());
  }
  std::string payload_ascii = format_ascii_string(frame);
  if (!payload_ascii.empty()) {
    ESP_LOGD(TAG, "XML @TG:C0 payload ASCII: %s", payload_ascii.c_str());
  }

  bool any_value = false;
  bool had_valid_sample = false;
  for (const auto &field : mapping.fields) {
    if (field.size < 4) {
      ESP_LOGW(TAG, "XML @TG:C0 Feld %s ignoriert (nicht unterstützte Größe %u)", field.name.c_str(),
               static_cast<unsigned>(field.size));
      continue;
    }
    if (field.offset + field.size > frame.size()) {
      ESP_LOGW(TAG, "XML @TG:C0 Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", field.name.c_str(),
               static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               static_cast<unsigned>(frame.size()));
      continue;
    }
    std::size_t header_offset = field.offset;
    uint16_t header_value = frame[header_offset];
    uint16_t encoded_value = frame[header_offset + 1];

    XmlField raw_field = field;
    raw_field.offset = field.offset + field.size - 2;
    raw_field.size = 2;
    std::uint64_t raw_value = 0;
    bool little_endian = raw_field.has_endian ? raw_field.little_endian : TGC0_TRY_LITTLE_ENDIAN_FIRST;
    bool little_endian_used = little_endian;
    if (!this->decode_field_value_(frame, raw_field, little_endian, raw_value)) {
      continue;
    }

    auto compute_percent = [&](std::uint64_t raw) -> double {
      double base = static_cast<double>(raw);
      if (field.has_add || std::fabs(field.scale - 1.0) > 1e-6) {
        double scaled = base * field.scale;
        if (field.has_add) {
          scaled += field.add;
        }
        return scaled;
      }
      return base / TGC0_DEFAULT_DIVISOR;
    };

    double percent = compute_percent(raw_value);
    if (!field.has_endian && raw_field.size == 2 && percent > XML_MEASUREMENT_MAX) {
      std::uint64_t alt_raw = 0;
      if (this->decode_field_value_(frame, raw_field, !little_endian, alt_raw)) {
        double alt_percent = compute_percent(alt_raw);
        if (alt_percent <= XML_MEASUREMENT_MAX) {
          raw_value = alt_raw;
          percent = alt_percent;
          little_endian_used = !little_endian;
        }
      }
    }

    if (!std::isfinite(percent)) {
      auto &state = this->tgc0_filters_[field.name];
      state.window.clear();
      state.consecutive_valid = 0;
      continue;
    }

    if (percent < XML_MEASUREMENT_MIN || percent > XML_MEASUREMENT_MAX) {
      ESP_LOGD(TAG, "TGC0 out-of-range verworfen: %s raw=%llu percent=%.2f", field.name.c_str(),
               static_cast<unsigned long long>(raw_value), percent);
      auto &state = this->tgc0_filters_[field.name];
      state.window.clear();
      state.consecutive_valid = 0;
      continue;
    }

    had_valid_sample = true;
    float clamped_percent = static_cast<float>(std::clamp(percent, XML_MEASUREMENT_MIN, XML_MEASUREMENT_MAX));
    if (first_frame_debug) {
      char buffer[128];
      std::snprintf(buffer, sizeof(buffer), "%s Byteorder=%s Wert=%.1f%%", field.name.c_str(),
                    little_endian_used ? "LE" : "BE", static_cast<double>(clamped_percent));
      first_frame_logs.emplace_back(buffer);
    }
    if (this->stage_tgc0_value_(field.name, field.label, clamped_percent, header_value, encoded_value,
                                static_cast<uint16_t>(raw_value & 0xFFFFu))) {
      any_value = true;
    }
  }

  if (!any_value && !had_valid_sample) {
    ESP_LOGW(TAG, "XML TGC0: keine Daten empfangen");
  }
  if (first_frame_debug && had_valid_sample) {
    ESP_LOGD(TAG, "TGC0 Debug: raw_len=%u payload_len=%u crlf=%s trimmed=%s",
             static_cast<unsigned>(raw_len), static_cast<unsigned>(payload_len), YESNO(had_crlf), YESNO(trimmed));
    for (const auto &entry : first_frame_logs) {
      ESP_LOGD(TAG, "  %s", entry.c_str());
    }
    this->tgc0_first_frame_logged_ = true;
  }
  return any_value;
}

void JuraComponent::add_configured_xml_sensor(const std::string &field, sensor::Sensor *sensor) {
  if (field.empty() || sensor == nullptr) {
    return;
  }
  this->xml_sensors_[field] = sensor;
  this->xml_unconfigured_sensor_logged_.erase(field);
  if (this->xml_sensor_meta_.find(field) != this->xml_sensor_meta_.end()) {
    this->get_or_create_sensor_(field, field);
  }
}

void JuraComponent::publish_xml_stats_() {
  if (!this->enable_xml_poll_) {
    return;
  }
  if (this->xml_stats_.empty()) {
    return;
  }

  struct PendingStat {
    std::string name;
    double value;
    std::string label;
  };

  const auto &stats = this->xml_stats_.values();
  std::vector<PendingStat> pending;
  pending.reserve(stats.size());
  for (const auto &entry : stats) {
    pending.push_back({entry.first, entry.second.value, entry.second.label});
  }

  auto to_lower = [](std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  };

  auto command_key_for = [&](const XmlSensorMeta &meta, const PendingStat &item) {
    std::string key = meta.command_label;
    if (key.empty()) {
      key = item.name;
    }
    return to_lower(key);
  };

  std::unordered_map<std::string, double> partial_sums;
  std::unordered_map<std::string, std::vector<PendingStat>> totals;

  for (const auto &item : pending) {
    auto meta_it = this->xml_sensor_meta_.find(item.name);
    if (meta_it == this->xml_sensor_meta_.end()) {
      continue;
    }
    const auto &meta = meta_it->second;
    if (meta.kind != XmlSensorKind::Counter) {
      continue;
    }
    std::string lower_name = to_lower(item.name);
    std::string lower_label = to_lower(item.label);
    bool is_total = lower_name.find("total") != std::string::npos ||
                    lower_name.find("gesamt") != std::string::npos ||
                    lower_label.find("total") != std::string::npos ||
                    lower_label.find("gesamt") != std::string::npos;
    std::string command_key = command_key_for(meta, item);
    if (is_total) {
      totals[command_key].push_back(item);
    } else {
      partial_sums[command_key] += item.value;
    }
  }

  for (const auto &item : pending) {
    bool skip = false;
    auto meta_it = this->xml_sensor_meta_.find(item.name);
    if (meta_it != this->xml_sensor_meta_.end()) {
      const auto &meta = meta_it->second;
      if (meta.kind == XmlSensorKind::Counter) {
        std::string command_key = command_key_for(meta, item);
        auto total_it = totals.find(command_key);
        if (total_it != totals.end() && total_it->second.size() == 1 && total_it->second.front().name == item.name) {
          double sum_value = partial_sums[command_key];
          if (sum_value > item.value + 0.5) {
            ESP_LOGW(TAG, "XML counter incoherent total, skipped: %s total=%.1f sum=%.1f", item.name.c_str(),
                     static_cast<double>(item.value), static_cast<double>(sum_value));
            skip = true;
          }
        }
      }
    }
    if (!skip) {
      this->publish_single_stat_(item.name, item.value, item.label);
    }
  }
}

void JuraComponent::publish_single_stat_(const std::string &name, double value, const std::string &label) {
  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it == this->xml_sensor_meta_.end()) {
    XmlSensorMeta fallback_meta;
    fallback_meta.kind = XmlSensorKind::Measurement;
    fallback_meta.min_value = XML_MEASUREMENT_MIN;
    fallback_meta.max_value = XML_MEASUREMENT_MAX;
    fallback_meta.accuracy_decimals = 2;
    fallback_meta.configured = true;
    meta_it = this->xml_sensor_meta_.emplace(name, fallback_meta).first;
  }
  auto &meta = meta_it->second;
  auto *sensor = this->get_or_create_sensor_(name, label);
  if (sensor == nullptr) {
    auto logged_it = this->xml_unconfigured_sensor_logged_.find(name);
    bool already_logged = logged_it != this->xml_unconfigured_sensor_logged_.end() && logged_it->second;
    if (!already_logged) {
      ESP_LOGD(TAG, "XML Feld %s ist nicht in der YAML als Sensor verlinkt – Wert wird verworfen", name.c_str());
      this->xml_unconfigured_sensor_logged_[name] = true;
    }
    return;
  }

  uint32_t now = esphome::millis();
  float publish_value = static_cast<float>(value);
  float raw_value = publish_value;
  if (meta.is_percent) {
    float scaled_value = publish_value;
    bool scaled = false;
    if (scaled_value > 200.0f) {
      scaled_value /= 10.0f;
      scaled = true;
    }
    float clamped_value = std::clamp(scaled_value, 0.0f, 100.0f);
    if (scaled || std::fabs(clamped_value - scaled_value) > 0.1f) {
      ESP_LOGD(TAG, "XML percent clamp %s: raw=%.1f scaled=%.1f clamped=%.1f", name.c_str(),
               static_cast<double>(raw_value), static_cast<double>(scaled_value),
               static_cast<double>(clamped_value));
    }
    publish_value = clamped_value;
  } else {
    double min_value = meta.min_value;
    double max_value = meta.max_value;
    if (value < min_value || value > max_value) {
      ESP_LOGD(TAG, "XML publish_state unterdrückt: %s=%.3f außerhalb %.3f..%.3f", name.c_str(), value, min_value,
               max_value);
      return;
    }
  }

  if (!std::isfinite(publish_value)) {
    ESP_LOGW(TAG, "XML publish_state unterdrückt: %s ist nicht endlich", name.c_str());
    return;
  }

  if (meta.kind == XmlSensorKind::Counter) {
    publish_value = static_cast<float>(std::round(static_cast<double>(publish_value)));
  } else if (!meta.is_percent && meta.accuracy_decimals > 0) {
    double factor = std::pow(10.0, static_cast<double>(meta.accuracy_decimals));
    publish_value = static_cast<float>(std::round(static_cast<double>(publish_value) * factor) / factor);
  }

  float tolerance = (meta.kind == XmlSensorKind::Counter) ? XML_COUNTER_TOLERANCE : XML_MEASUREMENT_TOLERANCE;
  if (meta.kind == XmlSensorKind::Measurement && meta.accuracy_decimals > 0) {
    float factor = std::pow(10.0f, static_cast<float>(meta.accuracy_decimals));
    if (factor > 0.0f) {
      tolerance = 0.5f / factor;
    }
  }
  if (meta.is_percent && tolerance < 0.05f) {
    tolerance = 0.05f;
  }

  if (meta.kind == XmlSensorKind::Counter && meta.has_last_value) {
    constexpr uint32_t COUNTER_RESET_WINDOW_MS = 6U * 60U * 60U * 1000U;
    float last_value = meta.last_value;
    float drop = last_value - publish_value;
    if (drop > tolerance) {
      bool allow_reset = false;
      float drop_ratio = last_value > 0.0f ? drop / last_value : 0.0f;
      if (drop_ratio >= 0.8f) {
        float tg43_percent = this->last_tg43_percent_;
        if (std::isfinite(tg43_percent) && tg43_percent < 5.0f) {
          allow_reset = true;
        }
      }
      uint32_t age_ms = meta.last_update_ms == 0 ? std::numeric_limits<uint32_t>::max() : now - meta.last_update_ms;
      if (!allow_reset && age_ms < COUNTER_RESET_WINDOW_MS) {
        ESP_LOGW(TAG, "XML counter drop ignored: %s last=%.1f new=%.1f", name.c_str(), static_cast<double>(last_value),
                 static_cast<double>(publish_value));
        return;
      }
    }
  }

  if (meta.has_last_value && std::fabs(publish_value - meta.last_value) < tolerance) {
    return;
  }

  sensor->publish_state(publish_value);
  meta.last_value = publish_value;
  meta.has_last_value = true;
  meta.last_update_ms = now;

  if (meta.is_percent) {
    std::string lower_name = name;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower_name.find("tg43") != std::string::npos) {
      this->last_tg43_percent_ = publish_value;
    }
  }

  if (meta.kind == XmlSensorKind::Counter) {
    ESP_LOGD(TAG, "XML publish_state: %s=%u", name.c_str(),
             static_cast<unsigned>(std::lround(static_cast<double>(publish_value))));
  } else {
    ESP_LOGD(TAG, "XML publish_state: %s=%.3f", name.c_str(), static_cast<double>(publish_value));
  }
}

void JuraComponent::register_xml_sensor_(const XmlField &field, XmlSensorKind kind, const char *command_label) {
  auto &meta = this->xml_sensor_meta_[field.name];
  meta.kind = kind;
  meta.min_value = (kind == XmlSensorKind::Counter) ? XML_COUNTER_MIN : XML_MEASUREMENT_MIN;
  meta.max_value = (kind == XmlSensorKind::Counter) ? XML_COUNTER_MAX : XML_MEASUREMENT_MAX;
  meta.accuracy_decimals = determine_accuracy(kind, field.scale);
  meta.is_tgc0 = false;
  meta.is_percent = false;
  meta.has_unit = false;
  meta.unit_of_measurement.clear();
  meta.has_icon = false;
  meta.icon.clear();

  std::string lower_name = field.name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::string command = command_label != nullptr ? command_label : "";
  meta.command_label = command;
  std::string lower_command = command;
  std::transform(lower_command.begin(), lower_command.end(), lower_command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  bool is_tgc0 = lower_command == "@tg:c0" || lower_name.find("tgc0") != std::string::npos;
  bool is_tg43 = lower_command == "@tg:43" || lower_name.find("tg43") != std::string::npos;
  bool label_has_percent = field.label.find('%') != std::string::npos;
  bool name_has_percent = lower_name.find("percent") != std::string::npos;

  if (kind == XmlSensorKind::Measurement && (is_tgc0 || is_tg43 || label_has_percent || name_has_percent)) {
    meta.min_value = 0.0;
    meta.max_value = 100.0;
    meta.is_percent = true;
    if (meta.accuracy_decimals < 1) {
      meta.accuracy_decimals = 1;
    }
    meta.has_unit = true;
    meta.unit_of_measurement = "%";
    if (is_tgc0) {
      meta.has_icon = true;
      meta.icon = "mdi:percent";
      meta.is_tgc0 = true;
    }
  }

  meta.configured = true;
  this->get_or_create_sensor_(field.name, field.label);
}

void JuraComponent::ensure_xml_sensors_created_() {
  if (!this->enable_xml_poll_) {
    return;
  }
  auto ensure_block = [&](const XmlCommandMapping &mapping, XmlSensorKind kind, const char *command) {
    if (mapping.empty()) {
      return;
    }
    for (const auto &field : mapping.fields) {
      this->register_xml_sensor_(field, kind, command);
    }
  };
  ensure_block(this->xml_mapping_.tr32, XmlSensorKind::Counter, "@TR:32");
  ensure_block(this->xml_mapping_.tg43, XmlSensorKind::Measurement, "@TG:43");
  ensure_block(this->xml_mapping_.tgc0, XmlSensorKind::Measurement, "@TG:C0");

  for (const auto &entry : this->xml_sensors_) {
    if (this->xml_sensor_meta_.find(entry.first) != this->xml_sensor_meta_.end()) {
      continue;
    }
    if (this->xml_missing_sensor_logged_[entry.first]) {
      continue;
    }
    ESP_LOGW(TAG, "XML Sensor %s ist im aktuellen Mapping nicht vorhanden", entry.first.c_str());
    this->xml_missing_sensor_logged_[entry.first] = true;
  }
}

bool JuraComponent::ensure_xml_mapping_loaded_() {
  if (!this->enable_xml_poll_) {
    return false;
  }
  if (this->xml_mapping_loaded_) {
    return this->xml_mapping_.valid;
  }

  if (this->xml_mapping_data_ == nullptr || this->xml_mapping_length_ == 0) {
    ESP_LOGW(TAG, "Kein XML-Mapping verfügbar (Quelle: %s)",
             this->xml_mapping_path_.c_str());
    this->xml_mapping_loaded_ = true;
    this->xml_mapping_ = {};
    this->xml_stats_.clear();
    this->xml_missing_sensor_logged_.clear();
    this->log_xml_mapping_status_();
    return false;
  }

  std::string xml_source(this->xml_mapping_data_, this->xml_mapping_length_);
  bool valid = load_mapping_from_string(xml_source);
  this->xml_mapping_ = get_xml_mapping();
  this->xml_mapping_loaded_ = true;
  this->xml_mapping_logged_ = false;
  this->xml_stats_.clear();
  this->xml_sensor_meta_.clear();
  this->tgc0_filters_.clear();
  this->tgc0_first_frame_logged_ = false;
  this->xml_missing_sensor_logged_.clear();
  this->log_xml_mapping_status_();
  if (this->xml_mapping_.valid) {
    this->ensure_xml_sensors_created_();
  }
  return this->xml_mapping_.valid;
}

void JuraComponent::log_xml_mapping_status_(bool force) {
  if (this->xml_mapping_logged_ && !force) {
    return;
  }
  ESP_LOGCONFIG(TAG,
                "  XML mapping Status: geladen=%s, gültig=%s, TR32=%s, TG43=%s, TGC0=%s",
                YESNO(this->xml_mapping_loaded_), YESNO(this->xml_mapping_.valid),
                YESNO(!this->xml_mapping_.tr32.empty()), YESNO(!this->xml_mapping_.tg43.empty()),
                YESNO(!this->xml_mapping_.tgc0.empty()));
  this->xml_mapping_logged_ = true;
}

void JuraComponent::reset_xml_poll_state_() {
  this->xml_state_ = XmlPollState::IDLE;
  this->xml_deadline_ms_ = 0;
  this->xml_next_action_ms_ = 0;
  this->xml_inflight_ = false;
  this->xml_last_command_.clear();
  this->xml_rx_buffer_.clear();
  this->xml_stats_.clear();
  this->tgc0_first_frame_logged_ = false;
  this->xml_next_poll_ = esphome::millis();
}

void JuraComponent::process_xml_polling() {
  if (!this->enable_xml_poll_) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  if (!this->ensure_xml_mapping_loaded_()) {
    return;
  }
  this->ensure_xml_sensors_created_();
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }

  uint32_t now = esphome::millis();
  this->handle_xml_state_machine_(now);
}


void JuraComponent::handle_xml_state_machine_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }

  if (this->xml_next_action_ms_ != 0 && static_cast<int32_t>(now - this->xml_next_action_ms_) < 0) {
    return;
  }

  auto *connection = this->coffee_maker_->connection.get();

  if (this->xml_state_ == XmlPollState::IDLE) {
    if (this->xml_next_poll_ != 0 && !time_reached(now, this->xml_next_poll_)) {
      return;
    }
    bool has_mapping = this->xml_state_has_mapping_(XmlPollState::SEND_TR32) ||
                       this->xml_state_has_mapping_(XmlPollState::SEND_TG43) ||
                       this->xml_state_has_mapping_(XmlPollState::SEND_TGC0);
    if (!has_mapping) {
      ESP_LOGW(TAG, "XML Polling übersprungen - kein Mapping aktiv");
      uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
      uint32_t deadline = now + sleep;
      this->xml_deadline_ms_ = deadline;
      this->xml_next_poll_ = deadline;
      this->transition_to_state_(XmlPollState::SLEEP, now);
      return;
    }
    this->xml_stats_.clear();
    this->xml_rx_buffer_.clear();
    this->xml_inflight_ = false;
    this->xml_last_command_.clear();
    this->transition_to_state_(XmlPollState::SEND_TR32, now);
  }

  if (this->xml_next_action_ms_ != 0 && static_cast<int32_t>(now - this->xml_next_action_ms_) < 0) {
    return;
  }

  switch (this->xml_state_) {
    case XmlPollState::SEND_TR32: {
      if (this->xml_inflight_) {
        return;
      }
      if (!this->xml_state_has_mapping_(XmlPollState::SEND_TR32)) {
        this->transition_to_state_(XmlPollState::SEND_TG43, now, kInterCmdGapMs);
        return;
      }
      this->send_xml_command_(this->xml_state_command_(XmlPollState::SEND_TR32), XmlPollState::WAIT_TR32, now);
      return;
    }
    case XmlPollState::SEND_TG43: {
      if (this->xml_inflight_) {
        return;
      }
      if (!this->xml_state_has_mapping_(XmlPollState::SEND_TG43)) {
        this->transition_to_state_(XmlPollState::SEND_TGC0, now, kInterCmdGapMs);
        return;
      }
      this->send_xml_command_(this->xml_state_command_(XmlPollState::SEND_TG43), XmlPollState::WAIT_TG43, now);
      return;
    }
    case XmlPollState::SEND_TGC0: {
      if (this->xml_inflight_) {
        return;
      }
      if (!this->xml_state_has_mapping_(XmlPollState::SEND_TGC0)) {
        uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
        uint32_t deadline = now + sleep;
        this->xml_deadline_ms_ = deadline;
        this->xml_next_poll_ = deadline;
        this->transition_to_state_(XmlPollState::SLEEP, now);
        return;
      }
      this->send_xml_command_(this->xml_state_command_(XmlPollState::SEND_TGC0), XmlPollState::WAIT_TGC0, now);
      return;
    }
    case XmlPollState::WAIT_TR32:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::WAIT_TGC0: {
      if (!this->xml_inflight_) {
        XmlPollState next = (this->xml_state_ == XmlPollState::WAIT_TR32)
                                ? XmlPollState::SEND_TG43
                                : (this->xml_state_ == XmlPollState::WAIT_TG43 ? XmlPollState::SEND_TGC0
                                                                               : XmlPollState::SLEEP);
        if (next == XmlPollState::SLEEP) {
          uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
          uint32_t deadline = now + sleep;
          this->xml_deadline_ms_ = deadline;
          this->xml_next_poll_ = deadline;
          this->transition_to_state_(XmlPollState::SLEEP, now);
        } else {
          this->transition_to_state_(next, now, kInterCmdGapMs);
        }
        return;
      }
      std::vector<uint8_t> decoded;
      if (connection->read_db_frame(decoded, 0)) {
        const char *expected_command = this->xml_state_command_(this->xml_state_);
        if (!this->xml_last_command_.empty() && expected_command != nullptr && expected_command[0] != '\0' &&
            this->xml_last_command_ != expected_command) {
          ESP_LOGW(TAG, "RX_DB Frame ignoriert: erwartet %s, letzter Befehl %s", expected_command,
                   this->xml_last_command_.c_str());
          return;
        }
        this->xml_inflight_ = false;
        this->xml_deadline_ms_ = 0;
        this->xml_rx_buffer_ = std::move(decoded);
        XmlPollState parse_state = (this->xml_state_ == XmlPollState::WAIT_TR32)
                                       ? XmlPollState::PARSE_TR32
                                       : (this->xml_state_ == XmlPollState::WAIT_TG43 ? XmlPollState::PARSE_TG43
                                                                                      : XmlPollState::PARSE_TGC0);
        this->transition_to_state_(parse_state, now);
        return;
      }
      if (this->xml_deadline_ms_ != 0 && static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        XmlPollState next_state = (this->xml_state_ == XmlPollState::WAIT_TR32)
                                      ? XmlPollState::SEND_TG43
                                      : (this->xml_state_ == XmlPollState::WAIT_TG43 ? XmlPollState::SEND_TGC0
                                                                                     : XmlPollState::SLEEP);
        this->handle_xml_timeout_(next_state, this->xml_state_label_(this->xml_state_), now);
      }
      return;
    }
    case XmlPollState::PARSE_TR32: {
      this->xml_inflight_ = false;
      this->xml_deadline_ms_ = 0;
      this->xml_stats_.clear();
      bool any = false;
      if (!this->xml_rx_buffer_.empty() && this->xml_state_has_mapping_(XmlPollState::PARSE_TR32)) {
        any = parse_TR32(this->xml_rx_buffer_, this->xml_stats_);
      }
      this->xml_rx_buffer_.clear();
      if (any) {
        this->publish_xml_stats_();
      }
      this->xml_stats_.clear();
      this->transition_to_state_(XmlPollState::SEND_TG43, now, kInterCmdGapMs);
      return;
    }
    case XmlPollState::PARSE_TG43: {
      this->xml_inflight_ = false;
      this->xml_deadline_ms_ = 0;
      this->xml_stats_.clear();
      bool any = false;
      if (!this->xml_rx_buffer_.empty() && this->xml_state_has_mapping_(XmlPollState::PARSE_TG43)) {
        any = parse_TG43(this->xml_rx_buffer_, this->xml_stats_);
      }
      this->xml_rx_buffer_.clear();
      if (any) {
        this->publish_xml_stats_();
      }
      this->xml_stats_.clear();
      this->transition_to_state_(XmlPollState::SEND_TGC0, now, kInterCmdGapMs);
      return;
    }
    case XmlPollState::PARSE_TGC0: {
      this->xml_inflight_ = false;
      this->xml_deadline_ms_ = 0;
      this->xml_stats_.clear();
      bool any = false;
      if (!this->xml_rx_buffer_.empty() && this->xml_state_has_mapping_(XmlPollState::PARSE_TGC0)) {
        any = this->process_tgc0_response_(this->xml_rx_buffer_);
      }
      this->xml_rx_buffer_.clear();
      if (any) {
        this->publish_xml_stats_();
      }
      this->xml_stats_.clear();
      uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
      uint32_t deadline = now + sleep;
      this->xml_deadline_ms_ = deadline;
      this->xml_next_poll_ = deadline;
      this->transition_to_state_(XmlPollState::SLEEP, now);
      return;
    }
    case XmlPollState::SLEEP: {
      if (this->xml_deadline_ms_ == 0) {
        this->xml_state_ = XmlPollState::IDLE;
        this->xml_next_action_ms_ = 0;
        this->xml_next_poll_ = now;
        return;
      }
      if (static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        this->xml_deadline_ms_ = 0;
        this->xml_next_action_ms_ = 0;
        this->xml_state_ = XmlPollState::IDLE;
        this->xml_next_poll_ = now;
        this->xml_inflight_ = false;
        this->xml_last_command_.clear();
      }
      return;
    }
    case XmlPollState::IDLE:
    default:
      return;
  }
}

bool JuraComponent::xml_state_has_mapping_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return !this->xml_mapping_.tr32.empty();
    case XmlPollState::SEND_TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::PARSE_TG43:
      return !this->xml_mapping_.tg43.empty();
    case XmlPollState::SEND_TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::PARSE_TGC0:
      return !this->xml_mapping_.tgc0.empty();
    case XmlPollState::IDLE:
    case XmlPollState::SLEEP:
    default:
      return true;
  }
}

const char *JuraComponent::xml_state_command_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return "@TR:32";
    case XmlPollState::SEND_TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::PARSE_TG43:
      return "@TG:43";
    case XmlPollState::SEND_TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::PARSE_TGC0:
      return "@TG:C0";
    case XmlPollState::IDLE:
    case XmlPollState::SLEEP:
    default:
      break;
  }
  return "";
}

const char *JuraComponent::xml_state_label_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return "TR32";
    case XmlPollState::SEND_TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::PARSE_TG43:
      return "TG43";
    case XmlPollState::SEND_TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::PARSE_TGC0:
      return "TGC0";
    case XmlPollState::IDLE:
      return "IDLE";
    case XmlPollState::SLEEP:
      return "SLEEP";
    default:
      break;
  }
  return "?";
}

void JuraComponent::transition_to_state_(XmlPollState state, uint32_t now, uint32_t delay_ms) {
  this->xml_state_ = state;
  if (delay_ms > 0) {
    this->xml_next_action_ms_ = now + delay_ms;
  } else {
    this->xml_next_action_ms_ = 0;
  }
  if (state != XmlPollState::WAIT_TR32 && state != XmlPollState::WAIT_TG43 && state != XmlPollState::WAIT_TGC0) {
    this->xml_deadline_ms_ = 0;
  }
}

bool JuraComponent::send_xml_command_(const char *command, XmlPollState wait_state, uint32_t now) {
  if (command == nullptr || command[0] == '\0') {
    this->transition_to_state_(wait_state, now);
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (this->xml_inflight_) {
    return false;
  }
  auto *connection = this->coffee_maker_->connection.get();
  if (wait_state == XmlPollState::WAIT_TGC0) {
    this->prepare_tgc0_request_();
  } else {
    connection->reset_db_rx_buffer();
  }
  this->xml_rx_buffer_.clear();
  ESP_LOGD(TAG, "TX_DB \"%s\"", command);
  connection->tx_db_command(command);
  this->xml_inflight_ = true;
  this->xml_last_command_ = command;
  this->xml_deadline_ms_ = now + kXmlRxTimeoutMs;
  this->xml_next_action_ms_ = 0;
  this->xml_state_ = wait_state;
  return true;
}

void JuraComponent::handle_xml_timeout_(XmlPollState next_state, const char *label, uint32_t now) {
  if (!this->xml_inflight_) {
    return;
  }
  if (label == nullptr) {
    label = "?";
  }
  ESP_LOGW(TAG, "RX_DB timeout %s", label);
  this->xml_inflight_ = false;
  this->xml_deadline_ms_ = 0;
  this->xml_rx_buffer_.clear();
  if (next_state == XmlPollState::SLEEP) {
    uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
    uint32_t deadline = now + sleep;
    this->xml_deadline_ms_ = deadline;
    this->xml_next_poll_ = deadline;
    this->transition_to_state_(XmlPollState::SLEEP, now);
  } else {
    this->transition_to_state_(next_state, now, kInterCmdGapMs);
  }
}

void JuraComponent::prepare_tgc0_request_() {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->drain_serial_input_nonblocking();
  connection->reset_db_rx_buffer();
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

