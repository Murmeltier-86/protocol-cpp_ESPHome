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
constexpr uint32_t XML_RX_TIMEOUT_MS = 900;
constexpr uint32_t XML_INTER_COMMAND_GAP_MS = 120;
constexpr uint32_t XML_CYCLE_SLEEP_MS = 2000;

constexpr double XML_COUNTER_MIN = 0.0;
constexpr double XML_COUNTER_MAX = 1'000'000.0;
constexpr double XML_MEASUREMENT_MIN = 0.0;
constexpr double XML_MEASUREMENT_MAX = 250.0;
constexpr float XML_COUNTER_TOLERANCE = 0.5f;
constexpr float XML_MEASUREMENT_TOLERANCE = 0.1f;

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

  this->reset_xml_cycle_state_();
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
    this->reset_xml_cycle_state_();
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
  this->reset_xml_cycle_state_();
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

bool JuraComponent::ensure_xml_mapping_loaded_() {
  if (!this->enable_xml_poll_) {
    return false;
  }
  if (this->xml_mapping_loaded_) {
    return this->xml_mapping_.valid;
  }

  if (this->xml_mapping_data_ == nullptr || this->xml_mapping_length_ == 0) {
    ESP_LOGW(TAG, "Kein XML-Mapping verfügbar (Quelle: %s)", this->xml_mapping_path_.c_str());
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
  this->xml_missing_sensor_logged_.clear();
  this->log_xml_mapping_status_();
  if (this->xml_mapping_.valid) {
    this->ensure_xml_sensors_created_();
  }
  return this->xml_mapping_.valid && valid;
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

void JuraComponent::register_xml_sensor_(const XmlField &field, XmlSensorKind kind) {
  auto &meta = this->xml_sensor_meta_[field.name];
  meta.kind = kind;
  meta.min_value = (kind == XmlSensorKind::Counter) ? XML_COUNTER_MIN : XML_MEASUREMENT_MIN;
  meta.max_value = (kind == XmlSensorKind::Counter) ? XML_COUNTER_MAX : XML_MEASUREMENT_MAX;
  meta.accuracy_decimals = determine_accuracy(kind, field.scale);
  std::string lower_name = field.name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  bool is_tgc0 = kind == XmlSensorKind::Measurement && lower_name.find("tgc0") != std::string::npos;
  if (is_tgc0) {
    meta.accuracy_decimals = 1;
    meta.has_unit = true;
    meta.unit_of_measurement = "%";
    meta.has_icon = true;
    meta.icon = "mdi:percent";
    meta.is_tgc0 = true;
  }
  meta.configured = true;
  this->get_or_create_sensor_(field.name, field.label);
}

void JuraComponent::ensure_xml_sensors_created_() {
  if (!this->enable_xml_poll_) {
    return;
  }
  auto ensure_block = [&](const XmlCommandMapping &mapping, XmlSensorKind kind) {
    if (mapping.empty()) {
      return;
    }
    for (const auto &field : mapping.fields) {
      this->register_xml_sensor_(field, kind);
    }
  };
  ensure_block(this->xml_mapping_.tr32, XmlSensorKind::Counter);
  ensure_block(this->xml_mapping_.tg43, XmlSensorKind::Counter);
  ensure_block(this->xml_mapping_.tgc0, XmlSensorKind::Measurement);

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



void JuraComponent::reset_xml_cycle_state_() {
  this->xml_state_ = XmlPollState::IDLE;
  this->xml_inflight_ = false;
  this->xml_deadline_ms_ = 0;
  this->xml_next_action_ms_ = 0;
  this->xml_last_command_.clear();
  this->xml_pending_frame_.clear();
  this->xml_cycle_had_value_ = false;
  this->xml_stats_.clear();
  this->xml_next_poll_ = 0;
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

  uint32_t now = esphome::millis();

  switch (this->xml_state_) {
    case XmlPollState::IDLE: {
      if (this->xml_next_poll_ != 0 && !time_reached(now, this->xml_next_poll_)) {
        return;
      }
      this->xml_stats_.clear();
      this->xml_pending_frame_.clear();
      this->xml_cycle_had_value_ = false;
      this->xml_inflight_ = false;
      this->xml_last_command_.clear();
      this->xml_deadline_ms_ = 0;
      this->xml_next_action_ms_ = now;
      this->xml_state_ = XmlPollState::SEND_TR32;
      ESP_LOGV(TAG, "XML Poll start");
      break;
    }
    case XmlPollState::SEND_TR32:
      if (this->send_xml_command_("@TR:32", "TR32", now)) {
        this->xml_state_ = XmlPollState::WAIT_TR32;
      }
      break;
    case XmlPollState::WAIT_TR32:
      this->handle_wait_state_("@TR:32", "TR32", XmlPollState::PARSE_TR32, XmlPollState::SEND_TG43,
                               this->xml_mapping_.tr32, now);
      break;
    case XmlPollState::PARSE_TR32:
      this->handle_parse_state_("@TR:32", "TR32", this->xml_mapping_.tr32, XmlPollState::SEND_TG43, now);
      break;
    case XmlPollState::SEND_TG43:
      if (this->send_xml_command_("@TG:43", "TG43", now)) {
        this->xml_state_ = XmlPollState::WAIT_TG43;
      }
      break;
    case XmlPollState::WAIT_TG43:
      this->handle_wait_state_("@TG:43", "TG43", XmlPollState::PARSE_TG43, XmlPollState::SEND_TGC0,
                               this->xml_mapping_.tg43, now);
      break;
    case XmlPollState::PARSE_TG43:
      this->handle_parse_state_("@TG:43", "TG43", this->xml_mapping_.tg43, XmlPollState::SEND_TGC0, now);
      break;
    case XmlPollState::SEND_TGC0:
      if (this->send_xml_command_("@TG:C0", "TGC0", now)) {
        this->xml_state_ = XmlPollState::WAIT_TGC0;
      }
      break;
    case XmlPollState::WAIT_TGC0:
      this->handle_wait_state_("@TG:C0", "TGC0", XmlPollState::PARSE_TGC0, XmlPollState::SLEEP,
                               this->xml_mapping_.tgc0, now);
      break;
    case XmlPollState::PARSE_TGC0:
      this->handle_parse_state_("@TG:C0", "TGC0", this->xml_mapping_.tgc0, XmlPollState::SLEEP, now);
      break;
    case XmlPollState::SLEEP:
      if (this->xml_deadline_ms_ != 0 && static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        this->xml_state_ = XmlPollState::IDLE;
        this->xml_deadline_ms_ = 0;
      }
      break;
  }
}

bool JuraComponent::send_xml_command_(const char *command, const char *label, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    ESP_LOGW(TAG, "XML %s: keine Verbindung verfügbar", label);
    return false;
  }
  if (this->xml_inflight_) {
    return false;
  }
  if (this->xml_next_action_ms_ != 0 && static_cast<int32_t>(now - this->xml_next_action_ms_) < 0) {
    return false;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_db_rx_buffer();
  ESP_LOGD(TAG, "TX_DB \"%s\"", command);
  connection->tx_db_command(command);
  this->xml_inflight_ = true;
  this->xml_last_command_ = command;
  this->xml_deadline_ms_ = now + XML_RX_TIMEOUT_MS;
  this->xml_pending_frame_.clear();
  return true;
}

void JuraComponent::handle_wait_state_(const char *command, const char *label, XmlPollState parse_state,
                                       XmlPollState next_send_state, const XmlCommandMapping &mapping, uint32_t now) {
  (void) command;
  (void) mapping;
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    if (this->xml_inflight_) {
      ESP_LOGW(TAG, "XML %s: Verbindung verloren während des Empfangs", label);
    }
    this->xml_inflight_ = false;
    this->xml_pending_frame_.clear();
    if (next_send_state == XmlPollState::SLEEP) {
      this->complete_xml_cycle_(now);
    } else {
      this->xml_state_ = next_send_state;
    }
    return;
  }

  auto *connection = this->coffee_maker_->connection.get();
  if (!this->xml_inflight_) {
    if (!this->xml_pending_frame_.empty()) {
      this->xml_state_ = parse_state;
    }
    return;
  }

  std::vector<uint8_t> decoded;
  if (connection->read_db_frame(decoded, 0)) {
    if (!this->xml_last_command_.empty() && this->xml_last_command_ != command) {
      ESP_LOGW(TAG, "RX_DB %s ignoriert – pending=%s state=%s", label, this->xml_last_command_.c_str(), command);
      return;
    }
    ESP_LOGD(TAG, "RX_DB %s decoded_len=%u", label, static_cast<unsigned>(decoded.size()));
    this->xml_pending_frame_ = std::move(decoded);
    this->xml_inflight_ = false;
    this->xml_deadline_ms_ = 0;
    this->xml_state_ = parse_state;
    return;
  }

  if (this->xml_deadline_ms_ != 0 && static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
    ESP_LOGW(TAG, "RX_DB timeout %s", label);
    this->xml_inflight_ = false;
    this->xml_pending_frame_.clear();
    this->xml_deadline_ms_ = 0;
    this->xml_last_command_.clear();
    this->xml_next_action_ms_ = now + XML_INTER_COMMAND_GAP_MS;
    if (next_send_state == XmlPollState::SLEEP) {
      this->complete_xml_cycle_(now);
    } else {
      this->xml_state_ = next_send_state;
    }
  }
}

void JuraComponent::handle_parse_state_(const char *command, const char *label, const XmlCommandMapping &mapping,
                                        XmlPollState next_state, uint32_t now) {
  bool parsed = false;
  if (!this->xml_pending_frame_.empty()) {
    parsed = this->parse_and_stage_frame_(command, label, mapping, this->xml_pending_frame_);
  } else {
    ESP_LOGW(TAG, "XML %s: keine Daten empfangen", label);
  }

  this->xml_pending_frame_.clear();
  this->xml_last_command_.clear();
  this->xml_inflight_ = false;
  this->xml_deadline_ms_ = 0;
  this->xml_next_action_ms_ = now + XML_INTER_COMMAND_GAP_MS;

  if (parsed) {
    this->xml_cycle_had_value_ = true;
    this->publish_xml_stats_();
  } else {
    this->xml_stats_.clear();
  }

  if (next_state == XmlPollState::SLEEP) {
    this->complete_xml_cycle_(now);
  } else {
    this->xml_state_ = next_state;
  }
}

void JuraComponent::complete_xml_cycle_(uint32_t now) {
  uint32_t sleep_ms = std::max(XML_CYCLE_SLEEP_MS, this->xml_poll_interval_ms_);
  this->xml_deadline_ms_ = now + sleep_ms;
  this->xml_next_poll_ = this->xml_deadline_ms_;
  this->xml_state_ = XmlPollState::SLEEP;
  this->xml_inflight_ = false;
  this->xml_pending_frame_.clear();
  this->xml_last_command_.clear();
  if (!this->xml_cycle_had_value_) {
    ESP_LOGW(TAG, "XML poll cycle finished without values");
  }
}

bool JuraComponent::parse_and_stage_frame_(const char *command, const char *label, const XmlCommandMapping &mapping,
                                           const std::vector<uint8_t> &frame) {
  (void) mapping;
  if (frame.empty()) {
    ESP_LOGW(TAG, "XML %s: leeres Frame", label);
    return false;
  }

  bool parsed = false;
  if (std::strcmp(command, "@TR:32") == 0) {
    parsed = parse_TR32(frame, this->xml_stats_);
  } else if (std::strcmp(command, "@TG:43") == 0) {
    parsed = parse_TG43(frame, this->xml_stats_);
  } else if (std::strcmp(command, "@TG:C0") == 0) {
    parsed = parse_TGC0(frame, this->xml_stats_);
  }

  if (!parsed) {
    this->xml_stats_.clear();
    ESP_LOGW(TAG, "XML %s: Parsing fehlgeschlagen", label);
    return false;
  }
  return !this->xml_stats_.empty();
}

bool JuraComponent::check_counter_coherence_(const std::unordered_map<std::string, StatValue> &values,
                                            std::unordered_set<std::string> &skip) const {
  std::string total_name;
  double total_value = 0.0;
  double partial_sum = 0.0;
  bool has_total = false;

  for (const auto &entry : values) {
    auto meta_it = this->xml_sensor_meta_.find(entry.first);
    if (meta_it == this->xml_sensor_meta_.end()) {
      continue;
    }
    if (meta_it->second.kind != XmlSensorKind::Counter) {
      continue;
    }
    std::string name_lower = entry.first;
    std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    bool is_total = name_lower.find("total") != std::string::npos || name_lower.find("gesamt") != std::string::npos;
    if (is_total && !has_total) {
      total_name = entry.first;
      total_value = entry.second.value;
      has_total = true;
    } else if (!is_total) {
      partial_sum += entry.second.value;
    }
  }

  if (has_total && partial_sum > 0.0 && total_value + 0.5 < partial_sum) {
    ESP_LOGW(TAG, "XML incoherent total, skipped %s (total=%.1f < sum=%.1f)", total_name.c_str(), total_value,
             partial_sum);
    skip.insert(total_name);
    return true;
  }
  return false;
}

void JuraComponent::publish_xml_stats_() {
  if (!this->enable_xml_poll_) {
    this->xml_stats_.clear();
    return;
  }
  if (this->xml_stats_.empty()) {
    return;
  }
  const auto &values = this->xml_stats_.values();
  std::unordered_set<std::string> skip_totals;
  this->check_counter_coherence_(values, skip_totals);
  for (const auto &entry : values) {
    if (skip_totals.find(entry.first) != skip_totals.end()) {
      continue;
    }
    this->publish_single_stat_(entry.first, entry.second.value, entry.second.label);
  }
  this->xml_stats_.clear();
}

void JuraComponent::publish_single_stat_(const std::string &name, double value, const std::string &label) {
  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it == this->xml_sensor_meta_.end()) {
    XmlSensorMeta meta;
    meta.kind = XmlSensorKind::Measurement;
    meta.min_value = XML_MEASUREMENT_MIN;
    meta.max_value = XML_MEASUREMENT_MAX;
    meta.accuracy_decimals = 2;
    meta.configured = true;
    meta_it = this->xml_sensor_meta_.emplace(name, meta).first;
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

  double processed_value = value;
  if (meta.kind == XmlSensorKind::Measurement) {
    bool is_percent = meta.is_tgc0 || (meta.has_unit && meta.unit_of_measurement == "%");
    double original_value = processed_value;
    if (is_percent) {
      bool scaled = false;
      if (processed_value > 200.0) {
        processed_value /= 10.0;
        scaled = true;
      }
      double clamped = std::clamp(processed_value, 0.0, 100.0);
      if (scaled || std::fabs(clamped - processed_value) > 0.001) {
        ESP_LOGD(TAG, "XML %s: clamped percent from %.1f to %.1f", name.c_str(), original_value, clamped);
      }
      processed_value = clamped;
      this->last_tgc0_percent_ = static_cast<float>(processed_value);
    }
    if (processed_value < meta.min_value || processed_value > meta.max_value) {
      ESP_LOGD(TAG, "XML publish_state unterdrückt: %s=%.3f außerhalb %.3f..%.3f", name.c_str(), processed_value,
               meta.min_value, meta.max_value);
      return;
    }
  } else {
    processed_value = std::max(processed_value, meta.min_value);
  }

  uint32_t now = esphome::millis();
  float publish_value = 0.0f;
  if (meta.kind == XmlSensorKind::Counter) {
    processed_value = std::max(processed_value, 0.0);
    publish_value = static_cast<float>(std::round(processed_value));
    if (meta.has_last_value) {
      float last = meta.last_value;
      if (publish_value + XML_COUNTER_TOLERANCE < last) {
        bool allow_reset = false;
        float ratio = (last > 0.0f) ? publish_value / last : 0.0f;
        if (ratio < 0.2f) {
          float guard = this->last_tgc0_percent_;
          if (std::isfinite(guard) && guard <= 5.0f) {
            allow_reset = true;
          }
        }
        constexpr uint32_t SIX_HOURS_MS = 6U * 60U * 60U * 1000U;
        uint32_t age = meta.last_publish_ms == 0 ? std::numeric_limits<uint32_t>::max()
                                                 : now - meta.last_publish_ms;
        if (!allow_reset && age < SIX_HOURS_MS) {
          ESP_LOGW(TAG, "XML counter drop ignored: %s last=%.0f new=%.0f", name.c_str(), last, publish_value);
          return;
        }
      }
    }
  } else {
    double rounded = processed_value;
    if (meta.accuracy_decimals > 0) {
      double factor = std::pow(10.0, static_cast<double>(meta.accuracy_decimals));
      rounded = std::round(processed_value * factor) / factor;
    }
    publish_value = static_cast<float>(rounded);
    float tolerance = XML_MEASUREMENT_TOLERANCE;
    if (meta.accuracy_decimals > 0) {
      float factor = std::pow(10.0f, static_cast<float>(meta.accuracy_decimals));
      if (factor > 0.0f) {
        tolerance = 0.5f / factor;
      }
    }
    if (meta.has_last_value && std::fabs(publish_value - meta.last_value) < tolerance) {
      return;
    }
  }

  sensor->publish_state(publish_value);
  meta.last_value = publish_value;
  meta.has_last_value = true;
  meta.last_publish_ms = now;
  if (meta.kind == XmlSensorKind::Counter) {
    ESP_LOGD(TAG, "XML publish_state: %s=%u", name.c_str(),
             static_cast<unsigned>(std::lround(static_cast<double>(publish_value))));
  } else {
    ESP_LOGD(TAG, "XML publish_state: %s=%.3f", name.c_str(), static_cast<double>(publish_value));
  }
}

sensor::Sensor *JuraComponent::get_or_create_sensor_(const std::string &name, const std::string &label) {
  auto it = this->xml_sensors_.find(name);
  sensor::Sensor *sensor_obj = nullptr;
  if (it != this->xml_sensors_.end()) {
    sensor_obj = it->second;
  }
  if (sensor_obj == nullptr) {
    sensor_obj = this->create_internal_sensor_(name, label);
    if (sensor_obj == nullptr) {
      return nullptr;
    }
    this->xml_sensors_[name] = sensor_obj;
  }

  sensor_obj->set_internal(false);

  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it != this->xml_sensor_meta_.end()) {
    const auto &meta = meta_it->second;
    sensor_obj->set_accuracy_decimals(meta.accuracy_decimals);
    sensor::StateClass state_class = meta.kind == XmlSensorKind::Counter
                                         ? sensor::StateClass::STATE_CLASS_TOTAL_INCREASING
                                         : sensor::StateClass::STATE_CLASS_MEASUREMENT;
    sensor_obj->set_state_class(state_class);
    if (meta.has_unit) {
      sensor_obj->set_unit_of_measurement(meta.unit_of_measurement.c_str());
    }
    if (meta.has_icon) {
      sensor_obj->set_icon(meta.icon.c_str());
    }
  }

  return sensor_obj;
}

sensor::Sensor *JuraComponent::create_internal_sensor_(const std::string &name, const std::string &label) {
  auto sensor_obj = std::make_unique<sensor::Sensor>();
  if (sensor_obj == nullptr) {
    return nullptr;
  }
  sensor::Sensor *raw_sensor = sensor_obj.get();
  std::string friendly_label = label.empty() ? name : label;
  if (!friendly_label.empty()) {
    try_set_name(raw_sensor, friendly_label);
  }
  auto sanitize = [](const std::string &value) {
    std::string out;
    out.reserve(value.size());
    for (unsigned char c : value) {
      if (std::isalnum(c) != 0) {
        out.push_back(static_cast<char>(std::tolower(c)));
      } else {
        out.push_back('_');
      }
    }
    return out;
  };
  std::string unique_part = sanitize(name.empty() ? friendly_label : name);
  if (unique_part.empty()) {
    unique_part = "field";
  }
  try_set_unique_id(raw_sensor, std::string("jutta_") + unique_part);
  raw_sensor->set_internal(false);
  try_register_sensor(App, raw_sensor, 0L);
  this->xml_owned_sensors_.push_back(std::move(sensor_obj));
  return raw_sensor;
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

