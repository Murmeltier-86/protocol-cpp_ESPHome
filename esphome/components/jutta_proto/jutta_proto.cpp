#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <iomanip>
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
constexpr uint32_t XML_RESPONSE_TIMEOUT_MS = 1000;
constexpr uint32_t XML_INTER_COMMAND_DELAY_MS = 180;

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

template<typename AppT>
void try_register_sensor(AppT &, sensor::Sensor *, ...) {}

inline void register_sensor_with_app(sensor::Sensor *sensor) {
  if (sensor == nullptr) {
    return;
  }
#ifdef __GXX_RTTI
  if (auto *component = dynamic_cast<Component *>(sensor)) {
    App.register_component(component);
  }
#else
  // Ohne RTTI können wir keinen sicheren Cross-Cast durchführen. In diesem Fall
  // verzichten wir lediglich darauf, die Komponente bei App zu registrieren.
  // Dies entspricht dem Verhalten, das auch mit dem Stub-Sensortyp greift, der
  // nicht von Component erbt.
#endif
  try_register_sensor(App, sensor, 0L);
}

template<typename SensorT>
auto try_set_parent(SensorT *sensor, Component *parent, int)
    -> decltype(sensor->set_parent(parent), void()) {
  sensor->set_parent(parent);
}

template<typename SensorT>
void try_set_parent(SensorT *, Component *, ...) {}

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

JuraComponent::~JuraComponent() {
  for (auto &entry : this->xml_sensors_) {
    delete entry.second;
  }
  this->xml_sensors_.clear();
}

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
  ESP_LOGCONFIG(TAG, "  XML mapping path: %s", this->xml_mapping_path_.c_str());
  ESP_LOGCONFIG(TAG, "  XML poll interval: %u ms", static_cast<unsigned>(this->xml_poll_interval_ms_));
  ESP_LOGCONFIG(TAG, "  XML mapping loaded: %s",
                YESNO(this->xml_mapping_loaded_ && this->xml_mapping_.valid));
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

void JuraComponent::ensure_xml_sensors_created_() {
  if (!this->enable_xml_poll_ || !this->xml_mapping_.valid) {
    return;
  }
  auto ensure_field = [&](const XmlField &field) {
    this->get_or_create_sensor_(field.name, field.label);
  };
  for (const auto &field : this->xml_mapping_.tr32_fields.fields) {
    ensure_field(field);
  }
  for (const auto &field : this->xml_mapping_.tg43_fields.fields) {
    ensure_field(field);
  }
  for (const auto &field : this->xml_mapping_.tgc0_fields.fields) {
    ensure_field(field);
  }
}

bool JuraComponent::ensure_xml_mapping_loaded_() {
  if (!this->enable_xml_poll_) {
    return false;
  }
  if (this->xml_mapping_loaded_) {
    return this->xml_mapping_.valid;
  }
  XmlMapping mapping;
  bool loaded = false;
  if (this->xml_mapping_has_blob_) {
    loaded = load_xml_mapping_from_content(this->xml_mapping_path_, this->xml_mapping_blob_, mapping);
  } else {
    loaded = load_xml_mapping(this->xml_mapping_path_, mapping);
  }
  if (!loaded) {
    ESP_LOGW(TAG, "Failed to load XML mapping at %s", this->xml_mapping_path_.c_str());
    this->xml_mapping_loaded_ = false;
    this->xml_mapping_.valid = false;
    this->xml_mapping_logged_ = false;
    this->xml_stats_.clear();
    this->log_xml_mapping_status_();
    return false;
  }
  this->xml_mapping_ = std::move(mapping);
  this->xml_mapping_loaded_ = this->xml_mapping_.valid;
  this->xml_mapping_logged_ = false;
  this->xml_stats_.clear();
  this->log_xml_mapping_status_();
  return this->xml_mapping_.valid;
}

void JuraComponent::log_xml_mapping_status_() {
  if (this->xml_mapping_logged_) {
    return;
  }
  const std::string &source = this->xml_mapping_.source_path.empty() ? this->xml_mapping_path_
                                                                     : this->xml_mapping_.source_path;
  ESP_LOGI(TAG, "XML mapping loaded from %s (valid=%d, fields=%u/%u/%u)", source.c_str(),
           static_cast<int>(this->xml_mapping_.valid),
           static_cast<unsigned>(this->xml_mapping_.tr32_fields.fields.size()),
           static_cast<unsigned>(this->xml_mapping_.tg43_fields.fields.size()),
           static_cast<unsigned>(this->xml_mapping_.tgc0_fields.fields.size()));
  this->xml_mapping_logged_ = true;
}

void JuraComponent::reset_xml_cycle_state_() {
  this->xml_cycle_.reset();
  this->xml_stats_.clear();
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
  if (this->xml_cycle_.phase == XmlCycleState::Phase::Idle) {
    if (this->xml_next_poll_ != 0 && !time_reached(now, this->xml_next_poll_)) {
      return;
    }
    this->start_xml_cycle_(now);
    return;
  }
  this->handle_xml_cycle_(now);
}

void JuraComponent::start_xml_cycle_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  this->xml_cycle_.reset();
  this->xml_cycle_.phase = XmlCycleState::Phase::WaitingForFrame;
  this->xml_cycle_.command_index = 0;
  const char *command = xml_command_for_index_(0);
  ESP_LOGD(TAG, "TX_DB \"%s\"", command);
  this->coffee_maker_->connection->tx_db_command(command);
  this->xml_cycle_.deadline_ms = now + XML_RESPONSE_TIMEOUT_MS;
  this->xml_cycle_.next_action_ms = 0;
}

void JuraComponent::handle_xml_cycle_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->finish_xml_cycle_(now, false);
    return;
  }

  switch (this->xml_cycle_.phase) {
    case XmlCycleState::Phase::Idle:
      break;
    case XmlCycleState::Phase::WaitingForFrame: {
      if (this->try_receive_xml_frame_(now)) {
        this->xml_cycle_.phase = XmlCycleState::Phase::DelayBeforeNext;
        this->xml_cycle_.next_action_ms = now + XML_INTER_COMMAND_DELAY_MS;
        return;
      }
      if (this->xml_cycle_.deadline_ms != 0 &&
          static_cast<int32_t>(now - this->xml_cycle_.deadline_ms) >= 0) {
        ESP_LOGW(TAG, "RX_DB timeout %s", xml_log_label_for_index_(this->xml_cycle_.command_index));
        this->xml_cycle_.phase = XmlCycleState::Phase::DelayBeforeNext;
        this->xml_cycle_.next_action_ms = now + XML_INTER_COMMAND_DELAY_MS;
      }
      break;
    }
    case XmlCycleState::Phase::DelayBeforeNext: {
      if (this->xml_cycle_.next_action_ms != 0 &&
          static_cast<int32_t>(now - this->xml_cycle_.next_action_ms) < 0) {
        return;
      }
      size_t next_index = this->xml_cycle_.command_index + 1;
      if (next_index >= 3) {
        this->finish_xml_cycle_(now, true);
        return;
      }
      const char *command = xml_command_for_index_(next_index);
      ESP_LOGD(TAG, "TX_DB \"%s\"", command);
      this->coffee_maker_->connection->tx_db_command(command);
      this->xml_cycle_.command_index = next_index;
      this->xml_cycle_.phase = XmlCycleState::Phase::WaitingForFrame;
      this->xml_cycle_.deadline_ms = now + XML_RESPONSE_TIMEOUT_MS;
      break;
    }
  }
}

bool JuraComponent::try_receive_xml_frame_(uint32_t now) {
  (void) now;
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  std::vector<uint8_t> decoded;
  if (!this->coffee_maker_->connection->read_db_frame(decoded, 0)) {
    return false;
  }
  size_t index = this->xml_cycle_.command_index;
  this->xml_cycle_.responses[index] = std::move(decoded);
  ESP_LOGD(TAG, "RX_DB %s decoded_len=%u", xml_log_label_for_index_(index),
           static_cast<unsigned>(this->xml_cycle_.responses[index].size()));
  return true;
}

void JuraComponent::finish_xml_cycle_(uint32_t now, bool success) {
  bool any_value = false;

  auto handle_response = [&](size_t index, auto mapper) {
    const auto &response = this->xml_cycle_.responses[index];
    const char *label = xml_log_label_for_index_(index);
    if (response.empty()) {
      ESP_LOGW(TAG, "XML %s: keine Daten empfangen", label);
      return;
    }
    if (mapper(response, this->xml_mapping_, this->xml_stats_)) {
      any_value = true;
    }
  };

  handle_response(0,
                   static_cast<bool (*)(const std::vector<uint8_t> &, const XmlMapping &, MachineStats &)>(
                       map_tr32));
  handle_response(1,
                   static_cast<bool (*)(const std::vector<uint8_t> &, const XmlMapping &, MachineStats &)>(
                       map_tg43));
  handle_response(2,
                   static_cast<bool (*)(const std::vector<uint8_t> &, const XmlMapping &, MachineStats &)>(
                       map_tgc0));

  if (any_value) {
    this->publish_xml_stats_();
  }

  if (!success) {
    ESP_LOGW(TAG, "XML poll aborted");
  }
  this->xml_cycle_.reset();
  this->xml_next_poll_ = now + this->xml_poll_interval_ms_;
}

void JuraComponent::publish_xml_stats_() {
  if (!this->enable_xml_poll_) {
    return;
  }
  if (this->xml_stats_.empty()) {
    return;
  }
  const auto &stats = this->xml_stats_.values();
  for (const auto &entry : stats) {
    this->publish_single_stat_(entry.first, entry.second.value, entry.second.label);
  }
}

void JuraComponent::publish_single_stat_(const std::string &name, double value, const std::string &label) {
  auto *sensor = this->get_or_create_sensor_(name, label);
  if (sensor == nullptr) {
    ESP_LOGW(TAG, "XML field %s konnte nicht veröffentlicht werden", name.c_str());
    return;
  }
  sensor->publish_state(static_cast<float>(value));
  ESP_LOGD(TAG, "XML publish: %s=%.3f", name.c_str(), value);
}

sensor::Sensor *JuraComponent::get_or_create_sensor_(const std::string &name, const std::string &label) {
  auto it = this->xml_sensors_.find(name);
  if (it != this->xml_sensors_.end()) {
    if (!label.empty() && this->xml_sensor_labels_[name] != label) {
      it->second->set_name(label);
      this->xml_sensor_labels_[name] = label;
    }
    return it->second;
  }
  auto *sensor_obj = new sensor::Sensor();
  sensor_obj->set_accuracy_decimals(0);
  sensor_obj->set_internal(false);
  try_set_parent(sensor_obj, this, 0);
  register_sensor_with_app(sensor_obj);
  std::string effective_label = label.empty() ? name : label;
  sensor_obj->set_name(effective_label);
  this->xml_sensor_labels_[name] = effective_label;
  this->xml_sensors_[name] = sensor_obj;
  return sensor_obj;
}

const char *JuraComponent::xml_command_for_index_(size_t index) {
  static const char *const COMMANDS[] = {"@TR:32", "@TG:43", "@TG:C0"};
  if (index >= sizeof(COMMANDS) / sizeof(COMMANDS[0])) {
    return "";
  }
  return COMMANDS[index];
}

const char *JuraComponent::xml_log_label_for_index_(size_t index) {
  static const char *const LABELS[] = {"TR32", "TG43", "TGC0"};
  if (index >= sizeof(LABELS) / sizeof(LABELS[0])) {
    return "?";
  }
  return LABELS[index];
}

void JuraComponent::XmlCycleState::reset() {
  this->phase = Phase::Idle;
  this->command_index = 0;
  this->deadline_ms = 0;
  this->next_action_ms = 0;
  for (auto &response : this->responses) {
    response.clear();
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

