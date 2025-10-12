#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <utility>
#include <map>
#include <cstdlib>

#include "esphome/core/time.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 2000;
const char *const MACHINE_DATA_COMMAND = "&STAT?\r\n";
constexpr uint32_t XML_RETRY_DELAY_MS = 5000;
constexpr uint32_t XML_PAUSE_AFTER_FAILURE_MS = 60000;

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

void JuraComponent::set_xml_poll_enabled(bool enabled) {
  this->xml_config_.enabled = enabled;
  if (!enabled) {
    this->xml_stage_ = XmlStage::Disabled;
    this->xml_attempt_counter_ = 0;
    this->xml_action_deadline_ = 0;
  } else if (this->xml_stage_ == XmlStage::Disabled) {
    this->xml_stage_ = XmlStage::AwaitHandshake;
  }
}

void JuraComponent::set_xml_poll_interval(uint32_t interval_ms) {
  this->xml_config_.poll_interval_ms = interval_ms;
}

void JuraComponent::set_xml_mapping_path(const std::string &path) {
  this->xml_config_.mapping_path = path;
}

void JuraComponent::add_xml_sensor(const std::string &field, esphome::sensor::Sensor *sensor) {
  if (sensor == nullptr) {
    return;
  }
  XmlSensorEntry entry;
  entry.field = field;
  entry.sensor = sensor;
  this->xml_config_.sensors.push_back(entry);
}

void JuraComponent::set_xml_status_sensor(esphome::text_sensor::TextSensor *sensor) {
  this->xml_config_.status_sensor = sensor;
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

  if (this->xml_config_.enabled) {
    this->xml_stage_ = XmlStage::AwaitHandshake;
    this->xml_attempt_counter_ = 0;
    this->xml_action_deadline_ = 0;
    this->publish_xml_status("XML-Handshakes ausstehend");
  } else {
    this->xml_stage_ = XmlStage::Disabled;
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
  this->process_xml_channel();
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

  if (this->xml_config_.enabled) {
    ESP_LOGCONFIG(TAG, "  XML-Polling: aktiviert (%u ms Intervall)", this->xml_config_.poll_interval_ms);
    if (!this->xml_config_.mapping_path.empty()) {
      ESP_LOGCONFIG(TAG, "  XML-Referenzdatei: %s", this->xml_config_.mapping_path.c_str());
    }
    ESP_LOGCONFIG(TAG, "  XML-Sensoren: %u", static_cast<unsigned>(this->xml_config_.sensors.size()));
  } else {
    ESP_LOGCONFIG(TAG, "  XML-Polling: deaktiviert");
  }
}

void JuraComponent::process_handshake() {
  using ::jutta_proto::JuttaConnection;
  using ::jutta_proto::JUTTA_GET_TYPE;

  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      break;
    case HandshakeStage::HELLO: {
      ESP_LOGD(TAG, "HELLO: requesting device type with payload '%s' (hex %s).",
               format_printable_string(JUTTA_GET_TYPE).c_str(),
               format_hex_string(JUTTA_GET_TYPE).c_str());
      auto response = this->connection_->write_decoded_with_response(JUTTA_GET_TYPE, std::chrono::milliseconds{1000});
      if (response != nullptr) {
        this->device_type_ = *response;
        ESP_LOGI(TAG, "Detected coffee maker response: %s", this->device_type_.c_str());
        this->handshake_buffer_.clear();
        this->handshake_stage_ = HandshakeStage::SEND_T1;
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
  this->handshake_stage_ = HandshakeStage::HELLO;
  this->last_logged_stage_ = HandshakeStage::FAILED;
}

bool JuraComponent::read_handshake_bytes() {
  if (this->connection_ == nullptr) {
    return false;
  }
  bool read_any = false;
  uint8_t byte = 0;
  while (this->connection_->read_decoded(&byte)) {
    read_any = true;
    this->handshake_buffer_.push_back(static_cast<char>(byte));
    if (this->handshake_buffer_.size() > 128) {
      this->handshake_buffer_.erase(0, this->handshake_buffer_.size() - 128);
    }
    ESP_LOGV(TAG,
             "Handshake buffered byte: '%s' (0x%02X); buffer size=%zu; buffer now '%s' (hex %s)",
             format_printable_char(byte).c_str(), static_cast<unsigned int>(byte),
             this->handshake_buffer_.size(),
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

  auto *connection = this->coffee_maker_->connection.get();
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
    auto response = connection->write_decoded_with_response(
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
  auto response = connection->write_decoded_with_response(
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
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->is_locked()) {
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

::jutta_proto::JuttaConnection *JuraComponent::get_active_connection() const {
  if (this->connection_ != nullptr) {
    return this->connection_.get();
  }
  if (this->coffee_maker_ != nullptr && this->coffee_maker_->connection != nullptr) {
    return this->coffee_maker_->connection.get();
  }
  return nullptr;
}

void JuraComponent::process_xml_channel() {
  if (!this->xml_config_.enabled) {
    return;
  }
  if (this->handshake_stage_ != HandshakeStage::DONE) {
    return;
  }

  auto *connection = this->get_active_connection();
  if (connection == nullptr) {
    return;
  }
  if (this->machine_data_request_pending_) {
    return;
  }

  uint32_t now = esphome::millis();
  if (this->xml_stage_ == XmlStage::Disabled) {
    return;
  }
  if (this->xml_stage_ == XmlStage::Paused) {
    if (this->xml_action_deadline_ != 0 && !time_reached(now, this->xml_action_deadline_)) {
      return;
    }
    this->xml_stage_ = XmlStage::AwaitHandshake;
    this->xml_attempt_counter_ = 0;
  }

  if (this->xml_stage_ == XmlStage::AwaitHandshake) {
    if (this->xml_action_deadline_ != 0 && !time_reached(now, this->xml_action_deadline_)) {
      return;
    }
    bool ok = this->perform_xml_handshake(connection);
    if (ok) {
      this->xml_stage_ = XmlStage::Idle;
      this->xml_attempt_counter_ = 0;
      this->xml_next_poll_ = now;
      this->xml_action_deadline_ = 0;
      this->publish_xml_status("XML-Kanal bereit");
    } else {
      this->handle_xml_failure(false);
    }
    return;
  }

  if (this->xml_stage_ == XmlStage::Idle) {
    if (time_reached(now, this->xml_next_poll_)) {
      this->xml_stage_ = XmlStage::Polling;
    } else {
      return;
    }
  }

  if (this->xml_stage_ == XmlStage::Polling) {
    bool ok = this->poll_xml_values(connection);
    if (ok) {
      this->xml_stage_ = XmlStage::Idle;
      this->xml_attempt_counter_ = 0;
      this->xml_next_poll_ = esphome::millis() + this->xml_config_.poll_interval_ms;
      this->xml_action_deadline_ = 0;
      return;
    }
    this->handle_xml_failure(true);
  }
}

bool JuraComponent::perform_xml_handshake(::jutta_proto::JuttaConnection *connection) {
  ESP_LOGD(TAG, "XML: Starte Handshake-Versuch %u", static_cast<unsigned>(this->xml_attempt_counter_ + 1));
  auto status = connection->start_xml_handshake();
  if (status.success) {
    ESP_LOGI(TAG, "XML: Handshake erfolgreich (%s)", status.summary.c_str());
    return true;
  }
  ESP_LOGW(TAG, "XML: Handshake fehlgeschlagen: %s", status.summary.c_str());
  return false;
}

bool JuraComponent::poll_xml_values(::jutta_proto::JuttaConnection *connection) {
  ESP_LOGD(TAG, "XML: Starte Polling");
  std::vector<std::string> lines;
  auto result = connection->request_xml_lines("@TR:32\r\n", std::chrono::milliseconds{2000}, lines);
  if (!result.success) {
    ESP_LOGW(TAG, "XML: Polling fehlgeschlagen: %s", result.summary.c_str());
    return false;
  }
  if (lines.empty()) {
    ESP_LOGW(TAG, "XML: Keine Daten empfangen");
    return false;
  }
  this->publish_xml_status("XML-Daten empfangen");
  this->update_xml_sensors(lines);
  return true;
}

void JuraComponent::handle_xml_failure(bool severe) {
  ++this->xml_attempt_counter_;
  if (this->xml_attempt_counter_ >= 3) {
    this->xml_stage_ = XmlStage::Paused;
    this->xml_action_deadline_ = esphome::millis() + XML_PAUSE_AFTER_FAILURE_MS;
    this->publish_xml_status("XML: pausiert nach Fehlern");
    ESP_LOGW(TAG, "XML: %u Fehlversuche, pausiere bis in %u ms", static_cast<unsigned>(this->xml_attempt_counter_),
             XML_PAUSE_AFTER_FAILURE_MS);
  } else {
    this->xml_stage_ = XmlStage::AwaitHandshake;
    this->xml_action_deadline_ = esphome::millis() + XML_RETRY_DELAY_MS;
    if (severe) {
      this->publish_xml_status("XML: erneuter Handshake erforderlich");
    }
    ESP_LOGW(TAG, "XML: Fehler, erneuter Versuch in %u ms (Versuch %u von 3)", XML_RETRY_DELAY_MS,
             static_cast<unsigned>(this->xml_attempt_counter_ + 1));
  }
}

void JuraComponent::publish_xml_status(const std::string &state) {
  if (this->xml_config_.status_sensor != nullptr) {
    this->xml_config_.status_sensor->publish_state(state);
  }
}

void JuraComponent::update_xml_sensors(const std::vector<std::string> &lines) {
  if (this->xml_config_.sensors.empty()) {
    return;
  }

  std::map<std::string, std::string> values;
  for (const std::string &line : lines) {
    std::string key;
    std::string value;
    if (JuraComponent::parse_key_value_line(line, key, value)) {
      values[key] = value;
    }
  }

  for (auto &entry : this->xml_config_.sensors) {
    if (entry.sensor == nullptr) {
      continue;
    }
    auto it = values.find(entry.field);
    if (it == values.end()) {
      continue;
    }
    const std::string &raw = it->second;
    char *end_ptr = nullptr;
    const char *c_str = raw.c_str();
    float value = std::strtof(c_str, &end_ptr);
    if (end_ptr == c_str) {
      ESP_LOGW(TAG, "XML: Wert für Feld '%s' konnte nicht interpretiert werden: '%s'", entry.field.c_str(), raw.c_str());
      continue;
    }
    entry.sensor->publish_state(value);
  }
}

bool JuraComponent::parse_key_value_line(const std::string &line, std::string &key, std::string &value) {
  if (line.empty()) {
    return false;
  }
  size_t start = (line[0] == '@' || line[0] == '&') ? 1 : 0;
  size_t sep = line.find_first_of(":=", start);
  if (sep == std::string::npos) {
    return false;
  }
  key = line.substr(start, sep - start);
  size_t value_start = sep + 1;
  while (value_start < line.size() && std::isspace(static_cast<unsigned char>(line[value_start])) != 0) {
    ++value_start;
  }
  size_t value_end = line.size();
  while (value_end > value_start && (line[value_end - 1] == '\r' || line[value_end - 1] == '\n' ||
                                     std::isspace(static_cast<unsigned char>(line[value_end - 1])) != 0)) {
    --value_end;
  }
  if (value_end <= value_start) {
    return false;
  }
  value = line.substr(value_start, value_end - value_start);
  return true;
}

}  // namespace jutta_component
}  // namespace esphome

