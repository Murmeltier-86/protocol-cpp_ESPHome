#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <unordered_set>

#include "esphome/core/application.h"
#include "esphome/core/time.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 2000;
const char *const MACHINE_DATA_COMMAND = "&STAT?\r\n";
constexpr uint32_t kReplyTimeoutMs = 1800;
constexpr uint32_t kInterCmdGapMs = 120;
constexpr uint32_t kTr32InterCmdGapMs = 220;
constexpr uint32_t kXmlQuietMs = 120;
constexpr uint32_t kXmlQuietTr32Ms = 220;
constexpr uint32_t kCycleSleepMs = 2000;
constexpr uint32_t kSettingsRefreshMs = 600000;
constexpr uint32_t kErrorPollIntervalMs = 5000;
constexpr uint32_t kCommandTimeoutMs = 1500;

constexpr double XML_COUNTER_MIN = 0.0;
constexpr double XML_COUNTER_MAX = 1'000'000.0;
constexpr double XML_MEASUREMENT_MIN = 0.0;
constexpr double XML_MEASUREMENT_MAX = 250.0;
constexpr float XML_COUNTER_TOLERANCE = 0.5f;
constexpr float XML_MEASUREMENT_TOLERANCE = 0.1f;
constexpr bool TGC0_TRY_LITTLE_ENDIAN_FIRST = true;
constexpr std::size_t kTR32MinFrameLength = 20;
constexpr std::size_t kTG43MinFrameLength = 13;
constexpr std::size_t kTGC0MinFrameLength = 13;

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
  this->poll_settings_once_();
  this->poll_error_cycle_();
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
        esphome::delay(500);
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

void JuraComponent::start_new_xml_cycle_(uint32_t now) {
  (void) now;
  this->xml_stats_.clear();
  this->xml_rx_buffer_.clear();
  this->xml_inflight_ = false;
  this->xml_last_command_.clear();
  this->xml_retry_count_.fill(0);
  this->xml_invalid_len_seen_.fill(false);
  this->xml_last_invalid_len_.fill(0);
}

size_t JuraComponent::xml_command_index_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return 0;
    case XmlPollState::SEND_TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::PARSE_TG43:
      return 1;
    case XmlPollState::SEND_TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::PARSE_TGC0:
      return 2;
    default:
      break;
  }
  return 0;
}

bool JuraComponent::validate_xml_frame_(XmlPollState state, const std::vector<uint8_t> &decoded, bool had_crlf,
                                        size_t decoded_len, std::vector<uint8_t> &payload,
                                        size_t &expected_min_len, uint8_t &head0) const {
  const XmlCommandMapping *mapping = nullptr;
  std::size_t minimum = 0;
  switch (state) {
    case XmlPollState::WAIT_TR32:
      mapping = &this->xml_mapping_.tr32;
      minimum = kTR32MinFrameLength;
      break;
    case XmlPollState::WAIT_TG43:
      mapping = &this->xml_mapping_.tg43;
      minimum = kTG43MinFrameLength;
      break;
    case XmlPollState::WAIT_TGC0:
      mapping = &this->xml_mapping_.tgc0;
      minimum = kTGC0MinFrameLength;
      break;
    default:
      ESP_LOGW(TAG, "XML validate: unerwarteter Zustand %d", static_cast<int>(state));
      return false;
  }

  if (mapping == nullptr || mapping->empty()) {
    ESP_LOGW(TAG, "XML %s: kein Mapping aktiv", this->xml_state_label_(state));
    return false;
  }

  if (!had_crlf) {
    ESP_LOGW(TAG,
             "XML %s: Frame ohne CRLF (decoded_len=%d) – akzeptiere (EOL wurde im Connection-Layer entfernt).",
             this->xml_state_label_(state), static_cast<int>(decoded_len));
    // kein return – weiterparsen
  }

  std::size_t expected_len = 0;
  for (const auto &field : mapping->fields) {
    expected_len = std::max(expected_len, field.offset + field.size);
  }
  expected_min_len = std::max(expected_len, minimum);

  int start_index = -1;
  for (std::size_t i = 0; i < decoded.size(); ++i) {
    if (decoded[i] == 0x26) {
      start_index = static_cast<int>(i);
      break;
    }
  }
  if (start_index < 0) {
    ESP_LOGW(TAG, "XML %s: Startbyte 0x26 nicht gefunden (decoded_len=%u)", this->xml_state_label_(state),
             static_cast<unsigned>(decoded.size()));
    return false;
  }

  payload.assign(decoded.begin() + start_index, decoded.end());
  head0 = payload.empty() ? 0x00 : payload.front();

  std::size_t avail = payload.size();
  if (avail < expected_min_len) {
    ESP_LOGW(TAG, "XML %s: decoded_len (%u) < expected_min_len (%u)", this->xml_state_label_(state),
             static_cast<unsigned>(avail), static_cast<unsigned>(expected_min_len));
    return false;
  }
  if (head0 != 0x26) {
    ESP_LOGW(TAG, "XML %s: erstes Payload-Byte 0x%02X statt 0x26", this->xml_state_label_(state),
             static_cast<unsigned>(head0));
    return false;
  }

  const char *expected_command = this->xml_state_command_(state);
  if (expected_command != nullptr && expected_command[0] != '\0') {
    std::size_t command_len = std::strlen(expected_command);
    constexpr std::size_t kCommandOffset = 1;
    if (payload.size() < kCommandOffset + command_len) {
      ESP_LOGW(TAG, "XML %s: Kommando-Präfix zu kurz (decoded_len=%u)", this->xml_state_label_(state),
               static_cast<unsigned>(payload.size()));
      return false;
    }
    auto begin = payload.begin() + static_cast<std::ptrdiff_t>(kCommandOffset);
    if (!std::equal(begin, begin + static_cast<std::ptrdiff_t>(command_len), expected_command,
                    expected_command + command_len)) {
      ESP_LOGW(TAG, "XML %s: unerwarteter Präfix '%s' – schneide bis zum ersten '<' ab.",
               this->xml_state_label_(state),
               format_printable_string(std::string(begin, begin + command_len)).c_str());
      auto pos = std::find(payload.begin(), payload.end(), static_cast<uint8_t>('<'));
      if (pos != payload.end()) {
        payload.erase(payload.begin(), pos);
        head0 = payload.empty() ? 0x00 : payload.front();
      } else {
        return false;
      }
    }
  }

  const char *command = this->xml_state_command_(state);
  ESP_LOGD(TAG, "XML RX cmd=%s decoded_len=%u expected_min_len=%u head0=0x%02X", command,
           static_cast<unsigned>(decoded_len), static_cast<unsigned>(expected_min_len),
           static_cast<unsigned>(head0));
  return true;
}

bool JuraComponent::stage_counter_frame_(const XmlCommandMapping &mapping, const std::vector<uint8_t> &frame,
                                         const char *command_label) {
  bool any = false;
  for (const auto &field : mapping.fields) {
    if (field.offset + field.size > frame.size()) {
      ESP_LOGW(TAG, "XML %s Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", command_label,
               field.name.c_str(), static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               static_cast<unsigned>(frame.size()));
      continue;
    }
    std::uint64_t raw = 0;
    if (field.little_endian) {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw |= static_cast<std::uint64_t>(frame[field.offset + i]) << (8U * i);
      }
    } else {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw = (raw << 8U) | static_cast<std::uint64_t>(frame[field.offset + i]);
      }
    }
    double value = static_cast<double>(raw) * field.scale;
    if (field.has_add) {
      value += field.add;
    }
    ESP_LOGD(TAG, "XML field %s offset=%u size=%u endian=%s value=%.3f", field.name.c_str(),
             static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
             field.little_endian ? "LE" : "BE", static_cast<double>(value));
    this->xml_stats_.set_value(field.name, value, field.label);
    any = true;
  }
  return any;
}

bool JuraComponent::process_valid_tgc0_frame_(const std::vector<uint8_t> &frame, bool stage_values) {
  const auto &mapping = this->xml_mapping_.tgc0;
  if (mapping.empty()) {
    ESP_LOGW(TAG, "XML @TG:C0: kein Mapping aktiv");
    return true;
  }
  bool any_value = false;
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

    uint16_t header_value = frame[field.offset];
    uint16_t encoded_value = frame[field.offset + 1];
    XmlField raw_field = field;
    raw_field.offset = field.offset + field.size - 2;
    raw_field.size = 2;
    bool little_endian = raw_field.has_endian ? raw_field.little_endian : TGC0_TRY_LITTLE_ENDIAN_FIRST;
    std::uint64_t raw_value = 0;
    if (!this->decode_field_value_(frame, raw_field, little_endian, raw_value)) {
      continue;
    }
    double percent = 0.0;
    if (field.has_scale) {
      percent = static_cast<double>(raw_value) * field.scale;
      if (field.has_add) {
        percent += field.add;
      }
    } else {
      percent = std::round(static_cast<double>(raw_value) * 100.0 / 65535.0);
      percent = std::clamp(percent, 0.0, 110.0);
    }
    if (!std::isfinite(percent)) {
      auto &state = this->tgc0_filters_[field.name];
      state.window.clear();
      state.consecutive_valid = 0;
      continue;
    }
    if (percent < 0.0 || percent > 130.0) {
      ESP_LOGW(TAG, "XML @TG:C0 ungültig: %s raw=%llu percent=%.2f", field.name.c_str(),
               static_cast<unsigned long long>(raw_value), percent);
      return false;
    }
    if (stage_values) {
      ESP_LOGD(TAG, "XML field %s offset=%u size=%u endian=%s value=%.2f", field.name.c_str(),
               static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               little_endian ? "LE" : "BE", percent);
      if (this->stage_tgc0_value_(field.name, field.label, static_cast<float>(percent), header_value, encoded_value,
                                  static_cast<uint16_t>(raw_value & 0xFFFFu))) {
        any_value = true;
      }
    }
  }
  if (stage_values && !any_value) {
    ESP_LOGW(TAG, "XML TGC0: keine gültigen Werte im Frame");
  }
  return true;
}

bool JuraComponent::should_retry_current_(XmlPollState wait_state, uint32_t now) {
  size_t index = this->xml_command_index_(wait_state);
  if (this->xml_retry_count_[index] >= 1) {
    return false;
  }
  this->xml_retry_count_[index] += 1;
  XmlPollState resend_state = wait_state == XmlPollState::WAIT_TR32
                                  ? XmlPollState::SEND_TR32
                                  : (wait_state == XmlPollState::WAIT_TG43 ? XmlPollState::SEND_TG43
                                                                           : XmlPollState::SEND_TGC0);
  ESP_LOGW(TAG, "XML %s: erneuter Versuch", this->xml_state_label_(wait_state));
  this->transition_to_state_(resend_state, now, this->inter_command_gap_after_(wait_state));
  return true;
}

void JuraComponent::handle_xml_failure_(XmlPollState wait_state, bool is_timeout, size_t decoded_len, uint32_t now) {
  size_t index = this->xml_command_index_(wait_state);
  if (wait_state == XmlPollState::WAIT_TGC0 && is_timeout) {
    if (this->xml_tgc0_timeout_streak_ < std::numeric_limits<uint8_t>::max()) {
      this->xml_tgc0_timeout_streak_ += 1;
    }
    if (this->xml_tgc0_timeout_streak_ >= 3) {
      ESP_LOGW(TAG, "XML @TG:C0 drei Timeouts – überspringe nächsten Versuch");
      this->xml_skip_tgc0_ = true;
      this->xml_tgc0_timeout_streak_ = 0;
    }
  }

  if (!is_timeout) {
    if (!this->xml_invalid_len_seen_[index]) {
      this->xml_invalid_len_seen_[index] = true;
      this->xml_last_invalid_len_[index] = decoded_len;
    } else if (decoded_len != 0 && decoded_len != this->xml_last_invalid_len_[index]) {
      ESP_LOGW(TAG, "XML %s: unterschiedliche decoded_len (%u vs %u) – überspringe", this->xml_state_label_(wait_state),
               static_cast<unsigned>(this->xml_last_invalid_len_[index]), static_cast<unsigned>(decoded_len));
      this->xml_retry_count_[index] = 1;
      this->xml_invalid_len_seen_[index] = false;
    }
  } else {
    this->xml_invalid_len_seen_[index] = false;
    this->xml_last_invalid_len_[index] = 0;
  }

  this->xml_inflight_ = false;
  this->xml_deadline_ms_ = 0;
  this->xml_rx_buffer_.clear();

  if (this->should_retry_current_(wait_state, now)) {
    return;
  }
  this->xml_retry_count_[index] = 0;
  XmlPollState next_state = wait_state == XmlPollState::WAIT_TR32
                                ? XmlPollState::SEND_TG43
                                : (wait_state == XmlPollState::WAIT_TG43 ? XmlPollState::SEND_TGC0
                                                                          : XmlPollState::SLEEP);
  if (next_state == XmlPollState::SLEEP) {
    uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
    uint32_t deadline = now + sleep;
    this->xml_deadline_ms_ = deadline;
    this->xml_next_poll_ = deadline;
    this->transition_to_state_(XmlPollState::SLEEP, now);
  } else {
    this->transition_to_state_(next_state, now, this->inter_command_gap_after_(wait_state));
  }
}

void JuraComponent::complete_command_success_(XmlPollState wait_state) {
  size_t index = this->xml_command_index_(wait_state);
  this->xml_retry_count_[index] = 0;
  this->xml_invalid_len_seen_[index] = false;
  this->xml_last_invalid_len_[index] = 0;
  if (wait_state == XmlPollState::WAIT_TGC0) {
    this->xml_tgc0_timeout_streak_ = 0;
  }
}

bool JuraComponent::stage_tgc0_value_(const std::string &name, const std::string &label, float raw_percent,
                                      uint16_t header_value, uint16_t encoded_value, uint16_t raw_value) {
  auto &state = this->tgc0_filters_[name];
  (void) header_value;
  (void) encoded_value;
  (void) raw_value;
  state.window.clear();
  state.consecutive_valid = 0;
  this->xml_stats_.set_value(name, raw_percent, label);
  return true;
}

void JuraComponent::add_configured_xml_sensor(const std::string &field, sensor::Sensor *sensor) {
  if (field.empty() || sensor == nullptr) {
    return;
  }
  this->xml_sensors_[field] = sensor;
  this->xml_unconfigured_sensor_logged_.erase(field);
  if (this->xml_sensor_meta_.find(field) != this->xml_sensor_meta_.end()) {
    this->apply_sensor_metadata_(field, sensor);
  }
}

void JuraComponent::register_setting_sensor(const std::string &id, sensor::Sensor *sensor) {
  if (id.empty() || sensor == nullptr) {
    return;
  }
  this->setting_sensors_[id] = sensor;
  this->settings_entities_created_ = false;
}

void JuraComponent::register_setting_text_sensor(const std::string &id, text_sensor::TextSensor *sensor) {
  if (id.empty() || sensor == nullptr) {
    return;
  }
  this->setting_text_sensors_[id] = sensor;
  this->settings_entities_created_ = false;
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
  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it == this->xml_sensor_meta_.end() || !meta_it->second.configured) {
    ESP_LOGD(TAG, "XML Feld %s hat keine Sensor-Metadaten – Wert wird verworfen", name.c_str());
    return;
  }
  auto &meta = meta_it->second;
  auto *sensor = this->find_configured_sensor_(name);
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
  if (meta.kind == XmlSensorKind::Counter) {
    if (publish_value < 0.0f) {
      ESP_LOGD(TAG, "XML counter negativ verworfen: %s=%.3f", name.c_str(), static_cast<double>(publish_value));
      return;
    }
    publish_value = static_cast<float>(std::round(static_cast<double>(publish_value)));
  }

  if (!std::isfinite(publish_value)) {
    ESP_LOGW(TAG, "XML publish_state unterdrückt: %s ist nicht endlich", name.c_str());
    return;
  }

  float tolerance = (meta.kind == XmlSensorKind::Counter) ? XML_COUNTER_TOLERANCE : XML_MEASUREMENT_TOLERANCE;
  if (meta.has_last_value && std::fabs(publish_value - meta.last_value) < tolerance) {
    return;
  }

  sensor->publish_state(publish_value);
  meta.last_value = publish_value;
  meta.has_last_value = true;
  meta.last_update_ms = now;
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

  std::string command = command_label != nullptr ? command_label : "";
  meta.command_label = command;
  std::string lower_command = command;
  std::transform(lower_command.begin(), lower_command.end(), lower_command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  bool is_tgc0 = lower_command == "@tg:c0";
  if (kind == XmlSensorKind::Measurement && is_tgc0) {
    meta.min_value = 0.0;
    meta.max_value = 150.0;
    meta.is_percent = true;
    if (meta.accuracy_decimals < 1) {
      meta.accuracy_decimals = 1;
    }
    meta.has_unit = true;
    meta.unit_of_measurement = "%";
    meta.has_icon = true;
    meta.icon = "mdi:percent";
    meta.is_tgc0 = true;
  }

  meta.configured = true;

  sensor::Sensor *sensor = this->find_configured_sensor_(field.name);
  if (sensor != nullptr) {
    this->apply_sensor_metadata_(field.name, sensor);
  }
}

sensor::Sensor *JuraComponent::find_configured_sensor_(const std::string &name) const {
  auto it = this->xml_sensors_.find(name);
  if (it == this->xml_sensors_.end()) {
    return nullptr;
  }
  return it->second;
}

void JuraComponent::apply_sensor_metadata_(const std::string &name, sensor::Sensor *sensor) {
  if (sensor == nullptr) {
    return;
  }
  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it == this->xml_sensor_meta_.end()) {
    return;
  }
  const auto &meta = meta_it->second;
  sensor->set_accuracy_decimals(meta.accuracy_decimals);
  sensor::StateClass state_class = meta.kind == XmlSensorKind::Counter
                                       ? sensor::StateClass::STATE_CLASS_TOTAL_INCREASING
                                       : sensor::StateClass::STATE_CLASS_MEASUREMENT;
  sensor->set_state_class(state_class);
  sensor->set_force_update(true);
  sensor->set_entity_category(EntityCategory::ENTITY_CATEGORY_DIAGNOSTIC);
  if (meta.has_unit) {
    sensor->set_unit_of_measurement(meta.unit_of_measurement.c_str());
  }
  if (meta.has_icon) {
    sensor->set_icon(meta.icon.c_str());
  }
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
  ensure_block(this->xml_mapping_.tg43, XmlSensorKind::Counter, "@TG:43");
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
  load_settings_from_xml(xml_source);
  load_errors_from_xml(xml_source);
  this->xml_mapping_ = get_xml_mapping();
  this->xml_mapping_loaded_ = true;
  this->xml_mapping_logged_ = false;
  this->xml_stats_.clear();
  this->xml_sensor_meta_.clear();
  this->tgc0_filters_.clear();
  this->xml_missing_sensor_logged_.clear();
  this->settings_entities_created_ = false;
  this->settings_boot_polled_ = false;
  this->last_error_code_ = 0;
  this->errors_entities_created_ = false;
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
  this->xml_next_poll_ = esphome::millis();
  this->xml_retry_count_.fill(0);
  this->xml_invalid_len_seen_.fill(false);
  this->xml_last_invalid_len_.fill(0);
  this->xml_tgc0_timeout_streak_ = 0;
  this->xml_skip_tgc0_ = false;
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
    this->start_new_xml_cycle_(now);
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
      if (this->xml_skip_tgc0_) {
        ESP_LOGW(TAG, "XML @TG:C0 übersprungen (Timeout-Streak)");
        this->xml_skip_tgc0_ = false;
        uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
        uint32_t deadline = now + sleep;
        this->xml_deadline_ms_ = deadline;
        this->xml_next_poll_ = deadline;
        this->transition_to_state_(XmlPollState::SLEEP, now);
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
          this->transition_to_state_(next, now, this->inter_command_gap_after_(this->xml_state_));
        }
        return;
      }
      std::vector<uint8_t> decoded;
      bool had_crlf = false;
      size_t decoded_len = 0;
      if (connection->read_db_frame(decoded, 0, &had_crlf, &decoded_len)) {
        const char *expected_command = this->xml_state_command_(this->xml_state_);
        if (!this->xml_last_command_.empty() && expected_command != nullptr && expected_command[0] != '\0' &&
            this->xml_last_command_ != expected_command) {
          ESP_LOGW(TAG, "RX_DB Frame ignoriert: erwartet %s, letzter Befehl %s", expected_command,
                   this->xml_last_command_.c_str());
          return;
        }
        std::vector<uint8_t> payload;
        size_t expected_min_len = 0;
        uint8_t head0 = 0;
        if (!this->validate_xml_frame_(this->xml_state_, decoded, had_crlf, decoded_len, payload, expected_min_len,
                                       head0)) {
          connection->drain_serial_input_quick();
          this->handle_xml_failure_(this->xml_state_, false, decoded_len, now);
          return;
        }
        if (this->xml_state_ == XmlPollState::WAIT_TGC0) {
          if (!this->process_valid_tgc0_frame_(payload, false)) {
            connection->drain_serial_input_quick();
            this->handle_xml_failure_(this->xml_state_, false, payload.size(), now);
            return;
          }
        }
        this->xml_inflight_ = false;
        this->xml_deadline_ms_ = 0;
        this->xml_rx_buffer_ = std::move(payload);
        this->complete_command_success_(this->xml_state_);
        XmlPollState parse_state = (this->xml_state_ == XmlPollState::WAIT_TR32)
                                       ? XmlPollState::PARSE_TR32
                                       : (this->xml_state_ == XmlPollState::WAIT_TG43 ? XmlPollState::PARSE_TG43
                                                                                      : XmlPollState::PARSE_TGC0);
        this->transition_to_state_(parse_state, now);
        return;
      }
      if (this->xml_deadline_ms_ != 0 && static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        this->handle_xml_timeout_(XmlPollState::SLEEP, this->xml_state_label_(this->xml_state_), now);
      }
      return;
    }
    case XmlPollState::PARSE_TR32: {
      this->xml_inflight_ = false;
      this->xml_deadline_ms_ = 0;
      this->xml_stats_.clear();
      bool any = false;
      if (!this->xml_rx_buffer_.empty() && this->xml_state_has_mapping_(XmlPollState::PARSE_TR32)) {
        any = this->stage_counter_frame_(this->xml_mapping_.tr32, this->xml_rx_buffer_, "@TR:32");
      }
      this->xml_rx_buffer_.clear();
      if (any) {
        this->publish_xml_stats_();
      }
      this->xml_stats_.clear();
      this->transition_to_state_(XmlPollState::SEND_TG43, now,
                                 this->inter_command_gap_after_(XmlPollState::WAIT_TR32));
      return;
    }
    case XmlPollState::PARSE_TG43: {
      this->xml_inflight_ = false;
      this->xml_deadline_ms_ = 0;
      this->xml_stats_.clear();
      bool any = false;
      if (!this->xml_rx_buffer_.empty() && this->xml_state_has_mapping_(XmlPollState::PARSE_TG43)) {
        any = this->stage_counter_frame_(this->xml_mapping_.tg43, this->xml_rx_buffer_, "@TG:43");
      }
      this->xml_rx_buffer_.clear();
      if (any) {
        this->publish_xml_stats_();
      }
      this->xml_stats_.clear();
      this->transition_to_state_(XmlPollState::SEND_TGC0, now,
                                 this->inter_command_gap_after_(XmlPollState::WAIT_TG43));
      return;
    }
    case XmlPollState::PARSE_TGC0: {
      this->xml_inflight_ = false;
      this->xml_deadline_ms_ = 0;
      this->xml_stats_.clear();
      bool any = false;
      if (!this->xml_rx_buffer_.empty() && this->xml_state_has_mapping_(XmlPollState::PARSE_TGC0)) {
        if (this->process_valid_tgc0_frame_(this->xml_rx_buffer_, true)) {
          any = !this->xml_stats_.empty();
        } else {
          this->handle_xml_failure_(XmlPollState::WAIT_TGC0, false, this->xml_rx_buffer_.size(), now);
          this->xml_rx_buffer_.clear();
          return;
        }
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

uint32_t JuraComponent::quiet_delay_for_state_(XmlPollState wait_state) const {
  if (wait_state == XmlPollState::WAIT_TR32) {
    return kXmlQuietTr32Ms;
  }
  return kXmlQuietMs;
}

uint32_t JuraComponent::inter_command_gap_after_(XmlPollState wait_state) const {
  if (wait_state == XmlPollState::WAIT_TR32) {
    return kTr32InterCmdGapMs;
  }
  return kInterCmdGapMs;
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
  connection->reset_db_rx_buffer();
  this->xml_rx_buffer_.clear();
  ESP_LOGD(TAG, "TX_DB \"%s\"", command);
  connection->tx_db_command(command, false);
  esphome::delay(80);
  this->xml_inflight_ = true;
  this->xml_last_command_ = command;
  this->xml_deadline_ms_ = now + kReplyTimeoutMs;
  uint32_t quiet_delay = this->quiet_delay_for_state_(wait_state);
  this->xml_next_action_ms_ = quiet_delay > 0 ? now + quiet_delay : 0;
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
  this->handle_xml_failure_(this->xml_state_, true, 0, now);
}

void JuraComponent::prepare_tgc0_request_() {
  // Keine zusätzlichen Flush-Operationen während des XML-Pollings.
}

void JuraComponent::ensure_setting_entities_created_() {
  if (this->settings_entities_created_) {
    return;
  }
  this->setting_descs_.clear();
  const auto &settings = get_settings();
  for (const auto &desc : settings) {
    if (!desc.id.empty()) {
      this->setting_descs_[desc.id] = desc;
    }
  }
  this->settings_entities_created_ = true;
}

bool JuraComponent::query_setting_command_(const std::string &command, std::vector<uint8_t> &decoded) {
  decoded.clear();
  if (command.empty()) {
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  std::string cmd = command;
  if (cmd.find("\r\n") == std::string::npos) {
    cmd.append("\r\n");
  }
  auto response = this->coffee_maker_->connection->write_decoded_with_response(
      cmd, std::chrono::milliseconds{kCommandTimeoutMs});
  if (response == nullptr) {
    ESP_LOGW(TAG, "Einstellungspoll: keine Antwort für %s", command.c_str());
    return false;
  }
  std::string payload = *response;
  payload.erase(std::remove(payload.begin(), payload.end(), '\r'), payload.end());
  payload.erase(std::remove(payload.begin(), payload.end(), '\n'), payload.end());
  std::string filtered;
  filtered.reserve(payload.size());
  for (unsigned char c : payload) {
    if (!std::isspace(c)) {
      filtered.push_back(static_cast<char>(c));
    }
  }
  for (std::size_t i = 0; i + 1 < filtered.size(); i += 2) {
    std::string byte_hex = filtered.substr(i, 2);
    char *end = nullptr;
    auto value = std::strtoul(byte_hex.c_str(), &end, 16);
    if (end == byte_hex.c_str()) {
      decoded.clear();
      break;
    }
    decoded.push_back(static_cast<uint8_t>(value));
  }
  if (decoded.empty() && !filtered.empty()) {
    decoded.assign(filtered.begin(), filtered.end());
  }
  return !decoded.empty();
}

bool JuraComponent::query_error_command_(const std::string &command, std::vector<uint8_t> &decoded) {
  return this->query_setting_command_(command, decoded);
}

void JuraComponent::publish_setting_value_(const SettingDesc &desc, float value, const std::string &raw_text) {
  if (desc.type != SettingValueType::String) {
    auto it = this->setting_sensors_.find(desc.id);
    if (it != this->setting_sensors_.end() && it->second != nullptr) {
      it->second->publish_state(value);
    }
  }
  auto text_it = this->setting_text_sensors_.find(desc.id);
  if (text_it != this->setting_text_sensors_.end() && text_it->second != nullptr) {
    text_it->second->publish_state(raw_text);
  }
}

void JuraComponent::poll_settings_refresh_() {
  this->ensure_setting_entities_created_();
  if (this->setting_descs_.empty()) {
    return;
  }
  std::unordered_map<std::string, std::vector<uint8_t>> command_cache;
  std::unordered_set<std::string> failed_commands;

  auto get_command_payload = [&](const std::string &command) -> const std::vector<uint8_t> * {
    if (command.empty()) {
      return nullptr;
    }
    if (failed_commands.find(command) != failed_commands.end()) {
      return nullptr;
    }
    auto cache_it = command_cache.find(command);
    if (cache_it != command_cache.end()) {
      return &cache_it->second;
    }
    std::vector<uint8_t> decoded;
    if (!this->query_setting_command_(command, decoded)) {
      failed_commands.insert(command);
      return nullptr;
    }
    auto inserted = command_cache.emplace(command, std::move(decoded));
    return &inserted.first->second;
  };

  auto format_numeric_text = [](float value) -> std::string {
    char buffer[32];
    int written = std::snprintf(buffer, sizeof(buffer), "%.6f", static_cast<double>(value));
    if (written < 0) {
      return std::to_string(static_cast<double>(value));
    }
    std::string text(buffer, static_cast<size_t>(written));
    auto dot = text.find('.');
    if (dot != std::string::npos) {
      while (!text.empty() && text.back() == '0') {
        text.pop_back();
      }
      if (!text.empty() && text.back() == '.') {
        text.pop_back();
      }
      if (text.empty()) {
        text = "0";
      }
    }
    return text;
  };

  for (const auto &entry : this->setting_descs_) {
    const auto &desc = entry.second;
    if (desc.source_cmd.empty()) {
      continue;
    }
    const auto *decoded_ptr = get_command_payload(desc.source_cmd);
    if (decoded_ptr == nullptr) {
      continue;
    }
    const auto &decoded = *decoded_ptr;
    if (desc.offset + desc.width > decoded.size()) {
      ESP_LOGW(TAG, "Einstellung %s: Antwort zu kurz (offset=%u width=%u size=%u)", desc.id.c_str(),
               static_cast<unsigned>(desc.offset), static_cast<unsigned>(desc.width),
               static_cast<unsigned>(decoded.size()));
      continue;
    }
    const uint8_t *ptr = decoded.data() + desc.offset;
    std::uint64_t raw = 0;
    for (std::size_t i = 0; i < desc.width; ++i) {
      raw = (raw << 8U) | static_cast<std::uint64_t>(ptr[i]);
    }
    float scaled = static_cast<float>(raw) * desc.scale;
    std::string text_value;
    switch (desc.type) {
      case SettingValueType::Bool:
        text_value = raw ? "on" : "off";
        scaled = raw ? 1.0f : 0.0f;
        break;
      case SettingValueType::Enum:
      case SettingValueType::U8:
      case SettingValueType::U16:
      case SettingValueType::U32:
        text_value = format_numeric_text(scaled);
        break;
      case SettingValueType::String:
        text_value.assign(reinterpret_cast<const char *>(ptr), desc.width);
        auto zero_pos = text_value.find('\0');
        if (zero_pos != std::string::npos) {
          text_value.resize(zero_pos);
        }
        this->publish_setting_value_(desc, 0.0f, text_value);
        continue;
    }
    this->publish_setting_value_(desc, scaled, text_value);
  }
}

void JuraComponent::poll_settings_once_() {
  if (!this->is_ready()) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  uint32_t now = esphome::millis();
  if (!this->settings_boot_polled_) {
    this->poll_settings_refresh_();
    this->settings_boot_polled_ = true;
    this->settings_next_refresh_ = now + kSettingsRefreshMs;
    return;
  }
  if (this->settings_next_refresh_ != 0 && !time_reached(now, this->settings_next_refresh_)) {
    return;
  }
  this->poll_settings_refresh_();
  this->settings_next_refresh_ = now + kSettingsRefreshMs;
}

void JuraComponent::publish_error_state_(uint32_t code) {
  if (this->error_code_sensor_ != nullptr) {
    this->error_code_sensor_->publish_state(static_cast<float>(code));
  }
  bool has_error = code != 0;
  if (this->error_active_sensor_ != nullptr) {
    this->error_active_sensor_->publish_state(has_error);
  }
  const ErrorDesc *desc = find_error(code);
  if (desc != nullptr) {
    if (this->error_text_sensor_ != nullptr) {
      this->error_text_sensor_->publish_state(desc->text);
    }
    if (this->error_severity_sensor_ != nullptr) {
      this->error_severity_sensor_->publish_state(desc->severity);
    }
  } else {
    if (this->error_text_sensor_ != nullptr) {
      this->error_text_sensor_->publish_state(has_error ? "unbekannt" : "kein Fehler");
    }
    if (this->error_severity_sensor_ != nullptr) {
      this->error_severity_sensor_->publish_state(has_error ? "unknown" : "none");
    }
  }
  this->last_error_code_ = code;
  this->errors_entities_created_ = true;
}

void JuraComponent::poll_error_cycle_() {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  uint32_t now = esphome::millis();
  if (this->errors_next_poll_ != 0 && !time_reached(now, this->errors_next_poll_)) {
    return;
  }
  this->errors_next_poll_ = now + kErrorPollIntervalMs;
  std::string command = error_source_command();
  if (command.empty()) {
    return;
  }
  std::vector<uint8_t> decoded;
  if (!this->query_error_command_(command, decoded)) {
    return;
  }
  if (decoded.empty()) {
    return;
  }
  uint32_t code = 0;
  for (uint8_t byte : decoded) {
    code = (code << 8U) | byte;
  }
  if (!this->errors_entities_created_ || code != this->last_error_code_) {
    this->publish_error_state_(code);
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

