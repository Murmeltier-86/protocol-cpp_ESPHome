#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <utility>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/time.h"
#ifdef USE_FILESYSTEM
#include "esphome/components/filesystem/filesystem.h"
#endif

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 2000;
const char *const MACHINE_DATA_COMMAND = "&STAT?\r\n";
constexpr size_t XML_TR32_COUNT = 10;
constexpr size_t XML_TG43_COUNT = 6;
constexpr size_t XML_TGC0_COUNT = 3;
constexpr uint32_t XML_RX_DRAIN_MS = 40;

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

template <size_t N>
void fill_default_labels(const char *prefix, std::array<std::string, N> &labels) {
  for (size_t i = 0; i < N; ++i) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%s %u", prefix, static_cast<unsigned>(i + 1));
    labels[i] = buffer;
  }
}

template <size_t N>
size_t count_non_empty(const std::array<std::string, N> &labels) {
  size_t count = 0;
  for (const auto &label : labels) {
    if (!label.empty()) {
      ++count;
    }
  }
  return count;
}

bool extract_command(const std::string &content, size_t anchor, std::string &command) {
  size_t search_start = content.find('@', anchor);
  if (search_start == std::string::npos) {
    return false;
  }
  size_t end = search_start;
  while (end < content.size()) {
    unsigned char ch = static_cast<unsigned char>(content[end]);
    if (std::isalnum(ch) != 0 || ch == '@' || ch == ':' || ch == '_' || ch == '-') {
      ++end;
    } else {
      break;
    }
  }
  if (end <= search_start + 1) {
    return false;
  }
  command = content.substr(search_start, end - search_start);
  return true;
}

template <size_t N>
void extract_labels(const std::string &content, size_t anchor, std::array<std::string, N> &labels) {
  size_t block_end = content.find("</", anchor);
  if (block_end == std::string::npos) {
    block_end = content.size();
  }
  size_t cursor = anchor;
  size_t count = 0;
  while (count < N) {
    size_t label_pos = content.find("label", cursor);
    if (label_pos == std::string::npos || label_pos >= block_end) {
      break;
    }
    size_t quote_pos = content.find_first_of("\"'", label_pos);
    if (quote_pos == std::string::npos) {
      break;
    }
    char quote_char = content[quote_pos];
    size_t value_start = quote_pos + 1;
    size_t value_end = content.find(quote_char, value_start);
    if (value_end == std::string::npos) {
      break;
    }
    if (value_end > value_start) {
      labels[count] = content.substr(value_start, value_end - value_start);
    }
    ++count;
    cursor = value_end + 1;
  }
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
  auto cleanup_pool = [](auto &pool) {
    for (auto *&sensor : pool) {
      delete sensor;
      sensor = nullptr;
    }
  };
  cleanup_pool(this->xml_tr32_sensors_);
  cleanup_pool(this->xml_tg43_sensors_);
  cleanup_pool(this->xml_tgc0_sensors_);
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
  if (this->xml_enabled_) {
    this->load_xml_mapping_();
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

  if (this->coffee_maker_ != nullptr && !this->xml_busy_) {
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

  ESP_LOGCONFIG(TAG, "  XML polling: %s", this->xml_enabled_ ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  XML mapping path: %s", this->xml_path_.c_str());
  if (this->xml_enabled_) {
    ESP_LOGCONFIG(TAG, "  XML poll interval: %u ms", static_cast<unsigned>(this->xml_poll_interval_ms_));
    ESP_LOGCONFIG(TAG, "  XML RX timeout: %u ms", static_cast<unsigned>(this->xml_rx_timeout_ms_));
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
  if (this->xml_busy_) {
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

void JuraComponent::reset_xml_cycle_state_() {
  this->xml_initialized_ = false;
  this->xml_has_values_ = false;
  this->xml_next_poll_ = 0;
  this->xml_mapping_logged_ = false;
  this->xml_busy_ = false;
}

void JuraComponent::ensure_xml_sensors_created_() {
  if (!this->xml_enabled_) {
    return;
  }
  auto create_pool = [&](auto &pool) {
    for (auto *&sensor_ptr : pool) {
      if (sensor_ptr == nullptr) {
        auto *sensor_obj = new sensor::Sensor();
        sensor_obj->set_accuracy_decimals(0);
        sensor_obj->set_internal(false);
        try_set_parent(sensor_obj, this, 0);
        register_sensor_with_app(sensor_obj);
        sensor_ptr = sensor_obj;
      }
    }
  };
  create_pool(this->xml_tr32_sensors_);
  create_pool(this->xml_tg43_sensors_);
  create_pool(this->xml_tgc0_sensors_);
  this->update_sensor_names_();
}

void JuraComponent::update_sensor_names_() {
  auto assign = [](auto &pool, const auto &labels, const char *default_prefix) {
    for (size_t i = 0; i < pool.size(); ++i) {
      if (pool[i] == nullptr) {
        continue;
      }
      std::string label = labels[i];
      if (label.empty()) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%s %u", default_prefix, static_cast<unsigned>(i + 1));
        label = buffer;
      }
      pool[i]->set_name(label);
    }
  };
  assign(this->xml_tr32_sensors_, this->xml_mapping_.tr32_labels, "TR32");
  assign(this->xml_tg43_sensors_, this->xml_mapping_.tg43_labels, "TG43");
  assign(this->xml_tgc0_sensors_, this->xml_mapping_.tgc0_labels, "TGC0");
}

void JuraComponent::load_xml_mapping_() {
  fill_default_labels("TR32", this->xml_mapping_.tr32_labels);
  fill_default_labels("TG43", this->xml_mapping_.tg43_labels);
  fill_default_labels("TGC0", this->xml_mapping_.tgc0_labels);
  this->xml_mapping_.tr32_command = "@TR:32";
  this->xml_mapping_.tg43_command = "@TG:43";
  this->xml_mapping_.tgc0_command = "@TG:C0";
  this->xml_mapping_.valid = false;

  if (!this->xml_enabled_) {
    return;
  }

  bool commands_ok = true;
  std::string content;
#if defined(USE_FILESYSTEM)
  auto *fs = ::esphome::filesystem::global_filesystem;
  if (fs == nullptr) {
    ESP_LOGW(TAG, "No filesystem available for XML mapping");
    commands_ok = false;
  } else {
    auto file = fs->open(this->xml_path_.c_str(), "r");
    if (!file) {
      ESP_LOGW(TAG, "Failed to open XML mapping at %s", this->xml_path_.c_str());
      commands_ok = false;
    } else {
      char buffer[256];
      while (true) {
        size_t read = file.readBytes(buffer, sizeof(buffer));
        if (read == 0) {
          break;
        }
        content.append(buffer, read);
        if (read < sizeof(buffer) && !file.available()) {
          break;
        }
      }
      file.close();

      auto parse_block = [&](const char *anchor_token, std::string &command, auto &labels) {
        size_t anchor_pos = content.find(anchor_token);
        if (anchor_pos == std::string::npos) {
          commands_ok = false;
          return;
        }
        std::string parsed_command;
        if (extract_command(content, anchor_pos, parsed_command)) {
          command = parsed_command;
        } else {
          commands_ok = false;
        }
        extract_labels(content, anchor_pos, labels);
      };

      parse_block("TR32", this->xml_mapping_.tr32_command, this->xml_mapping_.tr32_labels);
      parse_block("TG43", this->xml_mapping_.tg43_command, this->xml_mapping_.tg43_labels);
      parse_block("TGC0", this->xml_mapping_.tgc0_command, this->xml_mapping_.tgc0_labels);
    }
  }
#else
  ESP_LOGW(TAG, "No filesystem available for XML mapping");
  commands_ok = false;
#endif

  this->xml_mapping_.valid = commands_ok;
  if (!this->xml_mapping_logged_) {
    ESP_LOGI(TAG, "XML mapping path = %s", this->xml_path_.c_str());
    size_t tr32_labels = count_non_empty(this->xml_mapping_.tr32_labels);
    size_t tg43_labels = count_non_empty(this->xml_mapping_.tg43_labels);
    size_t tgc0_labels = count_non_empty(this->xml_mapping_.tgc0_labels);
    ESP_LOGI(TAG, "XML mapping valid=%d, cmds=<%s, %s, %s>", static_cast<int>(this->xml_mapping_.valid),
             this->xml_mapping_.tr32_command.c_str(), this->xml_mapping_.tg43_command.c_str(),
             this->xml_mapping_.tgc0_command.c_str());
    ESP_LOGI(TAG, "XML labels: TR32=%u, TG43=%u, TGC0=%u", static_cast<unsigned>(tr32_labels),
             static_cast<unsigned>(tg43_labels), static_cast<unsigned>(tgc0_labels));
    this->xml_mapping_logged_ = true;
  }
  this->update_sensor_names_();
}

void JuraComponent::process_xml_polling() {
  if (!this->xml_enabled_) {
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

  if (this->xml_busy_) {
    return;
  }

  uint32_t now = esphome::millis();

  if (!this->xml_initialized_) {
    this->load_xml_mapping_();
    this->ensure_xml_sensors_created_();
    this->xml_initialized_ = true;
    this->xml_next_poll_ = now;
    return;
  }

  if (this->xml_next_poll_ != 0 && !time_reached(now, this->xml_next_poll_)) {
    return;
  }

  ESP_LOGD(TAG, "XML poll start");
  if (this->perform_xml_cycle_()) {
    this->publish_xml_values_();
  }
  this->xml_next_poll_ = now + this->xml_poll_interval_ms_;
}

bool JuraComponent::perform_xml_cycle_() {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (this->xml_busy_) {
    ESP_LOGW(TAG, "XML poll skipped because connection is busy.");
    return false;
  }

  struct BusyGuard {
    bool &flag;
    explicit BusyGuard(bool &ref) : flag(ref) { flag = true; }
    ~BusyGuard() { flag = false; }
  } guard(this->xml_busy_);

  auto *link = this->coffee_maker_->connection.get();
  link->drain_db_stream(std::chrono::milliseconds{XML_RX_DRAIN_MS});

  std::vector<uint8_t> buffer;
  std::array<uint16_t, XML_TR32_COUNT> tr32_values{};
  std::array<uint16_t, XML_TG43_COUNT> tg43_values{};
  std::array<uint32_t, XML_TGC0_COUNT> tgc0_values{};

  auto parse_u16 = [](const std::vector<uint8_t> &src, auto &dest) {
    for (size_t i = 0; i < dest.size(); ++i) {
      size_t offset = 1 + i * 2;
      dest[i] = static_cast<uint16_t>((static_cast<uint16_t>(src[offset]) << 8) |
                                      static_cast<uint16_t>(src[offset + 1]));
    }
  };
  auto parse_u32 = [](const std::vector<uint8_t> &src, auto &dest) {
    for (size_t i = 0; i < dest.size(); ++i) {
      size_t offset = 1 + i * 4;
      dest[i] = (static_cast<uint32_t>(src[offset]) << 24) |
                (static_cast<uint32_t>(src[offset + 1]) << 16) |
                (static_cast<uint32_t>(src[offset + 2]) << 8) |
                static_cast<uint32_t>(src[offset + 3]);
    }
  };

  auto read_block = [&](const std::string &command, size_t expected_length, auto &&parser) -> bool {
    ESP_LOGD(TAG, "TX_DB \"%s\"", command.c_str());
    if (!this->send_db_command_(command)) {
      ESP_LOGW(TAG, "Failed to send XML command %s", command.c_str());
      return false;
    }
    if (!this->read_db_with_expected_len_(buffer, this->xml_rx_timeout_ms_, command, expected_length)) {
      return false;
    }
    parser(buffer);
    return true;
  };

  if (!read_block(this->xml_mapping_.tr32_command, 1 + XML_TR32_COUNT * 2, [&](const std::vector<uint8_t> &src) {
        parse_u16(src, tr32_values);
      })) {
    return false;
  }
  if (!read_block(this->xml_mapping_.tg43_command, 1 + XML_TG43_COUNT * 2, [&](const std::vector<uint8_t> &src) {
        parse_u16(src, tg43_values);
      })) {
    return false;
  }
  if (!read_block(this->xml_mapping_.tgc0_command, 1 + XML_TGC0_COUNT * 4, [&](const std::vector<uint8_t> &src) {
        parse_u32(src, tgc0_values);
      })) {
    return false;
  }

  this->xml_tr32_values_ = tr32_values;
  this->xml_tg43_values_ = tg43_values;
  this->xml_tgc0_values_ = tgc0_values;
  this->xml_has_values_ = true;
  return true;
}

bool JuraComponent::send_db_command_(const std::string &command) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  return this->coffee_maker_->connection->write_db_command(command);
}

bool JuraComponent::read_db_data_frame_(std::vector<uint8_t> &decoded, uint32_t timeout_ms) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  return this->coffee_maker_->connection->read_db_data_frame(decoded, std::chrono::milliseconds{timeout_ms});
}

bool JuraComponent::read_db_with_expected_len_(std::vector<uint8_t> &decoded, uint32_t total_timeout_ms,
                                               std::string_view cmd, size_t expected_len) {
  const uint32_t start = esphome::millis();
  bool echo_seen = false;
  while (true) {
    uint32_t now = esphome::millis();
    if (total_timeout_ms > 0 && now - start >= total_timeout_ms) {
      ESP_LOGW(TAG, "RX_DB timeout");
      return false;
    }

    uint32_t remaining = 0;
    if (total_timeout_ms > 0) {
      remaining = total_timeout_ms - (now - start);
    }

    if (!this->read_db_data_frame_(decoded, remaining)) {
      ESP_LOGW(TAG, "RX_DB timeout");
      return false;
    }

    if (!echo_seen) {
      bool ascii = std::all_of(decoded.begin(), decoded.end(), [](uint8_t c) {
        return c >= 0x20 && c <= 0x7E;
      });
      if (ascii && decoded.size() == cmd.size() &&
          std::memcmp(decoded.data(), cmd.data(), decoded.size()) == 0) {
        ESP_LOGD("jutta_proto", "RX_DB echo ignored: \"%.*s\"", static_cast<int>(decoded.size()),
                 reinterpret_cast<const char *>(decoded.data()));
        echo_seen = true;
        continue;
      }
      echo_seen = true;
    }

    if (decoded.size() == expected_len) {
      ESP_LOGD(TAG, "RX_DB decoded_len=%zu", decoded.size());
      return true;
    }

    ESP_LOGV(TAG, "RX_DB len=%u != %u → drop", static_cast<unsigned>(decoded.size()),
             static_cast<unsigned>(expected_len));
  }
}

void JuraComponent::publish_xml_values_() {
  if (!this->xml_enabled_ || !this->xml_has_values_) {
    return;
  }
  ESP_LOGD(TAG, "publish TR32=%s TG43=%s TGC0=%s",
           format_numeric_array(this->xml_tr32_values_).c_str(),
           format_numeric_array(this->xml_tg43_values_).c_str(),
           format_numeric_array(this->xml_tgc0_values_).c_str());
  for (size_t i = 0; i < this->xml_tr32_sensors_.size(); ++i) {
    if (this->xml_tr32_sensors_[i] != nullptr) {
      this->xml_tr32_sensors_[i]->publish_state(static_cast<float>(this->xml_tr32_values_[i]));
    }
  }
  for (size_t i = 0; i < this->xml_tg43_sensors_.size(); ++i) {
    if (this->xml_tg43_sensors_[i] != nullptr) {
      this->xml_tg43_sensors_[i]->publish_state(static_cast<float>(this->xml_tg43_values_[i]));
    }
  }
  for (size_t i = 0; i < this->xml_tgc0_sensors_.size(); ++i) {
    if (this->xml_tgc0_sensors_[i] != nullptr) {
      this->xml_tgc0_sensors_[i]->publish_state(static_cast<float>(this->xml_tgc0_values_[i]));
    }
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

