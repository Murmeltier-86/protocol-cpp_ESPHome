#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
constexpr uint32_t MACHINE_XML_TIMEOUT_MS = 1500;
constexpr std::size_t MACHINE_XML_MIN_LENGTH = 32;
const char *const MACHINE_XML_PRIMARY_COMMAND = "@hr:00\r\n";
const char *const MACHINE_XML_FALLBACK_COMMAND = "@hr:05\r\n";
constexpr uint32_t kXmlRxTimeoutMs = 1000;
constexpr uint32_t kInterCmdGapMs = 250;
constexpr uint32_t kXmlQuietMs = 120;
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
constexpr std::size_t kTR32MinFrameLength = 21;
constexpr std::size_t kTG43MinFrameLength = 13;
constexpr std::size_t kTGC0MinFrameLength = 13;


template<typename T>
auto set_sensor_entity_category_if_supported(T *sensor, EntityCategory category)
    -> decltype(sensor->set_entity_category(category), void()) {
  sensor->set_entity_category(category);
}

inline void set_sensor_entity_category_if_supported(...) {}

template<typename T>
auto set_sensor_unit_if_supported(T *sensor, const char *unit)
    -> decltype(sensor->set_unit_of_measurement(unit), void()) {
  sensor->set_unit_of_measurement(unit);
}

inline void set_sensor_unit_if_supported(...) {}

template<typename T>
auto set_sensor_icon_if_supported(T *sensor, const char *icon)
    -> decltype(sensor->set_icon(icon), void()) {
  sensor->set_icon(icon);
}

inline void set_sensor_icon_if_supported(...) {}


std::string sanitize_text_for_api(const std::string &input) {
  std::string out;
  out.reserve(input.size());
  constexpr char kHex[] = "0123456789ABCDEF";
  for (unsigned char c : input) {
    if (c >= 0x20 && c <= 0x7E) {
      out.push_back(static_cast<char>(c));
      continue;
    }
    if (c == '	') {
      out.push_back('	');
      continue;
    }
    out.push_back('\\');
    out.push_back('x');
    out.push_back(kHex[(c >> 4) & 0x0F]);
    out.push_back(kHex[c & 0x0F]);
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

void trim_in_place(std::string &value) {
  auto begin = value.find_first_not_of(" \t\r\n");
  auto end = value.find_last_not_of(" \t\r\n");
  if (begin == std::string::npos || end == std::string::npos) {
    value.clear();
    return;
  }
  value.assign(value.begin() + static_cast<std::ptrdiff_t>(begin),
               value.begin() + static_cast<std::ptrdiff_t>(end) + 1);
}

std::string collapse_whitespace(std::string value) {
  std::string result;
  result.reserve(value.size());
  bool last_space = false;
  for (unsigned char c : value) {
    if (std::isspace(c) != 0) {
      if (!last_space) {
        result.push_back(' ');
        last_space = true;
      }
    } else {
      result.push_back(static_cast<char>(c));
      last_space = false;
    }
  }
  trim_in_place(result);
  return result;
}

bool status_field_is_publishable(const JuraDecodedField &field) {
  return field.category == "status" || field.category == "alert";
}

std::string xml_table_for_decoded_category(const std::string &category) {
  if (category == "status") {
    return "PROGRESS_STATE_INTAKE";
  }
  if (category == "alert") {
    return "ALERTS";
  }
  if (category == "product_candidate") {
    return "PRODUCTS";
  }
  if (category == "process") {
    return "PROCESSES";
  }
  return "unknown";
}

void append_unique(std::vector<std::string> &values, const std::string &value) {
  if (value.empty()) {
    return;
  }
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

std::string join_values(const std::vector<std::string> &values, const char *separator) {
  std::string out;
  for (const auto &value : values) {
    if (!out.empty()) {
      out.append(separator);
    }
    out.append(value);
  }
  return out;
}

std::string truncate_for_log(const std::string &value, size_t limit = 240) {
  if (value.size() <= limit) {
    return value;
  }
  return value.substr(0, limit - 3) + "...";
}

std::string infer_response_command(const std::string &response, const std::string &active_command) {
  if (!active_command.empty()) {
    return active_command;
  }
  if (response.empty()) {
    return "unknown";
  }
  if (response[0] == '@') {
    auto end = response.find_first_of(":\r\n ");
    if (end != std::string::npos && end > 1) {
      return response.substr(0, end);
    }
  }
  if (response.size() >= 3 && std::isalpha(static_cast<unsigned char>(response[0])) != 0 &&
      std::isalpha(static_cast<unsigned char>(response[1])) != 0 && response[2] == ':') {
    return response.substr(0, 2);
  }
  return "status_hex";
}

std::string infer_response_family(const std::string &command) {
  if (command.empty() || command == "unknown") {
    return "unknown";
  }
  if (command == "status_hex") {
    return "status_hex";
  }
  if (command[0] == '@' && command.size() >= 3) {
    return command.substr(1, 2);
  }
  if (command.size() >= 2) {
    return command.substr(0, 2);
  }
  return command;
}

std::string format_decoded_field_trace(const std::vector<JuraDecodedField> &fields, std::vector<std::string> &tables,
                                       bool &has_publishable) {
  std::vector<std::string> details;
  has_publishable = false;
  for (const auto &field : fields) {
    if (field.category == "raw" || field.category == "unknown") {
      continue;
    }
    std::string table = xml_table_for_decoded_category(field.category);
    append_unique(tables, table);
    if (status_field_is_publishable(field)) {
      has_publishable = true;
    }

    std::string detail = table;
    detail.push_back(':');
    detail.append(field.key.empty() ? "?" : field.key);
    detail.append(" raw=");
    detail.append(field.raw_value.empty() ? "?" : field.raw_value);
    detail.append(" text='");
    detail.append(sanitize_text_for_api(field.decoded_text));
    detail.push_back('\'');
    if (field.category == "product_candidate") {
      detail.append(" publish=no reason=product_candidate_unverified");
    } else {
      detail.append(status_field_is_publishable(field) ? " publish=yes" : " publish=no");
    }
    details.push_back(detail);
  }
  if (details.empty()) {
    append_unique(tables, "unknown");
    return "none";
  }
  return truncate_for_log(join_values(details, " | "), 700);
}

std::string to_pascal_case(const std::string &value) {
  std::string result;
  result.reserve(value.size());
  bool capitalize = true;
  for (unsigned char c : value) {
    if (c == '_' || c == '-' || c == ' ') {
      capitalize = true;
      continue;
    }
    if (capitalize) {
      result.push_back(static_cast<char>(std::toupper(c)));
      capitalize = false;
    } else {
      result.push_back(static_cast<char>(std::tolower(c)));
    }
  }
  return result;
}

bool find_tag_in_scope(const std::string &xml, const std::string &tag, size_t scope_begin, size_t scope_end,
                       size_t &content_begin, size_t &content_end) {
  std::string open_tag = "<" + tag;
  std::string close_tag = "</" + tag + ">";
  size_t search_pos = scope_begin;
  while (true) {
    size_t open = xml.find(open_tag, search_pos);
    if (open == std::string::npos || open >= scope_end) {
      return false;
    }
    size_t name_end = open + open_tag.size();
    if (name_end < xml.size()) {
      unsigned char next = static_cast<unsigned char>(xml[name_end]);
      if (!(std::isspace(next) != 0 || next == '>' || next == '/')) {
        search_pos = name_end;
        continue;
      }
    }
    size_t open_end = xml.find('>', name_end);
    if (open_end == std::string::npos) {
      return false;
    }
    bool self_closing = open_end > open && xml[open_end - 1] == '/';
    size_t start = open_end + 1;
    if (self_closing) {
      content_begin = start;
      content_end = start;
      return true;
    }
    size_t close = xml.find(close_tag, start);
    if (close == std::string::npos) {
      search_pos = open_end + 1;
      continue;
    }
    if (close > scope_end) {
      search_pos = open_end + 1;
      continue;
    }
    content_begin = start;
    content_end = close;
    return true;
  }
}

bool xml_get_value_simple(const std::string &xml, const std::string &path, std::string &out) {
  if (path.empty()) {
    return false;
  }
  size_t scope_begin = 0;
  size_t scope_end = xml.size();
  size_t pos = 0;
  while (pos < path.size()) {
    size_t next = path.find('/', pos);
    std::string segment = (next == std::string::npos) ? path.substr(pos) : path.substr(pos, next - pos);
    if (segment.empty()) {
      pos = (next == std::string::npos) ? path.size() : next + 1;
      continue;
    }
    size_t content_begin = 0;
    size_t content_end = 0;
    if (!find_tag_in_scope(xml, segment, scope_begin, scope_end, content_begin, content_end)) {
      return false;
    }
    scope_begin = content_begin;
    scope_end = content_end;
    pos = (next == std::string::npos) ? path.size() : next + 1;
  }
  if (scope_begin > scope_end || scope_end > xml.size()) {
    return false;
  }
  out = xml.substr(scope_begin, scope_end - scope_begin);
  trim_in_place(out);
  return true;
}

std::string format_numeric_text(double value) {
  char buffer[32];
  int written = std::snprintf(buffer, sizeof(buffer), "%.6f", value);
  if (written < 0) {
    return std::to_string(value);
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
  this->connection_->set_log_decoded_tx(this->log_decoded_tx_);
  this->connection_->set_log_encoded_uart(this->log_encoded_uart_);
  this->connection_->set_response_callback([this](const std::string &response, const char *parser_branch) {
    this->handle_decoded_response_(response, parser_branch);
  });
  this->connection_->init();
  this->publish_machine_online_(false);
  this->publish_machine_ready_(false);

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
  ESP_LOGCONFIG(TAG, "  XML publish unstable counters: %s", YESNO(this->xml_publish_unstable_));
  ESP_LOGCONFIG(TAG, "  XML counter max: %u", static_cast<unsigned>(this->xml_counter_max_));
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
            this->publish_machine_type_();
            this->publish_last_command_result_("machine_type");
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
        this->publish_last_command_result_("handshake_done");
        this->publish_machine_status_("ready");
        this->publish_machine_online_(true);
        this->publish_machine_ready_(true);
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
    this->coffee_maker_->connection->set_log_decoded_tx(this->log_decoded_tx_);
    this->coffee_maker_->connection->set_log_encoded_uart(this->log_encoded_uart_);
    this->coffee_maker_->connection->set_response_callback([this](const std::string &response,
                                                                  const char *parser_branch) {
      this->handle_decoded_response_(response, parser_branch);
    });
    ESP_LOGI(TAG, "Coffee maker controller initialized.");
    this->reset_xml_poll_state_();
  }
}

void JuraComponent::restart_handshake(const char *reason) {
  if (reason != nullptr) {
    ESP_LOGW(TAG, "Restarting handshake: %s", reason);
    this->publish_last_command_result_(std::string("handshake_restart: ") + reason);
    this->publish_machine_status_(std::string("not_ready: ") + reason);
    this->publish_machine_online_(false);
    this->publish_machine_ready_(false);
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

void JuraComponent::handle_decoded_response_(const std::string &response, const char *parser_branch) {
  this->publish_raw_rx_(response, parser_branch);
  this->publish_machine_online_(true);

  std::string lowered = response;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered.rfind("ty:", 0) == 0) {
    if (response.size() > 3) {
      this->device_type_ = response;
    } else if (this->device_type_.empty()) {
      this->device_type_ = "TY:unknown";
    }
    this->publish_machine_type_();
    this->publish_last_command_result_("machine_type");
    return;
  }

  if (lowered == "ok:") {
    this->publish_last_command_result_("ok");
    return;
  }

  if (lowered == "@t1") {
    this->publish_last_command_result_("@t1");
    return;
  }

  if (lowered.rfind("@h", 0) == 0 && lowered.find(":error") != std::string::npos) {
    this->publish_machine_status_(response);
    this->publish_last_command_result_(response);
    return;
  }

  if (response.rfind("@T2", 0) == 0 || response.rfind("@T3", 0) == 0) {
    this->publish_last_command_result_(response);
    return;
  }

  if (parser_branch != nullptr && std::string(parser_branch) == "db_frame") {
    if (this->decode_and_publish_status_(response, parser_branch)) {
      return;
    }
    ESP_LOGD(TAG, "DB frame kept raw-only; no verified status text decoded");
  }
}

bool JuraComponent::decode_and_publish_status_(const std::string &response, const char *parser_branch) {
  if (response.find('<') != std::string::npos || response.find('>') != std::string::npos) {
    return false;
  }
  std::string active_command_raw = this->xml_last_command_;
  std::string active_command = active_command_raw;
  std::transform(active_command.begin(), active_command.end(), active_command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (active_command == "@tr:32" || active_command == "@tg:43" || active_command == "@tg:c0") {
    return false;
  }
  std::string branch = parser_branch != nullptr ? parser_branch : "unknown";
  std::string command = infer_response_command(response, active_command_raw);
  std::string family = infer_response_family(command);
  std::string payload_hex = format_hex_string(response);
  std::string raw_rx = sanitize_text_for_api(response);

  if (!this->ensure_xml_mapping_loaded_()) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=unknown fields=none final='' fallback=xml_mapping_unavailable",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str());
    return false;
  }

  std::vector<JuraDecodedField> fields;
  if (!decode_status_response(response, branch, fields) || fields.empty()) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=unknown fields=none final='' fallback=decoder_returned_no_fields",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str());
    return false;
  }
  this->last_decoded_fields_ = fields;
  std::vector<std::string> tables;
  bool has_publishable = false;
  std::string field_trace = format_decoded_field_trace(fields, tables, has_publishable);
  std::string table_trace = tables.empty() ? "unknown" : join_values(tables, ",");
  if (!has_publishable) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=%s fields=%s final='' fallback=no_verified_status_or_alert_match_raw_only",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str(),
             table_trace.c_str(), field_trace.c_str());
    return false;
  }
  std::string summary = this->format_decoded_status_(fields);
  if (summary.empty()) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=%s fields=%s final='' fallback=empty_summary",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str(),
             table_trace.c_str(), field_trace.c_str());
    return false;
  }
  ESP_LOGD(TAG,
           "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
           "xml_tables=%s fields=%s final='%s' fallback=%s",
           raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str(),
           table_trace.c_str(), field_trace.c_str(), sanitize_text_for_api(summary).c_str(), "none");
  this->publish_machine_status_(summary);
  this->publish_last_command_result_("decoded_status");
  return true;
}

std::string JuraComponent::format_decoded_status_(const std::vector<JuraDecodedField> &fields) const {
  std::vector<std::string> known;
  std::string raw;
  for (const auto &field : fields) {
    if (field.category == "raw") {
      raw = field.raw_value;
      continue;
    }
    if (field.category == "unknown" || field.category == "product_candidate") {
      continue;
    }
    std::string text = field.decoded_text.empty() ? field.raw_value : field.decoded_text;
    if (text.empty()) {
      continue;
    }
    std::string item;
    if (field.category == "status") {
      item = "Status: ";
    } else if (field.category == "alert") {
      item = "Alert: ";
    } else if (field.category == "product") {
      item = "Product: ";
    } else {
      item = field.category + ": ";
    }
    item.append(text);
    if (!field.raw_value.empty()) {
      item.append(" (");
      item.append(field.raw_value);
      item.push_back(')');
    }
    known.push_back(item);
  }

  if (known.empty()) {
    return raw.empty() ? std::string{} : std::string("Raw: ") + raw;
  }

  std::string summary;
  for (const auto &item : known) {
    if (!summary.empty()) {
      summary.append("; ");
    }
    if (summary.size() + item.size() > 240) {
      summary.append("...");
      break;
    }
    summary.append(item);
  }
  return sanitize_text_for_api(summary);
}

bool JuraComponent::is_printable_status_text_(const std::string &text) const {
  if (text.empty()) {
    return false;
  }
  size_t escaped_binary_markers = 0;
  size_t hex_like_tokens = 0;
  for (size_t i = 0; i < text.size(); ++i) {
    unsigned char c = static_cast<unsigned char>(text[i]);
    if (c == '\r' || c == '\n' || c == '\t') {
      continue;
    }
    if (c < 0x20 || c == 0x7F) {
      return false;
    }
    if (c >= 0x80) {
      size_t remaining = 0;
      if ((c & 0xE0) == 0xC0) {
        remaining = 1;
        if (c < 0xC2) {
          return false;
        }
      } else if ((c & 0xF0) == 0xE0) {
        remaining = 2;
      } else if ((c & 0xF8) == 0xF0) {
        remaining = 3;
        if (c > 0xF4) {
          return false;
        }
      } else {
        return false;
      }
      if (i + remaining >= text.size()) {
        return false;
      }
      for (size_t j = 1; j <= remaining; ++j) {
        unsigned char cc = static_cast<unsigned char>(text[i + j]);
        if ((cc & 0xC0) != 0x80) {
          return false;
        }
      }
      i += remaining;
      continue;
    }
    if (c == '\\' && i + 1 < text.size() && text[i + 1] == 'x') {
      ++escaped_binary_markers;
    }
    if (c == '0' && i + 1 < text.size() && (text[i + 1] == 'x' || text[i + 1] == 'X')) {
      ++hex_like_tokens;
    }
  }
  if (escaped_binary_markers > 0) {
    return false;
  }
  return hex_like_tokens <= 4 || text.find('<') != std::string::npos;
}

void JuraComponent::publish_raw_rx_(const std::string &response, const char *parser_branch) {
  std::string safe = sanitize_text_for_api(response);
  ESP_LOGV(TAG, "RX decoded branch=%s value='%s'", parser_branch != nullptr ? parser_branch : "unknown",
           safe.c_str());
  if (this->raw_rx_sensor_ != nullptr) {
    this->raw_rx_sensor_->publish_state(safe);
  }
}

void JuraComponent::publish_last_command_result_(const std::string &result) {
  std::string safe = sanitize_text_for_api(result);
  ESP_LOGV(TAG, "Command result: %s", safe.c_str());
  if (this->last_command_result_sensor_ != nullptr) {
    this->last_command_result_sensor_->publish_state(safe);
  }
}

void JuraComponent::publish_machine_type_() {
  if (this->machine_type_sensor_ != nullptr && !this->device_type_.empty()) {
    this->machine_type_sensor_->publish_state(sanitize_text_for_api(this->device_type_));
  }
}

void JuraComponent::publish_machine_status_(const std::string &status) {
  if (!this->is_printable_status_text_(status)) {
    ESP_LOGW(TAG, "Machine status publish suppressed (binary/non-printable payload)");
    return;
  }
  std::string safe = sanitize_text_for_api(status);
  ESP_LOGV(TAG, "Machine status: %s", safe.c_str());
  if (this->machine_status_sensor_ != nullptr) {
    this->machine_status_sensor_->publish_state(safe);
  }
}

void JuraComponent::publish_machine_online_(bool online) {
  ESP_LOGD(TAG, "Machine online: %s", YESNO(online));
  if (this->machine_online_sensor_ != nullptr) {
    this->machine_online_sensor_->publish_state(online);
  }
}

void JuraComponent::publish_machine_ready_(bool ready) {
  ESP_LOGD(TAG, "Machine ready: %s", YESNO(ready));
  if (this->machine_ready_sensor_ != nullptr) {
    this->machine_ready_sensor_->publish_state(ready);
  }
}

void JuraComponent::process_machine_data_query() {
  if (this->machine_data_sensor_ == nullptr && this->machine_status_sensor_ == nullptr) {
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
  bool xml_poll_active = this->xml_inflight_ || this->db_transaction_owner_ == DbTransactionOwner::XML_POLL ||
                         (this->enable_xml_poll_ && this->xml_state_ != XmlPollState::IDLE &&
                          this->xml_state_ != XmlPollState::SLEEP);
  if (xml_poll_active) {
    ESP_LOGD(TAG, "Machine-XML query skipped while XML DB polling is active");
    return;
  }

  uint32_t now = esphome::millis();
  if (this->machine_data_query_next_ != 0 && !time_reached(now, this->machine_data_query_next_)) {
    return;
  }
  this->machine_data_query_next_ = now + MACHINE_DATA_QUERY_INTERVAL_MS;

  std::string xml;
  if (!this->request_machine_xml_(xml)) {
    ESP_LOGW(TAG, "Machine-XML konnte nicht abgefragt werden.");
    return;
  }

  this->handle_machine_xml_(xml);
}

void JuraComponent::publish_machine_data_(const std::string &response) {
  std::string sanitized = response;
  sanitized.erase(std::remove_if(sanitized.begin(), sanitized.end(),
                                 [](unsigned char c) { return c == '\r' || c == '\n'; }),
                  sanitized.end());
  ESP_LOGD(TAG, "Machine data response: %s", sanitize_text_for_api(sanitized).c_str());
  if (!this->is_printable_status_text_(sanitized)) {
    ESP_LOGW(TAG, "Machine data publish skipped (binary/non-printable payload)");
    return;
  }
  if (this->machine_data_sensor_ != nullptr) {
    std::string safe = sanitize_text_for_api(sanitized);
    this->machine_data_sensor_->publish_state(safe);
  }
}

bool JuraComponent::request_machine_xml_(std::string &xml) {
  xml.clear();
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  bool xml_poll_active = this->xml_inflight_ || this->db_transaction_owner_ == DbTransactionOwner::XML_POLL ||
                         (this->enable_xml_poll_ && this->xml_state_ != XmlPollState::IDLE &&
                          this->xml_state_ != XmlPollState::SLEEP);
  if (xml_poll_active) {
    ESP_LOGD(TAG, "Machine-XML query skipped while XML DB polling is active");
    return false;
  }

  auto *connection = this->coffee_maker_->connection.get();

  auto send_and_receive = [&](const char *command, std::string &out) -> bool {
    if (command == nullptr || command[0] == '\0') {
      return false;
    }
    if (this->db_transaction_owner_ != DbTransactionOwner::NONE) {
      ESP_LOGD(TAG, "Machine-XML command %s skipped; DB transaction already active", command);
      return false;
    }
    this->db_transaction_owner_ = DbTransactionOwner::MACHINE_XML;
    connection->reset_db_rx_buffer();
    if (!connection->write_decoded(command)) {
      ESP_LOGW(TAG, "Machine-XML Befehl %s konnte nicht gesendet werden.", command);
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
    std::vector<uint8_t> decoded;
    bool had_crlf = false;
    size_t decoded_len = 0;
    if (!connection->read_db_frame(decoded, MACHINE_XML_TIMEOUT_MS, &had_crlf, &decoded_len)) {
      ESP_LOGW(TAG, "Machine-XML timeout for %s", command);
      connection->reset_db_rx_buffer();
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
    if (decoded.empty()) {
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
    out.assign(decoded.begin(), decoded.end());
    if (!this->is_printable_status_text_(out)) {
      ESP_LOGW(TAG, "Machine-XML ignored binary response");
      connection->reset_db_rx_buffer();
      out.clear();
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
    out.erase(std::remove(out.begin(), out.end(), '\r'), out.end());
    trim_in_place(out);
    bool ok = !out.empty();
    this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
    return ok;
  };

  std::string primary;
  if (send_and_receive(MACHINE_XML_PRIMARY_COMMAND, primary)) {
    if (primary.size() >= MACHINE_XML_MIN_LENGTH) {
      xml.swap(primary);
      return true;
    }
    ESP_LOGW(TAG, "Machine-XML Antwort zu kurz (%u Byte) – versuche Fallback.",
             static_cast<unsigned>(primary.size()));
  } else {
    ESP_LOGW(TAG, "Machine-XML Primärkommando ohne Antwort – versuche Fallback.");
  }

  std::string fallback;
  if (send_and_receive(MACHINE_XML_FALLBACK_COMMAND, fallback)) {
    xml.swap(fallback);
    return true;
  }

  if (!primary.empty()) {
    xml.swap(primary);
    return true;
  }

  return false;
}

void JuraComponent::handle_machine_xml_(const std::string &xml) {
  if (!this->is_printable_status_text_(xml)) {
    ESP_LOGW(TAG, "Machine-XML ignored binary response");
    return;
  }
  if (xml.find('<') == std::string::npos || xml.find('>') == std::string::npos) {
    ESP_LOGW(TAG, "Machine-XML ignored non-XML response");
    return;
  }
  uint32_t now = esphome::millis();
  std::string normalized = xml;
  normalized.erase(std::remove(normalized.begin(), normalized.end(), '\r'), normalized.end());
  this->machine_xml_cache_ = normalized;
  this->machine_xml_timestamp_ = now;

  std::string summary = this->format_machine_status_summary_(normalized);
  if (this->is_printable_status_text_(summary)) {
    this->publish_machine_status_(summary);
    this->publish_machine_data_(summary);
  }
  this->update_settings_from_xml_(normalized);
  this->update_errors_from_xml_(normalized);
}

bool JuraComponent::ensure_machine_xml_(uint32_t max_age_ms, std::string &xml_out) {
  uint32_t now = esphome::millis();
  if (!this->machine_xml_cache_.empty()) {
    bool fresh = max_age_ms == 0;
    if (!fresh && this->machine_xml_timestamp_ != 0) {
      fresh = static_cast<int32_t>(now - this->machine_xml_timestamp_) <= static_cast<int32_t>(max_age_ms);
    }
    if (fresh) {
      xml_out = this->machine_xml_cache_;
      return true;
    }
  }

  std::string fetched;
  if (!this->request_machine_xml_(fetched)) {
    return false;
  }
  this->handle_machine_xml_(fetched);
  xml_out = this->machine_xml_cache_;
  return !xml_out.empty();
}

std::string JuraComponent::format_machine_status_summary_(const std::string &xml) const {
  std::vector<std::pair<std::string, std::string>> fields;
  std::string value;
  if (this->xml_get_value_(xml, "Machine/Status/State", value) && !value.empty()) {
    fields.emplace_back("State", value);
  }
  if (this->xml_get_value_(xml, "Machine/Status/WaterLevel", value) && !value.empty()) {
    fields.emplace_back("Water", value);
  }
  if (this->xml_get_value_(xml, "Machine/Status/BeanLevel", value) && !value.empty()) {
    fields.emplace_back("Beans", value);
  }
  if (this->xml_get_value_(xml, "Machine/Status/ErrorCode", value) && !value.empty()) {
    fields.emplace_back("Error", value);
  }

  if (fields.empty()) {
    std::string collapsed = collapse_whitespace(xml);
    if (collapsed.size() > 160) {
      collapsed.resize(157);
      collapsed.append("...");
    }
    return collapsed;
  }

  std::string summary;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i > 0) {
      summary.append(", ");
    }
    summary.append(fields[i].first);
    summary.push_back('=');
    summary.append(fields[i].second);
  }
  return summary;
}

void JuraComponent::update_settings_from_xml_(const std::string &xml) {
  this->ensure_setting_entities_created_();
  if (this->setting_descs_.empty()) {
    return;
  }

  for (const auto &entry : this->setting_descs_) {
    const auto &desc = entry.second;
    if (!desc.source_cmd.empty()) {
      continue;
    }
    std::string path = this->determine_setting_path_(desc);
    if (path.empty()) {
      continue;
    }
    std::string raw_value;
    if (!this->xml_get_value_(xml, path, raw_value)) {
      continue;
    }
    if (desc.type == SettingValueType::String) {
      this->publish_setting_value_(desc, 0.0f, raw_value);
      continue;
    }
    if (raw_value.empty()) {
      continue;
    }

    double numeric = 0.0;
    bool parsed = false;
    {
      char *end = nullptr;
      numeric = std::strtod(raw_value.c_str(), &end);
      parsed = end != nullptr && end != raw_value.c_str();
      if (!parsed) {
        long long as_int = std::strtoll(raw_value.c_str(), &end, 0);
        if (end != nullptr && end != raw_value.c_str()) {
          numeric = static_cast<double>(as_int);
          parsed = true;
        }
      }
    }
    if (!parsed) {
      std::string lowered = raw_value;
      std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (desc.type == SettingValueType::Bool) {
        bool value_bool = (lowered == "true" || lowered == "on" || lowered == "1" || lowered == "yes");
        this->publish_setting_value_(desc, value_bool ? 1.0f : 0.0f, value_bool ? "on" : "off");
      }
      continue;
    }

    double scaled = numeric * static_cast<double>(desc.scale);
    std::string text_value;
    float publish_value = static_cast<float>(scaled);
    switch (desc.type) {
      case SettingValueType::Bool: {
        bool value_bool = scaled != 0.0;
        publish_value = value_bool ? 1.0f : 0.0f;
        text_value = value_bool ? "on" : "off";
        break;
      }
      case SettingValueType::String:
        text_value = raw_value;
        break;
      case SettingValueType::Enum:
      case SettingValueType::U8:
      case SettingValueType::U16:
      case SettingValueType::U32:
        text_value = format_numeric_text(scaled);
        break;
    }
    this->publish_setting_value_(desc, publish_value, text_value);
  }
}

void JuraComponent::update_errors_from_xml_(const std::string &xml) {
  std::string path = error_source_path();
  if (path.empty()) {
    path = "Machine/Status/ErrorCode";
  }
  if (path.empty()) {
    return;
  }
  std::string raw;
  if (!this->xml_get_value_(xml, path, raw)) {
    return;
  }
  if (raw.empty()) {
    if (this->errors_entities_created_ && this->last_error_code_ != 0) {
      this->publish_error_state_(0);
    }
    return;
  }
  char *end = nullptr;
  uint32_t code = static_cast<uint32_t>(std::strtoul(raw.c_str(), &end, 0));
  if (end == raw.c_str()) {
    return;
  }
  if (!this->errors_entities_created_ || code != this->last_error_code_) {
    this->publish_error_state_(code);
  }
}

bool JuraComponent::xml_get_value_(const std::string &xml, const std::string &path, std::string &out) const {
  return xml_get_value_simple(xml, path, out);
}

std::string JuraComponent::determine_setting_path_(const SettingDesc &desc) const {
  if (!desc.path.empty()) {
    return desc.path;
  }
  if (desc.id.empty()) {
    return {};
  }
  std::string tag = to_pascal_case(desc.id);
  if (tag.empty()) {
    tag = desc.id;
  }
  return "Machine/Settings/" + tag;
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
  this->db_transaction_owner_ = DbTransactionOwner::XML_POLL;
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

void JuraComponent::clear_db_transaction_(DbTransactionOwner owner) {
  if (owner == DbTransactionOwner::NONE || this->db_transaction_owner_ == owner) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
  }
}

bool JuraComponent::validate_xml_frame_(XmlPollState state, const std::vector<uint8_t> &decoded, bool had_crlf,
                                        size_t decoded_len, std::vector<uint8_t> &payload,
                                        size_t &expected_min_len, uint8_t &head0) const {
  (void) had_crlf;
  const char *expected_command = this->xml_state_command_(state);
  if (expected_command == nullptr || expected_command[0] == '\0') {
    ESP_LOGW(TAG, "XML validate: kein erwarteter Befehl für Zustand %d", static_cast<int>(state));
    return false;
  }
  if (this->xml_last_command_ != expected_command) {
    ESP_LOGW(TAG, "XML %s verworfen: pending command mismatch (pending=%s expected=%s)",
             this->xml_state_label_(state), this->xml_last_command_.c_str(), expected_command);
    return false;
  }

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
  if (start_index != 0) {
    ESP_LOGW(TAG, "XML %s verworfen: Startbyte 0x26 erst bei Index %d (decoded_len=%u)",
             this->xml_state_label_(state), start_index, static_cast<unsigned>(decoded.size()));
    return false;
  }

  payload.assign(decoded.begin() + start_index, decoded.end());
  head0 = payload.empty() ? 0x00 : payload.front();

  std::size_t avail = payload.size();
  if (avail != expected_min_len) {
    ESP_LOGW(TAG, "XML %s verworfen: decoded_len (%u) != expected_len (%u)",
             this->xml_state_label_(state), static_cast<unsigned>(avail), static_cast<unsigned>(expected_min_len));
    return false;
  }
  if (head0 != 0x26) {
    ESP_LOGW(TAG, "XML %s: erstes Payload-Byte 0x%02X statt 0x26", this->xml_state_label_(state),
             static_cast<unsigned>(head0));
    return false;
  }

  ESP_LOGD(TAG, "XML RX cmd=%s decoded_len=%u expected_len=%u head0=0x%02X", expected_command,
           static_cast<unsigned>(decoded_len), static_cast<unsigned>(expected_min_len),
           static_cast<unsigned>(head0));
  return true;
}

bool JuraComponent::validate_counter_frame_(XmlPollState state, const XmlCommandMapping &mapping,
                                            const std::vector<uint8_t> &frame, const char *command_label,
                                            std::string &reason) const {
  if (mapping.empty()) {
    reason = "mapping_empty";
    return false;
  }
  if (frame.empty() || frame.front() != 0x26) {
    reason = "missing_frame_header_0x26";
    return false;
  }
  for (const auto &field : mapping.fields) {
    if (field.offset + field.size > frame.size()) {
      reason = "field_overflow:" + field.name;
      return false;
    }
    if (field.size != 1 && field.size != 2 && field.size != 4) {
      reason = "unsupported_field_size:" + field.name;
      return false;
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
    if (!std::isfinite(value)) {
      reason = "non_finite_value:" + field.name;
      return false;
    }
    if (value < 0.0 || value > static_cast<double>(this->xml_counter_max_)) {
      reason = "counter_out_of_range:" + std::string(command_label != nullptr ? command_label : "?") + ":" +
               field.name + "=" + format_numeric_text(value) + " max=" + std::to_string(this->xml_counter_max_);
      return false;
    }
  }
  (void) state;
  reason.clear();
  return true;
}

bool JuraComponent::counter_frame_is_stable_(XmlPollState state, const std::vector<uint8_t> &frame,
                                             const char *command_label, std::string &reason) {
  if (this->xml_publish_unstable_) {
    reason.clear();
    return true;
  }
  size_t index = this->xml_command_index_(state);
  if (index >= this->xml_counter_candidate_frame_.size()) {
    reason = "invalid_command_index";
    return false;
  }
  if (this->xml_counter_candidate_frame_[index] == frame) {
    if (this->xml_counter_candidate_count_[index] < std::numeric_limits<uint8_t>::max()) {
      this->xml_counter_candidate_count_[index] += 1;
    }
  } else {
    this->xml_counter_candidate_frame_[index] = frame;
    this->xml_counter_candidate_count_[index] = 1;
  }
  if (this->xml_counter_candidate_count_[index] < 2) {
    reason = "unstable_frame_waiting_for_repeat:" + std::string(command_label != nullptr ? command_label : "?");
    return false;
  }
  reason.clear();
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
    double percent = static_cast<double>(raw_value) * field.scale;
    if (field.has_add) {
      percent += field.add;
    }
    if (!std::isfinite(percent)) {
      auto &state = this->tgc0_filters_[field.name];
      state.window.clear();
      state.consecutive_valid = 0;
      continue;
    }
    if (percent < 0.0 || percent > 100.0) {
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
  this->transition_to_state_(resend_state, now, kInterCmdGapMs);
  return true;
}

void JuraComponent::handle_xml_failure_(XmlPollState wait_state, bool is_timeout, size_t decoded_len, uint32_t now) {
  size_t index = this->xml_command_index_(wait_state);
  if (index < this->xml_counter_candidate_frame_.size()) {
    this->xml_counter_candidate_frame_[index].clear();
    this->xml_counter_candidate_count_[index] = 0;
  }
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
  if (is_timeout) {
    if (this->coffee_maker_ != nullptr && this->coffee_maker_->connection != nullptr) {
      this->coffee_maker_->connection->reset_db_rx_buffer();
    }
    this->clear_db_transaction_(DbTransactionOwner::XML_POLL);
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
    this->transition_to_state_(next_state, now, kInterCmdGapMs);
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
  set_sensor_entity_category_if_supported(sensor, EntityCategory::ENTITY_CATEGORY_DIAGNOSTIC);
  if (meta.has_unit) {
    set_sensor_unit_if_supported(sensor, meta.unit_of_measurement.c_str());
  }
  if (meta.has_icon) {
    set_sensor_icon_if_supported(sensor, meta.icon.c_str());
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
  for (auto &candidate : this->xml_counter_candidate_frame_) {
    candidate.clear();
  }
  this->xml_counter_candidate_count_.fill(0);
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
  this->clear_db_transaction_(DbTransactionOwner::NONE);
  this->xml_last_command_.clear();
  this->xml_rx_buffer_.clear();
  this->xml_stats_.clear();
  this->xml_next_poll_ = esphome::millis();
  this->xml_retry_count_.fill(0);
  this->xml_invalid_len_seen_.fill(false);
  this->xml_last_invalid_len_.fill(0);
  for (auto &candidate : this->xml_counter_candidate_frame_) {
    candidate.clear();
  }
  this->xml_counter_candidate_count_.fill(0);
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
  if (this->db_transaction_owner_ == DbTransactionOwner::MACHINE_XML) {
    ESP_LOGD(TAG, "XML DB polling skipped while Machine-XML transaction is active");
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
          this->transition_to_state_(next, now, kInterCmdGapMs);
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
          this->handle_xml_failure_(this->xml_state_, false, decoded_len, now);
          return;
        }
        if (this->xml_state_ == XmlPollState::WAIT_TGC0) {
          if (!this->process_valid_tgc0_frame_(payload, false)) {
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
        std::string reason;
        if (!this->validate_counter_frame_(XmlPollState::PARSE_TR32, this->xml_mapping_.tr32, this->xml_rx_buffer_,
                                           "@TR:32", reason)) {
          ESP_LOGW(TAG, "XML @TR:32 verworfen: %s", reason.c_str());
        } else if (!this->counter_frame_is_stable_(XmlPollState::PARSE_TR32, this->xml_rx_buffer_, "@TR:32", reason)) {
          ESP_LOGW(TAG, "XML @TR:32 nicht publiziert: %s", reason.c_str());
        } else {
          any = this->stage_counter_frame_(this->xml_mapping_.tr32, this->xml_rx_buffer_, "@TR:32");
        }
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
        std::string reason;
        if (!this->validate_counter_frame_(XmlPollState::PARSE_TG43, this->xml_mapping_.tg43, this->xml_rx_buffer_,
                                           "@TG:43", reason)) {
          ESP_LOGW(TAG, "XML @TG:43 verworfen: %s", reason.c_str());
        } else if (!this->counter_frame_is_stable_(XmlPollState::PARSE_TG43, this->xml_rx_buffer_, "@TG:43", reason)) {
          ESP_LOGW(TAG, "XML @TG:43 nicht publiziert: %s", reason.c_str());
        } else {
          any = this->stage_counter_frame_(this->xml_mapping_.tg43, this->xml_rx_buffer_, "@TG:43");
        }
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
        this->clear_db_transaction_(DbTransactionOwner::XML_POLL);
        return;
      }
      if (static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        this->xml_deadline_ms_ = 0;
        this->xml_next_action_ms_ = 0;
        this->xml_state_ = XmlPollState::IDLE;
        this->xml_next_poll_ = now;
        this->xml_inflight_ = false;
        this->xml_last_command_.clear();
        this->clear_db_transaction_(DbTransactionOwner::XML_POLL);
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
  if (state == XmlPollState::SLEEP || state == XmlPollState::IDLE) {
    this->clear_db_transaction_(DbTransactionOwner::XML_POLL);
  }
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
  if (this->db_transaction_owner_ == DbTransactionOwner::MACHINE_XML) {
    ESP_LOGD(TAG, "TX_DB \"%s\" skipped while Machine-XML transaction is active", command);
    return false;
  }
  if (this->db_transaction_owner_ == DbTransactionOwner::NONE) {
    this->db_transaction_owner_ = DbTransactionOwner::XML_POLL;
  }
  if (this->xml_inflight_) {
    return false;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_db_rx_buffer();
  this->xml_rx_buffer_.clear();
  ESP_LOGD(TAG, "TX_DB \"%s\"", command);
  connection->tx_db_command(command, false);
  this->xml_inflight_ = true;
  this->xml_last_command_ = command;
  this->xml_deadline_ms_ = now + kXmlRxTimeoutMs;
  this->xml_next_action_ms_ = now + kXmlQuietMs;
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
    this->publish_last_command_result_(std::string("timeout: ") + command);
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
  this->publish_last_command_result_(std::string("response: ") + command);
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
    text_it->second->publish_state(sanitize_text_for_api(raw_text));
  }
}

void JuraComponent::poll_settings_refresh_() {
  this->ensure_setting_entities_created_();
  if (this->setting_descs_.empty()) {
    return;
  }
  bool has_command_settings = false;
  bool has_xml_settings = false;
  for (const auto &entry : this->setting_descs_) {
    if (!entry.second.source_cmd.empty()) {
      has_command_settings = true;
    } else {
      has_xml_settings = true;
    }
  }

  if (has_command_settings) {
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
          text_value = format_numeric_text(static_cast<double>(scaled));
          break;
        case SettingValueType::String:
          text_value.assign(reinterpret_cast<const char *>(ptr), desc.width);
          if (auto zero_pos = text_value.find('\0'); zero_pos != std::string::npos) {
            text_value.resize(zero_pos);
          }
          this->publish_setting_value_(desc, 0.0f, text_value);
          continue;
      }
      this->publish_setting_value_(desc, scaled, text_value);
    }
  }

  if (has_xml_settings) {
    uint32_t previous_timestamp = this->machine_xml_timestamp_;
    std::string xml;
    if (this->ensure_machine_xml_(kSettingsRefreshMs, xml)) {
      if (previous_timestamp == this->machine_xml_timestamp_) {
        this->update_settings_from_xml_(xml);
      }
    }
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
  this->publish_last_command_result_(has_error ? "error_active" : "no_error");
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
  if (!command.empty()) {
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
    return;
  }

  uint32_t previous_timestamp = this->machine_xml_timestamp_;
  std::string xml;
  if (this->ensure_machine_xml_(kErrorPollIntervalMs, xml)) {
    if (previous_timestamp == this->machine_xml_timestamp_) {
      this->update_errors_from_xml_(xml);
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
