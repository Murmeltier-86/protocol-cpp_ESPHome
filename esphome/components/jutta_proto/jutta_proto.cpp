#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "esphome/core/application.h"
#include "esphome/core/time.h"
#include "machine_data_parser.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 2000;

struct MachineDataCommandDefinition {
  const char *command;
  const char *section;
  const char *element;
};

constexpr std::array<MachineDataCommandDefinition, 3> MACHINE_DATA_COMMANDS = {{{"@TR:32", "STATISTIC", "PRODUCTCOUNTER"},
                                                                              {"@TG:43", "STATISTIC", "MAINTENANCECOUNTER"},
                                                                              {"@TG:C0", "STATISTIC", "MAINTENANCEPERCENT"}}};

struct ProductCounterDefinition {
  const char *name;
  const char *code;
};

constexpr std::array<ProductCounterDefinition, 10> PRODUCT_COUNTER_DEFINITIONS = {{{"Espresso", "02"},
                                                                                   {"Coffee", "03"},
                                                                                   {"Cappuccino", "04"},
                                                                                   {"Espresso Macchiato", "06"},
                                                                                   {"Milk Foam", "08"},
                                                                                   {"Hotwater Portion", "0D"},
                                                                                   {"Cafe Barista", "28"},
                                                                                   {"Barista Lungo", "29"},
                                                                                   {"Espresso Doppio", "30"},
                                                                                   {"2 Espressi", "12"}}};

constexpr std::array<const char *, 6> MAINTENANCE_COUNTER_NAMES = {{"Cleaning", "FilterChange", "Decalc",
                                                                   "CappuRinse", "CoffeeRinse", "CappuClean"}};

constexpr std::array<const char *, 6> MAINTENANCE_PERCENT_NAMES = {{"Cleaning", "FilterChange", "Decalc",
                                                                   "CappuRinse", "CoffeeRinse", "CappuClean"}};

uint16_t read_u16_le(const uint8_t *ptr) {
  return static_cast<uint16_t>(static_cast<uint16_t>(ptr[0]) |
                               (static_cast<uint16_t>(ptr[1]) << 8));
}

uint32_t read_u32_le(const uint8_t *ptr) {
  return static_cast<uint32_t>(ptr[0]) | (static_cast<uint32_t>(ptr[1]) << 8) |
         (static_cast<uint32_t>(ptr[2]) << 16) | (static_cast<uint32_t>(ptr[3]) << 24);
}

std::optional<std::string> format_product_counter_payload(const std::string &payload) {
  if (payload.size() != 21) {
    return std::nullopt;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  uint8_t header = data[0];

  std::ostringstream stream;
  stream << "header=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<int>(header) << std::dec;

  for (size_t i = 0; i < PRODUCT_COUNTER_DEFINITIONS.size(); ++i) {
    uint16_t value = read_u16_le(&data[1 + i * 2]);
    stream << "; " << PRODUCT_COUNTER_DEFINITIONS[i].name;
    if (PRODUCT_COUNTER_DEFINITIONS[i].code != nullptr) {
      stream << '[' << PRODUCT_COUNTER_DEFINITIONS[i].code << ']';
    }
    stream << '=' << value;
  }

  return stream.str();
}

std::optional<std::string> format_maintenance_counter_payload(const std::string &payload) {
  if (payload.size() != 13) {
    return std::nullopt;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  uint8_t header = data[0];

  std::ostringstream stream;
  stream << "header=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<int>(header) << std::dec;

  for (size_t i = 0; i < MAINTENANCE_COUNTER_NAMES.size(); ++i) {
    uint16_t value = read_u16_le(&data[1 + i * 2]);
    stream << "; " << MAINTENANCE_COUNTER_NAMES[i] << '=' << value;
  }

  return stream.str();
}

std::optional<std::string> format_maintenance_percent_payload(const std::string &payload) {
  if (payload.size() != 13 && payload.size() != 14) {
    return std::nullopt;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  size_t header_size = payload.size() == 14 ? 2 : 1;
  uint16_t header = header_size == 2 ? read_u16_le(data) : static_cast<uint16_t>(data[0]);
  size_t value_offset = header_size;
  size_t values_bytes = payload.size() - value_offset;
  if (values_bytes % 2 != 0) {
    return std::nullopt;
  }
  size_t value_count = values_bytes / 2;

  std::ostringstream stream;
  stream << "header=0x" << std::uppercase << std::hex << std::setfill('0')
         << std::setw(static_cast<int>(header_size * 2)) << header << std::dec;

  for (size_t i = 0; i < value_count; ++i) {
    uint16_t raw_value = read_u16_le(&data[value_offset + i * 2]);
    const char *name = i < MAINTENANCE_PERCENT_NAMES.size() ? MAINTENANCE_PERCENT_NAMES[i] : nullptr;
    stream << "; ";
    if (name != nullptr) {
      stream << name;
    } else {
      stream << "Value" << (i + 1);
    }
    stream << '=' << raw_value;
    std::ostringstream percent_stream;
    percent_stream << std::fixed << std::setprecision(2)
                   << (static_cast<double>(raw_value) * 100.0 / 65535.0);
    stream << " (" << percent_stream.str() << "%)";
  }

  return stream.str();
}

std::optional<std::string> parse_machine_data_response(const std::string &command, const std::string &payload) {
  if (command == std::string("@TR:32")) {
    return format_product_counter_payload(payload);
  }
  if (command == std::string("@TG:43")) {
    return format_maintenance_counter_payload(payload);
  }
  if (command == std::string("@TG:C0")) {
    return format_maintenance_percent_payload(payload);
  }
  return std::nullopt;
}

std::string to_upper_hex(const std::string &value) {
  if (value.empty()) {
    return "";
  }
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0');
  for (unsigned char c : value) {
    stream << std::setw(2) << static_cast<int>(c);
  }
  return stream.str();
}

bool is_valid_utf8(const std::string &value);

bool is_safe_xml_text(const std::string &value) {
  if (value.empty()) {
    return true;
  }
  if (!is_valid_utf8(value)) {
    return false;
  }
  for (unsigned char c : value) {
    if (c < 0x20 || c == 0x7F) {
      return false;
    }
    switch (c) {
      case '&':
      case '<':
      case '>':
        return false;
      default:
        break;
    }
  }
  return true;
}

std::string xml_escape(const std::string &value) {
  std::ostringstream stream;
  for (unsigned char c : value) {
    switch (c) {
      case '&':
        stream << "&amp;";
        break;
      case '<':
        stream << "&lt;";
        break;
      case '>':
        stream << "&gt;";
        break;
      case '\"':
        stream << "&quot;";
        break;
      case 0x27:
        stream << "&apos;";
        break;
      default:
        stream << static_cast<char>(c);
        break;
    }
  }
  return stream.str();
}

std::string build_machine_data_payload(const std::vector<std::string> &responses) {
  std::ostringstream stream;
  stream << "<MACHINE_DATA>";

  std::string open_section;
  auto close_section = [&]() {
    if (!open_section.empty()) {
      stream << "</" << open_section << ">";
      open_section.clear();
    }
  };

  for (size_t i = 0; i < MACHINE_DATA_COMMANDS.size(); ++i) {
    const auto &definition = MACHINE_DATA_COMMANDS[i];
    if (open_section != definition.section) {
      close_section();
      open_section = definition.section;
      stream << '<' << open_section << '>';
    }

    std::string response = i < responses.size() ? responses[i] : std::string{};
    stream << '<' << definition.element << " command=\"" << definition.command << "\"";
    if (!response.empty()) {
      stream << " raw_hex=\"" << to_upper_hex(response) << "\"";
      auto parsed_text = parse_machine_data_response(definition.command, response);
      if (parsed_text.has_value()) {
        stream << " encoding=\"text\">" << xml_escape(parsed_text.value()) << "</" << definition.element << ">";
      } else if (is_safe_xml_text(response)) {
        stream << " encoding=\"text\">" << xml_escape(response) << "</" << definition.element << ">";
      } else {
        stream << " encoding=\"hex\"/>";
      }
    } else {
      stream << "/>";
    }
  }

  close_section();

  stream << "<ALERTS/>";
  stream << "<PROGRESS_STATE_INTAKE/>";
  stream << "<PROCESSES/>";
  stream << "</MACHINE_DATA>";
  return stream.str();
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

bool is_valid_utf8(const std::string &value) {
  size_t i = 0;
  while (i < value.size()) {
    unsigned char c = static_cast<unsigned char>(value[i]);
    if ((c & 0x80) == 0) {
      ++i;
      continue;
    }

    size_t sequence_length = 0;
    if ((c & 0xE0) == 0xC0) {
      sequence_length = 2;
      if ((c & 0xFE) == 0xC0) {
        return false;  // Overlong encoding of ASCII character.
      }
    } else if ((c & 0xF0) == 0xE0) {
      sequence_length = 3;
    } else if ((c & 0xF8) == 0xF0) {
      sequence_length = 4;
      if (c > 0xF4) {
        return false;  // Outside valid Unicode range.
      }
    } else {
      return false;  // Invalid first byte.
    }

    if (i + sequence_length > value.size()) {
      return false;  // Truncated sequence.
    }

    for (size_t j = 1; j < sequence_length; ++j) {
      unsigned char continuation = static_cast<unsigned char>(value[i + j]);
      if ((continuation & 0xC0) != 0x80) {
        return false;  // Invalid continuation byte.
      }
    }
    i += sequence_length;
  }
  return true;
}

std::string sanitize_text_sensor_value(const std::string &value) {
  std::string sanitized;
  sanitized.reserve(value.size());
  bool has_control_characters = false;

  for (unsigned char c : value) {
    if (c == '\r' || c == '\n' || c == '\0') {
      continue;
    }
    if (c < 0x20 || c == 0x7F) {
      has_control_characters = true;
    }
    sanitized.push_back(static_cast<char>(c));
  }

  if (!is_valid_utf8(sanitized) || has_control_characters) {
    return format_printable_string(sanitized);
  }

  return sanitized;
}

std::string canonicalize_machine_data_path(const std::string &path) {
  std::string canonical = path;
  std::transform(canonical.begin(), canonical.end(), canonical.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return canonical;
}

std::string build_machine_data_field_sensor_name(const std::string &prefix, const std::string &path) {
  std::string readable;
  readable.reserve(path.size());
  bool last_was_space = true;

  auto append_space = [&]() {
    if (!last_was_space) {
      readable.push_back(' ');
      last_was_space = true;
    }
  };

  for (char c : path) {
    switch (c) {
      case '/':
      case '_':
      case '-':
      case '@':
      case '[':
      case ']':
      case ':':
        append_space();
        break;
      default:
        if (std::isspace(static_cast<unsigned char>(c))) {
          append_space();
        } else {
          last_was_space = false;
          readable.push_back(c);
        }
        break;
    }
  }

  if (!readable.empty() && readable.back() == ' ') {
    readable.pop_back();
  }

  bool new_word = true;
  for (char &c : readable) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      new_word = true;
      continue;
    }
    if (new_word) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      new_word = false;
    } else {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
  }

  if (readable.empty()) {
    readable = "Field";
  }

  if (prefix.empty()) {
    return readable;
  }
  return prefix + " " + readable;
}

void collect_machine_data_fields(const MachineDataNode &node, const std::string &path, bool include_attributes,
                                std::vector<std::pair<std::string, std::string>> &fields) {
  if (node.name == "#text") {
    if (!path.empty() && !node.text.empty()) {
      fields.emplace_back(path, node.text);
    }
    return;
  }

  std::string current_path = path.empty() ? node.name : path + "/" + node.name;

  if (include_attributes) {
    for (const auto &attribute : node.attributes) {
      if (!attribute.first.empty()) {
        fields.emplace_back(current_path + "/@" + attribute.first, attribute.second);
      }
    }
  }

  if (!node.text.empty()) {
    fields.emplace_back(current_path, node.text);
  }

  std::unordered_map<std::string, size_t> child_counts;
  for (const auto &child : node.children) {
    if (child.name == "#text") {
      if (!child.text.empty()) {
        fields.emplace_back(current_path, child.text);
      }
      continue;
    }

    size_t &index = child_counts[child.name];
    std::string child_path = current_path.empty() ? child.name : current_path + "/" + child.name;
    if (index > 0) {
      child_path.append("[");
      child_path.append(std::to_string(index + 1));
      child_path.push_back(']');
    }
    ++index;
    collect_machine_data_fields(child, child_path, include_attributes, fields);
  }
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
  if (MACHINE_DATA_COMMANDS.empty()) {
    return;
  }

  uint32_t now = esphome::millis();

  if (this->machine_data_request_pending_) {
    if (this->machine_data_command_index_ >= MACHINE_DATA_COMMANDS.size()) {
      this->machine_data_request_pending_ = false;
    } else {
      const auto &definition = MACHINE_DATA_COMMANDS[this->machine_data_command_index_];
      auto response = this->coffee_maker_->connection->write_xml_with_response(
          definition.command, std::chrono::milliseconds{MACHINE_DATA_REQUEST_TIMEOUT_MS});
      if (response != nullptr) {
        if (this->machine_data_responses_.size() != MACHINE_DATA_COMMANDS.size()) {
          this->machine_data_responses_.assign(MACHINE_DATA_COMMANDS.size(), std::string{});
        }
        this->machine_data_responses_[this->machine_data_command_index_] = *response;
        ++this->machine_data_command_index_;
        this->machine_data_request_pending_ = false;
        this->machine_data_request_start_ = 0;
      } else {
        if (time_reached(now, this->machine_data_request_start_ + MACHINE_DATA_REQUEST_TIMEOUT_MS)) {
          ESP_LOGW(TAG, "Timeout while waiting for machine data response to command '%s'.",
                   definition.command);
          this->machine_data_request_pending_ = false;
          this->machine_data_request_start_ = 0;
          this->machine_data_command_index_ = 0;
          this->machine_data_responses_.clear();
          this->machine_data_query_next_ = now + MACHINE_DATA_QUERY_INTERVAL_MS;
        }
        return;
      }
    }
  }

  if (this->machine_data_command_index_ == 0) {
    if (this->machine_data_query_next_ != 0 &&
        !time_reached(now, this->machine_data_query_next_)) {
      return;
    }
    if (this->machine_data_responses_.size() != MACHINE_DATA_COMMANDS.size()) {
      this->machine_data_responses_.assign(MACHINE_DATA_COMMANDS.size(), std::string{});
    }
  }

  while (this->machine_data_command_index_ < MACHINE_DATA_COMMANDS.size()) {
    const auto &definition = MACHINE_DATA_COMMANDS[this->machine_data_command_index_];
    ESP_LOGV(TAG, "Requesting machine data command '%s' (%s/%s).", definition.command,
             definition.section, definition.element);
    this->machine_data_request_start_ = esphome::millis();
    auto response = this->coffee_maker_->connection->write_xml_with_response(
        definition.command, std::chrono::milliseconds{MACHINE_DATA_REQUEST_TIMEOUT_MS});
    if (response == nullptr) {
      this->machine_data_request_pending_ = true;
      return;
    }
    this->machine_data_responses_[this->machine_data_command_index_] = *response;
    ++this->machine_data_command_index_;
  }

  if (this->machine_data_command_index_ >= MACHINE_DATA_COMMANDS.size()) {
    std::string payload = build_machine_data_payload(this->machine_data_responses_);
    this->publish_machine_data_(payload);
    this->machine_data_responses_.clear();
    this->machine_data_request_pending_ = false;
    this->machine_data_request_start_ = 0;
    this->machine_data_command_index_ = 0;
    this->machine_data_query_next_ = esphome::millis() + MACHINE_DATA_QUERY_INTERVAL_MS;
  }
}

void JuraComponent::publish_machine_data_(const std::string &response) {
  auto parsed = MachineDataParser::parse(response);
  if (!parsed.has_value()) {
    ESP_LOGW(TAG, "Failed to parse machine data XML response.");
  }

  if (parsed.has_value()) {
    const auto &root = parsed.value();
    if (this->machine_data_statistic_sensor_ != nullptr) {
      auto formatted =
          format_machine_data_section(root.find_child_case_insensitive("STATISTIC"));
      this->machine_data_statistic_sensor_->publish_state(sanitize_text_sensor_value(formatted));
    }
    if (this->machine_data_errors_sensor_ != nullptr) {
      auto formatted =
          format_machine_data_section(root.find_child_case_insensitive("ALERTS"));
      this->machine_data_errors_sensor_->publish_state(sanitize_text_sensor_value(formatted));
    }
    if (this->machine_data_status_sensor_ != nullptr) {
      auto formatted = format_machine_data_section(
          root.find_child_case_insensitive("PROGRESS_STATE_INTAKE"));
      this->machine_data_status_sensor_->publish_state(sanitize_text_sensor_value(formatted));
    }
    if (this->machine_data_processes_sensor_ != nullptr) {
      auto formatted =
          format_machine_data_section(root.find_child_case_insensitive("PROCESSES"));
      this->machine_data_processes_sensor_->publish_state(sanitize_text_sensor_value(formatted));
    }
    if (this->machine_data_fields_enabled_) {
      this->publish_machine_data_fields_(root);
    }
  } else {
    if (this->machine_data_statistic_sensor_ != nullptr) {
      this->machine_data_statistic_sensor_->publish_state("");
    }
    if (this->machine_data_errors_sensor_ != nullptr) {
      this->machine_data_errors_sensor_->publish_state("");
    }
    if (this->machine_data_status_sensor_ != nullptr) {
      this->machine_data_status_sensor_->publish_state("");
    }
    if (this->machine_data_processes_sensor_ != nullptr) {
      this->machine_data_processes_sensor_->publish_state("");
    }
    if (this->machine_data_fields_enabled_) {
      this->clear_machine_data_field_sensors_();
    }
  }

  std::string sanitized = sanitize_text_sensor_value(response);
  ESP_LOGD(TAG, "Machine data response: %s", sanitized.c_str());
  if (this->machine_data_sensor_ != nullptr) {
    this->machine_data_sensor_->publish_state(sanitized);
  }
}

void JuraComponent::enable_machine_data_field_sensors(const std::string &prefix, bool include_attributes) {
  this->machine_data_fields_enabled_ = true;
  this->machine_data_field_prefix_ = prefix;
  this->machine_data_field_include_attributes_ = include_attributes;
}

text_sensor::TextSensor *JuraComponent::get_or_create_machine_data_field_sensor_(const std::string &key,
                                                                                const std::string &path) {
  auto it = this->machine_data_field_sensors_.find(key);
  if (it != this->machine_data_field_sensors_.end()) {
    return it->second;
  }

  auto sensor = std::make_unique<text_sensor::TextSensor>();
  auto sensor_name =
      build_machine_data_field_sensor_name(this->machine_data_field_prefix_, path);
  sensor->set_name(sensor_name.c_str());
  auto *raw_sensor = sensor.get();
  App.register_text_sensor(raw_sensor);
  this->machine_data_field_sensor_storage_.push_back(std::move(sensor));
  this->machine_data_field_sensors_.emplace(key, raw_sensor);
  return raw_sensor;
}

void JuraComponent::publish_machine_data_fields_(const MachineDataNode &root) {
  std::vector<std::pair<std::string, std::string>> fields;
  collect_machine_data_fields(root, "", this->machine_data_field_include_attributes_, fields);

  std::unordered_set<std::string> seen;
  seen.reserve(fields.size());

  for (auto &field : fields) {
    if (field.first.empty()) {
      continue;
    }
    std::string key = canonicalize_machine_data_path(field.first);
    auto *sensor = this->get_or_create_machine_data_field_sensor_(key, field.first);
    if (sensor != nullptr) {
      sensor->publish_state(sanitize_text_sensor_value(field.second));
      seen.insert(std::move(key));
    }
  }

  for (auto &entry : this->machine_data_field_sensors_) {
    if (seen.find(entry.first) == seen.end() && entry.second != nullptr) {
      entry.second->publish_state("");
    }
  }
}

void JuraComponent::clear_machine_data_field_sensors_() {
  for (auto &entry : this->machine_data_field_sensors_) {
    if (entry.second != nullptr) {
      entry.second->publish_state("");
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

