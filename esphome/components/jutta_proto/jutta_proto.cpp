#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/core/time.h"
#include "machine_data_parser.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_DATA_REQUEST_TIMEOUT_MS = 4500;
constexpr uint32_t MACHINE_DATA_PRE_REQUEST_GAP_MS = 30;
constexpr float MAINTENANCE_RAW_PERCENT_DIVISOR = 655.35f;

template<typename Sensor>
auto try_set_unique_id(Sensor *sensor, const std::string &unique_id)
    -> decltype(sensor->set_unique_id(unique_id), void()) {
  sensor->set_unique_id(unique_id);
}

inline void try_set_unique_id(...) {}

struct MachineDataCommandDefinition {
  const char *command;
  const char *section;
  const char *element;
};

constexpr std::array<MachineDataCommandDefinition, 3> MACHINE_DATA_COMMANDS = {{{"@TR:32", "STATISTIC", "PRODUCTCOUNTER"},
                                                                              {"@TG:43", "STATISTIC", "MAINTENANCECOUNTER"},
                                                                              {"@TG:C0", "STATISTIC", "MAINTENANCEPERCENT"}}};

std::optional<size_t> expected_machine_data_payload_size(const std::string &command) {
  if (command == "@TR:32") {
    return 21;
  }
  if (command == "@TG:43" || command == "@TG:C0") {
    return 13;
  }
  return std::nullopt;
}

std::string to_upper_ascii(const std::string &value) {
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return result;
}

bool equals_ignore_case(const std::string &lhs, const std::string &rhs) {
  return to_upper_ascii(lhs) == to_upper_ascii(rhs);
}

std::string trim_ascii_whitespace(const std::string &value) {
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
    ++start;
  }
  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    --end;
  }
  return value.substr(start, end - start);
}

int hex_digit_value(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + (c - 'a');
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + (c - 'A');
  }
  return -1;
}

bool decode_hex_string(const std::string &input, std::string &output) {
  output.clear();
  std::string sanitized;
  sanitized.reserve(input.size());
  for (unsigned char c : input) {
    if (std::isspace(c) != 0) {
      continue;
    }
    sanitized.push_back(static_cast<char>(c));
  }
  if (sanitized.size() % 2 != 0) {
    return false;
  }
  output.reserve(sanitized.size() / 2);
  for (size_t i = 0; i < sanitized.size(); i += 2) {
    int high = hex_digit_value(sanitized[i]);
    int low = hex_digit_value(sanitized[i + 1]);
    if (high < 0 || low < 0) {
      return false;
    }
    output.push_back(static_cast<char>((high << 4) | low));
  }
  return true;
}

const MachineDataNode *find_descendant_case_insensitive(const MachineDataNode &node, const std::string &name) {
  if (equals_ignore_case(node.name, name)) {
    return &node;
  }
  for (const auto &child : node.children) {
    if (child.name == "#text") {
      continue;
    }
    if (auto *match = find_descendant_case_insensitive(child, name)) {
      return match;
    }
  }
  return nullptr;
}

std::optional<std::string> extract_machine_data_payload(const MachineDataNode &machine_data_root,
                                                        const MachineDataCommandDefinition &definition,
                                                        std::string *encoding_out, std::string *error_out) {
  if (error_out != nullptr) {
    *error_out = "";
  }

  const MachineDataNode *section = machine_data_root.find_child_case_insensitive(definition.section);
  if (section == nullptr) {
    if (error_out != nullptr) {
      *error_out = "section missing";
    }
    return std::nullopt;
  }

  const MachineDataNode *element = section->find_child_case_insensitive(definition.element);
  if (element == nullptr) {
    if (error_out != nullptr) {
      *error_out = "element missing";
    }
    return std::nullopt;
  }

  bool command_mismatch = false;
  if (auto command_attr = element->get_attribute_case_insensitive("command"); command_attr.has_value()) {
    if (!equals_ignore_case(command_attr.value(), definition.command)) {
      command_mismatch = true;
    }
  }

  const MachineDataNode *payload_node = element;
  auto encoding_attr = payload_node->get_attribute_case_insensitive("encoding");
  if (!encoding_attr.has_value()) {
    for (const auto &child : element->children) {
      if (child.name == "#text") {
        continue;
      }
      auto child_encoding = child.get_attribute_case_insensitive("encoding");
      if (child_encoding.has_value()) {
        payload_node = &child;
        encoding_attr = child_encoding;
        break;
      }
    }
  }

  if (!encoding_attr.has_value()) {
    if (error_out != nullptr) {
      *error_out = "encoding attribute missing";
    }
    return std::nullopt;
  }

  std::string encoding = encoding_attr.value();
  if (encoding_out != nullptr) {
    *encoding_out = encoding;
  }

  std::string text = trim_ascii_whitespace(payload_node->collect_text_content());
  if (equals_ignore_case(encoding, "hex")) {
    std::string decoded;
    if (!decode_hex_string(text, decoded)) {
      if (error_out != nullptr) {
        *error_out = "invalid hex payload";
      }
      return std::nullopt;
    }
    if (command_mismatch && error_out != nullptr) {
      *error_out = "command attribute mismatch";
    }
    return decoded;
  }
  if (equals_ignore_case(encoding, "text")) {
    if (command_mismatch && error_out != nullptr) {
      *error_out = "command attribute mismatch";
    }
    return text;
  }

  if (error_out != nullptr) {
    *error_out = "unsupported encoding";
  }
  return std::nullopt;
}

std::string slugify(const std::string &value) {
  std::string result;
  result.reserve(value.size());
  bool last_was_separator = true;
  for (unsigned char c : value) {
    if (std::isalnum(c) != 0) {
      result.push_back(static_cast<char>(std::tolower(c)));
      last_was_separator = false;
    } else {
      if (!last_was_separator && !result.empty()) {
        result.push_back('_');
        last_was_separator = true;
      }
    }
  }
  while (!result.empty() && result.back() == '_') {
    result.pop_back();
  }
  if (result.empty()) {
    result = "field";
  }
  return result;
}

std::string prettify_identifier(const std::string &identifier) {
  if (identifier == "PRODUCTCOUNTER") {
    return "Product Counter";
  }
  if (identifier == "MAINTENANCECOUNTER") {
    return "Maintenance Counter";
  }
  if (identifier == "MAINTENANCEPERCENT") {
    return "Maintenance Percent";
  }
  if (identifier == "STATISTIC") {
    return "Statistic";
  }
  std::string lowered;
  lowered.reserve(identifier.size());
  for (unsigned char c : identifier) {
    if (c == '_' || c == '-' || c == '.' || c == ' ') {
      lowered.push_back(' ');
    } else {
      lowered.push_back(static_cast<char>(std::tolower(c)));
    }
  }
  std::string result;
  result.reserve(lowered.size());
  bool capitalize_next = true;
  for (unsigned char c : lowered) {
    if (std::isspace(c) != 0) {
      if (!result.empty() && result.back() != ' ') {
        result.push_back(' ');
      }
      capitalize_next = true;
      continue;
    }
    if (capitalize_next) {
      result.push_back(static_cast<char>(std::toupper(c)));
      capitalize_next = false;
    } else {
      result.push_back(static_cast<char>(c));
    }
  }
  if (!result.empty() && result.back() == ' ') {
    result.pop_back();
  }
  return result;
}

bool contains_alnum(const std::string &value) {
  for (unsigned char c : value) {
    if (std::isalnum(c) != 0) {
      return true;
    }
  }
  return false;
}

std::string normalize_machine_data_field_key(const std::string &key) {
  if (key.empty()) {
    return "";
  }

  std::string normalized;
  size_t start = 0;
  while (start <= key.size()) {
    size_t end = key.find('.', start);
    std::string segment = key.substr(start, end - start);
    if (contains_alnum(segment)) {
      std::string slug = slugify(segment);
      if (!slug.empty()) {
        if (!normalized.empty()) {
          normalized.push_back('.');
        }
        normalized.append(slug);
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }

  if (normalized.empty() && contains_alnum(key)) {
    normalized = slugify(key);
  }

  return normalized;
}

struct MachineDataFieldEntry {
  std::string name;
  std::string text_value;
  std::optional<float> numeric_value;
  std::string unit;
};

using MachineDataFieldList = std::vector<MachineDataFieldEntry>;

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

constexpr std::array<const char *, 3> MAINTENANCE_PERCENT_NAMES = {{"Cleaning", "FilterChange", "Decalc"}};

float decode_percent_fixed_point(uint32_t raw_value) {
  // Percentages in the maintenance payload are encoded in the upper 16 bits
  // using a Q8.8 fixed-point representation. The remaining 16 bits contain an
  // auxiliary counter that we expose separately as the "Raw" value. The
  // firmware dump in jura_joe_xml_bundle_final shows values that exceed 0xFFFF,
  // therefore we cannot treat the 32-bit word as a full Q16.16 number.
  uint16_t encoded_percent = static_cast<uint16_t>(raw_value >> 16);
  constexpr float SCALE = 256.0f;  // Q8.8 scaling factor for the encoded part.
  return static_cast<float>(encoded_percent) / SCALE;
}

uint16_t decode_percent_auxiliary(uint32_t raw_value) {
  return static_cast<uint16_t>(raw_value & 0xFFFF);
}

std::string format_percent_value(float value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1) << value;
  return stream.str();
}

uint16_t read_u16_be(const uint8_t *ptr) {
  return static_cast<uint16_t>((static_cast<uint16_t>(ptr[0]) << 8) |
                               static_cast<uint16_t>(ptr[1]));
}

uint32_t read_u32_be(const uint8_t *ptr) {
  return (static_cast<uint32_t>(ptr[0]) << 24) | (static_cast<uint32_t>(ptr[1]) << 16) |
         (static_cast<uint32_t>(ptr[2]) << 8) | static_cast<uint32_t>(ptr[3]);
}

std::optional<std::string> format_product_counter_payload(const std::string &payload) {
  if (payload.size() != 21) {
    return std::nullopt;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  uint8_t header = data[0];
  size_t value_offset = 1;

  std::ostringstream stream;
  stream << "header=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<int>(header) << std::dec;

  for (size_t i = 0; i < PRODUCT_COUNTER_DEFINITIONS.size(); ++i) {
    uint16_t value = read_u16_be(&data[value_offset + i * 2]);
    stream << "; " << PRODUCT_COUNTER_DEFINITIONS[i].name;
    if (PRODUCT_COUNTER_DEFINITIONS[i].code != nullptr) {
      stream << '[' << PRODUCT_COUNTER_DEFINITIONS[i].code << ']';
    }
    stream << '=' << value;
  }

  return stream.str();
}

MachineDataFieldList parse_product_counter_fields(const std::string &payload) {
  MachineDataFieldList fields;
  if (payload.size() != 21) {
    return fields;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  std::ostringstream header_stream;
  header_stream << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
                << static_cast<int>(data[0]);
  fields.push_back({"Header", header_stream.str(), std::nullopt, ""});

  for (size_t i = 0; i < PRODUCT_COUNTER_DEFINITIONS.size(); ++i) {
    uint16_t value = read_u16_be(&data[1 + i * 2]);
    fields.push_back({PRODUCT_COUNTER_DEFINITIONS[i].name, std::to_string(value),
                      static_cast<float>(value), ""});
  }

  return fields;
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
    uint16_t value = read_u16_be(&data[1 + i * 2]);
    stream << "; " << MAINTENANCE_COUNTER_NAMES[i] << '=' << value;
  }

  return stream.str();
}

MachineDataFieldList parse_maintenance_counter_fields(const std::string &payload) {
  MachineDataFieldList fields;
  if (payload.size() != 13) {
    return fields;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  uint8_t header = data[0];

  std::ostringstream header_stream;
  header_stream << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
                << static_cast<int>(header);
  fields.push_back({"Header", header_stream.str(), std::nullopt, ""});

  for (size_t i = 0; i < MAINTENANCE_COUNTER_NAMES.size(); ++i) {
    uint16_t value = read_u16_be(&data[1 + i * 2]);
    fields.push_back({MAINTENANCE_COUNTER_NAMES[i], std::to_string(value), static_cast<float>(value),
                      ""});
  }

  return fields;
}

std::optional<std::string> format_maintenance_percent_payload(const std::string &payload) {
  if (payload.size() != 13) {
    return std::nullopt;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());
  uint8_t header = data[0];

  std::ostringstream stream;
  stream << "header=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<int>(header) << std::dec;

  size_t value_count = (payload.size() - 1) / 4;
  for (size_t i = 0; i < value_count; ++i) {
    size_t index = 1 + i * 4;
    if (index + 3 >= payload.size()) {
      break;
    }
    uint32_t raw_value = read_u32_be(&data[index]);
    uint16_t encoded_percent = static_cast<uint16_t>(raw_value >> 16);
    uint16_t auxiliary_value = decode_percent_auxiliary(raw_value);
    float percent = decode_percent_fixed_point(raw_value);
    float raw_percent = static_cast<float>(auxiliary_value) / MAINTENANCE_RAW_PERCENT_DIVISOR;
    const char *name = i < MAINTENANCE_PERCENT_NAMES.size() ? MAINTENANCE_PERCENT_NAMES[i] : nullptr;
    stream << "; ";
    if (name != nullptr) {
      stream << name << " Percent=" << std::fixed << std::setprecision(1) << percent;
      stream << " (Encoded=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
             << encoded_percent << std::dec << "=" << encoded_percent;
      stream << "; Raw=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
             << auxiliary_value << std::dec << "=" << auxiliary_value << "; RawPercent="
             << std::fixed << std::setprecision(1) << raw_percent << '%'
             << ')';
      stream << std::setfill(' ');
    } else {
      stream << "Value" << (i + 1) << " Percent=" << std::fixed << std::setprecision(1) << percent;
      stream << " (Encoded=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
             << encoded_percent << std::dec << "=" << encoded_percent;
      stream << "; Raw=0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
             << auxiliary_value << std::dec << "=" << auxiliary_value << "; RawPercent="
             << std::fixed << std::setprecision(1) << raw_percent << '%'
             << ')';
      stream << std::setfill(' ');
    }
  }

  return stream.str();
}

MachineDataFieldList parse_maintenance_percent_fields(const std::string &payload) {
  MachineDataFieldList fields;
  if (payload.size() != 13) {
    return fields;
  }

  const auto *data = reinterpret_cast<const uint8_t *>(payload.data());

  std::ostringstream header_stream;
  header_stream << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
                << static_cast<int>(data[0]);
  fields.push_back({"Header", header_stream.str(), std::nullopt, ""});

  size_t value_count = (payload.size() - 1) / 4;
  for (size_t i = 0; i < value_count; ++i) {
    size_t index = 1 + i * 4;
    if (index + 3 >= payload.size()) {
      break;
    }
    uint32_t raw_value = read_u32_be(&data[index]);
    uint16_t auxiliary_value = decode_percent_auxiliary(raw_value);
    float percent = static_cast<float>(auxiliary_value) / MAINTENANCE_RAW_PERCENT_DIVISOR;
    std::string percent_text = format_percent_value(percent);
    const char *name = i < MAINTENANCE_PERCENT_NAMES.size() ? MAINTENANCE_PERCENT_NAMES[i] : nullptr;
    std::string field_name;
    if (name != nullptr) {
      field_name = std::string(name) + " Percent";
    } else {
      std::ostringstream fallback_name;
      fallback_name << "Value" << (i + 1) << " Percent";
      field_name = fallback_name.str();
    }
    fields.push_back({field_name, percent_text, percent, "%"});

    std::ostringstream raw_stream;
    raw_stream << auxiliary_value << " (" << percent_text << "%)";
    std::string raw_field_name;
    if (name != nullptr) {
      raw_field_name = std::string(name) + " Raw";
    } else {
      std::ostringstream fallback_name;
      fallback_name << "Value" << (i + 1) << " Raw";
      raw_field_name = fallback_name.str();
    }
    fields.push_back({raw_field_name, raw_stream.str(), std::nullopt, ""});
  }

  return fields;
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

MachineDataFieldList parse_machine_data_fields(const std::string &command, const std::string &payload) {
  if (command == std::string("@TR:32")) {
    return parse_product_counter_fields(payload);
  }
  if (command == std::string("@TG:43")) {
    return parse_maintenance_counter_fields(payload);
  }
  if (command == std::string("@TG:C0")) {
    return parse_maintenance_percent_fields(payload);
  }
  return {};
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

using MachineDataFieldMap = JuraComponent::MachineDataFieldMap;
using MachineDataFieldValue = JuraComponent::MachineDataFieldValue;

std::string build_machine_data_payload(const std::vector<std::string> &responses,
                                       MachineDataFieldMap *field_map = nullptr) {
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
    std::optional<std::string> parsed_text;
    if (!response.empty()) {
      parsed_text = parse_machine_data_response(definition.command, response);
    }
    stream << '<' << definition.element << " command=\"" << definition.command << "\"";
    if (!response.empty()) {
      stream << " raw_hex=\"" << to_upper_hex(response) << "\"";
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

    if (field_map != nullptr) {
      std::vector<std::string> base_labels;
      base_labels.push_back(prettify_identifier(definition.section));
      base_labels.push_back(prettify_identifier(definition.element));
      std::string key_prefix = slugify(definition.section);
      std::string element_slug = slugify(definition.element);
      if (!element_slug.empty()) {
        if (!key_prefix.empty()) {
          key_prefix.push_back('.');
        }
        key_prefix.append(element_slug);
      }
      if (!response.empty()) {
        MachineDataFieldList fields = parse_machine_data_fields(definition.command, response);
        for (const auto &field : fields) {
          std::vector<std::string> labels = base_labels;
          labels.push_back(field.name);
          std::string value_slug = slugify(field.name);
          std::string field_key = key_prefix.empty() ? value_slug : key_prefix + '.' + value_slug;
          MachineDataFieldValue value;
          value.labels = std::move(labels);
          value.text_value = field.text_value;
          value.numeric_value = field.numeric_value;
          value.unit = field.unit;
          (*field_map)[field_key] = std::move(value);
        }

        std::vector<std::string> raw_labels = base_labels;
        raw_labels.push_back("Raw Hex");
        std::string raw_key = key_prefix.empty() ? std::string("raw_hex") : key_prefix + ".raw_hex";
        MachineDataFieldValue raw_value;
        raw_value.labels = std::move(raw_labels);
        raw_value.text_value = to_upper_hex(response);
        (*field_map)[raw_key] = std::move(raw_value);

        if (parsed_text.has_value()) {
          std::vector<std::string> text_labels = base_labels;
          text_labels.push_back("Text");
          std::string text_key = key_prefix.empty() ? std::string("text") : key_prefix + ".text";
          MachineDataFieldValue text_value;
          text_value.labels = std::move(text_labels);
          text_value.text_value = parsed_text.value();
          (*field_map)[text_key] = std::move(text_value);
        } else if (is_safe_xml_text(response)) {
          std::vector<std::string> text_labels = base_labels;
          text_labels.push_back("Text");
          std::string text_key = key_prefix.empty() ? std::string("text") : key_prefix + ".text";
          MachineDataFieldValue text_value;
          text_value.labels = std::move(text_labels);
          text_value.text_value = response;
          (*field_map)[text_key] = std::move(text_value);
        }
      }
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

}  // namespace

void JuraComponent::publish_machine_data_fields_(const MachineDataFieldMap &fields) {
  if (fields.empty()) {
    ESP_LOGV(TAG, "No machine data field updates to publish.");
    return;
  }

  for (const auto &entry : fields) {
    const auto &value = entry.second;
    auto *text_sensor = this->find_machine_data_field_sensor_(entry.first);
    if (text_sensor != nullptr) {
      if (this->machine_data_auto_fields_ && this->machine_data_field_user_keys_.count(entry.first) == 0) {
        std::string name = this->make_machine_data_field_name_(value.labels);
        text_sensor->set_name(name.c_str());
      }
      text_sensor->publish_state(sanitize_text_sensor_value(value.text_value));
    }

    auto *numeric_sensor = this->find_machine_data_numeric_sensor_(entry.first);
    if (numeric_sensor != nullptr) {
      if (this->machine_data_auto_fields_ &&
          this->machine_data_numeric_field_user_keys_.count(entry.first) == 0) {
        std::string name = this->make_machine_data_field_name_(value.labels);
        numeric_sensor->set_name(name.c_str());
        if (!value.unit.empty()) {
          numeric_sensor->set_unit_of_measurement(value.unit.c_str());
        } else {
          numeric_sensor->set_unit_of_measurement(nullptr);
        }
      }
      if (value.numeric_value.has_value()) {
        numeric_sensor->publish_state(*value.numeric_value);
      }
    }
  }
}

text_sensor::TextSensor *JuraComponent::find_machine_data_field_sensor_(const std::string &key) {
  auto it = this->machine_data_field_sensors_.find(key);
  if (it == this->machine_data_field_sensors_.end()) {
    return nullptr;
  }
  return it->second;
}

void JuraComponent::add_machine_data_field_sensor(const std::string &key,
                                                  text_sensor::TextSensor *sensor) {
  if (sensor == nullptr) {
    return;
  }
  std::string normalized = normalize_machine_data_field_key(key);
  if (normalized.empty()) {
    ESP_LOGW(TAG, "Ignoring machine data field mapping with empty key '%s'.", key.c_str());
    return;
  }
  this->machine_data_field_sensors_[normalized] = sensor;
  this->machine_data_field_user_keys_.insert(normalized);
}

sensor::Sensor *JuraComponent::find_machine_data_numeric_sensor_(const std::string &key) {
  auto it = this->machine_data_numeric_field_sensors_.find(key);
  if (it == this->machine_data_numeric_field_sensors_.end()) {
    return nullptr;
  }
  return it->second;
}

void JuraComponent::add_machine_data_field_sensor(const std::string &key, sensor::Sensor *sensor) {
  if (sensor == nullptr) {
    return;
  }
  std::string normalized = normalize_machine_data_field_key(key);
  if (normalized.empty()) {
    ESP_LOGW(TAG, "Ignoring machine data field mapping with empty key '%s'.", key.c_str());
    return;
  }
  this->machine_data_numeric_field_sensors_[normalized] = sensor;
  this->machine_data_numeric_field_user_keys_.insert(normalized);
}

void JuraComponent::register_machine_data_text_sensor_(const std::string &key,
                                                       const std::vector<std::string> &labels) {
  if (!this->machine_data_auto_fields_ || key.empty()) {
    return;
  }

  auto *existing = this->find_machine_data_field_sensor_(key);
  std::string name = this->make_machine_data_field_name_(labels);
  if (existing != nullptr) {
    if (this->machine_data_field_user_keys_.count(key) == 0) {
      existing->set_name(name.c_str());
      std::string unique_id = this->make_machine_data_field_unique_id_(key);
      if (!unique_id.empty()) {

        try_set_unique_id(existing, unique_id);

      }
    }
    return;
  }

  auto *sensor = new text_sensor::TextSensor();
  sensor->set_name(name.c_str());
  std::string unique_id = this->make_machine_data_field_unique_id_(key);
  if (!unique_id.empty()) {

    try_set_unique_id(sensor, unique_id);

  }
  sensor->set_internal(false);
  App.register_text_sensor(sensor);
  this->machine_data_field_sensors_[key] = sensor;
}

void JuraComponent::register_machine_data_numeric_sensor_(const std::string &key,
                                                          const std::vector<std::string> &labels,
                                                          const std::string &unit) {
  if (!this->machine_data_auto_fields_ || key.empty()) {
    return;
  }

  auto *existing = this->find_machine_data_numeric_sensor_(key);
  std::string name = this->make_machine_data_field_name_(labels);
  if (existing != nullptr) {
    if (this->machine_data_numeric_field_user_keys_.count(key) == 0) {
      existing->set_name(name.c_str());
      if (!unit.empty()) {
        existing->set_unit_of_measurement(unit.c_str());
      } else {
        existing->set_unit_of_measurement(nullptr);
      }
      std::string unique_id = this->make_machine_data_field_unique_id_(key);
      if (!unique_id.empty()) {
        try_set_unique_id(existing, unique_id);
      }
    }
    return;
  }

  auto *sensor = new sensor::Sensor();
  sensor->set_name(name.c_str());
  if (!unit.empty()) {
    sensor->set_unit_of_measurement(unit.c_str());
  }
  std::string unique_id = this->make_machine_data_field_unique_id_(key);
  if (!unique_id.empty()) {
    try_set_unique_id(sensor, unique_id);
  }
  sensor->set_internal(false);
  App.register_sensor(sensor);
  this->machine_data_numeric_field_sensors_[key] = sensor;
}

void JuraComponent::initialize_machine_data_field_sensors_() {
  if (!this->machine_data_auto_fields_) {
    return;
  }

  for (const auto &definition : MACHINE_DATA_COMMANDS) {
    std::vector<std::string> base_labels;
    base_labels.push_back(prettify_identifier(definition.section));
    base_labels.push_back(prettify_identifier(definition.element));

    std::string key_prefix = slugify(definition.section);
    std::string element_slug = slugify(definition.element);
    if (!element_slug.empty()) {
      if (!key_prefix.empty()) {
        key_prefix.push_back('.');
      }
      key_prefix.append(element_slug);
    }

    auto register_text_field = [&](const std::string &field_name, const std::string &slug) {
      if (field_name.empty() || slug.empty()) {
        return;
      }
      std::vector<std::string> labels = base_labels;
      labels.push_back(field_name);
      std::string field_key;
      if (key_prefix.empty()) {
        field_key = slug;
      } else {
        field_key = key_prefix + '.' + slug;
      }
      this->register_machine_data_text_sensor_(field_key, labels);
    };

    auto register_numeric_field = [&](const std::string &field_name, const std::string &slug,
                                      const std::string &unit) {
      if (field_name.empty() || slug.empty()) {
        return;
      }
      std::vector<std::string> labels = base_labels;
      labels.push_back(field_name);
      std::string field_key;
      if (key_prefix.empty()) {
        field_key = slug;
      } else {
        field_key = key_prefix + '.' + slug;
      }
      this->register_machine_data_numeric_sensor_(field_key, labels, unit);
    };

    if (definition.command == std::string("@TR:32")) {
      register_text_field("Header", slugify("Header"));
      for (const auto &entry : PRODUCT_COUNTER_DEFINITIONS) {
        register_numeric_field(entry.name, slugify(entry.name), "");
      }
    } else if (definition.command == std::string("@TG:43")) {
      register_text_field("Header", slugify("Header"));
      for (const auto *name : MAINTENANCE_COUNTER_NAMES) {
        register_numeric_field(name, slugify(name), "");
      }
    } else if (definition.command == std::string("@TG:C0")) {
      register_text_field("Header", slugify("Header"));
      for (const auto *name : MAINTENANCE_PERCENT_NAMES) {
        register_numeric_field(std::string(name) + " Percent", slugify(std::string(name) + " Percent"), "%");
        register_numeric_field(std::string(name) + " Raw", slugify(std::string(name) + " Raw"), "");
      }
    }

    register_text_field("Raw Hex", "raw_hex");
    register_text_field("Text", "text");
  }
}

std::string JuraComponent::make_machine_data_field_name_(const std::vector<std::string> &labels) const {
  std::string name = this->machine_data_field_prefix_;
  for (const auto &label : labels) {
    if (label.empty()) {
      continue;
    }
    if (!name.empty()) {
      name.append(" ");
    }
    name.append(label);
  }
  if (name.empty()) {
    name = "Machine Data";
  }
  return name;
}

std::string JuraComponent::make_machine_data_field_unique_id_(const std::string &key) const {
  std::string node_slug = slugify(App.get_name());
  std::string device_slug = this->device_type_.empty() ? std::string("jura") : slugify(this->device_type_);

  std::string key_slug = key;
  std::replace(key_slug.begin(), key_slug.end(), '.', '_');

  if (node_slug.empty()) {
    node_slug = "esphome";
  }

  std::string unique_id = node_slug;
  unique_id.append("_jutta_");
  unique_id.append(device_slug);
  unique_id.append("_");
  unique_id.append(key_slug);
  return unique_id;
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

  if (this->machine_data_auto_fields_) {
    this->initialize_machine_data_field_sensors_();
  }

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

bool JuraComponent::has_machine_data_subscribers_() const {
  return this->machine_data_sensor_ != nullptr || this->machine_data_statistic_sensor_ != nullptr ||
         this->machine_data_errors_sensor_ != nullptr ||
         this->machine_data_status_sensor_ != nullptr ||
         this->machine_data_processes_sensor_ != nullptr || this->machine_data_auto_fields_ ||
         !this->machine_data_field_sensors_.empty() || !this->machine_data_numeric_field_sensors_.empty();
}

void JuraComponent::process_machine_data_query() {
  if (!this->has_machine_data_subscribers_()) {
    return;
  }
  bool has_field_subscribers = this->machine_data_auto_fields_ || !this->machine_data_field_sensors_.empty() ||
                               !this->machine_data_numeric_field_sensors_.empty();
  if (!this->is_ready()) {
    this->machine_data_command_index_ = 0;
    this->machine_data_responses_valid_ = false;
    this->machine_data_query_has_error_ = false;
    this->machine_data_responses_.clear();
    this->machine_data_payloads_.clear();
    this->machine_data_payload_ready_.clear();
    if (has_field_subscribers) {
      this->machine_data_field_values_.clear();
    }
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

  auto reset_query_state = [&]() {
    this->machine_data_command_index_ = 0;
    this->machine_data_responses_.clear();
    this->machine_data_payloads_.clear();
    this->machine_data_payload_ready_.clear();
    this->machine_data_query_next_ = esphome::millis() + MACHINE_DATA_QUERY_INTERVAL_MS;
  };

  auto fail_current_query = [&]() {
    this->machine_data_query_has_error_ = true;
    this->machine_data_responses_valid_ = false;
    reset_query_state();
  };

  auto handle_machine_data_response = [&](size_t index, const std::string &xml) -> bool {
    const auto &definition = MACHINE_DATA_COMMANDS[index];
    auto parsed = MachineDataParser::parse(xml);
    if (!parsed.has_value()) {
      ESP_LOGW(TAG, "Failed to parse XML response for command '%s'.", definition.command);
      return false;
    }
    const MachineDataNode *machine_data_node =
        find_descendant_case_insensitive(parsed.value(), "MACHINE_DATA");
    if (machine_data_node == nullptr) {
      ESP_LOGW(TAG, "Response for command '%s' does not contain <MACHINE_DATA> element.", definition.command);
      return false;
    }

    std::string encoding;
    std::string error;
    auto payload = extract_machine_data_payload(*machine_data_node, definition, &encoding, &error);
    if (!payload.has_value()) {
      ESP_LOGW(TAG, "Failed to extract payload for command '%s': %s.", definition.command, error.c_str());
      return false;
    }
    if (!error.empty()) {
      ESP_LOGW(TAG, "Machine data response for command '%s': %s.", definition.command, error.c_str());
    }

    if (this->machine_data_payloads_.size() != MACHINE_DATA_COMMANDS.size()) {
      this->machine_data_payloads_.assign(MACHINE_DATA_COMMANDS.size(), std::string{});
    }
    if (this->machine_data_payload_ready_.size() != MACHINE_DATA_COMMANDS.size()) {
      this->machine_data_payload_ready_.assign(MACHINE_DATA_COMMANDS.size(), false);
    }

    this->machine_data_payloads_[index] = *payload;
    auto expected_length = expected_machine_data_payload_size(definition.command);
    bool length_valid = true;
    if (expected_length.has_value()) {
      if (this->machine_data_payloads_[index].size() != expected_length.value()) {
        ESP_LOGW(TAG,
                 "Machine data payload for command '%s' has unexpected length (expected %zu byte%s, got %zu).",
                 definition.command, expected_length.value(), expected_length.value() == 1 ? "" : "s",
                 this->machine_data_payloads_[index].size());
        length_valid = false;
      }
    }

    this->machine_data_payload_ready_[index] = length_valid;
    if (!length_valid) {
      return false;
    }

    ESP_LOGD(TAG, "Parsed machine data command '%s' (encoding=%s, payload=%zu byte%s).",
             definition.command, encoding.c_str(), this->machine_data_payloads_[index].size(),
             this->machine_data_payloads_[index].size() == 1 ? "" : "s");
    return true;
  };

  uint32_t now = esphome::millis();

  if (this->machine_data_command_index_ == 0) {
    if (this->machine_data_query_next_ != 0 && !time_reached(now, this->machine_data_query_next_)) {
      return;
    }
    this->machine_data_responses_.assign(MACHINE_DATA_COMMANDS.size(), std::string{});
    this->machine_data_payloads_.assign(MACHINE_DATA_COMMANDS.size(), std::string{});
    this->machine_data_payload_ready_.assign(MACHINE_DATA_COMMANDS.size(), false);
    this->machine_data_query_has_error_ = false;
    this->machine_data_responses_valid_ = false;
    this->coffee_maker_->connection->flush_serial_input();
    esphome::delay(MACHINE_DATA_PRE_REQUEST_GAP_MS);
  }

  while (this->machine_data_command_index_ < MACHINE_DATA_COMMANDS.size()) {
    const auto &definition = MACHINE_DATA_COMMANDS[this->machine_data_command_index_];
    ESP_LOGV(TAG, "Requesting machine data command '%s' (%s/%s).", definition.command,
             definition.section, definition.element);
    auto response = this->coffee_maker_->connection->transact_db(
        definition.command, std::chrono::milliseconds{MACHINE_DATA_REQUEST_TIMEOUT_MS});
    if (response == nullptr) {
      ESP_LOGW(TAG, "Timeout while waiting for machine data response to command '%s'.", definition.command);
      fail_current_query();
      if (has_field_subscribers) {
        this->machine_data_field_values_.clear();
      }
      return;
    }
    this->machine_data_responses_[this->machine_data_command_index_] = *response;
    if (!handle_machine_data_response(this->machine_data_command_index_, *response)) {
      fail_current_query();
      if (has_field_subscribers) {
        this->machine_data_field_values_.clear();
      }
      return;
    }
    ++this->machine_data_command_index_;
  }

  if (this->machine_data_command_index_ >= MACHINE_DATA_COMMANDS.size()) {
    bool all_ready = !this->machine_data_payload_ready_.empty() &&
                     std::all_of(this->machine_data_payload_ready_.begin(),
                                 this->machine_data_payload_ready_.end(), [](bool ready) { return ready; });
    if (!all_ready || this->machine_data_query_has_error_) {
      ESP_LOGW(TAG, "Skipping machine data publish due to incomplete responses.");
      if (has_field_subscribers) {
        this->machine_data_field_values_.clear();
      }
      fail_current_query();
      return;
    }
    bool collect_fields = this->machine_data_auto_fields_ || !this->machine_data_field_sensors_.empty() ||
                          !this->machine_data_numeric_field_sensors_.empty();
    MachineDataFieldMap field_values;
    std::string payload = build_machine_data_payload(this->machine_data_payloads_,
                                                     collect_fields ? &field_values : nullptr);
    if (collect_fields) {
      this->machine_data_field_values_ = std::move(field_values);
    }
    this->machine_data_responses_valid_ = true;
    this->publish_machine_data_(payload);
    this->machine_data_query_has_error_ = false;
    reset_query_state();
  }
}

void JuraComponent::publish_machine_data_(const std::string &response) {
  bool has_field_subscribers = this->machine_data_auto_fields_ || !this->machine_data_field_sensors_.empty() ||
                               !this->machine_data_numeric_field_sensors_.empty();

  auto parsed = MachineDataParser::parse(response);
  if (!parsed.has_value()) {
    ESP_LOGW(TAG, "Failed to parse machine data XML response.");
    return;
  }

  const auto &root = parsed.value();
  if (this->machine_data_statistic_sensor_ != nullptr) {
    auto formatted = format_machine_data_section(root.find_child_case_insensitive("STATISTIC"));
    this->machine_data_statistic_sensor_->publish_state(sanitize_text_sensor_value(formatted));
  }
  if (this->machine_data_errors_sensor_ != nullptr) {
    auto formatted = format_machine_data_section(root.find_child_case_insensitive("ALERTS"));
    this->machine_data_errors_sensor_->publish_state(sanitize_text_sensor_value(formatted));
  }
  if (this->machine_data_status_sensor_ != nullptr) {
    auto formatted =
        format_machine_data_section(root.find_child_case_insensitive("PROGRESS_STATE_INTAKE"));
    this->machine_data_status_sensor_->publish_state(sanitize_text_sensor_value(formatted));
  }
  if (this->machine_data_processes_sensor_ != nullptr) {
    auto formatted = format_machine_data_section(root.find_child_case_insensitive("PROCESSES"));
    this->machine_data_processes_sensor_->publish_state(sanitize_text_sensor_value(formatted));
  }

  std::string sanitized = sanitize_text_sensor_value(response);
  ESP_LOGD(TAG, "Machine data response: %s", sanitized.c_str());
  if (has_field_subscribers && !this->machine_data_field_values_.empty()) {
    this->publish_machine_data_fields_(this->machine_data_field_values_);
  }
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

