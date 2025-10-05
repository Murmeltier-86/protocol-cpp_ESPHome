#include "esphome/components/jutta_proto/jutta_proto_xml.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <unordered_map>
#include <unordered_set>

#include "esphome/core/log.h"

namespace esphome {
namespace jutta_component {

namespace {
static const char *const TAG = "jutta_proto.xml";

std::string to_lower_copy(const std::string &input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return result;
}

void trim(std::string &value) {
  auto begin = value.find_first_not_of(" \t\r\n");
  auto end = value.find_last_not_of(" \t\r\n");
  if (begin == std::string::npos || end == std::string::npos) {
    value.clear();
    return;
  }
  value = value.substr(begin, end - begin + 1);
}

std::string trim_copy(std::string value) {
  trim(value);
  return value;
}

struct AttributeMap {
  std::unordered_map<std::string, std::string> values;

  std::string get(const std::string &key) const {
    auto it = this->values.find(to_lower_copy(key));
    if (it != this->values.end()) {
      return it->second;
    }
    return {};
  }
};

struct TextItemSpec {
  std::string label;
  std::string name_hint;
  bool has_offset{false};
  std::size_t offset{0};
  bool has_size{false};
  std::size_t size{0};
  bool has_endian{false};
  bool little_endian{false};
};

std::string format_hex_string(const uint8_t *data, std::size_t length, bool spaced = true) {
  if (data == nullptr || length == 0) {
    return {};
  }
  std::string out;
  out.reserve(length * (spaced ? 3U : 2U));
  char buf[4];
  for (std::size_t i = 0; i < length; ++i) {
    if (spaced && i > 0) {
      out.push_back(' ');
    }
    std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(data[i]));
    out.append(buf);
  }
  return out;
}

std::string format_hex_string(const std::vector<uint8_t> &data) {
  if (data.empty()) {
    return {};
  }
  return format_hex_string(data.data(), data.size(), true);
}

std::string format_hex_head(const std::vector<uint8_t> &data, std::size_t max_bytes) {
  if (data.empty() || max_bytes == 0) {
    return {};
  }
  std::size_t count = std::min<std::size_t>(data.size(), max_bytes);
  return format_hex_string(data.data(), count, true);
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

AttributeMap parse_attributes(const std::string &tag) {
  AttributeMap map;
  std::size_t pos = 0;
  while (pos < tag.size()) {
    pos = tag.find_first_not_of(" \t\r\n</", pos);
    if (pos == std::string::npos) {
      break;
    }
    if (tag[pos] == '>' || tag[pos] == '/') {
      break;
    }
    std::size_t name_end = tag.find_first_of("= \t\r\n/>", pos);
    if (name_end == std::string::npos) {
      break;
    }
    std::string key = tag.substr(pos, name_end - pos);
    key = to_lower_copy(key);
    std::size_t equal_pos = tag.find('=', name_end);
    if (equal_pos == std::string::npos) {
      break;
    }
    std::size_t next_non_space = tag.find_first_not_of(" \t\r\n", name_end);
    if (next_non_space == std::string::npos) {
      break;
    }
    if (tag[next_non_space] != '=') {
      pos = next_non_space;
      continue;
    }
    pos = equal_pos + 1;
    pos = tag.find_first_not_of(" \t\r\n", pos);
    if (pos == std::string::npos) {
      break;
    }
    char quote = tag[pos];
    std::size_t value_begin = pos;
    std::size_t value_end;
    if (quote == '\"' || quote == '\'') {
      ++value_begin;
      value_end = tag.find(quote, value_begin);
      if (value_end == std::string::npos) {
        value_end = tag.size();
      }
      pos = value_end + 1;
    } else {
      value_end = tag.find_first_of(" \t\r\n/>", value_begin);
      if (value_end == std::string::npos) {
        value_end = tag.size();
      }
      pos = value_end;
    }
    std::string value = tag.substr(value_begin, value_end - value_begin);
    trim(value);
    map.values[key] = value;
  }
  return map;
}

bool parse_size_t_attr(const AttributeMap &attrs, std::initializer_list<const char *> keys, std::size_t &out) {
  for (const char *key : keys) {
    std::string value = attrs.get(key);
    if (!value.empty()) {
      char *end = nullptr;
      auto parsed = std::strtoul(value.c_str(), &end, 0);
      if (end != value.c_str()) {
        out = static_cast<std::size_t>(parsed);
        return true;
      }
    }
  }
  return false;
}

bool parse_double_attr(const AttributeMap &attrs, std::initializer_list<const char *> keys, double &out) {
  for (const char *key : keys) {
    std::string value = attrs.get(key);
    if (!value.empty()) {
      char *end = nullptr;
      auto parsed = std::strtod(value.c_str(), &end);
      if (end != value.c_str()) {
        out = parsed;
        return true;
      }
    }
  }
  return false;
}

bool contains_percent_hint(const std::string &value) {
  if (value.find('%') != std::string::npos) {
    return true;
  }
  std::string lower = to_lower_copy(value);
  return lower.find("percent") != std::string::npos || lower.find("prozent") != std::string::npos ||
         lower.find("pct") != std::string::npos;
}

bool is_self_closing_tag(const std::string &tag_text) {
  std::string trimmed = trim_copy(tag_text);
  if (trimmed.size() < 2) {
    return false;
  }
  return trimmed[trimmed.size() - 2] == '/' && trimmed.back() == '>';
}

bool extract_section_content(const std::string &xml, const std::string &tag_name, std::string &out) {
  std::string lower_xml = to_lower_copy(xml);
  std::string open = "<" + to_lower_copy(tag_name);
  std::size_t start = lower_xml.find(open);
  if (start == std::string::npos) {
    return false;
  }
  std::size_t open_end = lower_xml.find('>', start);
  if (open_end == std::string::npos) {
    return false;
  }
  if (is_self_closing_tag(xml.substr(start, open_end - start + 1))) {
    out.clear();
    return true;
  }
  std::string close = "</" + to_lower_copy(tag_name);
  std::size_t close_pos = lower_xml.find(close, open_end);
  if (close_pos == std::string::npos) {
    return false;
  }
  out = xml.substr(open_end + 1, close_pos - open_end - 1);
  return true;
}

void for_each_tag(const std::string &block, const std::string &tag_name,
                  const std::function<void(const std::string &)> &callback) {
  std::string lower = to_lower_copy(block);
  std::string needle = "<" + to_lower_copy(tag_name);
  std::size_t pos = 0;
  while (true) {
    std::size_t tag_pos = lower.find(needle, pos);
    if (tag_pos == std::string::npos) {
      break;
    }
    std::size_t check_pos = tag_pos + needle.size();
    if (check_pos < lower.size()) {
      char next = lower[check_pos];
      if (!(next == '>' || next == '/' || std::isspace(static_cast<unsigned char>(next)))) {
        pos = tag_pos + 1;
        continue;
      }
    }
    std::size_t tag_end = lower.find('>', tag_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    std::string tag_text = block.substr(tag_pos, tag_end - tag_pos + 1);
    callback(tag_text);
    pos = tag_end + 1;
  }
}

std::string make_identifier(const std::string &prefix, const std::string &value, std::size_t index,
                            std::unordered_set<std::string> &used) {
  std::string sanitized;
  bool last_was_underscore = false;
  for (char c : value) {
    unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) {
      sanitized.push_back(static_cast<char>(std::tolower(uc)));
      last_was_underscore = false;
    } else {
      if (!last_was_underscore) {
        sanitized.push_back('_');
        last_was_underscore = true;
      }
    }
  }
  if (!sanitized.empty() && sanitized.front() == '_') {
    sanitized.erase(sanitized.begin());
  }
  while (!sanitized.empty() && sanitized.back() == '_') {
    sanitized.pop_back();
  }
  if (sanitized.empty()) {
    sanitized = prefix + "_" + std::to_string(index + 1);
  } else {
    sanitized = prefix + "_" + sanitized;
  }
  std::string candidate = sanitized;
  std::size_t suffix = 2;
  while (used.find(candidate) != used.end()) {
    candidate = sanitized + "_" + std::to_string(suffix++);
  }
  used.insert(candidate);
  return candidate;
}

void add_fields_from_labels(XmlCommandMapping &mapping, const std::vector<std::string> &labels,
                            const std::string &prefix, std::size_t field_size,
                            std::size_t base_offset = 0) {
  std::unordered_set<std::string> used;
  std::size_t offset = base_offset;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    XmlField field;
    field.label = labels[i];
    field.name = make_identifier(prefix, field.label, i, used);
    field.offset = offset;
    field.size = field_size;
    mapping.fields.push_back(field);
    offset += field_size;
  }
}

bool parse_field_tag(const std::string &tag_text, XmlCommandMapping &mapping) {
  auto attrs = parse_attributes(tag_text);
  XmlField field;
  field.name = attrs.get("name");
  trim(field.name);
  if (field.name.empty()) {
    ESP_LOGW(TAG, "XML Feld ohne 'name' ignoriert");
    return false;
  }
  field.label = attrs.get("label");
  trim(field.label);
  if (field.label.empty()) {
    field.label = field.name;
  }
  parse_size_t_attr(attrs, {"offset", "byte", "start"}, field.offset);
  if (!parse_size_t_attr(attrs, {"size", "bytes", "length"}, field.size)) {
    ESP_LOGW(TAG, "XML Feld %s ohne gültige 'size'-Angabe ignoriert", field.name.c_str());
    return false;
  }
  if (field.size != 1 && field.size != 2 && field.size != 4) {
    ESP_LOGW(TAG, "XML Feld %s verwendet nicht unterstützte Größe %u", field.name.c_str(),
             static_cast<unsigned>(field.size));
    return false;
  }
  std::string endian = to_lower_copy(attrs.get("endian"));
  if (endian == "le" || endian == "little") {
    field.little_endian = true;
  }
  double scale = field.scale;
  if (parse_double_attr(attrs, {"scale"}, scale)) {
    field.scale = scale;
  } else if (contains_percent_hint(field.name) || contains_percent_hint(field.label)) {
    field.scale = 0.01;
  }
  mapping.fields.push_back(field);
  return true;
}

bool parse_fields_block(const std::string &content, XmlCommandMapping &mapping) {
  bool any = false;
  std::string lower = to_lower_copy(content);
  std::size_t search_pos = 0;
  while (true) {
    std::size_t field_pos = lower.find("<field", search_pos);
    if (field_pos == std::string::npos) {
      break;
    }
    std::size_t tag_end = lower.find('>', field_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    std::string tag_text = content.substr(field_pos, tag_end - field_pos + 1);
    if (parse_field_tag(tag_text, mapping)) {
      any = true;
    }
    search_pos = tag_end + 1;
  }
  return any;
}

bool parse_command_block(const std::string &content, const std::string &target_id, XmlCommandMapping &mapping) {
  std::string lower = to_lower_copy(content);
  std::string lowered_id = to_lower_copy(target_id);
  std::size_t search_pos = 0;
  bool found = false;
  while (true) {
    std::size_t cmd_pos = lower.find("<cmd", search_pos);
    if (cmd_pos == std::string::npos) {
      break;
    }
    std::size_t tag_end = lower.find('>', cmd_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    std::string tag_text = content.substr(cmd_pos, tag_end - cmd_pos + 1);
    auto attrs = parse_attributes(tag_text);
    std::string id = attrs.get("id");
    trim(id);
    if (to_lower_copy(id) != lowered_id) {
      search_pos = tag_end + 1;
      continue;
    }
    std::size_t close_pos = lower.find("</cmd", tag_end);
    if (close_pos == std::string::npos) {
      ESP_LOGW(TAG, "XML CMD %s ohne schließendes </CMD>", id.c_str());
      break;
    }
    std::string inner = content.substr(tag_end + 1, close_pos - tag_end - 1);
    parse_fields_block(inner, mapping);
    found = true;
    search_pos = close_pos + 5;
    break;
  }
  return found;
}

bool find_bank_block(const std::string &xml, const std::string &command, std::string &content, AttributeMap *attrs_out) {
  std::string lower_xml = to_lower_copy(xml);
  std::string needle = "<bank";
  std::string lowered_command = to_lower_copy(command);
  std::size_t search_pos = 0;
  while (true) {
    std::size_t pos = lower_xml.find(needle, search_pos);
    if (pos == std::string::npos) {
      return false;
    }
    std::size_t tag_end = lower_xml.find('>', pos);
    if (tag_end == std::string::npos) {
      return false;
    }
    std::string tag_text = xml.substr(pos, tag_end - pos + 1);
    auto attrs = parse_attributes(tag_text);
    std::string cmd_attr = to_lower_copy(attrs.get("command"));
    trim(cmd_attr);
    if (cmd_attr == lowered_command) {
      if (attrs_out != nullptr) {
        *attrs_out = attrs;
      }
      if (is_self_closing_tag(tag_text)) {
        content.clear();
        return true;
      }
      std::size_t close_pos = lower_xml.find("</bank", tag_end);
      if (close_pos == std::string::npos) {
        return false;
      }
      content = xml.substr(tag_end + 1, close_pos - tag_end - 1);
      return true;
    }
    search_pos = tag_end + 1;
  }
}

std::vector<std::string> build_tr32_labels(const std::string &xml) {
  const std::size_t desired = 10;
  std::vector<std::string> labels;
  std::string products_block;
  if (!extract_section_content(xml, "products", products_block)) {
    return labels;
  }
  for_each_tag(products_block, "totalcounter", [&](const std::string &tag_text) {
    auto attrs = parse_attributes(tag_text);
    std::string name = attrs.get("name");
    trim(name);
    if (!name.empty() && labels.size() < desired) {
      labels.push_back(name);
    }
  });

  std::vector<std::string> singles;
  std::vector<std::string> doubles;
  for_each_tag(products_block, "product", [&](const std::string &tag_text) {
    auto attrs = parse_attributes(tag_text);
    std::string name = attrs.get("name");
    trim(name);
    if (name.empty()) {
      return;
    }
    std::string double_flag = to_lower_copy(attrs.get("doubleproduct"));
    trim(double_flag);
    if (double_flag == "true" || double_flag == "1" || double_flag == "yes") {
      doubles.push_back(name);
    } else {
      singles.push_back(name);
    }
  });

  for (const auto &name : singles) {
    if (labels.size() >= desired) {
      break;
    }
    labels.push_back(name);
  }
  for (const auto &name : doubles) {
    if (labels.size() >= desired) {
      break;
    }
    labels.push_back(name);
  }

  while (labels.size() < desired) {
    labels.push_back("TR32 " + std::to_string(labels.size() + 1));
  }
  if (labels.size() > desired) {
    labels.resize(desired);
  }
  return labels;
}

bool parse_tr32_mapping(const std::string &xml, XmlCommandMapping &mapping) {
  mapping.fields.clear();
  std::string content;
  if (!find_bank_block(xml, "@tr:32", content, nullptr)) {
    return false;
  }
  auto labels = build_tr32_labels(xml);
  if (labels.empty()) {
    for (std::size_t i = 0; i < 10; ++i) {
      labels.push_back("TR32 " + std::to_string(i + 1));
    }
  }
  add_fields_from_labels(mapping, labels, "tr32", 2, 1);
  return true;
}

std::vector<TextItemSpec> parse_textitem_specs(const std::string &block) {
  std::vector<TextItemSpec> specs;
  for_each_tag(block, "textitem", [&](const std::string &tag_text) {
    auto attrs = parse_attributes(tag_text);
    TextItemSpec spec;
    spec.name_hint = attrs.get("type");
    trim(spec.name_hint);
    if (spec.name_hint.empty()) {
      spec.name_hint = attrs.get("name");
      trim(spec.name_hint);
    }
    std::string label = attrs.get("text");
    trim(label);
    if (label.empty()) {
      label = spec.name_hint;
    }
    spec.label = label;

    std::size_t value = 0;
    if (parse_size_t_attr(attrs, {"offset", "byte", "start"}, value)) {
      spec.has_offset = true;
      spec.offset = value;
    }
    value = 0;
    if (parse_size_t_attr(attrs, {"size", "bytes", "length"}, value)) {
      spec.has_size = true;
      spec.size = value;
    }
    std::string endian = to_lower_copy(attrs.get("endian"));
    if (endian == "le" || endian == "little") {
      spec.has_endian = true;
      spec.little_endian = true;
    } else if (endian == "be" || endian == "big") {
      spec.has_endian = true;
      spec.little_endian = false;
    }
    if (!spec.label.empty() || !spec.name_hint.empty()) {
      specs.push_back(spec);
    }
  });
  return specs;
}

bool parse_textitem_mapping(const std::string &xml, const std::string &command, const std::string &prefix,
                            std::size_t field_size, std::size_t fallback_count, XmlCommandMapping &mapping) {
  mapping.fields.clear();
  std::string content;
  if (!find_bank_block(xml, command, content, nullptr)) {
    return false;
  }
  auto specs = parse_textitem_specs(content);
  if (specs.empty()) {
    std::vector<std::string> labels;
    for (std::size_t i = 0; i < fallback_count; ++i) {
      labels.push_back(prefix + " " + std::to_string(i + 1));
    }
    add_fields_from_labels(mapping, labels, to_lower_copy(prefix), field_size, 1);
    return true;
  }

  std::unordered_set<std::string> used_names;
  std::size_t running_offset = 1;
  for (std::size_t i = 0; i < specs.size(); ++i) {
    const auto &spec = specs[i];
    XmlField field;
    field.label = spec.label.empty() ? spec.name_hint : spec.label;
    field.name = make_identifier(to_lower_copy(prefix), field.label, i, used_names);
    if (spec.has_offset) {
      field.offset = spec.offset;
      running_offset = spec.offset;
    } else {
      field.offset = running_offset;
    }
    if (spec.has_size) {
      field.size = spec.size;
    } else {
      field.size = field_size;
    }
    if (!(field.size == 1 || field.size == 2 || field.size == 4)) {
      ESP_LOGW(TAG, "XML %s Feld %s ignoriert (nicht unterstützte Größe %u)",
               command, field.name.c_str(), static_cast<unsigned>(field.size));
      continue;
    }
    running_offset = field.offset + field.size;
    if (spec.has_endian) {
      field.little_endian = spec.little_endian;
    }
    mapping.fields.push_back(field);
  }
  return true;
}

XmlMapping g_mapping;
bool g_mapping_loaded = false;

bool parse_payload(const std::vector<uint8_t> &decoded, const XmlCommandMapping &mapping, Stats &out,
                   const char *command_label) {
  if (mapping.empty()) {
    ESP_LOGW(TAG, "XML %s: kein Mapping vorhanden", command_label);
    return false;
  }
  bool any = false;
  std::size_t expected_len = 0;
  for (const auto &field : mapping.fields) {
    expected_len = std::max(expected_len, field.offset + field.size);
  }
  std::string hex_head = format_hex_head(decoded, 32);
  ESP_LOGD(TAG, "XML frame: cmd=%s decoded_len=%u expected_len=%u hex_head=%s", command_label,
           static_cast<unsigned>(decoded.size()), static_cast<unsigned>(expected_len), hex_head.c_str());
  std::string payload_hex = format_hex_string(decoded);
  if (!payload_hex.empty()) {
    ESP_LOGD(TAG, "XML %s payload HEX: %s", command_label, payload_hex.c_str());
    std::string payload_ascii = format_ascii_string(decoded);
    ESP_LOGD(TAG, "XML %s payload ASCII: %s", command_label, payload_ascii.c_str());
  }
  if (expected_len != 0 && decoded.size() != expected_len) {
    std::string mismatch_head = format_hex_head(decoded, 32);
    ESP_LOGW(TAG,
             "XML %s: decoded_len (%u) != expected_len (%u), head32=%s",
             command_label, static_cast<unsigned>(decoded.size()), static_cast<unsigned>(expected_len),
             mismatch_head.c_str());
  }
  for (const auto &field : mapping.fields) {
    if (field.offset + field.size > decoded.size()) {
      ESP_LOGW(TAG, "XML %s Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", command_label,
               field.name.c_str(), static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               static_cast<unsigned>(decoded.size()));
      continue;
    }
    std::uint64_t raw = 0;
    if (field.little_endian) {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw |= static_cast<std::uint64_t>(decoded[field.offset + i]) << (8U * i);
      }
    } else {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw = (raw << 8U) | static_cast<std::uint64_t>(decoded[field.offset + i]);
      }
    }
    std::string raw_hex = format_hex_string(decoded.data() + field.offset, field.size, false);
    ESP_LOGD(TAG,
             "XML field %s offset=%u size=%u endian=%s raw=%s parsed_uint=%llu",
             field.name.c_str(), static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
             field.little_endian ? "LE" : "BE", raw_hex.c_str(),
             static_cast<unsigned long long>(raw));
    if ((field.size <= 2 && raw > 0xFFFFULL) || raw > 100000ULL) {
      ESP_LOGW(TAG, "XML %s Feld %s plausibility warning: value=%llu", command_label, field.name.c_str(),
               static_cast<unsigned long long>(raw));
    }
    double value = static_cast<double>(raw) * field.scale;
    out.set_value(field.name, value, field.label);
    ESP_LOGD(TAG, "XML field %s value_staged=%.3f", field.name.c_str(), value);
    any = true;
  }
  return any;
}

}  // namespace

void Stats::clear() { this->values_.clear(); }

bool Stats::empty() const { return this->values_.empty(); }

void Stats::set_value(const std::string &name, double value, const std::string &label) {
  this->values_[name] = StatValue{value, label};
}

bool Stats::has_value(const std::string &name) const { return this->values_.find(name) != this->values_.end(); }

const std::unordered_map<std::string, StatValue> &Stats::values() const { return this->values_; }

bool load_mapping_from_string(const std::string &xml) {
  g_mapping = XmlMapping{};
  bool tr32_found = parse_tr32_mapping(xml, g_mapping.tr32);
  bool tr32_has_fields = tr32_found && !g_mapping.tr32.empty();
  if (!tr32_has_fields) {
    g_mapping.tr32 = XmlCommandMapping{};
    bool legacy_tr32 = parse_command_block(xml, "@tr:32", g_mapping.tr32);
    tr32_found = tr32_found || legacy_tr32;
    tr32_has_fields = legacy_tr32 && !g_mapping.tr32.empty();
  }

  bool tg43_found = parse_textitem_mapping(xml, "@tg:43", "TG43", 2, 6, g_mapping.tg43);
  bool tg43_has_fields = tg43_found && !g_mapping.tg43.empty();
  if (!tg43_has_fields) {
    g_mapping.tg43 = XmlCommandMapping{};
    bool legacy_tg43 = parse_command_block(xml, "@tg:43", g_mapping.tg43);
    tg43_found = tg43_found || legacy_tg43;
    tg43_has_fields = legacy_tg43 && !g_mapping.tg43.empty();
  }

  bool tgc0_found = parse_textitem_mapping(xml, "@tg:c0", "TGC0", 4, 3, g_mapping.tgc0);
  bool tgc0_has_fields = tgc0_found && !g_mapping.tgc0.empty();
  if (!tgc0_has_fields) {
    g_mapping.tgc0 = XmlCommandMapping{};
    bool legacy_tgc0 = parse_command_block(xml, "@tg:c0", g_mapping.tgc0);
    tgc0_found = tgc0_found || legacy_tgc0;
    tgc0_has_fields = legacy_tgc0 && !g_mapping.tgc0.empty();
  }

  g_mapping.valid = tr32_has_fields || tg43_has_fields || tgc0_has_fields;
  g_mapping_loaded = true;

  if (!tr32_found || g_mapping.tr32.empty()) {
    ESP_LOGW(TAG, "XML Mapping enthält keine Felder für @TR:32");
  }
  if (!tg43_found || g_mapping.tg43.empty()) {
    ESP_LOGW(TAG, "XML Mapping enthält keine Felder für @TG:43");
  }
  if (!tgc0_found || g_mapping.tgc0.empty()) {
    ESP_LOGW(TAG, "XML Mapping enthält keine Felder für @TG:C0");
  }
  return g_mapping.valid;
}

const XmlMapping &get_xml_mapping() { return g_mapping; }

bool parse_TR32(const std::vector<uint8_t> &decoded, Stats &out) {
  if (!g_mapping_loaded) {
    ESP_LOGW(TAG, "XML Mapping wurde nicht geladen");
    return false;
  }
  return parse_payload(decoded, g_mapping.tr32, out, "@TR:32");
}

bool parse_TG43(const std::vector<uint8_t> &decoded, Stats &out) {
  if (!g_mapping_loaded) {
    ESP_LOGW(TAG, "XML Mapping wurde nicht geladen");
    return false;
  }
  return parse_payload(decoded, g_mapping.tg43, out, "@TG:43");
}

bool parse_TGC0(const std::vector<uint8_t> &decoded, Stats &out) {
  if (!g_mapping_loaded) {
    ESP_LOGW(TAG, "XML Mapping wurde nicht geladen");
    return false;
  }
  return parse_payload(decoded, g_mapping.tgc0, out, "@TG:C0");
}

}  // namespace jutta_component
}  // namespace esphome

