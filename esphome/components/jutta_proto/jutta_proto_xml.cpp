#include "esphome/components/jutta_proto/jutta_proto_xml.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <sstream>
#include <unordered_map>

#include "esphome/core/log.h"

namespace esphome {
namespace jutta_component {

namespace {
static const char *const TAG = "jutta_proto.xml";

constexpr std::size_t DEFAULT_TR32_COUNT = 10;
constexpr std::size_t DEFAULT_TG43_COUNT = 6;
constexpr std::size_t DEFAULT_TGC0_COUNT = 3;

constexpr std::size_t MAX_TR32_COUNT = 64;
constexpr std::size_t MAX_TG43_COUNT = 16;
constexpr std::size_t MAX_TGC0_COUNT = 16;

constexpr std::size_t FIELD_WIDTH_16BIT = 2;
constexpr std::size_t FIELD_WIDTH_32BIT = 4;

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

std::string tidy_label(std::string value) {
  std::string result;
  result.reserve(value.size() * 2);
  char prev = 0;
  for (char ch : value) {
    if (ch == '_' || ch == '-') {
      if (!result.empty() && result.back() != ' ') {
        result.push_back(' ');
      }
      prev = ' ';
      continue;
    }
    if (std::isupper(static_cast<unsigned char>(ch)) != 0 && prev != 0 &&
        std::islower(static_cast<unsigned char>(prev)) != 0) {
      if (!result.empty() && result.back() != ' ') {
        result.push_back(' ');
      }
    }
    result.push_back(ch);
    prev = ch;
  }
  trim(result);
  return result;
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
    pos = tag.find('=', name_end);
    if (pos == std::string::npos) {
      break;
    }
    ++pos;
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

std::string extract_block(const std::string &xml, const std::string &lower_xml, const std::string &tag) {
  std::string tag_lower = "<" + tag;
  std::size_t begin = lower_xml.find(tag_lower);
  if (begin == std::string::npos) {
    return {};
  }
  std::size_t open_end = lower_xml.find('>', begin);
  if (open_end == std::string::npos) {
    return {};
  }
  std::string close_tag = "</" + tag + ">";
  std::size_t close_pos = lower_xml.find(close_tag, open_end);
  if (close_pos == std::string::npos) {
    return {};
  }
  return xml.substr(open_end + 1, close_pos - open_end - 1);
}

std::vector<std::string> collect_tag_values(const std::string &content, const std::string &tag,
                                            std::initializer_list<const char *> attribute_priority) {
  std::vector<std::string> values;
  std::string lower = to_lower_copy(content);
  std::string needle = "<" + tag;
  std::size_t pos = 0;
  while (true) {
    std::size_t tag_pos = lower.find(needle, pos);
    if (tag_pos == std::string::npos) {
      break;
    }
    std::size_t tag_end = lower.find('>', tag_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    std::string tag_text = content.substr(tag_pos, tag_end - tag_pos + 1);
    auto attrs = parse_attributes(tag_text);
    std::string chosen;
    for (const char *key : attribute_priority) {
      chosen = attrs.get(key);
      trim(chosen);
      if (!chosen.empty()) {
        break;
      }
    }
    if (!chosen.empty()) {
      values.push_back(tidy_label(chosen));
    }
    pos = tag_end + 1;
  }
  return values;
}

std::string extract_bank_block(const std::string &xml, const std::string &lower_xml, const std::string &command) {
  std::string lowered_command = to_lower_copy(command);
  std::size_t pos = 0;
  while (true) {
    std::size_t bank_pos = lower_xml.find("<bank", pos);
    if (bank_pos == std::string::npos) {
      break;
    }
    std::size_t tag_end = lower_xml.find('>', bank_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    std::string tag_text = xml.substr(bank_pos, tag_end - bank_pos + 1);
    auto attrs = parse_attributes(tag_text);
    std::string cmd = to_lower_copy(attrs.get("command"));
    if (cmd == lowered_command) {
      if (tag_end > bank_pos && lower_xml[tag_end - 1] == '/') {
        return {};
      }
      std::size_t close_pos = lower_xml.find("</bank", tag_end);
      if (close_pos == std::string::npos) {
        return {};
      }
      return xml.substr(tag_end + 1, close_pos - tag_end - 1);
    }
    pos = tag_end + 1;
  }
  return {};
}

std::vector<std::string> collect_tr32_labels(const std::string &xml) {
  std::vector<std::string> labels;
  std::string lower_xml = to_lower_copy(xml);

  auto total_values = collect_tag_values(xml, "totalcounter", {"name", "text"});
  if (!total_values.empty()) {
    labels.push_back(total_values.front());
  }

  std::string products_block = extract_block(xml, lower_xml, "products");
  if (!products_block.empty()) {
    auto product_values = collect_tag_values(products_block, "product", {"name", "text"});
    labels.insert(labels.end(), product_values.begin(), product_values.end());
  }

  return labels;
}

std::vector<std::string> collect_textitem_labels(const std::string &xml, const std::string &command) {
  std::string lower_xml = to_lower_copy(xml);
  std::string bank_block = extract_bank_block(xml, lower_xml, command);
  if (bank_block.empty()) {
    return {};
  }
  return collect_tag_values(bank_block, "textitem", {"label", "type", "name", "text"});
}

std::string default_label_for_index(const std::string &display_prefix, std::size_t index) {
  std::ostringstream stream;
  stream << display_prefix << ' ' << (index + 1);
  return stream.str();
}

XmlCommandMapping build_sequential_mapping(const std::string &name_prefix, const std::string &display_prefix,
                                           std::size_t field_width, std::size_t count,
                                           const std::vector<std::string> &labels) {
  XmlCommandMapping mapping;
  if (count == 0) {
    return mapping;
  }
  mapping.fields.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    XmlField field;
    std::ostringstream name_stream;
    name_stream << name_prefix << '_' << (i + 1);
    field.name = name_stream.str();
    if (i < labels.size() && !labels[i].empty()) {
      field.label = labels[i];
    } else {
      field.label = default_label_for_index(display_prefix, i);
    }
    field.offset = i * field_width;
    field.size = field_width;
    field.little_endian = false;
    field.scale = 1.0;
    mapping.fields.push_back(field);
  }
  return mapping;
}

XmlMapping build_default_mapping() {
  XmlMapping mapping;
  mapping.tr32 = build_sequential_mapping("tr32", "TR32", FIELD_WIDTH_16BIT, DEFAULT_TR32_COUNT, {});
  mapping.tg43 = build_sequential_mapping("tg43", "TG43", FIELD_WIDTH_16BIT, DEFAULT_TG43_COUNT, {});
  mapping.tgc0 = build_sequential_mapping("tgc0", "TGC0", FIELD_WIDTH_32BIT, DEFAULT_TGC0_COUNT, {});
  mapping.valid = true;
  return mapping;
}

XmlMapping g_mapping;
bool g_mapping_loaded = false;

bool parse_payload(const std::vector<uint8_t> &decoded, const XmlCommandMapping &mapping, Stats &out,
                   const char *log_label) {
  if (mapping.empty()) {
    ESP_LOGW(TAG, "XML %s: kein Mapping vorhanden", log_label);
    return false;
  }
  bool any = false;
  for (const auto &field : mapping.fields) {
    if (field.offset + field.size > decoded.size()) {
      ESP_LOGW(TAG, "XML %s Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", log_label,
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
    double value = static_cast<double>(raw) * field.scale;
    out.set_value(field.name, value, field.label);
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

  bool tr32_cmd_found = parse_command_block(xml, "@tr:32", g_mapping.tr32);
  bool tg43_cmd_found = parse_command_block(xml, "@tg:43", g_mapping.tg43);
  bool tgc0_cmd_found = parse_command_block(xml, "@tg:c0", g_mapping.tgc0);

  std::string lower_xml = to_lower_copy(xml);
  bool has_tr32 = lower_xml.find("@tr:32") != std::string::npos;
  bool has_tg43 = lower_xml.find("@tg:43") != std::string::npos;
  bool has_tgc0 = lower_xml.find("@tg:c0") != std::string::npos;

  if ((!tr32_cmd_found || g_mapping.tr32.empty()) && has_tr32) {
    auto tr32_labels = collect_tr32_labels(xml);
    if (tr32_labels.size() > MAX_TR32_COUNT) {
      ESP_LOGW(TAG, "XML Mapping @TR:32 enthält %u Einträge, auf %u gekürzt",
               static_cast<unsigned>(tr32_labels.size()), static_cast<unsigned>(MAX_TR32_COUNT));
      tr32_labels.resize(MAX_TR32_COUNT);
    }
    std::size_t tr32_count = !tr32_labels.empty() ? tr32_labels.size() : DEFAULT_TR32_COUNT;
    if (tr32_count > MAX_TR32_COUNT) {
      tr32_count = MAX_TR32_COUNT;
    }
    g_mapping.tr32 = build_sequential_mapping("tr32", "TR32", FIELD_WIDTH_16BIT, tr32_count, tr32_labels);
  }
  if ((!tg43_cmd_found || g_mapping.tg43.empty()) && has_tg43) {
    auto tg43_labels = collect_textitem_labels(xml, "@tg:43");
    if (tg43_labels.size() > MAX_TG43_COUNT) {
      ESP_LOGW(TAG, "XML Mapping @TG:43 enthält %u Einträge, auf %u gekürzt",
               static_cast<unsigned>(tg43_labels.size()), static_cast<unsigned>(MAX_TG43_COUNT));
      tg43_labels.resize(MAX_TG43_COUNT);
    }
    std::size_t tg43_count = !tg43_labels.empty() ? tg43_labels.size() : DEFAULT_TG43_COUNT;
    if (tg43_count > MAX_TG43_COUNT) {
      tg43_count = MAX_TG43_COUNT;
    }
    g_mapping.tg43 = build_sequential_mapping("tg43", "TG43", FIELD_WIDTH_16BIT, tg43_count, tg43_labels);
  }
  if ((!tgc0_cmd_found || g_mapping.tgc0.empty()) && has_tgc0) {
    auto tgc0_labels = collect_textitem_labels(xml, "@tg:c0");
    if (tgc0_labels.size() > MAX_TGC0_COUNT) {
      ESP_LOGW(TAG, "XML Mapping @TG:C0 enthält %u Einträge, auf %u gekürzt",
               static_cast<unsigned>(tgc0_labels.size()), static_cast<unsigned>(MAX_TGC0_COUNT));
      tgc0_labels.resize(MAX_TGC0_COUNT);
    }
    std::size_t tgc0_count = !tgc0_labels.empty() ? tgc0_labels.size() : DEFAULT_TGC0_COUNT;
    if (tgc0_count > MAX_TGC0_COUNT) {
      tgc0_count = MAX_TGC0_COUNT;
    }
    g_mapping.tgc0 = build_sequential_mapping("tgc0", "TGC0", FIELD_WIDTH_32BIT, tgc0_count, tgc0_labels);
  }

  if (!has_tr32 && !has_tg43 && !has_tgc0) {
    ESP_LOGW(TAG, "XML Mapping enthält keine bekannten Kommandos, verwende Standardzuordnung");
    g_mapping = build_default_mapping();
  } else {
    if (has_tr32 && g_mapping.tr32.empty()) {
      ESP_LOGW(TAG, "XML Mapping @TR:32 ohne Felder, verwende Standardzuordnung");
      g_mapping.tr32 = build_sequential_mapping("tr32", "TR32", FIELD_WIDTH_16BIT, DEFAULT_TR32_COUNT, {});
    }
    if (has_tg43 && g_mapping.tg43.empty()) {
      ESP_LOGW(TAG, "XML Mapping @TG:43 ohne Felder, verwende Standardzuordnung");
      g_mapping.tg43 = build_sequential_mapping("tg43", "TG43", FIELD_WIDTH_16BIT, DEFAULT_TG43_COUNT, {});
    }
    if (has_tgc0 && g_mapping.tgc0.empty()) {
      ESP_LOGW(TAG, "XML Mapping @TG:C0 ohne Felder, verwende Standardzuordnung");
      g_mapping.tgc0 = build_sequential_mapping("tgc0", "TGC0", FIELD_WIDTH_32BIT, DEFAULT_TGC0_COUNT, {});
    }
  }

  g_mapping.valid = !g_mapping.tr32.empty() || !g_mapping.tg43.empty() || !g_mapping.tgc0.empty();
  g_mapping_loaded = true;
  return g_mapping.valid;
}

const XmlMapping &get_xml_mapping() { return g_mapping; }

bool parse_TR32(const std::vector<uint8_t> &decoded, Stats &out) {
  if (!g_mapping_loaded) {
    ESP_LOGW(TAG, "XML Mapping wurde nicht geladen");
    return false;
  }
  return parse_payload(decoded, g_mapping.tr32, out, "TR32");
}

bool parse_TG43(const std::vector<uint8_t> &decoded, Stats &out) {
  if (!g_mapping_loaded) {
    ESP_LOGW(TAG, "XML Mapping wurde nicht geladen");
    return false;
  }
  return parse_payload(decoded, g_mapping.tg43, out, "TG43");
}

bool parse_TGC0(const std::vector<uint8_t> &decoded, Stats &out) {
  if (!g_mapping_loaded) {
    ESP_LOGW(TAG, "XML Mapping wurde nicht geladen");
    return false;
  }
  return parse_payload(decoded, g_mapping.tgc0, out, "TGC0");
}

}  // namespace jutta_component
}  // namespace esphome

