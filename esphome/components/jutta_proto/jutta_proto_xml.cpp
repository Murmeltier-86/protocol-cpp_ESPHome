#include "esphome/components/jutta_proto/jutta_proto_xml.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <unordered_map>

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
  bool tr32_found = parse_command_block(xml, "@tr:32", g_mapping.tr32);
  bool tg43_found = parse_command_block(xml, "@tg:43", g_mapping.tg43);
  bool tgc0_found = parse_command_block(xml, "@tg:c0", g_mapping.tgc0);

  bool tr32_has_fields = tr32_found && !g_mapping.tr32.empty();
  bool tg43_has_fields = tg43_found && !g_mapping.tg43.empty();
  bool tgc0_has_fields = tgc0_found && !g_mapping.tgc0.empty();

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

