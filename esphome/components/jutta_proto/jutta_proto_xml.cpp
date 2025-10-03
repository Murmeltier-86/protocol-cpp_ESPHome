#include "esphome/components/jutta_proto/jutta_proto_xml.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "esphome/core/log.h"
#ifdef USE_FILESYSTEM
#include "esphome/components/filesystem/filesystem.h"
#endif

namespace esphome {
namespace jutta_component {

namespace {
static const char *const TAG = "jutta_proto.xml";

struct AttributeMap {
  std::unordered_map<std::string, std::string> values;

  std::string get(const std::string &key) const {
    auto it = this->values.find(key);
    if (it != this->values.end()) {
      return it->second;
    }
    return {};
  }
};

inline std::string to_lower(const std::string &input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return result;
}

AttributeMap parse_attributes(const std::string &tag) {
  AttributeMap map;
  std::size_t pos = 0;
  while (pos < tag.size()) {
    std::size_t name_start = tag.find_first_not_of(" \t\r\n<", pos);
    if (name_start == std::string::npos) {
      break;
    }
    std::size_t name_end = tag.find_first_of("= \t\r\n>", name_start);
    if (name_end == std::string::npos) {
      break;
    }
    std::string key = to_lower(tag.substr(name_start, name_end - name_start));
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
    if (quote == '\"' || quote == '\'') {
      ++pos;
    } else {
      quote = ' ';
    }
    std::size_t value_end;
    if (quote == ' ') {
      value_end = tag.find_first_of(" \t\r\n>", pos);
    } else {
      value_end = tag.find(quote, pos);
    }
    if (value_end == std::string::npos) {
      break;
    }
    std::string value = tag.substr(pos, value_end - pos);
    map.values[key] = value;
    pos = value_end + 1;
  }
  return map;
}

bool parse_size_t(const AttributeMap &attrs, std::initializer_list<const char *> keys, std::size_t &out) {
  for (const char *key : keys) {
    std::string value = attrs.get(to_lower(key));
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

bool parse_double(const AttributeMap &attrs, std::initializer_list<const char *> keys, double &out) {
  for (const char *key : keys) {
    std::string value = attrs.get(to_lower(key));
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

bool load_file_content(const std::string &path, std::string &content) {
  if (path.empty()) {
    return false;
  }

  std::vector<std::string> candidates;
  candidates.push_back(path);

  if (!path.empty() && path[0] == '/') {
    candidates.emplace_back(path.begin() + 1, path.end());
  }

  constexpr char CONFIG_PREFIX[] = "/config/";
  if (path.rfind(CONFIG_PREFIX, 0) == 0) {
    candidates.emplace_back(path.begin() + sizeof(CONFIG_PREFIX) - 1, path.end());
  }

  auto slash = path.find_last_of("/\\");
  if (slash != std::string::npos && slash + 1 < path.size()) {
    candidates.push_back(path.substr(slash + 1));
  }

  // remove duplicates while keeping order
  std::vector<std::string> unique_candidates;
  for (const auto &candidate : candidates) {
    if (candidate.empty()) {
      continue;
    }
    if (std::find(unique_candidates.begin(), unique_candidates.end(), candidate) ==
        unique_candidates.end()) {
      unique_candidates.push_back(candidate);
    }
  }

  for (const auto &candidate : unique_candidates) {
#ifdef USE_FILESYSTEM
    if (auto *fs = esphome::filesystem::global_filesystem; fs != nullptr) {
      auto file = fs->open(candidate.c_str(), "r");
      if (file) {
        ESP_LOGI(TAG, "Lade XML Mapping aus '%s' (Filesystem)", candidate.c_str());
        std::ostringstream stream;
        while (file.available()) {
          stream << static_cast<char>(file.read());
        }
        content = stream.str();
        return true;
      }
    }
#endif
    std::ifstream file(candidate, std::ios::binary);
    if (file.is_open()) {
      ESP_LOGI(TAG, "Lade XML Mapping aus '%s'", candidate.c_str());
      std::ostringstream stream;
      stream << file.rdbuf();
      content = stream.str();
      return true;
    }
  }

  ESP_LOGW(TAG, "XML Mapping konnte unter keinem bekannten Pfad geladen werden (Original: %s)", path.c_str());
  return false;
}

void trim(std::string &value) {
  auto begin = value.find_first_not_of(" \t\r\n");
  auto end = value.find_last_not_of(" \t\r\n");
  if (begin == std::string::npos || end == std::string::npos) {
    value.clear();
  } else {
    value = value.substr(begin, end - begin + 1);
  }
}

bool decode_field_value(const XmlField &field, const std::vector<uint8_t> &decoded, double &value) {
  if (field.byte_length == 0) {
    return false;
  }
  std::size_t required = field.byte_offset + field.byte_length;
  if (decoded.size() < required) {
    return false;
  }
  std::uint64_t raw = 0;
  for (std::size_t i = 0; i < field.byte_length; ++i) {
    std::size_t index = field.little_endian ? (field.byte_offset + field.byte_length - 1 - i)
                                           : (field.byte_offset + i);
    raw = (raw << 8) | static_cast<std::uint64_t>(decoded[index]);
  }
  if (field.bit_length > 0) {
    std::size_t shift = field.bit_offset;
    raw >>= shift;
    if (field.bit_length < 64U) {
      std::uint64_t mask = (static_cast<std::uint64_t>(1) << field.bit_length) - 1ULL;
      raw &= mask;
    }
  }
  value = static_cast<double>(raw) * field.scale;
  return true;
}

bool map_fields(const std::vector<uint8_t> &decoded, const XmlCommandMapping &mapping,
                MachineStats &out, const char *label) {
  bool any = false;
  for (const auto &field : mapping.fields) {
    double numeric = 0.0;
    if (!decode_field_value(field, decoded, numeric)) {
      ESP_LOGW(TAG, "XML %s Feld %s konnte nicht gelesen werden (Offset=%u, Bytes=%u, Daten=%u)", label,
               field.name.c_str(), static_cast<unsigned>(field.byte_offset),
               static_cast<unsigned>(field.byte_length), static_cast<unsigned>(decoded.size()));
      continue;
    }
    out.set_value(field.name, numeric, field.label);
    any = true;
  }
  return any;
}

XmlCommandMapping *mapping_for_id(XmlMapping &mapping, const std::string &id) {
  std::string lowered = to_lower(id);
  std::string canonical;
  canonical.reserve(lowered.size());
  for (char ch : lowered) {
    if (std::isalnum(static_cast<unsigned char>(ch)) != 0) {
      canonical.push_back(ch);
    }
  }
  if (canonical == "tr32") {
    return &mapping.tr32_fields;
  }
  if (canonical == "tg43") {
    return &mapping.tg43_fields;
  }
  if (canonical == "tgc0") {
    return &mapping.tgc0_fields;
  }
  return nullptr;
}

}  // namespace

void MachineStats::clear() { this->values_.clear(); }

bool MachineStats::empty() const { return this->values_.empty(); }

void MachineStats::set_value(const std::string &name, double value, const std::string &label) {
  this->values_[name] = StatValue{value, label};
}

bool MachineStats::has_value(const std::string &name) const {
  return this->values_.find(name) != this->values_.end();
}

const std::unordered_map<std::string, StatValue> &MachineStats::values() const { return this->values_; }

namespace {

bool parse_fields(const std::string &content, XmlCommandMapping &mapping) {
  std::string lower = to_lower(content);
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
    auto attrs = parse_attributes(tag_text);
    XmlField field;
    field.name = attrs.get("name");
    trim(field.name);
    if (field.name.empty()) {
      ESP_LOGW(TAG, "XML Feld ohne Namen ignoriert");
      search_pos = tag_end + 1;
      continue;
    }
    field.label = attrs.get("label");
    trim(field.label);
    if (field.label.empty()) {
      field.label = field.name;
    }
    parse_size_t(attrs, {"byte", "offset", "start", "index"}, field.byte_offset);
    parse_size_t(attrs, {"bits", "bit_length"}, field.bit_length);
    parse_size_t(attrs, {"bit", "bit_offset"}, field.bit_offset);
    if (!parse_size_t(attrs, {"bytes", "length", "size"}, field.byte_length)) {
      if (field.bit_length > 0) {
        field.byte_length = (field.bit_offset + field.bit_length + 7) / 8;
      } else {
        std::string type = to_lower(attrs.get("type"));
        if (type.find("32") != std::string::npos) {
          field.byte_length = 4;
        } else if (type.find("16") != std::string::npos) {
          field.byte_length = 2;
        } else if (type.find("8") != std::string::npos || type == "u8") {
          field.byte_length = 1;
        }
      }
    }
    if (field.byte_length == 0) {
      field.byte_length = 2;
    }
    std::string endian = to_lower(attrs.get("endian"));
    if (endian == "little" || attrs.get("little_endian") == "1") {
      field.little_endian = true;
    }
    double scale = field.scale;
    if (parse_double(attrs, {"scale", "factor", "multiplier"}, scale)) {
      field.scale = scale;
    }
    double divisor = 0.0;
    if (parse_double(attrs, {"divisor"}, divisor) && divisor != 0.0) {
      field.scale /= divisor;
    }
    mapping.fields.push_back(field);
    search_pos = tag_end + 1;
  }
  return !mapping.fields.empty();
}

bool parse_xml_mapping_content(const std::string &content, XmlMapping &mapping) {
  std::string lower = to_lower(content);
  std::size_t search_pos = 0;
  bool any_fields = false;
  while (true) {
    std::size_t frame_pos = lower.find("<frame", search_pos);
    if (frame_pos == std::string::npos) {
      break;
    }
    std::size_t tag_end = lower.find('>', frame_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    std::size_t close_pos = lower.find("</frame", tag_end);
    if (close_pos == std::string::npos) {
      break;
    }
    std::string tag_text = content.substr(frame_pos, tag_end - frame_pos + 1);
    auto attrs = parse_attributes(tag_text);
    std::string id = attrs.get("id");
    trim(id);
    auto *command = mapping_for_id(mapping, id);
    if (command == nullptr) {
      ESP_LOGW(TAG, "Unbekannter Frame-Typ '%s'", id.c_str());
      search_pos = close_pos + 7;
      continue;
    }
    std::string inner = content.substr(tag_end + 1, close_pos - tag_end - 1);
    if (parse_fields(inner, *command)) {
      any_fields = true;
    }
    search_pos = close_pos + 7;
  }
  mapping.valid = any_fields;
  return mapping.valid;
}

}  // namespace

bool load_xml_mapping(const std::string &path, XmlMapping &mapping) {
  std::string content;
  if (!load_file_content(path, content)) {
    ESP_LOGW(TAG, "XML Mapping %s konnte nicht geladen werden", path.c_str());
    mapping.valid = false;
    return false;
  }
  mapping = XmlMapping{};
  mapping.source_path = path;
  if (!parse_xml_mapping_content(content, mapping)) {
    ESP_LOGW(TAG, "XML Mapping %s enthält keine bekannten Felder", path.c_str());
    mapping.valid = false;
    return false;
  }
  return true;
}

namespace {
XmlMapping g_current_mapping{};
bool g_has_mapping = false;
}

bool load_xml_mapping(const std::string &path) {
  XmlMapping mapping;
  if (!load_xml_mapping(path, mapping)) {
    g_has_mapping = false;
    g_current_mapping = XmlMapping{};
    return false;
  }
  g_current_mapping = mapping;
  g_has_mapping = mapping.valid;
  return g_has_mapping;
}

const XmlMapping &get_loaded_xml_mapping() { return g_current_mapping; }

bool map_tr32(const std::vector<uint8_t> &decoded, const XmlMapping &mapping, MachineStats &out) {
  return map_fields(decoded, mapping.tr32_fields, out, "TR32");
}

bool map_tg43(const std::vector<uint8_t> &decoded, const XmlMapping &mapping, MachineStats &out) {
  return map_fields(decoded, mapping.tg43_fields, out, "TG43");
}

bool map_tgc0(const std::vector<uint8_t> &decoded, const XmlMapping &mapping, MachineStats &out) {
  return map_fields(decoded, mapping.tgc0_fields, out, "TGC0");
}

bool map_tr32(const std::vector<uint8_t> &decoded, MachineStats &out) {
  if (!g_has_mapping) {
    ESP_LOGW(TAG, "Kein XML-Mapping geladen");
    return false;
  }
  return map_tr32(decoded, g_current_mapping, out);
}

bool map_tg43(const std::vector<uint8_t> &decoded, MachineStats &out) {
  if (!g_has_mapping) {
    ESP_LOGW(TAG, "Kein XML-Mapping geladen");
    return false;
  }
  return map_tg43(decoded, g_current_mapping, out);
}

bool map_tgc0(const std::vector<uint8_t> &decoded, MachineStats &out) {
  if (!g_has_mapping) {
    ESP_LOGW(TAG, "Kein XML-Mapping geladen");
    return false;
  }
  return map_tgc0(decoded, g_current_mapping, out);
}

}  // namespace jutta_component
}  // namespace esphome

