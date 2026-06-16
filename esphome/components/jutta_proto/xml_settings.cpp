#include "esphome/components/jutta_proto/xml_settings.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <regex>
#include <unordered_map>

#include "esphome/core/log.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto.settings";

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

std::string to_lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
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

AttributeMap parse_attributes(const std::string &tag_text) {
  AttributeMap map;
  static const std::regex attr_regex(R"(([A-Za-z0-9_:\-]+)\s*=\s*\"([^\"]*)\")");
  auto begin = std::sregex_iterator(tag_text.begin(), tag_text.end(), attr_regex);
  auto end = std::sregex_iterator();
  for (auto it = begin; it != end; ++it) {
    std::string key = to_lower_copy((*it)[1].str());
    std::string value = (*it)[2].str();
    trim(value);
    map.values[key] = value;
  }
  return map;
}

bool parse_size_t(const AttributeMap &attrs, const std::string &key, std::size_t &out) {
  std::string value = attrs.get(key);
  if (value.empty()) {
    return false;
  }
  char *end = nullptr;
  auto parsed = std::strtoul(value.c_str(), &end, 0);
  if (end == value.c_str()) {
    return false;
  }
  out = static_cast<std::size_t>(parsed);
  return true;
}

bool parse_float(const AttributeMap &attrs, const std::string &key, float &out) {
  std::string value = attrs.get(key);
  if (value.empty()) {
    return false;
  }
  char *end = nullptr;
  auto parsed = std::strtof(value.c_str(), &end);
  if (end == value.c_str()) {
    return false;
  }
  out = parsed;
  return true;
}

bool parse_type(const AttributeMap &attrs, SettingValueType &out) {
  std::string value = to_lower_copy(attrs.get("type"));
  if (value.empty()) {
    return false;
  }
  if (value == "u8" || value == "uint8" || value == "byte") {
    out = SettingValueType::U8;
    return true;
  }
  if (value == "u16" || value == "uint16") {
    out = SettingValueType::U16;
    return true;
  }
  if (value == "u32" || value == "uint32") {
    out = SettingValueType::U32;
    return true;
  }
  if (value == "bool" || value == "boolean") {
    out = SettingValueType::Bool;
    return true;
  }
  if (value == "enum") {
    out = SettingValueType::Enum;
    return true;
  }
  if (value == "str" || value == "string") {
    out = SettingValueType::String;
    return true;
  }
  ESP_LOGW(TAG, "Unbekannter Setting-Typ '%s'", value.c_str());
  return false;
}

bool parse_setting_tag(const std::string &tag_text, SettingDesc &out) {
  auto attrs = parse_attributes(tag_text);
  out = SettingDesc{};
  out.id = attrs.get("id");
  trim(out.id);
  if (out.id.empty()) {
    ESP_LOGW(TAG, "Setting ohne id ignoriert");
    return false;
  }
  out.name = attrs.get("name");
  trim(out.name);
  out.unit = attrs.get("unit");
  trim(out.unit);
  out.source_cmd = attrs.get("source_cmd");
  trim(out.source_cmd);
  out.path = attrs.get("path");
  trim(out.path);
  parse_size_t(attrs, "offset", out.offset);
  parse_size_t(attrs, "width", out.width);
  parse_float(attrs, "scale", out.scale);
  parse_type(attrs, out.type);
  if (out.width == 0) {
    switch (out.type) {
      case SettingValueType::U16:
        out.width = 2;
        break;
      case SettingValueType::U32:
        out.width = 4;
        break;
      default:
        out.width = 1;
        break;
    }
  }
  return true;
}

bool extract_section(const std::string &xml, const std::string &tag, std::string &out) {
  std::string lower = to_lower_copy(xml);
  std::string open = "<" + to_lower_copy(tag);
  auto start = lower.find(open);
  if (start == std::string::npos) {
    return false;
  }
  auto open_end = lower.find('>', start);
  if (open_end == std::string::npos) {
    return false;
  }
  auto close = lower.find("</" + to_lower_copy(tag), open_end);
  if (close == std::string::npos) {
    return false;
  }
  out = xml.substr(open_end + 1, close - open_end - 1);
  return true;
}

void for_each_setting_tag(const std::string &block, const std::function<void(const std::string &)> &callback) {
  std::string lower = to_lower_copy(block);
  std::size_t pos = 0;
  while (true) {
    auto tag_pos = lower.find("<setting", pos);
    if (tag_pos == std::string::npos) {
      break;
    }
    auto tag_end = lower.find('>', tag_pos);
    if (tag_end == std::string::npos) {
      break;
    }
    callback(block.substr(tag_pos, tag_end - tag_pos + 1));
    pos = tag_end + 1;
  }
}

std::vector<SettingDesc> g_settings;
bool g_settings_loaded = false;

}  // namespace

bool load_settings_from_xml(const std::string &xml_content) {
  g_settings.clear();
  g_settings_loaded = true;
  std::string settings_block;
  if (!extract_section(xml_content, "settings", settings_block)) {
    ESP_LOGW(TAG, "XML enthält keinen <Settings>-Block");
    settings_block = xml_content;
  }
  for_each_setting_tag(settings_block, [](const std::string &tag) {
    SettingDesc desc;
    if (parse_setting_tag(tag, desc)) {
      g_settings.push_back(desc);
    }
  });
  return !g_settings.empty();
}

const std::vector<SettingDesc> &get_settings() { return g_settings; }

}  // namespace jutta_component
}  // namespace esphome

