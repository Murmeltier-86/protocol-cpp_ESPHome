#include "esphome/components/jutta_proto/xml_errors.hpp"

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

static const char *const TAG = "jutta_proto.errors";

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

void for_each_error_tag(const std::string &block, const std::function<void(const std::string &)> &callback) {
  std::string lower = to_lower_copy(block);
  std::size_t pos = 0;
  while (true) {
    auto tag_pos = lower.find("<error", pos);
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

bool parse_uint32(const std::string &value, uint32_t &out) {
  if (value.empty()) {
    return false;
  }
  char *end = nullptr;
  auto parsed = std::strtoul(value.c_str(), &end, 0);
  if (end == value.c_str()) {
    return false;
  }
  out = static_cast<uint32_t>(parsed);
  return true;
}

std::vector<ErrorDesc> g_errors;
std::string g_error_source_cmd;

}  // namespace

bool load_errors_from_xml(const std::string &xml_content) {
  g_errors.clear();
  g_error_source_cmd.clear();
  std::string lower = to_lower_copy(xml_content);
  std::string needle = "<errors";
  auto pos = lower.find(needle);
  if (pos != std::string::npos) {
    auto tag_end = lower.find('>', pos);
    if (tag_end != std::string::npos) {
      std::string tag_text = xml_content.substr(pos, tag_end - pos + 1);
      auto attrs = parse_attributes(tag_text);
      g_error_source_cmd = attrs.get("source_cmd");
      trim(g_error_source_cmd);
    }
  }
  std::string block;
  if (!extract_section(xml_content, "errors", block)) {
    ESP_LOGW(TAG, "XML enthält keinen <Errors>-Block");
    block = xml_content;
  }
  for_each_error_tag(block, [](const std::string &tag_text) {
    auto attrs = parse_attributes(tag_text);
    ErrorDesc desc;
    if (!parse_uint32(attrs.get("code"), desc.code)) {
      ESP_LOGW(TAG, "Error ohne gültigen Code ignoriert");
      return;
    }
    desc.text = attrs.get("text");
    trim(desc.text);
    desc.severity = attrs.get("severity");
    trim(desc.severity);
    g_errors.push_back(desc);
  });
  std::sort(g_errors.begin(), g_errors.end(), [](const ErrorDesc &a, const ErrorDesc &b) { return a.code < b.code; });
  g_errors.erase(std::unique(g_errors.begin(), g_errors.end(),
                             [](const ErrorDesc &a, const ErrorDesc &b) { return a.code == b.code; }),
                 g_errors.end());
  return !g_errors.empty();
}

const ErrorDesc *find_error(uint32_t code) {
  auto it = std::lower_bound(g_errors.begin(), g_errors.end(), code,
                             [](const ErrorDesc &err, uint32_t value) { return err.code < value; });
  if (it != g_errors.end() && it->code == code) {
    return &*it;
  }
  return nullptr;
}

const std::vector<ErrorDesc> &all_errors() { return g_errors; }

const std::string &error_source_command() { return g_error_source_cmd; }

}  // namespace jutta_component
}  // namespace esphome

