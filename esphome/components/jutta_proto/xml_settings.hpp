#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace jutta_component {

enum class SettingValueType {
  U8,
  U16,
  U32,
  Bool,
  Enum,
  String,
};

struct SettingDesc {
  std::string id;
  std::string name;
  std::string unit;
  std::string source_cmd;
  std::size_t offset{0};
  std::size_t width{0};
  float scale{1.0f};
  SettingValueType type{SettingValueType::U8};
};

bool load_settings_from_xml(const std::string &xml_content);
const std::vector<SettingDesc> &get_settings();

}  // namespace jutta_component
}  // namespace esphome

