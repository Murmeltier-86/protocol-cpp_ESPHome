#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace jutta_component {

struct ErrorDesc {
  uint32_t code{0};
  std::string text;
  std::string severity;
};

bool load_errors_from_xml(const std::string &xml_content);
const ErrorDesc *find_error(uint32_t code);
const std::vector<ErrorDesc> &all_errors();
const std::string &error_source_command();

}  // namespace jutta_component
}  // namespace esphome

