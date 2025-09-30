#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace esphome {
namespace jutta_component {

struct MachineDataNode {
  std::string name;
  std::string text;
  std::vector<std::pair<std::string, std::string>> attributes;
  std::vector<MachineDataNode> children;

  const MachineDataNode *find_child_case_insensitive(const std::string &name) const;
  std::optional<std::string> get_attribute_case_insensitive(const std::string &name) const;
  std::string collect_text_content() const;
};

class MachineDataParser {
 public:
  static std::optional<MachineDataNode> parse(const std::string &input);
};

std::string format_machine_data_tree(const MachineDataNode &node);
std::string format_machine_data_section(const MachineDataNode *node);

}  // namespace jutta_component
}  // namespace esphome

