#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace esphome {
namespace jutta_component {

struct XmlField {
  std::string name;
  std::string label;
  std::size_t byte_offset{0};
  std::size_t byte_length{0};
  std::size_t bit_offset{0};
  std::size_t bit_length{0};
  bool little_endian{false};
  double scale{1.0};
};

struct XmlCommandMapping {
  std::vector<XmlField> fields;
};

struct XmlMapping {
  bool valid{false};
  std::string source_path;
  XmlCommandMapping tr32_fields;
  XmlCommandMapping tg43_fields;
  XmlCommandMapping tgc0_fields;
};

struct StatValue {
  double value{0.0};
  std::string label;
};

class MachineStats {
 public:
  void clear();
  bool empty() const;
  void set_value(const std::string &name, double value, const std::string &label);
  bool has_value(const std::string &name) const;
  const std::unordered_map<std::string, StatValue> &values() const;

 private:
  std::unordered_map<std::string, StatValue> values_{};
};

bool load_xml_mapping(const std::string &path, XmlMapping &mapping);
bool load_xml_mapping(const std::string &path);
const XmlMapping &get_loaded_xml_mapping();

bool map_tr32(const std::vector<uint8_t> &decoded, const XmlMapping &mapping, MachineStats &out);
bool map_tg43(const std::vector<uint8_t> &decoded, const XmlMapping &mapping, MachineStats &out);
bool map_tgc0(const std::vector<uint8_t> &decoded, const XmlMapping &mapping, MachineStats &out);

bool map_tr32(const std::vector<uint8_t> &decoded, MachineStats &out);
bool map_tg43(const std::vector<uint8_t> &decoded, MachineStats &out);
bool map_tgc0(const std::vector<uint8_t> &decoded, MachineStats &out);

}  // namespace jutta_component
}  // namespace esphome

