#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace esphome {
namespace jutta_component {

struct XmlField {
  enum class SensorKind { None, Measurement, TotalIncreasing };

  std::string name;
  std::string label;
  std::size_t offset{0};
  std::size_t size{0};
  bool little_endian{false};
  double scale{1.0};
  bool publish_sensor{false};
  SensorKind sensor_kind{SensorKind::None};
  bool has_min{false};
  double min_value{0.0};
  bool has_max{false};
  double max_value{0.0};
  bool has_accuracy{false};
  int accuracy_decimals{0};
  std::string unit;
  std::string unique_id;
};

struct XmlCommandMapping {
  std::vector<XmlField> fields;
  bool empty() const { return this->fields.empty(); }
};

struct XmlMapping {
  bool valid{false};
  XmlCommandMapping tr32;
  XmlCommandMapping tg43;
  XmlCommandMapping tgc0;
};

struct StatValue {
  double value{0.0};
  std::string label;
};

class Stats {
 public:
  void clear();
  bool empty() const;
  void set_value(const std::string &name, double value, const std::string &label);
  bool has_value(const std::string &name) const;
  const std::unordered_map<std::string, StatValue> &values() const;

 private:
  std::unordered_map<std::string, StatValue> values_{};
};

bool load_mapping_from_string(const std::string &xml);
const XmlMapping &get_xml_mapping();

bool parse_TR32(const std::vector<uint8_t> &decoded, Stats &out);
bool parse_TG43(const std::vector<uint8_t> &decoded, Stats &out);
bool parse_TGC0(const std::vector<uint8_t> &decoded, Stats &out);

}  // namespace jutta_component
}  // namespace esphome

