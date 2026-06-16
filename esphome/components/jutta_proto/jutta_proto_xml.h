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
  std::size_t offset{0};
  std::size_t size{0};
  bool has_endian{false};
  bool little_endian{false};
  double scale{1.0};
  bool has_add{false};
  double add{0.0};
};

struct XmlCommandMapping {
  std::vector<XmlField> fields;
  bool empty() const { return this->fields.empty(); }
};

struct JuraDecodedField {
  std::string category;
  std::string key;
  std::string name;
  std::string raw_value;
  std::string decoded_text;
  float numeric_value{0.0f};
  bool has_numeric_value{false};
};

struct JuraProductDesc {
  uint32_t code{0};
  std::string name;
  std::string kind;
  bool active{true};
  bool double_product{false};
};

struct JuraStatusDesc {
  uint32_t value{0};
  std::string name;
  std::string title;
  std::string message;
  std::string accept_command;
  bool progress{false};
};

struct JuraAlertDesc {
  uint32_t bit{0};
  std::string name;
  std::string title;
  std::string message;
  std::string type;
  std::string process;
};

struct JuraProcessDesc {
  std::string type;
  std::string execute_command;
  std::string title;
  bool progress{false};
};

struct XmlMapping {
  bool valid{false};
  XmlCommandMapping tr32;
  XmlCommandMapping tg43;
  XmlCommandMapping tgc0;
  std::vector<JuraProductDesc> products;
  std::vector<JuraStatusDesc> status_values;
  std::vector<JuraAlertDesc> alerts;
  std::vector<JuraProcessDesc> processes;
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
bool decode_status_response(const std::string &response, const std::string &parser_branch,
                            std::vector<JuraDecodedField> &out);
bool decode_status_payload(const std::vector<uint8_t> &payload, const std::string &source,
                           std::vector<JuraDecodedField> &out);

}  // namespace jutta_component
}  // namespace esphome
