#include "esphome/components/jutta_proto/jura_mapping_embed_impl.h"

#include <cstddef>
#include <cstdint>

extern "C" {
extern const std::uint8_t jura_xml_mapping_start[];
extern const std::uint8_t jura_xml_mapping_end[];
}

asm(
    ".section .rodata\n"
    ".global jura_xml_mapping_start\n"
    "jura_xml_mapping_start:\n"
    ".incbin " JURA_XML_MAPPING_INCLUDE "\n"
    ".global jura_xml_mapping_end\n"
    "jura_xml_mapping_end:\n"
    ".byte 0\n");

namespace esphome {
namespace jutta_component {

const char *get_joe_xml_data() {
  return reinterpret_cast<const char *>(jura_xml_mapping_start);
}

std::size_t get_joe_xml_length() {
  return static_cast<std::size_t>(jura_xml_mapping_end - jura_xml_mapping_start);
}

}  // namespace jutta_component
}  // namespace esphome
