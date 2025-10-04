#pragma once

#include "esphome/components/jutta_proto/jura_mapping_embed.h"

#ifndef JURA_XML_MAPPING_INCLUDE
#define JURA_XML_MAPPING_INCLUDE "esphome/components/jutta_proto/jura_mapping_embed.xml"
#endif

namespace esphome {
namespace jutta_component {

constexpr const char *kXmlMappingIncludePath = JURA_XML_MAPPING_INCLUDE;

}  // namespace jutta_component
}  // namespace esphome

