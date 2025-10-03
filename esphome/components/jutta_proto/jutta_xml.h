#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace uart {
class UARTComponent;
}
}  // namespace esphome

namespace jutta_xml {

bool send_db_cmd(esphome::uart::UARTComponent &uart, const char *ascii_at);
bool read_db_frame(esphome::uart::UARTComponent &uart, std::vector<uint8_t> &decoded, uint32_t timeout_ms);
bool query_TR32(esphome::uart::UARTComponent &uart, std::vector<uint16_t> &out10);
bool query_TG43(esphome::uart::UARTComponent &uart, std::vector<uint16_t> &out6);
bool query_TGC0(esphome::uart::UARTComponent &uart, uint32_t &a, uint32_t &b, uint32_t &c);

}  // namespace jutta_xml

