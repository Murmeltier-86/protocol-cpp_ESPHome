#include "jutta_xml.h"

#include <algorithm>

#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"

#include "jutta_config.h"

namespace {

constexpr uint8_t TERM[8] = {0xDF, 0xFF, 0xDB, 0xDB, 0xFB, 0xFB, 0xDB, 0xDB};

inline void db_encode(const char *s, std::vector<uint8_t> &out) {
  out.clear();
  for (const uint8_t *p = reinterpret_cast<const uint8_t *>(s); *p != 0; ++p) {
    uint8_t b = *p;
    if (b == 0xDB || b == 0xDF || b == 0xFB || b == 0xFF) {
      out.push_back(0xDB);
      out.push_back(static_cast<uint8_t>(b ^ 0x20));
    } else {
      out.push_back(b);
    }
  }
  out.insert(out.end(), std::begin(TERM), std::end(TERM));
}

inline uint16_t u16be(const std::vector<uint8_t> &v, size_t offset) {
  return static_cast<uint16_t>((static_cast<uint16_t>(v[offset]) << 8) |
                               static_cast<uint16_t>(v[offset + 1]));
}

inline uint32_t u32be(const std::vector<uint8_t> &v, size_t offset) {
  return (static_cast<uint32_t>(v[offset]) << 24) | (static_cast<uint32_t>(v[offset + 1]) << 16) |
         (static_cast<uint32_t>(v[offset + 2]) << 8) | static_cast<uint32_t>(v[offset + 3]);
}

}  // namespace

namespace jutta_xml {

bool send_db_cmd(esphome::uart::UARTComponent &uart, const char *ascii_at) {
  std::vector<uint8_t> encoded;
  db_encode(ascii_at, encoded);
  uart.write_array(encoded.data(), encoded.size());
  return true;
}

bool read_db_frame(esphome::uart::UARTComponent &uart, std::vector<uint8_t> &decoded, uint32_t timeout_ms) {
  std::vector<uint8_t> raw;
  raw.reserve(256);
  uint32_t start = esphome::millis();
  while (esphome::millis() - start < timeout_ms) {
    while (uart.available()) {
      uint8_t value;
      if (!uart.read_byte(&value)) {
        continue;
      }
      raw.push_back(value);
      start = esphome::millis();
      if (raw.size() >= std::size(TERM) &&
          std::equal(raw.end() - std::size(TERM), raw.end(), std::begin(TERM))) {
        raw.resize(raw.size() - std::size(TERM));
        decoded.clear();
        decoded.reserve(raw.size());
        for (size_t i = 0; i < raw.size();) {
          uint8_t b = raw[i++];
          if (b == 0xDB && i < raw.size()) {
            decoded.push_back(static_cast<uint8_t>(raw[i++] ^ 0x20));
          } else {
            decoded.push_back(b);
          }
        }
        return true;
      }
    }
    esphome::delay(1);
  }
  return false;
}

bool query_TR32(esphome::uart::UARTComponent &uart, std::vector<uint16_t> &out10) {
  out10.clear();
  if (!send_db_cmd(uart, "@TR:32")) {
    return false;
  }
  std::vector<uint8_t> decoded;
  if (!read_db_frame(uart, decoded, JUTTA_XML_RX_TIMEOUT_MS)) {
    return false;
  }
  if (decoded.size() != 21) {
    return false;
  }
  for (size_t i = 0; i < 10; ++i) {
    out10.push_back(u16be(decoded, 1 + i * 2));
  }
  return true;
}

bool query_TG43(esphome::uart::UARTComponent &uart, std::vector<uint16_t> &out6) {
  out6.clear();
  if (!send_db_cmd(uart, "@TG:43")) {
    return false;
  }
  std::vector<uint8_t> decoded;
  if (!read_db_frame(uart, decoded, JUTTA_XML_RX_TIMEOUT_MS)) {
    return false;
  }
  if (decoded.size() != 13) {
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    out6.push_back(u16be(decoded, 1 + i * 2));
  }
  return true;
}

bool query_TGC0(esphome::uart::UARTComponent &uart, uint32_t &a, uint32_t &b, uint32_t &c) {
  if (!send_db_cmd(uart, "@TG:C0")) {
    return false;
  }
  std::vector<uint8_t> decoded;
  if (!read_db_frame(uart, decoded, JUTTA_XML_RX_TIMEOUT_MS)) {
    return false;
  }
  if (decoded.size() != 13) {
    return false;
  }
  a = u32be(decoded, 1);
  b = u32be(decoded, 5);
  c = u32be(decoded, 9);
  return true;
}

}  // namespace jutta_xml

