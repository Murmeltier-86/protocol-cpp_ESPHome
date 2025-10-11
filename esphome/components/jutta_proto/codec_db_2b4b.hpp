#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace jutta_proto {

class DbCodec2B4B {
 public:
  struct DecodeResult {
    bool success{false};
    size_t consumed{0};
    bool msb_first{true};
    uint8_t xor_key{0x00};
    float printable_ratio{0.0f};
    std::string ascii{};
  };

  DbCodec2B4B() = default;

  void reset();
  bool has_detection() const { return this->have_detection_; }
  bool msb_first() const { return this->msb_first_; }
  uint8_t xor_key() const { return this->xor_key_; }
  void update_detection(bool msb_first, uint8_t xor_key);

  static bool decode_best(const std::vector<uint8_t> &symbols, DecodeResult &result);
  static std::vector<uint8_t> encode(const uint8_t *data, size_t length, bool msb_first, uint8_t xor_key);
  static std::vector<uint8_t> encode(const std::vector<uint8_t> &data, bool msb_first, uint8_t xor_key);
  static std::vector<uint8_t> encode(const std::string &text, bool msb_first, uint8_t xor_key);

 private:
  bool have_detection_{false};
  bool msb_first_{true};
  uint8_t xor_key_{0x00};
};

}  // namespace jutta_proto

