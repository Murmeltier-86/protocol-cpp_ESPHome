#include "codec_db_2b4b.hpp"

#include <algorithm>
#include <cctype>

namespace jutta_proto {
namespace {
static constexpr uint8_t CODEWORDS[4] = {0xFF, 0xDF, 0xFB, 0xDB};
static constexpr uint8_t XOR_KEYS[3] = {0x00, 0xFF, 0xA5};
static constexpr float MIN_PRINTABLE_RATIO = 0.7f;
static constexpr size_t MAX_FRAME_BYTES = 1024;

int symbol_to_pair(uint8_t symbol) {
  for (int i = 0; i < 4; ++i) {
    if (symbol == CODEWORDS[i]) {
      return i;
    }
  }
  return -1;
}

float printable_ratio(const std::vector<uint8_t> &data) {
  if (data.empty()) {
    return 0.0f;
  }
  size_t printable = 0;
  for (uint8_t byte : data) {
    if (std::isprint(static_cast<int>(byte)) != 0 || byte == '\r' || byte == '\n' || byte == '\t') {
      ++printable;
    }
  }
  return static_cast<float>(printable) / static_cast<float>(data.size());
}

bool has_crlf(const std::vector<uint8_t> &data) {
  if (data.size() < 2) {
    return false;
  }
  for (size_t i = 0; i + 1 < data.size(); ++i) {
    if (data[i] == '\r' && data[i + 1] == '\n') {
      return true;
    }
  }
  return false;
}

int score_ascii(const std::vector<uint8_t> &data) {
  if (data.empty()) {
    return -1000;
  }
  int score = 0;
  for (uint8_t byte : data) {
    if (std::isprint(static_cast<int>(byte)) != 0 || byte == '\r' || byte == '\n' || byte == '\t') {
      ++score;
    }
  }
  if (has_crlf(data)) {
    score += 5;
  }
  if (!data.empty() && (data[0] == '@' || data[0] == '&')) {
    score += 5;
  }
  return score;
}

}  // namespace

void DbCodec2B4B::reset() {
  this->have_detection_ = false;
  this->msb_first_ = true;
  this->xor_key_ = 0x00;
}

void DbCodec2B4B::update_detection(bool msb_first, uint8_t xor_key) {
  this->have_detection_ = true;
  this->msb_first_ = msb_first;
  this->xor_key_ = xor_key;
}

bool DbCodec2B4B::decode_best(const std::vector<uint8_t> &symbols, DecodeResult &result) {
  result = {};
  if (symbols.size() < 4) {
    return false;
  }

  int best_score = -1000;
  bool found = false;

  for (int align = 0; align < 4; ++align) {
    if (symbols.size() <= static_cast<size_t>(align)) {
      break;
    }

    for (bool msb_first : {true, false}) {
      std::vector<uint8_t> raw;
      raw.reserve(MAX_FRAME_BYTES);
      size_t used_symbols = 0;
      bool invalid = false;

      for (size_t idx = static_cast<size_t>(align); idx + 3 < symbols.size() && raw.size() < MAX_FRAME_BYTES; idx += 4) {
        int p0 = symbol_to_pair(symbols[idx + 0]);
        int p1 = symbol_to_pair(symbols[idx + 1]);
        int p2 = symbol_to_pair(symbols[idx + 2]);
        int p3 = symbol_to_pair(symbols[idx + 3]);
        if ((p0 | p1 | p2 | p3) < 0) {
          invalid = true;
          break;
        }
        uint8_t decoded = 0;
        if (msb_first) {
          decoded = static_cast<uint8_t>((p0 << 6) | (p1 << 4) | (p2 << 2) | p3);
        } else {
          decoded = static_cast<uint8_t>((p3 << 6) | (p2 << 4) | (p1 << 2) | p0);
        }
        raw.push_back(decoded);
        used_symbols = (idx + 4) - static_cast<size_t>(align);
        if (raw.size() >= 2 && raw[raw.size() - 2] == '\r' && raw.back() == '\n') {
          break;
        }
      }

      if (invalid || raw.empty()) {
        continue;
      }

      for (uint8_t key : XOR_KEYS) {
        std::vector<uint8_t> ascii(raw.size());
        for (size_t i = 0; i < raw.size(); ++i) {
          ascii[i] = static_cast<uint8_t>(raw[i] ^ key);
        }

        if (!has_crlf(ascii)) {
          continue;
        }
        float ratio = printable_ratio(ascii);
        if (ratio < MIN_PRINTABLE_RATIO) {
          continue;
        }
        int score = score_ascii(ascii);
        if (score > best_score) {
          best_score = score;
          result.success = true;
          result.consumed = static_cast<size_t>(align) + used_symbols;
          result.msb_first = msb_first;
          result.xor_key = key;
          result.printable_ratio = ratio;
          result.ascii.assign(reinterpret_cast<const char *>(ascii.data()), ascii.size());
          found = true;
        }
      }
    }
  }

  return found;
}

std::vector<uint8_t> DbCodec2B4B::encode(const uint8_t *data, size_t length, bool msb_first, uint8_t xor_key) {
  std::vector<uint8_t> encoded;
  if (data == nullptr || length == 0) {
    return encoded;
  }
  encoded.reserve(length * 4);
  for (size_t i = 0; i < length; ++i) {
    uint8_t value = static_cast<uint8_t>(data[i] ^ xor_key);
    if (msb_first) {
      encoded.push_back(CODEWORDS[(value >> 6) & 0x03]);
      encoded.push_back(CODEWORDS[(value >> 4) & 0x03]);
      encoded.push_back(CODEWORDS[(value >> 2) & 0x03]);
      encoded.push_back(CODEWORDS[value & 0x03]);
    } else {
      encoded.push_back(CODEWORDS[value & 0x03]);
      encoded.push_back(CODEWORDS[(value >> 2) & 0x03]);
      encoded.push_back(CODEWORDS[(value >> 4) & 0x03]);
      encoded.push_back(CODEWORDS[(value >> 6) & 0x03]);
    }
  }
  return encoded;
}

std::vector<uint8_t> DbCodec2B4B::encode(const std::vector<uint8_t> &data, bool msb_first, uint8_t xor_key) {
  if (data.empty()) {
    return {};
  }
  return encode(data.data(), data.size(), msb_first, xor_key);
}

std::vector<uint8_t> DbCodec2B4B::encode(const std::string &text, bool msb_first, uint8_t xor_key) {
  if (text.empty()) {
    return {};
  }
  return encode(reinterpret_cast<const uint8_t *>(text.data()), text.size(), msb_first, xor_key);
}

}  // namespace jutta_proto

