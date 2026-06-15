#include "esphome/components/jutta_proto/jutta_proto.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <unordered_set>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/core/time.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;
constexpr uint32_t MACHINE_DATA_QUERY_INTERVAL_MS = 30000;
constexpr uint32_t MACHINE_XML_TIMEOUT_MS = 1500;
constexpr uint32_t MACHINE_XML_BUSY_BACKOFF_MS = 5000;
constexpr std::size_t MACHINE_XML_MIN_LENGTH = 32;
const char *const MACHINE_XML_PRIMARY_COMMAND = "@hr:00\r\n";
const char *const MACHINE_XML_FALLBACK_COMMAND = "@hr:05\r\n";
constexpr uint32_t kXmlRxTimeoutMs = 5000;
constexpr uint32_t kStatsRxCaptureWindowMs = 1500;
constexpr uint32_t kPostGateStatsTimeoutMs = 5000;
constexpr uint32_t kPostGateControlTimeoutMs = 5000;
constexpr uint32_t kTabletSeqRxWindowMs = 1500;
constexpr size_t kTabletSeqMaxRxBytes = 256;
constexpr uint32_t kInterCmdGapMs = 250;
constexpr uint32_t kStatsNextCommandDelayMs = 0;
constexpr uint32_t kXmlQuietMs = 120;
constexpr uint32_t kCycleSleepMs = 2000;
constexpr uint8_t kTr32PageCount = 16;
constexpr uint8_t kTr32ProductsPerPage = 4;
constexpr uint8_t kTr32BytesPerProduct = 2;
constexpr uint8_t kStatsMaxConsecutiveFailures = 4;
constexpr uint32_t kDongleStartupProbeDelayMs = 250;
constexpr uint32_t kDongleStartupTimeoutMs = 1500;
constexpr uint32_t kDongleStartupT0AfterT3TimeoutMs = 5000;
constexpr uint32_t kDongleStartupT3QuietMs = 1000;
constexpr uint32_t kDongleStartupGateOnlyQuietMs = 2000;
constexpr uint32_t kDongleStartupMaxWaitAfterT3Ms = 5000;
constexpr uint32_t kDongleStartupTr37TimeoutMs = 3000;
constexpr size_t kDongleStartupMaxRxBytes = 512;
constexpr uint8_t kDongleStartupMaxProbeAttempts = 6;
constexpr uint8_t kDongleStartupMaxT1Attempts = 6;
constexpr uint8_t kDongleStartupMaxTr37Attempts = 3;
constexpr uint32_t DONGLE_EVENT_TY = 0x01;
constexpr uint32_t DONGLE_EVENT_T0 = 0x02;
constexpr uint32_t DONGLE_EVENT_T1 = 0x04;
constexpr uint32_t DONGLE_EVENT_T2 = 0x08;
constexpr uint32_t DONGLE_EVENT_T3 = 0x10;
constexpr uint32_t DONGLE_EVENT_TR = 0x40;
constexpr uint32_t DONGLE_EVENT_TF = 0x80;
constexpr uint32_t DONGLE_STARTUP_CLEAR_MASK =
    DONGLE_EVENT_TY | DONGLE_EVENT_T0 | DONGLE_EVENT_T1 | DONGLE_EVENT_T2 | DONGLE_EVENT_T3 | DONGLE_EVENT_TR;
constexpr uint32_t DONGLE_STARTUP_READY_MASK = DONGLE_EVENT_T2 | DONGLE_EVENT_T3 | DONGLE_EVENT_TR;
constexpr uint8_t INNER_UART_MODE0_A[16] = {0x08, 0x0E, 0x0C, 0x04, 0x03, 0x0D, 0x0A, 0x0B,
                                            0x00, 0x0F, 0x06, 0x07, 0x02, 0x05, 0x01, 0x09};
constexpr uint8_t INNER_UART_MODE0_B[16] = {0x04, 0x0B, 0x0D, 0x0A, 0x00, 0x07, 0x0F, 0x05,
                                            0x09, 0x08, 0x03, 0x01, 0x0E, 0x02, 0x0C, 0x06};
constexpr uint8_t INNER_BLE2_A[16] = {14, 4, 3, 2, 1, 13, 8, 11, 6, 15, 12, 7, 10, 5, 0, 9};
constexpr uint8_t INNER_BLE2_B[16] = {10, 6, 13, 12, 14, 11, 1, 9, 15, 7, 0, 5, 3, 2, 4, 8};
constexpr uint8_t kBle2ProbeKey = 0xA7;
constexpr uint32_t kSettingsRefreshMs = 600000;
constexpr uint32_t kErrorPollIntervalMs = 5000;
constexpr uint32_t kCommandTimeoutMs = 1500;
constexpr uint32_t kStatusProbeTimeoutMs = 5000;
constexpr uint32_t kBle2ProbeTimeoutMs = 5000;
constexpr uint32_t kDebugCommandTimeoutMs = 5000;
constexpr size_t kDebugCommandMaxLength = 80;

constexpr double XML_COUNTER_MIN = 0.0;
constexpr double XML_COUNTER_MAX = 1'000'000.0;
constexpr double XML_MEASUREMENT_MIN = 0.0;
constexpr double XML_MEASUREMENT_MAX = 250.0;
constexpr float XML_COUNTER_TOLERANCE = 0.5f;
constexpr float XML_MEASUREMENT_TOLERANCE = 0.1f;
constexpr bool TGC0_TRY_LITTLE_ENDIAN_FIRST = true;
constexpr std::size_t kTR32MinFrameLength = 21;
constexpr std::size_t kTG43MinFrameLength = 13;
constexpr std::size_t kTGC0MinFrameLength = 13;


template<typename T>
auto set_sensor_entity_category_if_supported(T *sensor, EntityCategory category)
    -> decltype(sensor->set_entity_category(category), void()) {
  sensor->set_entity_category(category);
}

inline void set_sensor_entity_category_if_supported(...) {}

template<typename T>
auto set_sensor_unit_if_supported(T *sensor, const char *unit)
    -> decltype(sensor->set_unit_of_measurement(unit), void()) {
  sensor->set_unit_of_measurement(unit);
}

inline void set_sensor_unit_if_supported(...) {}

template<typename T>
auto set_sensor_icon_if_supported(T *sensor, const char *icon)
    -> decltype(sensor->set_icon(icon), void()) {
  sensor->set_icon(icon);
}

inline void set_sensor_icon_if_supported(...) {}


std::string sanitize_text_for_api(const std::string &input) {
  std::string out;
  out.reserve(input.size());
  constexpr char kHex[] = "0123456789ABCDEF";
  for (unsigned char c : input) {
    if (c >= 0x20 && c <= 0x7E) {
      out.push_back(static_cast<char>(c));
      continue;
    }
    if (c == '\t') {
      out.push_back('\t');
      continue;
    }
    out.push_back('\\');
    out.push_back('x');
    out.push_back(kHex[(c >> 4) & 0x0F]);
    out.push_back(kHex[c & 0x0F]);
  }
  return out;
}

std::string escape_control_text_for_log(const std::string &input) {
  std::string out;
  out.reserve(input.size() + 8);
  constexpr char kHex[] = "0123456789ABCDEF";
  for (unsigned char c : input) {
    if (c == '\r') {
      out.append("\\r");
    } else if (c == '\n') {
      out.append("\\n");
    } else if (c == '\t') {
      out.append("\\t");
    } else if (c >= 0x20 && c <= 0x7E) {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('\\');
      out.push_back('x');
      out.push_back(kHex[(c >> 4) & 0x0F]);
      out.push_back(kHex[c & 0x0F]);
    }
  }
  return out;
}

bool has_binary_bytes(const std::string &input) {
  for (unsigned char c : input) {
    if (c == '\r' || c == '\n' || c == '\t') {
      continue;
    }
    if (c < 0x20 || c == 0x7F || c >= 0x80) {
      return true;
    }
  }
  return false;
}

bool is_inner_transport_start(uint8_t byte) {
  return byte == '&' || byte == '*' || byte == '+';
}

bool is_inner_transport_reserved_byte(uint8_t byte) {
  return byte == '\n' || byte == '\r' || byte == '*' || byte == '+' || byte == '&' || byte == 0x1B || byte == 0x00;
}

std::string compact_hex_string(const std::string &value, size_t max_bytes = 64) {
  std::ostringstream stream;
  size_t count = std::min(value.size(), max_bytes);
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
           << static_cast<int>(static_cast<unsigned char>(value[i]));
  }
  if (value.size() > max_bytes) {
    stream << " ...";
  }
  return stream.str();
}

std::string format_uint8_list(const std::string &value, size_t max_items = 24) {
  std::ostringstream stream;
  size_t count = std::min(value.size(), max_items);
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) {
      stream << ',';
    }
    stream << static_cast<unsigned>(static_cast<uint8_t>(value[i]));
  }
  if (value.size() > max_items) {
    stream << ",...";
  }
  return stream.str();
}

std::string format_u16_pairs(const std::string &value, bool big_endian, size_t max_pairs = 12) {
  std::ostringstream stream;
  size_t pair_count = std::min(value.size() / 2, max_pairs);
  for (size_t i = 0; i < pair_count; ++i) {
    if (i > 0) {
      stream << ',';
    }
    uint8_t first = static_cast<uint8_t>(value[i * 2]);
    uint8_t second = static_cast<uint8_t>(value[i * 2 + 1]);
    uint16_t number = big_endian ? static_cast<uint16_t>((first << 8) | second)
                                 : static_cast<uint16_t>((second << 8) | first);
    stream << static_cast<unsigned>(number);
  }
  if (value.size() / 2 > max_pairs) {
    stream << ",...";
  }
  if ((value.size() & 1U) != 0U) {
    if (pair_count > 0) {
      stream << ',';
    }
    stream << "odd:" << static_cast<unsigned>(static_cast<uint8_t>(value.back()));
  }
  return stream.str();
}

std::string format_nibble_dump(const std::string &value, size_t max_bytes = 24) {
  static constexpr char HEX[] = "0123456789ABCDEF";
  std::string out;
  size_t count = std::min(value.size(), max_bytes);
  out.reserve(count * 4 + 4);
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) {
      out.push_back(' ');
    }
    uint8_t byte = static_cast<uint8_t>(value[i]);
    out.push_back(HEX[(byte >> 4) & 0x0F]);
    out.push_back('/');
    out.push_back(HEX[byte & 0x0F]);
  }
  if (value.size() > max_bytes) {
    out.append(" ...");
  }
  return out;
}

std::string format_bcd_like(const std::string &value, size_t max_bytes = 24) {
  static constexpr char HEX[] = "0123456789ABCDEF";
  std::string out;
  size_t count = std::min(value.size(), max_bytes);
  out.reserve(count * 3 + 4);
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) {
      out.push_back(' ');
    }
    uint8_t byte = static_cast<uint8_t>(value[i]);
    uint8_t hi = (byte >> 4) & 0x0F;
    uint8_t lo = byte & 0x0F;
    out.push_back(hi <= 9 ? static_cast<char>('0' + hi) : HEX[hi]);
    out.push_back(lo <= 9 ? static_cast<char>('0' + lo) : HEX[lo]);
  }
  if (value.size() > max_bytes) {
    out.append(" ...");
  }
  return out;
}

struct InnerBinaryProbePayload {
  uint8_t key{0};
  bool key_escaped{false};
  size_t esc_count{0};
  std::string raw_payload{};
  std::string unescaped_payload{};
  std::string reason{};
};

InnerBinaryProbePayload extract_current_inner_binary_payload(const std::string &frame) {
  InnerBinaryProbePayload result;
  if (frame.size() < 2 || !is_inner_transport_start(static_cast<uint8_t>(frame.front()))) {
    result.reason = "unsupported_frame";
    return result;
  }

  size_t index = 1;
  if (static_cast<uint8_t>(frame[index]) == 0x1B) {
    result.key_escaped = true;
    result.esc_count += 1;
    ++index;
    if (index >= frame.size()) {
      result.reason = "truncated_key_escape";
      return result;
    }
  }
  result.key = static_cast<uint8_t>(frame[index]);
  if (result.key_escaped) {
    result.key &= 0x7F;
  }
  ++index;

  result.raw_payload.reserve(frame.size());
  result.unescaped_payload.reserve(frame.size());
  while (index < frame.size()) {
    uint8_t byte = static_cast<uint8_t>(frame[index]);
    if (byte == '\r' || byte == '\n') {
      break;
    }
    result.raw_payload.push_back(static_cast<char>(byte));
    if (byte == 0x1B) {
      result.esc_count += 1;
      ++index;
      if (index >= frame.size()) {
        result.reason = "truncated_payload_escape";
        return result;
      }
      uint8_t escaped = static_cast<uint8_t>(frame[index]);
      result.raw_payload.push_back(static_cast<char>(escaped));
      result.unescaped_payload.push_back(static_cast<char>(escaped & 0x7F));
      ++index;
      continue;
    }
    result.unescaped_payload.push_back(static_cast<char>(byte));
    ++index;
  }
  return result;
}

bool extract_crlf_line(std::string &buffer, std::string &line) {
  auto terminator = buffer.find("\r\n");
  if (terminator == std::string::npos) {
    return false;
  }
  line = buffer.substr(0, terminator);
  buffer.erase(0, terminator + 2);
  return true;
}

size_t first_raw_crlf_len(const std::string &buffer) {
  auto terminator = buffer.find("\r\n");
  return terminator == std::string::npos ? 0 : terminator;
}

bool has_unescaped_inner_transport_cr(const std::string &buffer) {
  bool escaped = false;
  for (unsigned char c : buffer) {
    if (escaped) {
      escaped = false;
      continue;
    }
    if (c == 0x1B) {
      escaped = true;
      continue;
    }
    if (c == '\r') {
      return true;
    }
  }
  return false;
}

struct ProbeRxLine {
  std::string data{};
  bool complete{false};
};

std::vector<ProbeRxLine> split_probe_rx_lines(const std::string &buffer) {
  std::vector<ProbeRxLine> lines;
  size_t line_start = 0;
  bool escaped = false;
  for (size_t i = 0; i < buffer.size(); ++i) {
    uint8_t byte = static_cast<uint8_t>(buffer[i]);
    if (escaped) {
      escaped = false;
      continue;
    }
    if (byte == 0x1B) {
      escaped = true;
      continue;
    }
    if (byte == '\r') {
      lines.push_back({buffer.substr(line_start, i - line_start), true});
      if (i + 1 < buffer.size() && static_cast<uint8_t>(buffer[i + 1]) == '\n') {
        ++i;
      }
      line_start = i + 1;
    }
  }
  if (line_start < buffer.size()) {
    lines.push_back({buffer.substr(line_start), false});
  }
  return lines;
}

std::vector<std::string> split_inner_transport_frames(const std::string &buffer) {
  std::vector<std::string> frames;
  size_t frame_start = std::string::npos;
  bool escaped = false;
  for (size_t i = 0; i < buffer.size(); ++i) {
    uint8_t byte = static_cast<uint8_t>(buffer[i]);
    if (frame_start == std::string::npos) {
      if (is_inner_transport_start(byte)) {
        frame_start = i;
        escaped = false;
      }
      continue;
    }
    if (escaped) {
      escaped = false;
      continue;
    }
    if (byte == 0x1B) {
      escaped = true;
      continue;
    }
    if (byte == '\r' && i + 1 < buffer.size() && static_cast<uint8_t>(buffer[i + 1]) == '\n') {
      if (i > frame_start) {
        frames.push_back(buffer.substr(frame_start, i - frame_start));
      }
      frame_start = std::string::npos;
      escaped = false;
      ++i;
    }
  }
  if (frame_start != std::string::npos && frame_start < buffer.size()) {
    frames.push_back(buffer.substr(frame_start));
  }
  return frames;
}

std::string lower_trimmed_transport_payload(const std::string &line) {
  std::string lower = line;
  while (!lower.empty() && (lower.back() == '\r' || lower.back() == '\n' || lower.back() == ' ' ||
                            lower.back() == '\t')) {
    lower.pop_back();
  }
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower;
}

bool payload_starts_with_ci(const std::string &line, const std::string &prefix) {
  std::string lower = lower_trimmed_transport_payload(line);
  std::string expected = prefix;
  std::transform(expected.begin(), expected.end(), expected.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower.rfind(expected, 0) == 0;
}

std::string transport_payload_log_text(const std::string &line) {
  std::string trimmed = line;
  while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n' || trimmed.back() == ' ' ||
                             trimmed.back() == '\t')) {
    trimmed.pop_back();
  }
  return sanitize_text_for_api(trimmed);
}

const char *classify_decoded_inner_response(const std::string &line) {
  std::string lower = lower_trimmed_transport_payload(line);
  if (lower.rfind("@tr", 0) == 0) {
    return "tr_response";
  }
  if (lower.rfind("@tg", 0) == 0) {
    return "tg_response";
  }
  if (lower.rfind("@ts", 0) == 0 || lower.rfind("ok", 0) == 0) {
    return "control_response";
  }
  if (lower.rfind("@tf", 0) == 0) {
    return "tf_status";
  }
  if (lower.rfind("@tv", 0) == 0) {
    return "tv_progress";
  }
  return "unknown";
}

bool is_stats_ascii_response(const std::string &line) {
  const char *decoded_class = classify_decoded_inner_response(line);
  return std::strcmp(decoded_class, "unknown") != 0;
}

std::string expected_stats_prefix_for_command(const std::string &command) {
  std::string lower = command;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lower.rfind("@tr:32", 0) == 0) {
    return "@tr:32";
  }
  if (lower.rfind("@tg:43", 0) == 0) {
    return "@tg:43";
  }
  if (lower.rfind("@tg:c0", 0) == 0) {
    return "@tg:c0";
  }
  if (lower.rfind("@ts:", 0) == 0) {
    return "@ts";
  }
  return "";
}

bool is_printable_transport_payload(const std::string &line) {
  if (line.empty()) {
    return false;
  }
  for (unsigned char c : line) {
    if (c == '\r' || c == '\n' || c == '\t') {
      continue;
    }
    if (c < 0x20 || c >= 0x7F) {
      return false;
    }
  }
  return true;
}

uint8_t printable_ratio_percent(const std::string &line) {
  if (line.empty()) {
    return 0;
  }
  size_t printable = 0;
  for (unsigned char c : line) {
    if ((c >= 0x20 && c <= 0x7E) || c == '\r' || c == '\n' || c == '\t') {
      ++printable;
    }
  }
  return static_cast<uint8_t>((printable * 100U) / line.size());
}

std::string printable_preview(const std::string &line, size_t max_bytes = 32) {
  std::string out;
  size_t count = std::min(line.size(), max_bytes);
  out.reserve(count + (line.size() > max_bytes ? 4 : 0));
  for (size_t i = 0; i < count; ++i) {
    unsigned char c = static_cast<unsigned char>(line[i]);
    out.push_back(c >= 0x20 && c <= 0x7E ? static_cast<char>(c) : '.');
  }
  if (line.size() > max_bytes) {
    out.append("...");
  }
  return out;
}

std::string printable_or_dot_ascii(const std::string &line, size_t max_bytes = 80) {
  return printable_preview(line, max_bytes);
}

bool matches_known_repeated_tr32_frame(const std::string &frame) {
  static constexpr uint8_t KNOWN_FRAME[] = {
      0x26, 0x3D, 0x29, 0xE1, 0xBE, 0xE8, 0x53, 0x2F, 0xE6, 0x50, 0xFC,
      0x47, 0x07, 0xFF, 0xA5, 0x04, 0xDE, 0xA3, 0xD1, 0x1B, 0xA6};
  if (frame.size() != sizeof(KNOWN_FRAME)) {
    return false;
  }
  for (size_t i = 0; i < sizeof(KNOWN_FRAME); ++i) {
    if (static_cast<uint8_t>(frame[i]) != KNOWN_FRAME[i]) {
      return false;
    }
  }
  return true;
}

bool payload_starts_with_at(const std::string &line) { return !line.empty() && line.front() == '@'; }
bool payload_starts_with_tr(const std::string &line) {
  std::string lower = line.substr(0, std::min<size_t>(3, line.size()));
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower == "@tr";
}
bool payload_starts_with_tg(const std::string &line) {
  std::string lower = line.substr(0, std::min<size_t>(3, line.size()));
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower == "@tg";
}
bool payload_starts_with_ts(const std::string &line) {
  std::string lower = line.substr(0, std::min<size_t>(3, line.size()));
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower == "@ts";
}
bool payload_starts_with_ok(const std::string &line) {
  std::string lower = line.substr(0, std::min<size_t>(2, line.size()));
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower == "ok";
}
bool payload_starts_with_tf(const std::string &line) {
  std::string lower = line.substr(0, std::min<size_t>(3, line.size()));
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower == "@tf";
}

uint8_t fw_mod8(int value) {
  value %= 256;
  if (value < 0) {
    value += 256;
  }
  return static_cast<uint8_t>(value);
}

uint8_t fw_mod4(int value) {
  return static_cast<uint8_t>(fw_mod8(value) & 0x0F);
}

uint8_t fw_nibble_transform(uint8_t nibble, uint8_t position, uint8_t key_hi, uint8_t key_lo,
                            const uint8_t table_a[16], const uint8_t table_b[16]) {
  int pos = position;
  int pos_hi = position >> 4;
  int first = fw_mod4(nibble + pos + key_hi);
  int second = fw_mod4((fw_mod8(pos_hi) + table_a[first] + key_lo) - pos - key_hi);
  int third = fw_mod4((table_b[second] + key_hi + pos) - key_lo - fw_mod8(pos_hi));
  return fw_mod4(table_a[third] - pos - key_hi);
}

bool find_inner_encoded_nibble_uart_mode0(uint8_t plain_nibble, uint8_t position, uint8_t key_hi, uint8_t key_lo,
                                          uint8_t &encoded_nibble) {
  for (uint8_t candidate = 0; candidate < 16; ++candidate) {
    if (fw_nibble_transform(candidate, position, key_hi, key_lo, INNER_UART_MODE0_A, INNER_UART_MODE0_B) ==
        (plain_nibble & 0x0F)) {
      encoded_nibble = candidate;
      return true;
    }
  }
  return false;
}

bool find_inner_encoded_nibble_with_tables(uint8_t plain_nibble, uint8_t position, uint8_t key_hi, uint8_t key_lo,
                                           const uint8_t table_a[16], const uint8_t table_b[16],
                                           uint8_t &encoded_nibble) {
  for (uint8_t candidate = 0; candidate < 16; ++candidate) {
    if (fw_nibble_transform(candidate, position, key_hi, key_lo, table_a, table_b) == (plain_nibble & 0x0F)) {
      encoded_nibble = candidate;
      return true;
    }
  }
  return false;
}

void append_inner_transport_escaped_byte(std::vector<uint8_t> &encoded, uint8_t byte) {
  if (is_inner_transport_reserved_byte(byte)) {
    encoded.push_back(0x1B);
    encoded.push_back(byte | 0x80);
  } else {
    encoded.push_back(byte);
  }
}

bool encode_inner_transport_uart_mode0(const std::string &plain, uint8_t key, std::vector<uint8_t> &encoded) {
  encoded.clear();
  encoded.reserve(plain.size() + 8);
  encoded.push_back('&');
  append_inner_transport_escaped_byte(encoded, key);

  uint8_t key_hi = static_cast<uint8_t>((key >> 4) & 0x0F);
  uint8_t key_lo = static_cast<uint8_t>(key & 0x0F);
  uint8_t position = 0;
  for (unsigned char plain_byte : plain) {
    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!find_inner_encoded_nibble_uart_mode0(static_cast<uint8_t>((plain_byte >> 4) & 0x0F), position, key_hi,
                                              key_lo, hi)) {
      return false;
    }
    uint8_t next_position = static_cast<uint8_t>(position + 1);
    if (!find_inner_encoded_nibble_uart_mode0(static_cast<uint8_t>(plain_byte & 0x0F), next_position, key_hi, key_lo,
                                              lo)) {
      return false;
    }
    append_inner_transport_escaped_byte(encoded, static_cast<uint8_t>((hi << 4) | lo));
    position = static_cast<uint8_t>(position + 2);
  }

  encoded.push_back('\r');
  encoded.push_back('\n');
  return true;
}

bool encode_ble2_transport_plus(const std::string &plain, std::vector<uint8_t> &encoded) {
  encoded.clear();
  encoded.reserve(plain.size() + 8);
  encoded.push_back('+');
  append_inner_transport_escaped_byte(encoded, kBle2ProbeKey);

  uint8_t key_hi = static_cast<uint8_t>((kBle2ProbeKey >> 4) & 0x0F);
  uint8_t key_lo = static_cast<uint8_t>(kBle2ProbeKey & 0x0F);
  uint8_t position = 0;
  for (unsigned char plain_byte : plain) {
    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!find_inner_encoded_nibble_with_tables(static_cast<uint8_t>((plain_byte >> 4) & 0x0F), position, key_hi,
                                               key_lo, INNER_BLE2_A, INNER_BLE2_B, hi)) {
      return false;
    }
    uint8_t next_position = static_cast<uint8_t>(position + 1);
    if (!find_inner_encoded_nibble_with_tables(static_cast<uint8_t>(plain_byte & 0x0F), next_position, key_hi,
                                               key_lo, INNER_BLE2_A, INNER_BLE2_B, lo)) {
      return false;
    }
    append_inner_transport_escaped_byte(encoded, static_cast<uint8_t>((hi << 4) | lo));
    position = static_cast<uint8_t>(position + 2);
  }

  encoded.push_back('\r');
  encoded.push_back('\n');
  return true;
}

enum class InnerEscapeVariant {
  CURRENT,
  RAW,
  MASK_7F,
  XOR_20,
  PASSTHROUGH,
};

const char *inner_escape_variant_name(InnerEscapeVariant variant) {
  switch (variant) {
    case InnerEscapeVariant::CURRENT:
      return "current";
    case InnerEscapeVariant::RAW:
      return "esc_raw";
    case InnerEscapeVariant::MASK_7F:
      return "esc_mask_7f";
    case InnerEscapeVariant::XOR_20:
      return "esc_xor_20";
    case InnerEscapeVariant::PASSTHROUGH:
      return "esc_passthrough";
  }
  return "unknown";
}

bool inner_escape_variant_consumes_escape(InnerEscapeVariant variant) {
  return variant != InnerEscapeVariant::RAW;
}

uint8_t inner_unescape_byte(uint8_t escaped, InnerEscapeVariant variant) {
  switch (variant) {
    case InnerEscapeVariant::CURRENT:
    case InnerEscapeVariant::MASK_7F:
      return escaped & 0x7F;
    case InnerEscapeVariant::XOR_20:
      return escaped ^ 0x20;
    case InnerEscapeVariant::PASSTHROUGH:
    case InnerEscapeVariant::RAW:
      return escaped;
  }
  return escaped;
}

struct InnerTransportDecodeResult {
  bool ok{false};
  std::string payload{};
  std::string encoded_payload{};
  std::string reason{};
  const char *table_name{"unknown"};
  const char *escape_name{"current"};
  const char *key_variant_name{"key_byte_current"};
  uint8_t start{0};
  uint8_t key{0};
  bool key_escaped{false};
  size_t payload_len{0};
  size_t esc_count{0};
  uint8_t printable_ratio{0};
};

InnerTransportDecodeResult decode_inner_transport_with_tables(const std::string &frame, const uint8_t table_a[16],
                                                              const uint8_t table_b[16], const char *table_name,
                                                              InnerEscapeVariant escape_variant =
                                                                  InnerEscapeVariant::CURRENT) {
  InnerTransportDecodeResult result;
  result.table_name = table_name;
  result.escape_name = inner_escape_variant_name(escape_variant);
  result.start = frame.empty() ? 0 : static_cast<uint8_t>(frame.front());
  if (frame.size() < 2) {
    result.reason = "short_frame";
    return result;
  }
  uint8_t start = static_cast<uint8_t>(frame[0]);
  if (!is_inner_transport_start(start)) {
    result.reason = "unsupported_start";
    return result;
  }

  size_t index = 1;
  if (static_cast<uint8_t>(frame[index]) == 0x1B && inner_escape_variant_consumes_escape(escape_variant)) {
    result.key_escaped = true;
    result.esc_count += 1;
    ++index;
    if (index >= frame.size()) {
      result.reason = "truncated_escaped_key";
      return result;
    }
  }
  uint8_t key = static_cast<uint8_t>(frame[index]);
  if (result.key_escaped) {
    key = inner_unescape_byte(key, escape_variant);
  }
  result.key = key;
  uint8_t key_hi = (key >> 4) & 0x0F;
  uint8_t key_lo = key & 0x0F;
  ++index;

  uint8_t position = 0;
  std::string payload;
  std::string encoded_payload;
  payload.reserve(frame.size());
  encoded_payload.reserve(frame.size());
  while (index < frame.size()) {
    uint8_t encoded = static_cast<uint8_t>(frame[index]);
    if (encoded == '\r' || encoded == '\n') {
      break;
    }
    if (encoded == 0x1B && inner_escape_variant_consumes_escape(escape_variant)) {
      result.esc_count += 1;
      ++index;
      if (index >= frame.size()) {
        result.reason = "truncated_escape";
        return result;
      }
      encoded = inner_unescape_byte(static_cast<uint8_t>(frame[index]), escape_variant);
    }

    encoded_payload.push_back(static_cast<char>(encoded));
    uint8_t hi = fw_nibble_transform((encoded >> 4) & 0x0F, position, key_hi, key_lo, table_a, table_b);
    uint8_t next_position = static_cast<uint8_t>(position + 1);
    uint8_t lo = fw_nibble_transform(encoded & 0x0F, next_position, key_hi, key_lo, table_a, table_b);
    payload.push_back(static_cast<char>((hi << 4) | lo));
    position = static_cast<uint8_t>(position + 2);
    ++index;
  }

  result.encoded_payload = encoded_payload;
  result.payload_len = encoded_payload.size();
  result.printable_ratio = printable_ratio_percent(payload);
  if (payload.empty()) {
    result.reason = "empty_payload";
    return result;
  }
  result.payload = payload;
  result.ok = is_stats_ascii_response(payload);
  result.reason = result.ok ? "" : "inner_decode_not_ascii";
  return result;
}

std::vector<InnerTransportDecodeResult> decode_inner_transport_candidates(const std::string &frame) {
  // Tables are the 16-byte substitution pairs used by FUN_40101408. The WiFi
  // and BLE2 pairs are also present verbatim in the Android transport layer.
  static constexpr uint8_t UART_MODE0_A[16] = {0x08, 0x0E, 0x0C, 0x04, 0x03, 0x0D, 0x0A, 0x0B,
                                               0x00, 0x0F, 0x06, 0x07, 0x02, 0x05, 0x01, 0x09};
  static constexpr uint8_t UART_MODE0_B[16] = {0x04, 0x0B, 0x0D, 0x0A, 0x00, 0x07, 0x0F, 0x05,
                                               0x09, 0x08, 0x03, 0x01, 0x0E, 0x02, 0x0C, 0x06};
  static constexpr uint8_t WIFI_A[16] = {1, 0, 3, 2, 15, 14, 8, 10, 6, 13, 7, 12, 11, 9, 5, 4};
  static constexpr uint8_t WIFI_B[16] = {9, 12, 6, 11, 10, 15, 2, 14, 13, 0, 4, 3, 1, 8, 7, 5};
  static constexpr uint8_t BLE2_A[16] = {14, 4, 3, 2, 1, 13, 8, 11, 6, 15, 12, 7, 10, 5, 0, 9};
  static constexpr uint8_t BLE2_B[16] = {10, 6, 13, 12, 14, 11, 1, 9, 15, 7, 0, 5, 3, 2, 4, 8};

  struct Candidate {
    const char *name;
    const uint8_t *a;
    const uint8_t *b;
  };

  uint8_t start = frame.empty() ? 0 : static_cast<uint8_t>(frame.front());
  const Candidate amp_candidates[] = {{"uart_mode0", UART_MODE0_A, UART_MODE0_B},
                                      {"wifi", WIFI_A, WIFI_B},
                                      {"ble2", BLE2_A, BLE2_B}};
  const Candidate star_candidates[] = {{"wifi", WIFI_A, WIFI_B}, {"uart_mode0", UART_MODE0_A, UART_MODE0_B}};
  const Candidate plus_candidates[] = {{"ble2", BLE2_A, BLE2_B}, {"wifi", WIFI_A, WIFI_B}};

  const Candidate *candidates = amp_candidates;
  size_t count = sizeof(amp_candidates) / sizeof(amp_candidates[0]);
  if (start == '*') {
    candidates = star_candidates;
    count = sizeof(star_candidates) / sizeof(star_candidates[0]);
  } else if (start == '+') {
    candidates = plus_candidates;
    count = sizeof(plus_candidates) / sizeof(plus_candidates[0]);
  }

  std::vector<InnerTransportDecodeResult> decoded_candidates;
  decoded_candidates.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    decoded_candidates.push_back(
        decode_inner_transport_with_tables(frame, candidates[i].a, candidates[i].b, candidates[i].name));
  }
  return decoded_candidates;
}

std::vector<InnerTransportDecodeResult> decode_inner_transport_variant_candidates(const std::string &frame,
                                                                                 InnerEscapeVariant escape_variant) {
  static constexpr uint8_t UART_MODE0_A[16] = {0x08, 0x0E, 0x0C, 0x04, 0x03, 0x0D, 0x0A, 0x0B,
                                               0x00, 0x0F, 0x06, 0x07, 0x02, 0x05, 0x01, 0x09};
  static constexpr uint8_t UART_MODE0_B[16] = {0x04, 0x0B, 0x0D, 0x0A, 0x00, 0x07, 0x0F, 0x05,
                                               0x09, 0x08, 0x03, 0x01, 0x0E, 0x02, 0x0C, 0x06};
  static constexpr uint8_t WIFI_A[16] = {1, 0, 3, 2, 15, 14, 8, 10, 6, 13, 7, 12, 11, 9, 5, 4};
  static constexpr uint8_t WIFI_B[16] = {9, 12, 6, 11, 10, 15, 2, 14, 13, 0, 4, 3, 1, 8, 7, 5};
  static constexpr uint8_t BLE2_A[16] = {14, 4, 3, 2, 1, 13, 8, 11, 6, 15, 12, 7, 10, 5, 0, 9};
  static constexpr uint8_t BLE2_B[16] = {10, 6, 13, 12, 14, 11, 1, 9, 15, 7, 0, 5, 3, 2, 4, 8};

  struct Candidate {
    const char *name;
    const uint8_t *a;
    const uint8_t *b;
  };

  const Candidate candidates[] = {{"uart_mode0", UART_MODE0_A, UART_MODE0_B},
                                  {"wifi", WIFI_A, WIFI_B},
                                  {"ble2", BLE2_A, BLE2_B}};

  std::vector<InnerTransportDecodeResult> decoded_candidates;
  decoded_candidates.reserve(sizeof(candidates) / sizeof(candidates[0]));
  for (const auto &candidate : candidates) {
    decoded_candidates.push_back(
        decode_inner_transport_with_tables(frame, candidate.a, candidate.b, candidate.name, escape_variant));
  }
  return decoded_candidates;
}

InnerTransportDecodeResult decode_inner_payload_with_key(const std::string &encoded_payload, uint8_t key_hi,
                                                         uint8_t key_lo, const uint8_t table_a[16],
                                                         const uint8_t table_b[16], const char *table_name,
                                                         const char *key_variant_name) {
  InnerTransportDecodeResult result;
  result.table_name = table_name;
  result.key_variant_name = key_variant_name;
  result.key = static_cast<uint8_t>(((key_hi & 0x0F) << 4) | (key_lo & 0x0F));
  result.encoded_payload = encoded_payload;
  result.payload_len = encoded_payload.size();

  std::string payload;
  payload.reserve(encoded_payload.size());
  uint8_t position = 0;
  for (unsigned char byte : encoded_payload) {
    uint8_t hi = fw_nibble_transform((byte >> 4) & 0x0F, position, key_hi & 0x0F, key_lo & 0x0F, table_a, table_b);
    uint8_t next_position = static_cast<uint8_t>(position + 1);
    uint8_t lo = fw_nibble_transform(byte & 0x0F, next_position, key_hi & 0x0F, key_lo & 0x0F, table_a, table_b);
    payload.push_back(static_cast<char>((hi << 4) | lo));
    position = static_cast<uint8_t>(position + 2);
  }

  result.payload = payload;
  result.printable_ratio = printable_ratio_percent(payload);
  if (payload.empty()) {
    result.reason = "empty_payload";
    return result;
  }
  result.ok = is_stats_ascii_response(payload);
  result.reason = result.ok ? "" : "inner_decode_not_ascii";
  return result;
}

std::vector<InnerTransportDecodeResult> decode_inner_transport_key_variant_candidates(
    const InnerBinaryProbePayload &probe, bool has_t2_word, uint16_t t2_word) {
  static constexpr uint8_t UART_MODE0_A[16] = {0x08, 0x0E, 0x0C, 0x04, 0x03, 0x0D, 0x0A, 0x0B,
                                               0x00, 0x0F, 0x06, 0x07, 0x02, 0x05, 0x01, 0x09};
  static constexpr uint8_t UART_MODE0_B[16] = {0x04, 0x0B, 0x0D, 0x0A, 0x00, 0x07, 0x0F, 0x05,
                                               0x09, 0x08, 0x03, 0x01, 0x0E, 0x02, 0x0C, 0x06};
  static constexpr uint8_t WIFI_A[16] = {1, 0, 3, 2, 15, 14, 8, 10, 6, 13, 7, 12, 11, 9, 5, 4};
  static constexpr uint8_t WIFI_B[16] = {9, 12, 6, 11, 10, 15, 2, 14, 13, 0, 4, 3, 1, 8, 7, 5};
  static constexpr uint8_t BLE2_A[16] = {14, 4, 3, 2, 1, 13, 8, 11, 6, 15, 12, 7, 10, 5, 0, 9};
  static constexpr uint8_t BLE2_B[16] = {10, 6, 13, 12, 14, 11, 1, 9, 15, 7, 0, 5, 3, 2, 4, 8};

  struct TableCandidate {
    const char *name;
    const uint8_t *a;
    const uint8_t *b;
  };
  struct KeyCandidate {
    const char *name;
    uint8_t hi;
    uint8_t lo;
    std::string payload;
  };

  const TableCandidate tables[] = {{"uart_mode0", UART_MODE0_A, UART_MODE0_B},
                                   {"wifi", WIFI_A, WIFI_B},
                                   {"ble2", BLE2_A, BLE2_B}};

  std::vector<KeyCandidate> keys;
  keys.reserve(8);
  uint8_t current_hi = (probe.key >> 4) & 0x0F;
  uint8_t current_lo = probe.key & 0x0F;
  keys.push_back({"key_byte_current", current_hi, current_lo, probe.unescaped_payload});
  keys.push_back({"key_hi_lo_nibbles", current_hi, current_lo, probe.unescaped_payload});
  keys.push_back({"key_low_high_nibbles", current_lo, current_hi, probe.unescaped_payload});
  if (!probe.unescaped_payload.empty()) {
    uint8_t next = static_cast<uint8_t>(probe.unescaped_payload.front());
    const uint8_t key_lo = static_cast<uint8_t>(probe.key & 0x0F);
    const uint8_t next_lo = static_cast<uint8_t>(next & 0x0F);
    keys.push_back({"key_byte_and_next_byte", key_lo, next_lo, probe.unescaped_payload.substr(1)});
  }
  if (has_t2_word) {
    uint8_t t2_low = static_cast<uint8_t>(t2_word & 0xFF);
    uint8_t t2_high = static_cast<uint8_t>((t2_word >> 8) & 0xFF);
    keys.push_back({"key_from_t2_word_low_byte", static_cast<uint8_t>((t2_low >> 4) & 0x0F),
                    static_cast<uint8_t>(t2_low & 0x0F), probe.unescaped_payload});
    keys.push_back({"key_from_t2_word_high_byte", static_cast<uint8_t>((t2_high >> 4) & 0x0F),
                    static_cast<uint8_t>(t2_high & 0x0F), probe.unescaped_payload});
    keys.push_back({"key_from_t2_word_nibbles", static_cast<uint8_t>((t2_word >> 12) & 0x0F),
                    static_cast<uint8_t>(t2_word & 0x0F), probe.unescaped_payload});
  }

  std::vector<InnerTransportDecodeResult> decoded_candidates;
  decoded_candidates.reserve(keys.size() * (sizeof(tables) / sizeof(tables[0])));
  for (const auto &key : keys) {
    for (const auto &table : tables) {
      decoded_candidates.push_back(
          decode_inner_payload_with_key(key.payload, key.hi, key.lo, table.a, table.b, table.name, key.name));
    }
  }
  return decoded_candidates;
}

bool parse_t2_word_from_response(const std::string &response, uint16_t &word) {
  if (response.size() < 14) {
    return false;
  }
  std::string candidate = response.substr(10, 4);
  char *end = nullptr;
  unsigned long parsed = std::strtoul(candidate.c_str(), &end, 16);
  if (end == candidate.c_str() || end == nullptr || *end != '\0' || parsed > 0xFFFFUL) {
    return false;
  }
  word = static_cast<uint16_t>(parsed);
  return true;
}

int determine_accuracy(XmlSensorKind kind, double scale) {
  if (kind == XmlSensorKind::Counter) {
    return 0;
  }
  double abs_scale = std::fabs(scale);
  constexpr double EPSILON = 1e-6;
  if (abs_scale <= 0.01 + EPSILON) {
    return 2;
  }
  if (abs_scale < 1.0 - EPSILON) {
    return 1;
  }
  return 0;
}

std::string format_printable_char(uint8_t byte) {
  switch (byte) {
    case '\r':
      return "\\r";
    case '\n':
      return "\\n";
    case '\t':
      return "\\t";
    default:
      break;
  }
  if (std::isprint(static_cast<int>(byte)) != 0) {
    return std::string(1, static_cast<char>(byte));
  }
  std::ostringstream stream;
  stream << "\\x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
         << static_cast<int>(byte);
  return stream.str();
}

std::string format_printable_string(const std::string &value) {
  std::ostringstream stream;
  for (unsigned char c : value) {
    stream << format_printable_char(c);
  }
  return stream.str();
}

std::string format_hex_string(const std::string &value) {
  if (value.empty()) {
    return "[]";
  }
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < value.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
           << static_cast<int>(static_cast<unsigned char>(value[i]));
  }
  stream << "]";
  return stream.str();
}

std::string format_hex_string(const std::vector<uint8_t> &value) {
  if (value.empty()) {
    return "[]";
  }
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < value.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
           << static_cast<int>(value[i]);
  }
  stream << "]";
  return stream.str();
}

std::string format_buffer_preview(const std::string &value) {
  if (value.size() <= HANDSHAKE_LOG_PREVIEW_LIMIT) {
    return format_printable_string(value);
  }
  std::string suffix = value.substr(value.size() - HANDSHAKE_LOG_PREVIEW_LIMIT);
  return std::string("...") + format_printable_string(suffix);
}

std::string format_buffer_hex_preview(const std::string &value) {
  if (value.size() <= HANDSHAKE_LOG_PREVIEW_LIMIT) {
    return format_hex_string(value);
  }
  std::string suffix = value.substr(value.size() - HANDSHAKE_LOG_PREVIEW_LIMIT);
  std::string formatted_suffix = format_hex_string(suffix);
  if (formatted_suffix.size() > 1) {
    return std::string("...") + formatted_suffix;
  }
  return formatted_suffix;
}

void trim_in_place(std::string &value) {
  auto begin = value.find_first_not_of(" \t\r\n");
  auto end = value.find_last_not_of(" \t\r\n");
  if (begin == std::string::npos || end == std::string::npos) {
    value.clear();
    return;
  }
  value.assign(value.begin() + static_cast<std::ptrdiff_t>(begin),
               value.begin() + static_cast<std::ptrdiff_t>(end) + 1);
}

std::string collapse_whitespace(std::string value) {
  std::string result;
  result.reserve(value.size());
  bool last_space = false;
  for (unsigned char c : value) {
    if (std::isspace(c) != 0) {
      if (!last_space) {
        result.push_back(' ');
        last_space = true;
      }
    } else {
      result.push_back(static_cast<char>(c));
      last_space = false;
    }
  }
  trim_in_place(result);
  return result;
}

bool status_field_is_publishable(const JuraDecodedField &field) {
  return field.category == "status" || field.category == "alert";
}

std::string xml_table_for_decoded_category(const std::string &category) {
  if (category == "status") {
    return "PROGRESS_STATE_INTAKE";
  }
  if (category == "alert") {
    return "ALERTS";
  }
  if (category == "product_candidate") {
    return "PRODUCTS";
  }
  if (category == "process") {
    return "PROCESSES";
  }
  return "unknown";
}

void append_unique(std::vector<std::string> &values, const std::string &value) {
  if (value.empty()) {
    return;
  }
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

std::string join_values(const std::vector<std::string> &values, const char *separator) {
  std::string out;
  for (const auto &value : values) {
    if (!out.empty()) {
      out.append(separator);
    }
    out.append(value);
  }
  return out;
}

std::string to_lower_copy(const std::string &value) {
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return result;
}

std::string sanitize_identifier(const std::string &value) {
  std::string result;
  bool last_was_underscore = false;
  for (unsigned char c : value) {
    if (std::isalnum(c) != 0) {
      result.push_back(static_cast<char>(std::tolower(c)));
      last_was_underscore = false;
    } else if (!last_was_underscore) {
      result.push_back('_');
      last_was_underscore = true;
    }
  }
  while (!result.empty() && result.front() == '_') {
    result.erase(result.begin());
  }
  while (!result.empty() && result.back() == '_') {
    result.pop_back();
  }
  return result;
}

bool is_hex_text(const std::string &value) {
  return std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isxdigit(c) != 0; });
}

std::string truncate_for_log(const std::string &value, size_t limit = 240) {
  if (value.size() <= limit) {
    return value;
  }
  return value.substr(0, limit - 3) + "...";
}

std::string infer_response_command(const std::string &response, const std::string &active_command) {
  if (!active_command.empty()) {
    return active_command;
  }
  if (response.empty()) {
    return "unknown";
  }
  if (response[0] == '@') {
    auto end = response.find_first_of(":\r\n ");
    if (end != std::string::npos && end > 1) {
      return response.substr(0, end);
    }
  }
  if (response.size() >= 3 && std::isalpha(static_cast<unsigned char>(response[0])) != 0 &&
      std::isalpha(static_cast<unsigned char>(response[1])) != 0 && response[2] == ':') {
    return response.substr(0, 2);
  }
  return "status_hex";
}

std::string infer_response_family(const std::string &command) {
  if (command.empty() || command == "unknown") {
    return "unknown";
  }
  if (command == "status_hex") {
    return "status_hex";
  }
  if (command[0] == '@' && command.size() >= 3) {
    return command.substr(1, 2);
  }
  if (command.size() >= 2) {
    return command.substr(0, 2);
  }
  return command;
}

std::string format_decoded_field_trace(const std::vector<JuraDecodedField> &fields, std::vector<std::string> &tables,
                                       bool &has_publishable) {
  std::vector<std::string> details;
  has_publishable = false;
  for (const auto &field : fields) {
    if (field.category == "raw" || field.category == "unknown") {
      continue;
    }
    std::string table = xml_table_for_decoded_category(field.category);
    append_unique(tables, table);
    if (status_field_is_publishable(field)) {
      has_publishable = true;
    }

    std::string detail = table;
    detail.push_back(':');
    detail.append(field.key.empty() ? "?" : field.key);
    detail.append(" raw=");
    detail.append(field.raw_value.empty() ? "?" : field.raw_value);
    detail.append(" text='");
    detail.append(sanitize_text_for_api(field.decoded_text));
    detail.push_back('\'');
    if (field.category == "product_candidate") {
      detail.append(" publish=no reason=product_candidate_unverified");
    } else {
      detail.append(status_field_is_publishable(field) ? " publish=yes" : " publish=no");
    }
    details.push_back(detail);
  }
  if (details.empty()) {
    append_unique(tables, "unknown");
    return "none";
  }
  return truncate_for_log(join_values(details, " | "), 700);
}

std::string to_pascal_case(const std::string &value) {
  std::string result;
  result.reserve(value.size());
  bool capitalize = true;
  for (unsigned char c : value) {
    if (c == '_' || c == '-' || c == ' ') {
      capitalize = true;
      continue;
    }
    if (capitalize) {
      result.push_back(static_cast<char>(std::toupper(c)));
      capitalize = false;
    } else {
      result.push_back(static_cast<char>(std::tolower(c)));
    }
  }
  return result;
}

bool find_tag_in_scope(const std::string &xml, const std::string &tag, size_t scope_begin, size_t scope_end,
                       size_t &content_begin, size_t &content_end) {
  std::string open_tag = "<" + tag;
  std::string close_tag = "</" + tag + ">";
  size_t search_pos = scope_begin;
  while (true) {
    size_t open = xml.find(open_tag, search_pos);
    if (open == std::string::npos || open >= scope_end) {
      return false;
    }
    size_t name_end = open + open_tag.size();
    if (name_end < xml.size()) {
      unsigned char next = static_cast<unsigned char>(xml[name_end]);
      if (!(std::isspace(next) != 0 || next == '>' || next == '/')) {
        search_pos = name_end;
        continue;
      }
    }
    size_t open_end = xml.find('>', name_end);
    if (open_end == std::string::npos) {
      return false;
    }
    bool self_closing = open_end > open && xml[open_end - 1] == '/';
    size_t start = open_end + 1;
    if (self_closing) {
      content_begin = start;
      content_end = start;
      return true;
    }
    size_t close = xml.find(close_tag, start);
    if (close == std::string::npos) {
      search_pos = open_end + 1;
      continue;
    }
    if (close > scope_end) {
      search_pos = open_end + 1;
      continue;
    }
    content_begin = start;
    content_end = close;
    return true;
  }
}

bool xml_get_value_simple(const std::string &xml, const std::string &path, std::string &out) {
  if (path.empty()) {
    return false;
  }
  size_t scope_begin = 0;
  size_t scope_end = xml.size();
  size_t pos = 0;
  while (pos < path.size()) {
    size_t next = path.find('/', pos);
    std::string segment = (next == std::string::npos) ? path.substr(pos) : path.substr(pos, next - pos);
    if (segment.empty()) {
      pos = (next == std::string::npos) ? path.size() : next + 1;
      continue;
    }
    size_t content_begin = 0;
    size_t content_end = 0;
    if (!find_tag_in_scope(xml, segment, scope_begin, scope_end, content_begin, content_end)) {
      return false;
    }
    scope_begin = content_begin;
    scope_end = content_end;
    pos = (next == std::string::npos) ? path.size() : next + 1;
  }
  if (scope_begin > scope_end || scope_end > xml.size()) {
    return false;
  }
  out = xml.substr(scope_begin, scope_end - scope_begin);
  trim_in_place(out);
  return true;
}

std::string format_numeric_text(double value) {
  char buffer[32];
  int written = std::snprintf(buffer, sizeof(buffer), "%.6f", value);
  if (written < 0) {
    return std::to_string(value);
  }
  std::string text(buffer, static_cast<size_t>(written));
  auto dot = text.find('.');
  if (dot != std::string::npos) {
    while (!text.empty() && text.back() == '0') {
      text.pop_back();
    }
    if (!text.empty() && text.back() == '.') {
      text.pop_back();
    }
    if (text.empty()) {
      text = "0";
    }
  }
  return text;
}

template<typename ArrayT>
std::string format_numeric_array(const ArrayT &values) {
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

}  // namespace

const char *JuraComponent::handshake_stage_name(JuraComponent::HandshakeStage stage) {
  switch (stage) {
    case JuraComponent::HandshakeStage::IDLE:
      return "idle";
    case JuraComponent::HandshakeStage::HELLO:
      return "hello";
    case JuraComponent::HandshakeStage::SEND_T1:
      return "send_t1";
    case JuraComponent::HandshakeStage::WAIT_T2:
      return "wait_t2";
    case JuraComponent::HandshakeStage::SEND_T2:
      return "send_t2";
    case JuraComponent::HandshakeStage::WAIT_T3:
      return "wait_t3";
    case JuraComponent::HandshakeStage::SEND_T3:
      return "send_t3";
    case JuraComponent::HandshakeStage::DONE:
      return "done";
    case JuraComponent::HandshakeStage::FAILED:
      return "failed";
  }
  return "unknown";
}

JuraComponent::~JuraComponent() { this->xml_sensors_.clear(); }

void JuraComponent::setup() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not configured for JUTTA Proto component.");
    this->mark_failed();
    return;
  }

  this->connection_ = std::make_unique<::jutta_proto::JuttaConnection>(this->parent_);
  this->connection_->set_log_decoded_tx(this->log_decoded_tx_);
  this->connection_->set_log_encoded_uart(this->log_encoded_uart_);
  this->connection_->set_response_callback([this](const std::string &response, const char *parser_branch) {
    this->handle_decoded_response_(response, parser_branch);
  });
  this->connection_->init();
  this->publish_machine_online_(false);
  this->publish_machine_ready_(false);

  this->handshake_stage_ = HandshakeStage::HELLO;
  ESP_LOGI(TAG, "Starting handshake with coffee maker...");

  this->reset_xml_poll_state_();
  if (this->xml_command_probe_) {
    ESP_LOGI(TAG, "xml_command_probe_enabled with_ts_lock=%s", YESNO(this->xml_command_probe_with_ts_lock_));
  }
  if (this->xml_session_probe_) {
    ESP_LOGI(TAG, "xml_session_probe_enabled variant=%s", this->xml_session_probe_variant_.c_str());
    if (this->xml_session_probe_variant_ == "dongle_full") {
      ESP_LOGW(TAG, "xml_session_probe variant=dongle_full is experimental and may disturb the machine session");
    }
  }
  if (this->xml_dongle_startup_) {
    ESP_LOGI(TAG, "xml_dongle_startup enabled; mode=%s; XML statistics will wait for @TR:37 gate",
             this->xml_dongle_startup_mode_.c_str());
  }
  if (this->status_debug_) {
    ESP_LOGI(TAG,
             "status_path_app_firmware_summary app_udp=0010A5F3_to_51515 firmware_cache=DAT_400d0738_TF "
             "DAT_400d073c_progress esphome=passive_uart_tf_tv");
  }
  if (this->status_forensics_) {
    ESP_LOGI(TAG,
             "status_forensics enabled; passive RX diagnostics only, interval=%u ms, verbose_candidates=%s",
             static_cast<unsigned>(this->status_forensics_log_interval_ms_),
             YESNO(this->status_forensics_verbose_candidates_));
  }
  if (this->status_probe_enabled_) {
    ESP_LOGW(TAG, "status_probe_disabled reason=tf_tv_direct_commands_not_valid");
    this->status_probe_enabled_ = false;
  }
  if (this->enable_xml_poll_) {
    this->ensure_xml_mapping_loaded_();
    this->ensure_xml_sensors_created_();
  }
}

void JuraComponent::loop() {
  if (this->handshake_stage_ != this->last_logged_stage_) {
    ESP_LOGI(TAG, "Handshake stage changed: %s -> %s (buffer size=%zu, preview='%s', hex %s)",
             JuraComponent::handshake_stage_name(this->last_logged_stage_),
             JuraComponent::handshake_stage_name(this->handshake_stage_), this->handshake_buffer_.size(),
             format_buffer_preview(this->handshake_buffer_).c_str(),
             format_buffer_hex_preview(this->handshake_buffer_).c_str());
    this->last_logged_stage_ = this->handshake_stage_;
  }

  if (this->connection_ != nullptr && this->handshake_stage_ != HandshakeStage::DONE &&
      this->handshake_stage_ != HandshakeStage::FAILED) {
    this->process_handshake();
  }

  if (this->coffee_maker_ != nullptr) {
    this->coffee_maker_->loop();
    if (!this->coffee_maker_->is_locked()) {
      this->custom_cancel_flag_ = false;
    }
  }

  this->process_xml_polling();
  this->process_machine_data_query();
  this->poll_settings_once_();
  this->poll_error_cycle_();
}

void JuraComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "JUTTA Proto");
  if (!this->device_type_.empty()) {
    ESP_LOGCONFIG(TAG, "  Detected device: %s", this->device_type_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Detected device: (pending)");
  }

  if (this->enable_xml_poll_) {
    // Stelle sicher, dass der aktuelle Status des XML-Mappings bereits zur
    // Konfigurationsausgabe geladen und die Sensoren angelegt wurden.
    this->ensure_xml_mapping_loaded_();
    this->ensure_xml_sensors_created_();
  }

  const char *state = "unknown";
  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      state = "idle";
      break;
    case HandshakeStage::HELLO:
      state = "awaiting type";
      break;
    case HandshakeStage::SEND_T1:
      state = "waiting for @t1";
      break;
    case HandshakeStage::WAIT_T2:
      state = "waiting for @T2";
      break;
    case HandshakeStage::SEND_T2:
      state = "sending @t2";
      break;
    case HandshakeStage::WAIT_T3:
      state = "waiting for @T3";
      break;
    case HandshakeStage::SEND_T3:
      state = "sending @t3";
      break;
    case HandshakeStage::DONE:
      state = "ready";
      break;
    case HandshakeStage::FAILED:
      state = "failed";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Handshake state: %s", state);

  if (!this->handshake_t2_response_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last key exchange T2: %s", this->handshake_t2_response_.c_str());
  }
  if (!this->handshake_t3_response_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last key exchange T3: %s", this->handshake_t3_response_.c_str());
  }

  if (this->coffee_maker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(true));
  } else {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(false));
  }

  ESP_LOGCONFIG(TAG, "  XML polling: %s", this->enable_xml_poll_ ? "enabled" : "disabled");
  ESP_LOGCONFIG(TAG, "  Machine-XML polling: %s", YESNO(this->enable_machine_xml_poll_));
  ESP_LOGCONFIG(TAG, "  XML mapping Quelle: %s", this->xml_mapping_path_.c_str());
  ESP_LOGCONFIG(TAG, "  XML poll interval: %u ms", static_cast<unsigned>(this->xml_poll_interval_ms_));
  ESP_LOGCONFIG(TAG, "  XML startup delay: %u ms", static_cast<unsigned>(this->xml_startup_delay_ms_));
  ESP_LOGCONFIG(TAG, "  XML publish unstable counters: %s", YESNO(this->xml_publish_unstable_));
  ESP_LOGCONFIG(TAG, "  XML stats TS lock: %s", YESNO(this->xml_stats_use_ts_lock_));
  ESP_LOGCONFIG(TAG, "  XML compact debug: %s", YESNO(this->xml_debug_compact_));
  ESP_LOGCONFIG(TAG, "  XML inner transport decode: %s", YESNO(this->xml_decode_inner_transport_));
  ESP_LOGCONFIG(TAG, "  XML inner decode trace: %s", YESNO(this->xml_inner_decode_trace_));
  ESP_LOGCONFIG(TAG, "  XML binary probe: %s", YESNO(this->xml_binary_probe_));
  ESP_LOGCONFIG(TAG, "  XML key probe: %s", YESNO(this->xml_key_probe_));
  ESP_LOGCONFIG(TAG, "  XML deep debug: %s", YESNO(this->xml_deep_debug_));
  ESP_LOGCONFIG(TAG, "  XML transport selftest: %s", YESNO(this->xml_transport_selftest_));
  ESP_LOGCONFIG(TAG, "  XML command probe: %s", YESNO(this->xml_command_probe_));
  ESP_LOGCONFIG(TAG, "  XML command probe TS lock: %s", YESNO(this->xml_command_probe_with_ts_lock_));
  ESP_LOGCONFIG(TAG, "  XML session probe: %s", YESNO(this->xml_session_probe_));
  ESP_LOGCONFIG(TAG, "  XML session probe variant: %s", this->xml_session_probe_variant_.c_str());
  ESP_LOGCONFIG(TAG, "  Status debug: %s", YESNO(this->status_debug_));
  ESP_LOGCONFIG(TAG, "  Status forensics: %s", YESNO(this->status_forensics_));
  ESP_LOGCONFIG(TAG, "  Status forensics interval: %u ms",
                static_cast<unsigned>(this->status_forensics_log_interval_ms_));
  ESP_LOGCONFIG(TAG, "  Status forensics verbose candidates: %s",
                YESNO(this->status_forensics_verbose_candidates_));
  ESP_LOGCONFIG(TAG, "  Status probe: %s", YESNO(this->status_probe_enabled_));
  ESP_LOGCONFIG(TAG, "  Status probe interval: %u ms", static_cast<unsigned>(this->status_probe_interval_ms_));
  ESP_LOGCONFIG(TAG, "  Unsafe debug commands: %s", YESNO(this->allow_unsafe_debug_commands_));
  ESP_LOGCONFIG(TAG, "  XML dongle startup: %s", YESNO(this->xml_dongle_startup_));
  ESP_LOGCONFIG(TAG, "  XML dongle startup debug: %s", YESNO(this->xml_dongle_startup_debug_));
  ESP_LOGCONFIG(TAG, "  XML dongle startup mode: %s", this->xml_dongle_startup_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  XML dongle wait @t0 after @t3: %s", YESNO(this->xml_dongle_wait_t0_after_t3_));
  ESP_LOGCONFIG(TAG, "  XML dongle inner TX debug: %s", YESNO(this->xml_dongle_inner_tx_debug_));
  ESP_LOGCONFIG(TAG, "  XML tablet start sequence: %s", YESNO(this->xml_run_tablet_start_sequence_));
  ESP_LOGCONFIG(TAG, "  XML tablet sequence mode: %s", this->xml_tablet_sequence_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  XML counter max: %u", static_cast<unsigned>(this->xml_counter_max_));
  if (this->enable_xml_poll_) {
    this->log_xml_mapping_status_(true);
    ESP_LOGCONFIG(TAG, "  XML mapping loaded: %s", YESNO(this->xml_mapping_loaded_));
    ESP_LOGCONFIG(TAG, "  XML mapping valid: %s", YESNO(this->xml_mapping_.valid));
    ESP_LOGCONFIG(TAG, "  XML mapping commands: TR32=%s (%u Felder), TG43=%s (%u Felder), TGC0=%s (%u Felder)",
                  YESNO(!this->xml_mapping_.tr32.empty()),
                  static_cast<unsigned>(this->xml_mapping_.tr32.fields.size()),
                  YESNO(!this->xml_mapping_.tg43.empty()),
                  static_cast<unsigned>(this->xml_mapping_.tg43.fields.size()),
                  YESNO(!this->xml_mapping_.tgc0.empty()),
                  static_cast<unsigned>(this->xml_mapping_.tgc0.fields.size()));
  } else {
    ESP_LOGCONFIG(TAG, "  XML mapping loaded: %s", YESNO(false));
  }
  ESP_LOGCONFIG(TAG, "  XML sensors: %u", static_cast<unsigned>(this->xml_sensors_.size()));
}

void JuraComponent::process_handshake() {
  using ::jutta_proto::JuttaConnection;
  using ::jutta_proto::JUTTA_GET_TYPE;

  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      break;
    case HandshakeStage::HELLO: {
      if (!this->handshake_hello_request_sent_) {
        ESP_LOGD(TAG, "HELLO: requesting device type with payload '%s' (hex %s).",
                 format_printable_string(JUTTA_GET_TYPE).c_str(),
                 format_hex_string(JUTTA_GET_TYPE).c_str());
        if (this->connection_->write_decoded(JUTTA_GET_TYPE)) {
          this->connection_->reset_response_line_buffer();
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = esphome::millis() + 2000;
          this->handshake_hello_request_sent_ = true;
          ESP_LOGD(TAG, "HELLO: device type request sent, waiting for response (deadline in 2000 ms).");
        } else {
          this->restart_handshake("failed to request device type");
          break;
        }
      }

      if (this->read_handshake_bytes()) {
        bool handled = false;
        while (!handled) {
          auto newline = this->handshake_buffer_.find("\r\n");
          if (newline == std::string::npos) {
            break;
          }

          std::string line = this->handshake_buffer_.substr(0, newline);
          this->handshake_buffer_.erase(0, newline + 2);

          std::string lowercase_line = line;
          std::transform(lowercase_line.begin(), lowercase_line.end(), lowercase_line.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

          if (lowercase_line.rfind("ty:", 0) == 0) {
            if (line.size() <= 3) {
              ESP_LOGW(TAG,
                       "HELLO: device type response line '%s' has no payload, proceeding with unknown type.",
                       format_printable_string(line).c_str());
              this->device_type_ = "TY:unknown";
            } else {
              this->device_type_ = line;
              ESP_LOGI(TAG, "Detected coffee maker response: %s", this->device_type_.c_str());
            }
            this->publish_machine_type_();
            this->publish_last_command_result_("machine_type");
            this->handshake_buffer_.clear();
            this->handshake_deadline_ = 0;
            this->handshake_stage_ = HandshakeStage::SEND_T1;
            this->handshake_hello_request_sent_ = false;
            handled = true;
          } else {
            ESP_LOGD(TAG, "HELLO: ignoring unexpected response line: '%s'",
                     format_printable_string(line).c_str());
          }
        }
      }

      if (this->handshake_stage_ == HandshakeStage::HELLO && this->handshake_deadline_ != 0 &&
          time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for device type");
      }
      break;
    }
    case HandshakeStage::SEND_T1: {
      ESP_LOGD(TAG, "SEND_T1: writing '@T1\\r\\n' and waiting for '@t1\\r\\n' (timeout=1000 ms).");
      auto wait_result = this->connection_->write_decoded_wait_for("@T1\r\n", "@t1\r\n", std::chrono::milliseconds{1000});
      if (wait_result == JuttaConnection::WaitResult::Success) {
        ESP_LOGD(TAG, "Received @t1 acknowledgment.");
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
        this->handshake_stage_ = HandshakeStage::WAIT_T2;
      } else if (wait_result == JuttaConnection::WaitResult::Timeout) {
        this->restart_handshake("timeout waiting for @t1");
      } else if (wait_result == JuttaConnection::WaitResult::Error) {
        this->restart_handshake("failed to send @T1");
      }
      break;
    }
    case HandshakeStage::WAIT_T2: {
      if (this->handshake_deadline_ == 0) {
        this->handshake_deadline_ = esphome::millis() + 5000;
        ESP_LOGD(TAG, "WAIT_T2: started response timer (deadline in 5000 ms).");
      }
      bool any = this->read_handshake_bytes();
      if (any) {
        auto pos = this->handshake_buffer_.find("@T2");
        if (pos != std::string::npos) {
          auto end = this->handshake_buffer_.find("\r\n", pos);
          if (end != std::string::npos) {
            this->handshake_t2_response_ = this->handshake_buffer_.substr(pos, end - pos);
          } else {
            this->handshake_t2_response_ = this->handshake_buffer_.substr(pos);
          }
          ESP_LOGD(TAG, "Received %s", this->handshake_t2_response_.c_str());
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = 0;
          this->handshake_stage_ = HandshakeStage::SEND_T2;
        }
      }
      if (this->handshake_deadline_ != 0 && time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for @T2");
      }
      break;
    }
    case HandshakeStage::SEND_T2: {
      ESP_LOGD(TAG, "SEND_T2: sending '@t2:8120000000\\r\\n'.");
      if (this->connection_->write_decoded("@t2:8120000000\r\n")) {
        ESP_LOGD(TAG, "Sent @t2 response.");
        this->handshake_stage_ = HandshakeStage::WAIT_T3;
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
      } else {
        this->restart_handshake("failed to send @t2");
      }
      break;
    }
    case HandshakeStage::WAIT_T3: {
      if (this->handshake_deadline_ == 0) {
        this->handshake_deadline_ = esphome::millis() + 5000;
        ESP_LOGD(TAG, "WAIT_T3: started response timer (deadline in 5000 ms).");
      }
      bool any = this->read_handshake_bytes();
      if (any) {
        auto pos = this->handshake_buffer_.find("@T3");
        if (pos != std::string::npos) {
          auto end = this->handshake_buffer_.find("\r\n", pos);
          if (end != std::string::npos) {
            this->handshake_t3_response_ = this->handshake_buffer_.substr(pos, end - pos);
          } else {
            this->handshake_t3_response_ = this->handshake_buffer_.substr(pos);
          }
          ESP_LOGD(TAG, "Received %s", this->handshake_t3_response_.c_str());
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = 0;
          this->handshake_stage_ = HandshakeStage::SEND_T3;
        }
      }
      if (this->handshake_deadline_ != 0 && time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for @T3");
      }
      break;
    }
    case HandshakeStage::SEND_T3: {
      ESP_LOGD(TAG, "SEND_T3: sending '@t3\\r\\n' to finish handshake.");
      if (this->connection_->write_decoded("@t3\r\n")) {
        ESP_LOGI(TAG, "Handshake finished successfully.");
        this->handshake_stage_ = HandshakeStage::DONE;
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
        if (this->enable_xml_poll_) {
          this->ensure_xml_mapping_loaded_();
        }
        this->publish_last_command_result_("handshake_done");
        this->publish_machine_online_(true);
        this->update_machine_status_from_state_("handshake");
      } else {
        this->restart_handshake("failed to send @t3");
      }
      break;
    }
    case HandshakeStage::DONE:
    case HandshakeStage::FAILED:
      break;
  }

  if (this->handshake_stage_ == HandshakeStage::DONE && this->connection_ != nullptr &&
      this->coffee_maker_ == nullptr) {
    auto connection = std::move(this->connection_);
    this->coffee_maker_ = std::make_unique<::jutta_proto::CoffeeMaker>(std::move(connection));
    this->coffee_maker_->connection->set_log_decoded_tx(this->log_decoded_tx_);
    this->coffee_maker_->connection->set_log_encoded_uart(this->log_encoded_uart_);
    this->coffee_maker_->connection->set_response_callback([this](const std::string &response,
                                                                  const char *parser_branch) {
      this->handle_decoded_response_(response, parser_branch);
    });
    ESP_LOGI(TAG, "Coffee maker controller initialized.");
    this->reset_xml_poll_state_();
  }
}

void JuraComponent::restart_handshake(const char *reason) {
  if (reason != nullptr) {
    ESP_LOGW(TAG, "Restarting handshake: %s", reason);
    this->publish_last_command_result_(std::string("handshake_restart: ") + reason);
    this->publish_machine_status_(std::string("not_ready: ") + reason);
    this->publish_machine_online_(false);
    this->publish_machine_ready_(false);
  }
  this->handshake_buffer_.clear();
  this->handshake_deadline_ = 0;
  this->handshake_hello_request_sent_ = false;
  this->handshake_stage_ = HandshakeStage::HELLO;
  this->last_logged_stage_ = HandshakeStage::FAILED;
  if (this->connection_ != nullptr) {
    this->connection_->reset_response_line_buffer();
  }
  this->stats_session_ready_ = false;
  this->dongle_events_ = 0;
  this->dongle_startup_state_ = DongleStartupState::IDLE;
  this->dongle_startup_rx_buffer_.clear();
  this->dongle_startup_deadline_ms_ = 0;
  this->dongle_startup_next_action_ms_ = 0;
  this->dongle_startup_quiet_start_ms_ = 0;
  this->dongle_startup_t3_seen_during_quiet_ = false;
  this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
  this->reset_xml_poll_state_();
}

bool JuraComponent::read_handshake_bytes() {
  if (this->connection_ == nullptr) {
    return false;
  }
  bool read_any = false;
  std::string line;
  while (this->connection_->read_line_until(line)) {
    read_any = true;
    this->handshake_buffer_.append(line);
    this->handshake_buffer_.append("\r\n");
    if (this->handshake_buffer_.size() > 128) {
      this->handshake_buffer_.erase(0, this->handshake_buffer_.size() - 128);
    }
    ESP_LOGV(TAG,
             "Handshake buffered line: '%s'; buffer size=%zu; buffer now '%s' (hex %s)",
             format_printable_string(line).c_str(), this->handshake_buffer_.size(),
             format_buffer_preview(this->handshake_buffer_).c_str(),
             format_buffer_hex_preview(this->handshake_buffer_).c_str());
  }
  return read_any;
}

bool JuraComponent::time_reached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

void JuraComponent::handle_decoded_response_(const std::string &response, const char *parser_branch) {
  this->publish_raw_rx_(response, parser_branch);
  this->publish_machine_online_(true);
  this->update_dongle_events_from_line_(response);

  std::string lowered = response;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered.rfind("ty:", 0) == 0) {
    if (response.size() > 3) {
      this->device_type_ = response;
    } else if (this->device_type_.empty()) {
      this->device_type_ = "TY:unknown";
    }
    this->publish_machine_type_();
    this->publish_last_command_result_("machine_type");
    return;
  }

  if (lowered == "ok:") {
    this->publish_last_command_result_("ok");
    return;
  }

  if (lowered == "@t1") {
    this->publish_last_command_result_("@t1");
    return;
  }

  if (lowered.rfind("@tf:", 0) == 0) {
    if (this->publish_tf_status_(response)) {
      this->publish_last_command_result_("tf_status");
    }
    return;
  }

  if (lowered.rfind("@tv:", 0) == 0) {
    if (this->handle_tv_progress_(response)) {
      this->publish_last_command_result_("tv_progress");
    }
    return;
  }

  if (lowered.rfind("@h", 0) == 0 && lowered.find(":error") != std::string::npos) {
    this->publish_machine_status_(response);
    this->publish_last_command_result_(response);
    return;
  }

  if (response.rfind("@T2", 0) == 0 || response.rfind("@T3", 0) == 0) {
    this->publish_last_command_result_(response);
    return;
  }

  if (parser_branch != nullptr && std::string(parser_branch) == "db_frame") {
    if (this->decode_and_publish_status_(response, parser_branch)) {
      return;
    }
    ESP_LOGD(TAG, "DB frame kept raw-only; no verified status text decoded");
  }
}

bool JuraComponent::decode_and_publish_status_(const std::string &response, const char *parser_branch) {
  if (response.find('<') != std::string::npos || response.find('>') != std::string::npos) {
    return false;
  }
  std::string active_command_raw = this->xml_last_command_;
  std::string active_command = active_command_raw;
  std::transform(active_command.begin(), active_command.end(), active_command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (active_command == "@tr:32" || active_command == "@tg:43" || active_command == "@tg:c0") {
    return false;
  }
  std::string branch = parser_branch != nullptr ? parser_branch : "unknown";
  std::string command = infer_response_command(response, active_command_raw);
  std::string family = infer_response_family(command);
  std::string payload_hex = format_hex_string(response);
  std::string raw_rx = sanitize_text_for_api(response);

  if (!this->ensure_xml_mapping_loaded_()) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=unknown fields=none final='' fallback=xml_mapping_unavailable",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str());
    return false;
  }

  std::vector<JuraDecodedField> fields;
  if (!decode_status_response(response, branch, fields) || fields.empty()) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=unknown fields=none final='' fallback=decoder_returned_no_fields",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str());
    return false;
  }
  this->last_decoded_fields_ = fields;
  std::vector<std::string> tables;
  bool has_publishable = false;
  std::string field_trace = format_decoded_field_trace(fields, tables, has_publishable);
  std::string table_trace = tables.empty() ? "unknown" : join_values(tables, ",");
  if (!has_publishable) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=%s fields=%s final='' fallback=no_verified_status_or_alert_match_raw_only",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str(),
             table_trace.c_str(), field_trace.c_str());
    return false;
  }
  std::string summary = this->format_decoded_status_(fields);
  if (summary.empty()) {
    ESP_LOGD(TAG,
             "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
             "xml_tables=%s fields=%s final='' fallback=empty_summary",
             raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str(),
             table_trace.c_str(), field_trace.c_str());
    return false;
  }
  ESP_LOGD(TAG,
           "status_decode raw_rx='%s' family=%s command=%s payload_hex=%s branch=%s decoder=status_xml "
           "xml_tables=%s fields=%s final='%s' fallback=%s",
           raw_rx.c_str(), family.c_str(), command.c_str(), payload_hex.c_str(), branch.c_str(),
           table_trace.c_str(), field_trace.c_str(), sanitize_text_for_api(summary).c_str(), "none");
  this->publish_machine_status_(summary);
  this->publish_last_command_result_("decoded_status");
  return true;
}

std::string JuraComponent::format_decoded_status_(const std::vector<JuraDecodedField> &fields) const {
  std::vector<std::string> known;
  std::string raw;
  for (const auto &field : fields) {
    if (field.category == "raw") {
      raw = field.raw_value;
      continue;
    }
    if (field.category == "unknown" || field.category == "product_candidate") {
      continue;
    }
    std::string text = field.decoded_text.empty() ? field.raw_value : field.decoded_text;
    if (text.empty()) {
      continue;
    }
    std::string item;
    if (field.category == "status") {
      item = "Status: ";
    } else if (field.category == "alert") {
      item = "Alert: ";
    } else if (field.category == "product") {
      item = "Product: ";
    } else {
      item = field.category + ": ";
    }
    item.append(text);
    if (!field.raw_value.empty()) {
      item.append(" (");
      item.append(field.raw_value);
      item.push_back(')');
    }
    known.push_back(item);
  }

  if (known.empty()) {
    return raw.empty() ? std::string{} : std::string("Raw: ") + raw;
  }

  std::string summary;
  for (const auto &item : known) {
    if (!summary.empty()) {
      summary.append("; ");
    }
    if (summary.size() + item.size() > 240) {
      summary.append("...");
      break;
    }
    summary.append(item);
  }
  return sanitize_text_for_api(summary);
}

bool JuraComponent::is_printable_status_text_(const std::string &text) const {
  if (text.empty()) {
    return false;
  }
  size_t escaped_binary_markers = 0;
  size_t hex_like_tokens = 0;
  for (size_t i = 0; i < text.size(); ++i) {
    unsigned char c = static_cast<unsigned char>(text[i]);
    if (c == '\r' || c == '\n' || c == '\t') {
      continue;
    }
    if (c < 0x20 || c == 0x7F) {
      return false;
    }
    if (c >= 0x80) {
      size_t remaining = 0;
      if ((c & 0xE0) == 0xC0) {
        remaining = 1;
        if (c < 0xC2) {
          return false;
        }
      } else if ((c & 0xF0) == 0xE0) {
        remaining = 2;
      } else if ((c & 0xF8) == 0xF0) {
        remaining = 3;
        if (c > 0xF4) {
          return false;
        }
      } else {
        return false;
      }
      if (i + remaining >= text.size()) {
        return false;
      }
      for (size_t j = 1; j <= remaining; ++j) {
        unsigned char cc = static_cast<unsigned char>(text[i + j]);
        if ((cc & 0xC0) != 0x80) {
          return false;
        }
      }
      i += remaining;
      continue;
    }
    if (c == '\\' && i + 1 < text.size() && text[i + 1] == 'x') {
      ++escaped_binary_markers;
    }
    if (c == '0' && i + 1 < text.size() && (text[i + 1] == 'x' || text[i + 1] == 'X')) {
      ++hex_like_tokens;
    }
  }
  if (escaped_binary_markers > 0) {
    return false;
  }
  return hex_like_tokens <= 4 || text.find('<') != std::string::npos;
}

void JuraComponent::publish_raw_rx_(const std::string &response, const char *parser_branch) {
  std::string safe;
  if (has_binary_bytes(response)) {
    safe = "hex:" + compact_hex_string(response, 96);
  } else {
    safe = sanitize_text_for_api(response);
    constexpr size_t kRawRxTextLimit = 240;
    if (safe.size() > kRawRxTextLimit) {
      safe = safe.substr(0, kRawRxTextLimit) + "...";
    }
  }
  ESP_LOGV(TAG, "RX decoded branch=%s value='%s'", parser_branch != nullptr ? parser_branch : "unknown", safe.c_str());
  if (this->raw_rx_sensor_ != nullptr) {
    this->raw_rx_sensor_->publish_state(safe);
  }
}

void JuraComponent::publish_last_command_result_(const std::string &result) {
  std::string safe = sanitize_text_for_api(result);
  ESP_LOGV(TAG, "Command result: %s", safe.c_str());
  if (this->last_command_result_sensor_ != nullptr) {
    this->last_command_result_sensor_->publish_state(safe);
  }
}

void JuraComponent::publish_machine_type_() {
  if (this->machine_type_sensor_ != nullptr && !this->device_type_.empty()) {
    this->machine_type_sensor_->publish_state(sanitize_text_for_api(this->device_type_));
  }
}

void JuraComponent::publish_machine_status_(const std::string &status) {
  if (!this->is_printable_status_text_(status)) {
    ESP_LOGW(TAG, "Machine status publish suppressed (binary/non-printable payload)");
    return;
  }
  std::string safe = sanitize_text_for_api(status);
  ESP_LOGV(TAG, "Machine status: %s", safe.c_str());
  if (this->machine_status_sensor_ != nullptr) {
    this->machine_status_sensor_->publish_state(safe);
  }
}

void JuraComponent::publish_machine_online_(bool online) {
  this->machine_online_state_ = online;
  ESP_LOGD(TAG, "Machine online: %s", YESNO(online));
  if (this->machine_online_sensor_ != nullptr) {
    this->machine_online_sensor_->publish_state(online);
  }
}

void JuraComponent::publish_machine_ready_(bool ready) {
  this->machine_ready_state_ = ready;
  ESP_LOGD(TAG, "Machine ready: %s", YESNO(ready));
  if (this->machine_ready_sensor_ != nullptr) {
    this->machine_ready_sensor_->publish_state(ready);
  }
}

void JuraComponent::update_machine_status_from_state_(const char *source) {
  const char *safe_source = source != nullptr ? source : "unknown";
  const bool online = this->machine_online_state_ || this->is_ready();
  const bool live_status_seen = this->has_valid_tf_status_ || this->has_valid_tv_status_;
  const bool blocking_alert = this->fill_water_required_ || !this->current_machine_warning_.empty();
  std::string status;
  bool ready = this->machine_ready_state_ || this->is_ready();

  if (this->live_status_source_sensor_ != nullptr) {
    this->live_status_source_sensor_->publish_state(
        live_status_seen ? sanitize_text_for_api(this->current_live_status_source_) : "nicht verfügbar");
  }

  if (this->machine_display_status_sensor_ != nullptr) {
    this->machine_display_status_sensor_->publish_state(
        live_status_seen ? (this->current_display_status_.empty() ? "keine"
                                                                  : sanitize_text_for_api(this->current_display_status_))
                         : "nicht verfügbar");
  }
  if (this->machine_warning_sensor_ != nullptr) {
    this->machine_warning_sensor_->publish_state(
        live_status_seen ? (this->current_machine_warning_.empty() ? "keine"
                                                                   : sanitize_text_for_api(this->current_machine_warning_))
                         : "nicht verfügbar");
  }
  if (this->active_alerts_sensor_ != nullptr) {
    this->active_alerts_sensor_->publish_state(
        live_status_seen ? (this->current_active_alerts_.empty() ? "keine"
                                                                 : sanitize_text_for_api(this->current_active_alerts_))
                         : "nicht verfügbar");
  }
  if (this->fill_water_required_sensor_ != nullptr && live_status_seen) {
    this->fill_water_required_sensor_->publish_state(this->fill_water_required_);
  }

  if (!online) {
    status = "offline";
    ready = false;
    ESP_LOGD(TAG, "machine_status_update source=%s priority=offline status=\"offline\"", safe_source);
  } else if (!live_status_seen) {
    status = "Online";
    ESP_LOGD(TAG, "machine_status_update source=%s priority=protocol_online_no_live_status status=\"Online\"",
             safe_source);
  } else if (blocking_alert) {
    status = this->current_machine_warning_.empty() ? "Warnung" : this->current_machine_warning_;
    ready = false;
    if (std::strcmp(safe_source, "handshake") == 0) {
      ESP_LOGD(TAG, "machine_status_update source=handshake ignored_reason=active_blocking_alert");
    }
    ESP_LOGD(TAG, "machine_status_update source=%s priority=alert status=\"%s\"", safe_source,
             sanitize_text_for_api(status).c_str());
  } else if (!this->current_display_status_.empty()) {
    status = this->current_display_status_;
    ready = (status == "Bereit");
    ESP_LOGD(TAG, "machine_status_update source=%s priority=display status=\"%s\"", safe_source,
             sanitize_text_for_api(status).c_str());
  } else if (this->tf_coffee_ready_active_) {
    status = "Bereit";
    ready = true;
    ESP_LOGD(TAG, "machine_status_update source=%s priority=tf_ready status=\"Bereit\"", safe_source);
  } else if (ready) {
    status = "Bereit";
    ESP_LOGD(TAG, "machine_status_update source=%s priority=protocol_ready status=\"Bereit\"", safe_source);
  } else {
    status = "Online";
    ESP_LOGD(TAG, "machine_status_update source=%s priority=online status=\"Online\"", safe_source);
  }

  const std::string live_source_log =
      live_status_seen ? sanitize_text_for_api(this->current_live_status_source_) : "nicht verfügbar";
  const std::string display_status_log = sanitize_text_for_api(this->current_display_status_);
  const std::string machine_status_log = sanitize_text_for_api(status);
  ESP_LOGD(TAG,
           "machine_status_decision online=%s ready=%s has_valid_tf=%s has_valid_tv=%s blocking_alert=%s "
           "live_status_source=\"%s\" display_state=\"%s\" result=\"%s\"",
           YESNO(online), YESNO(ready), YESNO(this->has_valid_tf_status_), YESNO(this->has_valid_tv_status_),
           YESNO(blocking_alert), live_source_log.c_str(), display_status_log.c_str(), machine_status_log.c_str());

  this->publish_machine_status_(status);
  this->publish_machine_ready_(ready);
}

bool JuraComponent::publish_tf_status_(const std::string &response) {
  std::string trimmed = response;
  trim_in_place(trimmed);
  std::string lower = to_lower_copy(trimmed);
  if (lower.rfind("@tf:", 0) != 0) {
    return false;
  }

  std::string payload = trimmed.substr(4);
  if (payload.empty() || (payload.size() % 2) != 0) {
    ESP_LOGD(TAG, "tf_status ignored decoded=\"%s\" reason=invalid_hex_length", sanitize_text_for_api(trimmed).c_str());
    return false;
  }

  std::vector<uint8_t> data;
  data.reserve(payload.size() / 2);
  for (size_t i = 0; i < payload.size(); i += 2) {
    if (!std::isxdigit(static_cast<unsigned char>(payload[i])) ||
        !std::isxdigit(static_cast<unsigned char>(payload[i + 1]))) {
      ESP_LOGD(TAG, "tf_status ignored decoded=\"%s\" reason=invalid_hex", sanitize_text_for_api(trimmed).c_str());
      return false;
    }
    char byte_text[3] = {payload[i], payload[i + 1], '\0'};
    data.push_back(static_cast<uint8_t>(std::strtoul(byte_text, nullptr, 16)));
  }

  std::vector<uint16_t> bits;
  bits.reserve(data.size() * 8);
  for (size_t bit = 0; bit < data.size() * 8; ++bit) {
    const uint8_t mask = static_cast<uint8_t>(1U << (7U - (bit % 8U)));
    if ((data[bit / 8U] & mask) != 0) {
      bits.push_back(static_cast<uint16_t>(bit));
    }
  }

  std::ostringstream bits_stream;
  for (size_t i = 0; i < bits.size(); ++i) {
    if (i != 0) {
      bits_stream << ',';
    }
    bits_stream << bits[i];
  }
  const std::string bits_text = bits_stream.str();
  ESP_LOGD(TAG, "tf_status decoded=\"%s\" bits=\"%s\"", sanitize_text_for_api(trimmed).c_str(), bits_text.c_str());

  auto has_bit = [&data](size_t bit) -> bool {
    if (bit >= data.size() * 8) {
      return false;
    }
    const uint8_t mask = static_cast<uint8_t>(1U << (7U - (bit % 8U)));
    return (data[bit / 8U] & mask) != 0;
  };

  this->has_valid_tf_status_ = true;
  this->current_live_status_source_ = "@TF";
  const bool fill_water = has_bit(1);
  const bool coffee_ready = has_bit(13);
  this->fill_water_required_ = fill_water;
  this->tf_coffee_ready_active_ = coffee_ready;

  auto log_alert_check = [&has_bit](size_t xml_bit) {
    const size_t zero_based_index = xml_bit;
    const size_t one_based_index = xml_bit > 0 ? xml_bit - 1 : 0;
    const uint8_t zero_based_mask = static_cast<uint8_t>(1U << (7U - (zero_based_index % 8U)));
    const uint8_t one_based_mask = static_cast<uint8_t>(1U << (7U - (one_based_index % 8U)));
    const bool active_zero_based = has_bit(zero_based_index);
    const bool active_one_based = xml_bit > 0 && has_bit(one_based_index);
    ESP_LOGD(TAG,
             "tf_alert_check xml_bit=%u zero_based_mask=0x%02X one_based_mask=0x%02X "
             "active_zero_based=%s active_one_based=%s",
             static_cast<unsigned>(xml_bit), static_cast<unsigned>(zero_based_mask),
             static_cast<unsigned>(one_based_mask), YESNO(active_zero_based), YESNO(active_one_based));
  };

  std::vector<std::string> active_alerts;
  if (fill_water) {
    active_alerts.emplace_back("Wassertank füllen");
    this->current_machine_warning_ = "Wassertank füllen";
    this->current_display_status_ = "Wassertank füllen";
  } else if (this->current_machine_warning_ == "Wassertank füllen") {
    this->current_machine_warning_.clear();
  }
  if (coffee_ready) {
    if (!fill_water) {
      this->current_display_status_ = "Bereit";
    }
  } else if (!fill_water && this->current_display_status_ == "Wassertank füllen") {
    this->current_display_status_.clear();
  }
  this->current_active_alerts_ = join_values(active_alerts, ", ");

  const std::string active_alerts_log =
      this->current_active_alerts_.empty() ? "keine" : sanitize_text_for_api(this->current_active_alerts_);
  ESP_LOGD(TAG, "tf_decode raw=\"%s\" valid=YES active_alerts=\"%s\"",
           sanitize_text_for_api(trimmed).c_str(), active_alerts_log.c_str());
  ESP_LOGD(TAG, "tf_status decoded payload=\"%s\" active_alerts=\"%s\"",
           sanitize_text_for_api(payload).c_str(), active_alerts_log.c_str());
  ESP_LOGD(TAG, "tf_payload raw=%s bytes=%s", sanitize_text_for_api(payload).c_str(),
           format_hex_string(data).c_str());
  log_alert_check(1);
  log_alert_check(13);
  ESP_LOGD(TAG, "tf_alert bit=1 name=\"fill water\" active=%s type=block", YESNO(fill_water));
  ESP_LOGD(TAG, "tf_alert bit=13 name=\"coffee ready\" active=%s", YESNO(coffee_ready));

  if (this->tf_welcome_sensor_ != nullptr) {
    this->tf_welcome_sensor_->publish_state(has_bit(11));
  }
  if (this->tf_coffee_ready_sensor_ != nullptr) {
    this->tf_coffee_ready_sensor_->publish_state(has_bit(13));
  }
  if (this->tf_energy_safe_sensor_ != nullptr) {
    this->tf_energy_safe_sensor_->publish_state(has_bit(36));
  }
  if (this->tf_active_rf_filter_sensor_ != nullptr) {
    this->tf_active_rf_filter_sensor_->publish_state(has_bit(37));
  }
  if (this->tf_status_bits_sensor_ != nullptr) {
    this->tf_status_bits_sensor_->publish_state(bits_text);
  }
  this->update_machine_status_from_state_("tf");
  return true;
}

bool JuraComponent::handle_tv_progress_(const std::string &response) {
  std::string trimmed = response;
  trim_in_place(trimmed);
  std::string lower = to_lower_copy(trimmed);
  if (lower.rfind("@tv:", 0) != 0) {
    return false;
  }

  const std::string payload = trimmed.substr(4);
  if (payload.size() < 2 || !std::isxdigit(static_cast<unsigned char>(payload[0])) ||
      !std::isxdigit(static_cast<unsigned char>(payload[1]))) {
    ESP_LOGD(TAG, "tv_progress ignored decoded=\"%s\" reason=invalid_hex",
             sanitize_text_for_api(trimmed).c_str());
    return false;
  }

  char code_text[3] = {payload[0], payload[1], '\0'};
  const uint8_t code = static_cast<uint8_t>(std::strtoul(code_text, nullptr, 16));
  this->has_valid_tv_status_ = true;
  this->current_live_status_source_ = "@TV";
  const char *state = nullptr;
  bool blocking = false;
  switch (code) {
    case 0x02:
      state = "Wassertank füllen";
      blocking = true;
      break;
    case 0x24:
      state = "Bereit";
      break;
    case 0x0E:
      state = "Alarm";
      blocking = true;
      break;
    default:
      ESP_LOGD(TAG, "tv_progress code=%02X state=\"unknown\" raw=\"%s\"", code,
               sanitize_text_for_api(trimmed).c_str());
      ESP_LOGD(TAG, "tv_decode raw=\"%s\" valid=YES state=\"unknown\"", sanitize_text_for_api(trimmed).c_str());
      return true;
  }

  this->current_display_status_ = state;
  if (blocking) {
    this->current_machine_warning_ = state;
    this->current_active_alerts_ = state;
    if (code == 0x02) {
      this->fill_water_required_ = true;
    }
  } else {
    if (this->current_machine_warning_ == state || this->current_machine_warning_ == "Wassertank füllen" ||
        this->current_machine_warning_ == "Alarm") {
      this->current_machine_warning_.clear();
    }
    if (code == 0x24) {
      this->fill_water_required_ = false;
      this->tf_coffee_ready_active_ = true;
      this->current_active_alerts_.clear();
    }
  }

  ESP_LOGD(TAG, "tv_progress code=%02X state=\"%s\"", code, sanitize_text_for_api(state).c_str());
  ESP_LOGD(TAG, "tv_decode raw=\"%s\" valid=YES state=\"%s\"", sanitize_text_for_api(trimmed).c_str(),
           sanitize_text_for_api(state).c_str());
  this->update_machine_status_from_state_("tv");
  return true;
}

bool JuraComponent::handle_t2_status_debug_(const std::string &response) {
  std::string trimmed = response;
  trim_in_place(trimmed);
  if (trimmed.rfind("@T2", 0) != 0) {
    return false;
  }

  std::string payload;
  if (trimmed.rfind("@T2:", 0) == 0) {
    payload = trimmed.substr(4);
  } else {
    payload = trimmed.substr(3);
  }

  std::vector<uint8_t> payload_bytes;
  bool payload_is_hex = !payload.empty() && (payload.size() % 2U) == 0;
  if (payload_is_hex) {
    payload_bytes.reserve(payload.size() / 2U);
    for (size_t i = 0; i < payload.size(); i += 2U) {
      if (!std::isxdigit(static_cast<unsigned char>(payload[i])) ||
          !std::isxdigit(static_cast<unsigned char>(payload[i + 1U]))) {
        payload_is_hex = false;
        payload_bytes.clear();
        break;
      }
      char byte_text[3] = {payload[i], payload[i + 1U], '\0'};
      payload_bytes.push_back(static_cast<uint8_t>(std::strtoul(byte_text, nullptr, 16)));
    }
  }

  const std::string payload_hex =
      payload_is_hex ? format_hex_string(payload_bytes) : compact_hex_string(payload, payload.size());
  const std::string candidate073c = payload.substr(0, std::min<size_t>(payload.size(), 16U));
  const std::string candidate0740 = trimmed.size() > 10U ? trimmed.substr(10U) : std::string{};
  const std::string decoded = "payload=" + payload + " len=" + std::to_string(payload_bytes.size()) +
                              " bytes=" + payload_hex + " candidate073c=" + candidate073c +
                              " candidate0740_offset10=" + candidate0740;

  ESP_LOGD(TAG, "t2_status_decode raw=\"%s\" payload=\"%s\" len=%u",
           sanitize_text_for_api(trimmed).c_str(), sanitize_text_for_api(payload).c_str(),
           static_cast<unsigned>(payload_bytes.size()));
  ESP_LOGD(TAG, "t2_status_cache candidate073c=\"%s\"",
           sanitize_text_for_api(candidate073c).c_str());
  ESP_LOGD(TAG, "t2_status_word candidate0740_offset10=\"%s\"",
           sanitize_text_for_api(candidate0740).c_str());
  ESP_LOGD(TAG, "t2_status_event set=0x%02X events=0x%02X", static_cast<unsigned>(DONGLE_EVENT_T2),
           static_cast<unsigned>(this->dongle_events_ | DONGLE_EVENT_T2));

  if (this->last_t2_status_raw_sensor_ != nullptr) {
    this->last_t2_status_raw_sensor_->publish_state(sanitize_text_for_api(trimmed));
  }
  if (this->last_t2_status_decoded_sensor_ != nullptr) {
    this->last_t2_status_decoded_sensor_->publish_state(sanitize_text_for_api(decoded));
  }
  return true;
}

void JuraComponent::publish_status_probe_last_response_(const std::string &text) {
  if (this->status_probe_last_response_sensor_ == nullptr) {
    return;
  }
  this->status_probe_last_response_sensor_->publish_state(sanitize_text_for_api(text));
}

void JuraComponent::run_status_probe_command(const std::string &command) {
  (void) command;
  ESP_LOGW(TAG, "status_probe_disabled reason=stability_rollback");
}

void JuraComponent::run_ble2_transport_probe(const std::string &probe) {
  (void) probe;
  ESP_LOGW(TAG, "ble2_probe_disabled reason=stability_rollback");
}

void JuraComponent::run_debug_command(const std::string &command, const std::string &transport) {
  (void) command;
  (void) transport;
  ESP_LOGW(TAG, "debug_command_disabled reason=stability_rollback");
}

void JuraComponent::publish_debug_command_last_response_(const std::string &text) {
  if (this->debug_command_last_response_sensor_ == nullptr) {
    return;
  }
  this->debug_command_last_response_sensor_->publish_state(sanitize_text_for_api(text));
}

std::string JuraComponent::normalize_debug_command_(const std::string &command) const {
  std::string normalized = command;
  trim_in_place(normalized);
  while (!normalized.empty() && (normalized.back() == '\r' || normalized.back() == '\n')) {
    normalized.pop_back();
  }
  trim_in_place(normalized);
  return normalized;
}

bool JuraComponent::is_unsafe_debug_command_(const std::string &command) const {
  std::string trimmed = this->normalize_debug_command_(command);
  std::string upper = trimmed;
  std::transform(upper.begin(), upper.end(), upper.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  static const char *const UNSAFE_PREFIXES[] = {
      "FN:",   "FA:",    "PR:",    "AN:",    "@TG:2",  "@TG:10", "@TG:21", "@TG:23",
      "@TG:24", "@TG:25", "@TG:26", "@TS:F1", "@TV:81", "@TV:82", "@TV:84", "@HR:81",
      "@HW:82",
  };
  for (const char *prefix : UNSAFE_PREFIXES) {
    if (upper.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

void JuraComponent::log_status_forensics_decoded_(const std::string &line, const char *source,
                                                  const char *table_name) {
  if (!this->status_forensics_) {
    return;
  }
  if (!this->status_forensics_decode_log_allowed_) {
    return;
  }
  std::string trimmed = line;
  trim_in_place(trimmed);
  if (trimmed.empty()) {
    return;
  }
  std::string lower = to_lower_copy(trimmed);
  const char *type = nullptr;
  if (lower.rfind("@tf:", 0) == 0) {
    type = "tf";
  } else if (lower.rfind("@tv:", 0) == 0) {
    type = "tv";
  } else if (trimmed.rfind("@T2", 0) == 0) {
    type = "t2";
  } else if (trimmed.rfind("@T3", 0) == 0) {
    type = "t3";
  } else if (lower.rfind("@t0", 0) == 0) {
    type = "t0";
  }

  const char *safe_source = source != nullptr ? source : "unknown";
  const char *safe_table = table_name != nullptr ? table_name : "ascii";
  bool logged = false;
  if (type != nullptr) {
    ESP_LOGD(TAG, "status_forensics_known ts_ms=%u source=%s table=%s type=%s line=\"%s\"",
             static_cast<unsigned>(esphome::millis()), safe_source, safe_table, type,
             sanitize_text_for_api(trimmed).c_str());
    logged = true;
  } else if (this->is_printable_status_text_(trimmed)) {
    ESP_LOGD(TAG, "status_forensics_unknown_printable ts_ms=%u source=%s table=%s line=\"%s\"",
             static_cast<unsigned>(esphome::millis()), safe_source, safe_table,
             sanitize_text_for_api(trimmed).c_str());
    logged = true;
  }
  if (logged) {
    this->status_forensics_decode_log_allowed_ = false;
  }
}

void JuraComponent::log_status_forensics_frame_(const std::string &raw, const char *source) {
  if (!this->status_forensics_) {
    return;
  }
  this->status_forensics_decode_log_allowed_ = false;
  const char *safe_source = source != nullptr ? source : "unknown";
  const uint32_t now = esphome::millis();
  const std::string raw_hex = compact_hex_string(raw, raw.size());

  bool important = false;
  if (!raw.empty() && this->is_printable_status_text_(raw)) {
    std::string trimmed = raw;
    trim_in_place(trimmed);
    std::string lower = to_lower_copy(trimmed);
    important = lower.rfind("@tf:", 0) == 0 || lower.rfind("@tv:", 0) == 0 || lower.rfind("@tm", 0) == 0 ||
                trimmed.rfind("@T2", 0) == 0 || trimmed.rfind("@T3", 0) == 0 || lower.rfind("@t0", 0) == 0 ||
                lower.rfind("@tr", 0) == 0 || lower.rfind("@tg", 0) == 0 || lower.rfind("ty:", 0) == 0 ||
                !trimmed.empty();
  }
  if (!raw.empty() && is_inner_transport_start(static_cast<uint8_t>(raw.front()))) {
    std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(raw);
    int selected = -1;
    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto &candidate = candidates[i];
      if (selected < 0 && !candidate.payload.empty() && this->is_printable_status_text_(candidate.payload)) {
        selected = static_cast<int>(i);
      }
      std::string payload_lower = to_lower_copy(candidate.payload);
      trim_in_place(payload_lower);
      important = important || payload_lower.rfind("@tf:", 0) == 0 || payload_lower.rfind("@tv:", 0) == 0 ||
                  payload_lower.rfind("@tm", 0) == 0 || payload_lower.rfind("@t0", 0) == 0 ||
                  payload_lower.rfind("@tr", 0) == 0 || payload_lower.rfind("@tg", 0) == 0 ||
                  payload_lower.rfind("ty:", 0) == 0;
    }
    const bool changed = raw_hex != this->status_forensics_last_raw_hex_;
    const bool rate_ok = this->status_forensics_next_log_ms_ == 0 || time_reached(now, this->status_forensics_next_log_ms_);
    if (!(changed || important) || !rate_ok) {
      if (this->status_forensics_next_suppressed_log_ms_ == 0 ||
          time_reached(now, this->status_forensics_next_suppressed_log_ms_)) {
        ESP_LOGD(TAG, "status_forensics_suppressed reason=%s",
                 !rate_ok ? "rate_limit" : "unchanged");
        this->status_forensics_next_suppressed_log_ms_ =
            now + std::max<uint32_t>(this->status_forensics_log_interval_ms_ * 5U, 10000U);
      }
      return;
    }
    this->status_forensics_last_raw_hex_ = raw_hex;
    this->status_forensics_next_log_ms_ = now + this->status_forensics_log_interval_ms_;
    this->status_forensics_decode_log_allowed_ = true;
    ESP_LOGD(TAG, "status_forensics_raw changed=%s source=%s len=%u hex=\"%s\"", YESNO(changed), safe_source,
             static_cast<unsigned>(raw.size()), raw_hex.c_str());
    if (selected >= 0) {
      const auto &candidate = candidates[static_cast<size_t>(selected)];
      ESP_LOGD(TAG, "status_forensics_selected table=%s line=\"%s\"", candidate.table_name,
               transport_payload_log_text(candidate.payload).c_str());
    } else {
      const InnerTransportDecodeResult *best = candidates.empty() ? nullptr : &candidates.front();
      ESP_LOGD(TAG, "status_forensics_unknown best_table=%s decoded=\"%s\"",
               best != nullptr ? best->table_name : "none",
               best != nullptr ? transport_payload_log_text(best->payload).c_str() : "");
    }
    if (this->status_forensics_verbose_candidates_ || selected < 0) {
      for (size_t i = 0; i < candidates.size(); ++i) {
        const auto &candidate = candidates[i];
        ESP_LOGD(TAG,
                 "status_forensics_candidate source=%s idx=%u table=%s ok=%s len=%u printable=%u first_ascii=\"%s\" "
                 "hex=\"%s\"",
                 safe_source, static_cast<unsigned>(i), candidate.table_name, YESNO(!candidate.payload.empty()),
                 static_cast<unsigned>(candidate.payload.size()), static_cast<unsigned>(candidate.printable_ratio),
                 printable_preview(candidate.payload, 32).c_str(), compact_hex_string(candidate.payload, 32).c_str());
      }
    }
    return;
  }

  const bool changed = raw_hex != this->status_forensics_last_raw_hex_;
  const bool rate_ok = this->status_forensics_next_log_ms_ == 0 || time_reached(now, this->status_forensics_next_log_ms_);
  if (!(changed || important) || !rate_ok) {
    if (this->status_forensics_next_suppressed_log_ms_ == 0 ||
        time_reached(now, this->status_forensics_next_suppressed_log_ms_)) {
      ESP_LOGD(TAG, "status_forensics_suppressed reason=%s", !rate_ok ? "rate_limit" : "unchanged");
      this->status_forensics_next_suppressed_log_ms_ =
          now + std::max<uint32_t>(this->status_forensics_log_interval_ms_ * 5U, 10000U);
    }
    return;
  }
  this->status_forensics_last_raw_hex_ = raw_hex;
  this->status_forensics_next_log_ms_ = now + this->status_forensics_log_interval_ms_;
  this->status_forensics_decode_log_allowed_ = true;
  ESP_LOGD(TAG, "status_forensics_raw changed=%s source=%s len=%u hex=\"%s\"", YESNO(changed), safe_source,
           static_cast<unsigned>(raw.size()), raw_hex.c_str());
}

void JuraComponent::start_status_probe_(uint32_t now) {
  this->status_probe_state_ = StatusProbeState::SEND;
  this->status_probe_index_ = 0;
  this->status_probe_current_cmd_.clear();
  this->status_probe_rx_buffer_.clear();
  this->status_probe_deadline_ms_ = 0;
  this->status_probe_frames_ = 0;
  ESP_LOGD(TAG, "status_probe_start candidates=%u", static_cast<unsigned>(this->status_probe_candidate_count_()));
  this->publish_status_probe_last_response_("started");
  (void) now;
}

const char *JuraComponent::status_probe_candidate_(size_t index) const {
  static const char *const CANDIDATES[] = {"@hf", "@ha:03,20", "@ha:03,21", "@ha:03,22", "@ha:03,23"};
  return index < (sizeof(CANDIDATES) / sizeof(CANDIDATES[0])) ? CANDIDATES[index] : nullptr;
}

size_t JuraComponent::status_probe_candidate_count_() const {
  return 5;
}

bool JuraComponent::status_probe_command_allowed_(const std::string &command) const {
  return command == "@hf" || command == "@ha:03,20" || command == "@ha:03,21" || command == "@ha:03,22" ||
         command == "@ha:03,23";
}

void JuraComponent::start_manual_status_probe_(const std::string &command, uint32_t now) {
  if (!this->status_probe_command_allowed_(command)) {
    ESP_LOGW(TAG, "status_probe_skip reason=unsupported_candidate cmd=%s", command.c_str());
    this->publish_status_probe_last_response_(command + " -> unsupported_candidate");
    return;
  }
  if (this->status_probe_state_ != StatusProbeState::IDLE) {
    ESP_LOGD(TAG, "status_probe_skip reason=uart_busy_or_stats_active cmd=%s", command.c_str());
    this->publish_status_probe_last_response_(command + " -> busy");
    return;
  }
  if (!this->stats_session_ready_ || !this->stats_inner_tx_required_) {
    ESP_LOGD(TAG, "status_probe_skip reason=post_gate_not_ready cmd=%s", command.c_str());
    this->publish_status_probe_last_response_(command + " -> post_gate_not_ready");
    return;
  }
  if (!this->post_gate_tx_ready_event_ || this->xml_inflight_ ||
      this->db_transaction_owner_ != DbTransactionOwner::NONE || this->is_busy()) {
    ESP_LOGD(TAG, "status_probe_skip reason=uart_busy_or_stats_active cmd=%s owner=%s", command.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->publish_status_probe_last_response_(command + " -> busy");
    return;
  }

  ESP_LOGD(TAG, "manual_status_probe_start cmd=%s", command.c_str());
  this->publish_status_probe_last_response_(command + " -> started");
  this->send_status_probe_candidate_(command, now);
}

bool JuraComponent::send_status_probe_candidate_(const std::string &command, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (!this->stats_session_ready_ || !this->stats_inner_tx_required_) {
    ESP_LOGD(TAG, "status_probe_skip reason=post_gate_not_ready cmd=%s", command.c_str());
    return false;
  }
  if (!this->post_gate_tx_ready_event_) {
    ESP_LOGD(TAG, "status_probe_skip reason=uart_busy_or_stats_active cmd=%s", command.c_str());
    return false;
  }
  if (this->xml_inflight_ || this->db_transaction_owner_ != DbTransactionOwner::NONE || this->is_busy()) {
    ESP_LOGD(TAG, "status_probe_skip reason=uart_busy_or_stats_active owner=%s cmd=%s",
             this->db_transaction_owner_name_(this->db_transaction_owner_), command.c_str());
    return false;
  }

  this->db_transaction_owner_ = DbTransactionOwner::STATUS_PROBE;
  this->post_gate_tx_ready_event_ = false;
  this->status_probe_current_cmd_ = command;
  this->status_probe_rx_buffer_.clear();
  this->status_probe_frames_ = 0;
  this->status_probe_deadline_ms_ = now + kStatusProbeTimeoutMs;
  if (this->coffee_maker_->connection != nullptr) {
    this->coffee_maker_->connection->reset_response_line_buffer();
  }

  ESP_LOGD(TAG, "manual_status_probe_tx cmd=%s mode=inner_uart0 timeout_ms=%u", command.c_str(),
           static_cast<unsigned>(kStatusProbeTimeoutMs));
  if (!this->write_inner_uart0_command_(command, now, true)) {
    ESP_LOGD(TAG, "manual_status_probe_done cmd=%s result=tx_failed frames=0", command.c_str());
    this->finish_status_probe_candidate_(now, "tx_failed");
    return false;
  }
  this->status_probe_state_ = StatusProbeState::WAIT;
  return true;
}

bool JuraComponent::handle_status_probe_line_(const std::string &line, bool complete, uint32_t now) {
  (void) complete;
  std::string lower = to_lower_copy(line);
  trim_in_place(lower);
  if (lower.empty()) {
    return false;
  }

  this->publish_status_probe_last_response_(line);
  this->update_dongle_events_from_line_(line);
  ESP_LOGD(TAG, "manual_status_probe_rx_decoded cmd=%s line=\"%s\"", this->status_probe_current_cmd_.c_str(),
           sanitize_text_for_api(line).c_str());

  if (lower.rfind("@tf:", 0) == 0) {
    ESP_LOGD(TAG, "manual_status_probe_detected_tf line=\"%s\"", sanitize_text_for_api(line).c_str());
    ESP_LOGD(TAG, "tf_decode source=manual_probe raw=\"%s\"", sanitize_text_for_api(line).c_str());
    this->publish_tf_status_(line);
    this->finish_status_probe_cycle_(now, "detected_tf");
    return true;
  }
  if (lower.rfind("@tv:", 0) == 0) {
    ESP_LOGD(TAG, "manual_status_probe_detected_tv line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->handle_tv_progress_(line);
    this->finish_status_probe_cycle_(now, "detected_tv");
    return true;
  }
  if (lower.rfind("@t2", 0) == 0) {
    ESP_LOGD(TAG, "manual_status_probe_detected_t2 line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->handle_t2_status_debug_(line);
    return false;
  }
  if (lower.rfind("@", 0) == 0) {
    ESP_LOGD(TAG, "manual_status_probe_detected_status_like line=\"%s\"", sanitize_text_for_api(line).c_str());
    return false;
  }

  ESP_LOGD(TAG, "manual_status_probe_rx_unmatched cmd=%s decoded_or_ascii=\"%s\"",
           this->status_probe_current_cmd_.c_str(), sanitize_text_for_api(line).c_str());
  return false;
}

void JuraComponent::finish_status_probe_candidate_(uint32_t now, const char *reason) {
  const char *safe_reason = reason != nullptr ? reason : "done";
  const std::string command = this->status_probe_current_cmd_;
  const uint16_t frames = this->status_probe_frames_;
  ESP_LOGD(TAG, "manual_status_probe_done cmd=%s result=%s frames=%u",
           command.empty() ? "(none)" : command.c_str(), safe_reason, static_cast<unsigned>(frames));
  if (this->db_transaction_owner_ == DbTransactionOwner::STATUS_PROBE) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
  }
  this->post_gate_tx_ready_event_ = true;
  this->status_probe_current_cmd_.clear();
  this->status_probe_rx_buffer_.clear();
  this->status_probe_deadline_ms_ = 0;
  this->status_probe_state_ = StatusProbeState::IDLE;
  this->publish_status_probe_last_response_((command.empty() ? std::string{} : command + " -> ") + safe_reason +
                                            " frames=" + std::to_string(frames));
  (void) now;
}

void JuraComponent::finish_status_probe_cycle_(uint32_t now, const char *result) {
  const char *safe_result = result != nullptr ? result : "done";
  std::string command = this->status_probe_current_cmd_;
  if (this->db_transaction_owner_ == DbTransactionOwner::STATUS_PROBE) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
  }
  this->post_gate_tx_ready_event_ = true;
  this->status_probe_state_ = StatusProbeState::IDLE;
  this->status_probe_index_ = 0;
  this->status_probe_current_cmd_.clear();
  this->status_probe_rx_buffer_.clear();
  this->status_probe_deadline_ms_ = 0;
  this->status_probe_next_ms_ = 0;
  ESP_LOGD(TAG, "manual_status_probe_done cmd=%s result=%s frames=%u",
           command.empty() ? "(none)" : command.c_str(), safe_result, static_cast<unsigned>(this->status_probe_frames_));
  if (std::strcmp(safe_result, "detected_tf") != 0 && std::strcmp(safe_result, "detected_tv") != 0) {
    this->publish_status_probe_last_response_((command.empty() ? std::string{} : command + " -> ") + safe_result +
                                              " frames=" + std::to_string(this->status_probe_frames_));
  }
  (void) now;
}

void JuraComponent::process_status_probe_(uint32_t now) {
  if (this->status_probe_state_ == StatusProbeState::IDLE || this->status_probe_state_ == StatusProbeState::DONE) {
    return;
  }

  if (this->status_probe_state_ == StatusProbeState::SEND) {
    const char *candidate = this->status_probe_candidate_(this->status_probe_index_);
    if (candidate == nullptr) {
      this->finish_status_probe_cycle_(now, "no_candidates");
      return;
    }
    this->send_status_probe_candidate_(candidate, now);
    return;
  }

  if (this->status_probe_state_ != StatusProbeState::WAIT) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->finish_status_probe_candidate_(now, "controller_not_ready");
    return;
  }

  std::string raw_line;
  while (this->coffee_maker_->connection->read_line_until(raw_line)) {
    this->status_probe_frames_++;
    ESP_LOGD(TAG, "manual_status_probe_rx_raw cmd=%s hex=\"%s\"", this->status_probe_current_cmd_.c_str(),
             compact_hex_string(raw_line, raw_line.size()).c_str());
    if (!raw_line.empty() && this->xml_decode_inner_transport_ &&
        is_inner_transport_start(static_cast<uint8_t>(raw_line.front()))) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(raw_line);
      if (!candidates.empty() && !candidates.front().payload.empty()) {
        const auto &decoded = candidates.front();
        ESP_LOGD(TAG, "manual_status_probe_rx_decoded cmd=%s line=\"%s\" table=%s",
                 this->status_probe_current_cmd_.c_str(), transport_payload_log_text(decoded.payload).c_str(),
                 decoded.table_name);
        if (this->handle_status_probe_line_(decoded.payload, true, now)) {
          return;
        }
      } else {
        ESP_LOGD(TAG, "manual_status_probe_rx_noise cmd=%s reason=inner_decode_empty",
                 this->status_probe_current_cmd_.c_str());
      }
    } else if (this->is_printable_status_text_(raw_line)) {
      if (this->handle_status_probe_line_(raw_line, true, now)) {
        return;
      }
    } else {
      ESP_LOGD(TAG, "manual_status_probe_rx_noise cmd=%s reason=non_printable hex=\"%s\"",
               this->status_probe_current_cmd_.c_str(), compact_hex_string(raw_line, raw_line.size()).c_str());
    }
  }

  if (time_reached(now, this->status_probe_deadline_ms_)) {
    ESP_LOGD(TAG, "manual_status_probe_timeout cmd=%s frames=%u", this->status_probe_current_cmd_.c_str(),
             static_cast<unsigned>(this->status_probe_frames_));
    this->finish_status_probe_candidate_(now, "timeout");
  }
}

bool JuraComponent::start_ble2_transport_probe_(const std::string &probe, uint32_t now) {
  std::string probe_name;
  std::string command;
  if (probe == "ty" || probe == "ty:") {
    probe_name = "ty";
    command = "TY:";
  } else if (probe == "tr37" || probe == "@tr:37") {
    probe_name = "tr37";
    command = "@TR:37";
  } else {
    ESP_LOGW(TAG, "ble2_probe_skip reason=unsupported_probe probe=%s", probe.c_str());
    this->publish_status_probe_last_response_("ble2 " + probe + " -> unsupported_probe");
    return false;
  }

  if (this->ble2_probe_state_ != Ble2ProbeState::IDLE || this->status_probe_state_ != StatusProbeState::IDLE) {
    ESP_LOGD(TAG, "ble2_probe_skip reason=uart_busy_or_stats_active probe=%s", probe_name.c_str());
    this->publish_status_probe_last_response_("ble2 " + probe_name + " -> busy");
    return false;
  }
  if (!this->stats_session_ready_ || !this->stats_inner_tx_required_) {
    ESP_LOGD(TAG, "ble2_probe_skip reason=post_gate_not_ready probe=%s", probe_name.c_str());
    this->publish_status_probe_last_response_("ble2 " + probe_name + " -> post_gate_not_ready");
    return false;
  }
  if (!this->post_gate_tx_ready_event_ || this->xml_inflight_ ||
      this->db_transaction_owner_ != DbTransactionOwner::NONE || this->is_busy()) {
    ESP_LOGD(TAG, "ble2_probe_skip reason=uart_busy_or_stats_active probe=%s owner=%s", probe_name.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->publish_status_probe_last_response_("ble2 " + probe_name + " -> busy");
    return false;
  }

  ESP_LOGD(TAG, "ble2_probe_start probe=%s cmd=%s", probe_name.c_str(), command.c_str());
  this->publish_status_probe_last_response_("ble2 " + probe_name + " -> started");
  return this->send_ble2_transport_probe_(probe_name, command, now);
}

bool JuraComponent::send_ble2_transport_probe_(const std::string &probe, const std::string &command, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    ESP_LOGD(TAG, "ble2_probe_skip reason=controller_not_ready probe=%s", probe.c_str());
    return false;
  }

  std::string framed = command;
  if (framed.size() < 2 || framed.substr(framed.size() - 2) != "\r\n") {
    framed.append("\r\n");
  }

  std::vector<uint8_t> encoded;
  if (!encode_ble2_transport_plus(framed, encoded)) {
    ESP_LOGD(TAG, "ble2_probe_done probe=%s result=encode_failed frames=0", probe.c_str());
    this->publish_status_probe_last_response_("ble2 " + probe + " -> encode_failed");
    return false;
  }

  std::string encoded_text(encoded.begin(), encoded.end());
  InnerTransportDecodeResult roundtrip =
      decode_inner_transport_with_tables(encoded_text, INNER_BLE2_A, INNER_BLE2_B, "ble2");
  bool roundtrip_ok = roundtrip.payload == framed;
  ESP_LOGD(TAG, "ble2_probe_roundtrip plain=\"%s\" ok=%s encoded_hex=\"%s\"",
           escape_control_text_for_log(command).c_str(), YESNO(roundtrip_ok),
           compact_hex_string(encoded_text, encoded_text.size()).c_str());
  if (!roundtrip_ok) {
    ESP_LOGD(TAG, "ble2_probe_done probe=%s result=roundtrip_failed frames=0", probe.c_str());
    this->publish_status_probe_last_response_("ble2 " + probe + " -> roundtrip_failed");
    return false;
  }

  this->db_transaction_owner_ = DbTransactionOwner::BLE2_PROBE;
  this->post_gate_tx_ready_event_ = false;
  this->ble2_probe_state_ = Ble2ProbeState::WAIT;
  this->ble2_probe_name_ = probe;
  this->ble2_probe_command_ = command;
  this->ble2_probe_deadline_ms_ = now + kBle2ProbeTimeoutMs;
  this->ble2_probe_frames_ = 0;
  this->coffee_maker_->connection->reset_response_line_buffer();

  ESP_LOGD(TAG, "ble2_probe_tx probe=%s cmd=%s mode=ble2_plus timeout_ms=%u", probe.c_str(), command.c_str(),
           static_cast<unsigned>(kBle2ProbeTimeoutMs));
  if (!this->coffee_maker_->connection->write_decoded_no_flush(encoded)) {
    ESP_LOGD(TAG, "ble2_probe_done probe=%s result=tx_failed frames=0", probe.c_str());
    this->finish_ble2_transport_probe_(now, "tx_failed");
    return false;
  }
  return true;
}

bool JuraComponent::handle_ble2_probe_line_(const std::string &line, const char *table_name, uint32_t now) {
  std::string lower = to_lower_copy(line);
  trim_in_place(lower);
  if (lower.empty()) {
    return false;
  }

  this->update_dongle_events_from_line_(line);
  this->publish_status_probe_last_response_("ble2 " + this->ble2_probe_name_ + " -> " + sanitize_text_for_api(line));

  const char *table = table_name != nullptr ? table_name : "ascii";
  ESP_LOGD(TAG, "ble2_probe_rx_decoded probe=%s line=\"%s\" table=%s", this->ble2_probe_name_.c_str(),
           sanitize_text_for_api(line).c_str(), table);

  if (this->ble2_probe_name_ == "ty" && lower.rfind("ty:", 0) == 0) {
    ESP_LOGD(TAG, "ble2_probe_detected_expected probe=ty line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->finish_ble2_transport_probe_(now, "expected");
    return true;
  }
  if (this->ble2_probe_name_ == "tr37" && lower.rfind("@tr:37", 0) == 0) {
    ESP_LOGD(TAG, "ble2_probe_detected_expected probe=tr37 line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->finish_ble2_transport_probe_(now, "expected");
    return true;
  }
  if (lower.rfind("@tf:", 0) == 0) {
    ESP_LOGD(TAG, "ble2_probe_detected_tf line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->publish_tf_status_(line);
    this->finish_ble2_transport_probe_(now, "detected_tf");
    return true;
  }
  if (lower.rfind("@tv:", 0) == 0) {
    ESP_LOGD(TAG, "ble2_probe_detected_tv line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->handle_tv_progress_(line);
    this->finish_ble2_transport_probe_(now, "detected_tv");
    return true;
  }
  if (lower.rfind("@t2", 0) == 0) {
    ESP_LOGD(TAG, "ble2_probe_detected_t2 line=\"%s\"", sanitize_text_for_api(line).c_str());
    this->handle_t2_status_debug_(line);
    return false;
  }
  if (lower.rfind("@", 0) == 0) {
    ESP_LOGD(TAG, "ble2_probe_detected_status_like line=\"%s\"", sanitize_text_for_api(line).c_str());
  }
  return false;
}

void JuraComponent::process_ble2_transport_probe_(uint32_t now) {
  if (this->ble2_probe_state_ != Ble2ProbeState::WAIT) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->finish_ble2_transport_probe_(now, "controller_not_ready");
    return;
  }

  std::string raw_line;
  while (this->coffee_maker_->connection->read_line_until(raw_line)) {
    this->ble2_probe_frames_++;
    ESP_LOGD(TAG, "ble2_probe_rx_raw probe=%s hex=\"%s\"", this->ble2_probe_name_.c_str(),
             compact_hex_string(raw_line, raw_line.size()).c_str());
    if (!raw_line.empty() && this->xml_decode_inner_transport_ &&
        is_inner_transport_start(static_cast<uint8_t>(raw_line.front()))) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(raw_line);
      bool had_payload = false;
      for (const auto &candidate : candidates) {
        if (candidate.payload.empty()) {
          continue;
        }
        had_payload = true;
        if (this->handle_ble2_probe_line_(candidate.payload, candidate.table_name, now)) {
          return;
        }
      }
      if (!had_payload) {
        ESP_LOGD(TAG, "ble2_probe_rx_noise probe=%s reason=inner_decode_empty",
                 this->ble2_probe_name_.c_str());
      }
    } else if (this->is_printable_status_text_(raw_line)) {
      if (this->handle_ble2_probe_line_(raw_line, "ascii", now)) {
        return;
      }
    } else {
      ESP_LOGD(TAG, "ble2_probe_rx_noise probe=%s reason=non_printable hex=\"%s\"",
               this->ble2_probe_name_.c_str(), compact_hex_string(raw_line, raw_line.size()).c_str());
    }
  }

  if (time_reached(now, this->ble2_probe_deadline_ms_)) {
    ESP_LOGD(TAG, "ble2_probe_timeout probe=%s frames=%u", this->ble2_probe_name_.c_str(),
             static_cast<unsigned>(this->ble2_probe_frames_));
    this->finish_ble2_transport_probe_(now, "timeout");
  }
}

void JuraComponent::finish_ble2_transport_probe_(uint32_t now, const char *result) {
  const char *safe_result = result != nullptr ? result : "done";
  const std::string probe = this->ble2_probe_name_;
  const uint16_t frames = this->ble2_probe_frames_;
  ESP_LOGD(TAG, "ble2_probe_done probe=%s result=%s frames=%u", probe.empty() ? "(none)" : probe.c_str(), safe_result,
           static_cast<unsigned>(frames));
  if (this->db_transaction_owner_ == DbTransactionOwner::BLE2_PROBE) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
  }
  this->post_gate_tx_ready_event_ = true;
  this->ble2_probe_state_ = Ble2ProbeState::IDLE;
  this->ble2_probe_name_.clear();
  this->ble2_probe_command_.clear();
  this->ble2_probe_deadline_ms_ = 0;
  this->publish_status_probe_last_response_("ble2 " + probe + " -> " + safe_result +
                                            " frames=" + std::to_string(frames));
  (void) now;
}

bool JuraComponent::start_debug_command_(const std::string &command, const std::string &transport, uint32_t now) {
  std::string normalized = this->normalize_debug_command_(command);
  std::string transport_normalized = to_lower_copy(transport);
  trim_in_place(transport_normalized);
  if (transport_normalized.empty()) {
    transport_normalized = "inner_uart0";
  }

  if (normalized.empty()) {
    ESP_LOGW(TAG, "debug_command_blocked reason=empty");
    this->publish_debug_command_last_response_("blocked empty");
    return false;
  }
  if (normalized.size() > kDebugCommandMaxLength) {
    ESP_LOGW(TAG, "debug_command_blocked reason=too_long len=%u", static_cast<unsigned>(normalized.size()));
    this->publish_debug_command_last_response_(normalized + " -> blocked too_long");
    return false;
  }
  if (transport_normalized != "inner_uart0") {
    ESP_LOGW(TAG, "debug_command_blocked cmd=\"%s\" reason=unsupported_transport transport=%s",
             escape_control_text_for_log(normalized).c_str(), transport_normalized.c_str());
    this->publish_debug_command_last_response_(normalized + " -> blocked unsupported_transport");
    return false;
  }
  if (this->is_unsafe_debug_command_(normalized)) {
    if (!this->allow_unsafe_debug_commands_) {
      ESP_LOGW(TAG, "debug_command_blocked cmd=\"%s\" reason=unsafe_prefix allow_unsafe=false",
               escape_control_text_for_log(normalized).c_str());
      this->publish_debug_command_last_response_("blocked unsafe " + normalized);
      return false;
    }
    ESP_LOGW(TAG, "debug_command_unsafe_allowed cmd=\"%s\"", escape_control_text_for_log(normalized).c_str());
  }

  if (this->debug_command_state_ != DebugCommandState::IDLE || this->ble2_probe_state_ != Ble2ProbeState::IDLE ||
      this->status_probe_state_ != StatusProbeState::IDLE) {
    ESP_LOGD(TAG, "debug_command_skip reason=uart_busy_or_stats_active cmd=\"%s\"",
             escape_control_text_for_log(normalized).c_str());
    this->publish_debug_command_last_response_(normalized + " -> busy");
    return false;
  }
  if (!this->stats_session_ready_ || !this->stats_inner_tx_required_) {
    ESP_LOGD(TAG, "debug_command_skip reason=post_gate_not_ready cmd=\"%s\"",
             escape_control_text_for_log(normalized).c_str());
    this->publish_debug_command_last_response_(normalized + " -> post_gate_not_ready");
    return false;
  }
  if (!this->post_gate_tx_ready_event_ || this->xml_inflight_ ||
      this->db_transaction_owner_ != DbTransactionOwner::NONE || this->is_busy()) {
    ESP_LOGD(TAG, "debug_command_skip reason=uart_busy_or_stats_active cmd=\"%s\" owner=%s",
             escape_control_text_for_log(normalized).c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->publish_debug_command_last_response_(normalized + " -> busy");
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    ESP_LOGD(TAG, "debug_command_skip reason=controller_not_ready cmd=\"%s\"",
             escape_control_text_for_log(normalized).c_str());
    this->publish_debug_command_last_response_(normalized + " -> controller_not_ready");
    return false;
  }

  this->db_transaction_owner_ = DbTransactionOwner::DEBUG_COMMAND;
  this->post_gate_tx_ready_event_ = false;
  this->debug_command_state_ = DebugCommandState::WAIT;
  this->debug_command_cmd_ = normalized;
  this->debug_command_transport_ = transport_normalized;
  this->debug_command_deadline_ms_ = now + kDebugCommandTimeoutMs;
  this->debug_command_frames_ = 0;
  this->coffee_maker_->connection->reset_response_line_buffer();

  ESP_LOGD(TAG, "debug_command_start cmd=\"%s\" transport=%s",
           escape_control_text_for_log(normalized).c_str(), transport_normalized.c_str());
  ESP_LOGD(TAG, "debug_command_tx cmd=\"%s\" transport=%s timeout_ms=%u",
           escape_control_text_for_log(normalized).c_str(), transport_normalized.c_str(),
           static_cast<unsigned>(kDebugCommandTimeoutMs));
  if (!this->write_inner_uart0_command_(normalized, now, true)) {
    this->finish_debug_command_(now, "tx_failed");
    return false;
  }
  this->publish_debug_command_last_response_(normalized + " -> sent");
  return true;
}

bool JuraComponent::handle_debug_command_line_(const std::string &line, const char *table_name, uint32_t now) {
  std::string trimmed = line;
  trim_in_place(trimmed);
  if (trimmed.empty()) {
    return false;
  }
  std::string lower = to_lower_copy(trimmed);
  const char *table = table_name != nullptr ? table_name : "ascii";
  this->update_dongle_events_from_line_(trimmed);
  this->log_status_forensics_decoded_(trimmed, "debug_command", table);
  this->publish_debug_command_last_response_(this->debug_command_cmd_ + " -> " + sanitize_text_for_api(trimmed));
  ESP_LOGD(TAG, "debug_command_rx_decoded cmd=\"%s\" line=\"%s\"",
           escape_control_text_for_log(this->debug_command_cmd_).c_str(), sanitize_text_for_api(trimmed).c_str());

  if (lower.rfind("@tf:", 0) == 0) {
    ESP_LOGD(TAG, "debug_command_detected_tf line=\"%s\"", sanitize_text_for_api(trimmed).c_str());
    this->publish_tf_status_(trimmed);
    this->finish_debug_command_(now, "detected_tf");
    return true;
  }
  if (lower.rfind("@tv:", 0) == 0) {
    ESP_LOGD(TAG, "debug_command_detected_tv line=\"%s\"", sanitize_text_for_api(trimmed).c_str());
    this->handle_tv_progress_(trimmed);
    this->finish_debug_command_(now, "detected_tv");
    return true;
  }
  if (trimmed.rfind("@T2", 0) == 0) {
    ESP_LOGD(TAG, "debug_command_detected_t2 line=\"%s\"", sanitize_text_for_api(trimmed).c_str());
    this->handle_t2_status_debug_(trimmed);
  }
  if (lower.rfind("@", 0) == 0) {
    ESP_LOGD(TAG, "debug_command_detected_status_like line=\"%s\"", sanitize_text_for_api(trimmed).c_str());
  }
  return false;
}

void JuraComponent::process_debug_command_(uint32_t now) {
  if (this->debug_command_state_ != DebugCommandState::WAIT) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->finish_debug_command_(now, "controller_not_ready");
    return;
  }

  std::string raw_line;
  while (this->coffee_maker_->connection->read_line_until(raw_line)) {
    this->debug_command_frames_++;
    ESP_LOGD(TAG, "debug_command_rx_raw cmd=\"%s\" hex=\"%s\"",
             escape_control_text_for_log(this->debug_command_cmd_).c_str(),
             compact_hex_string(raw_line, raw_line.size()).c_str());
    this->log_status_forensics_frame_(raw_line, "debug_command");
    if (!raw_line.empty() && this->xml_decode_inner_transport_ &&
        is_inner_transport_start(static_cast<uint8_t>(raw_line.front()))) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(raw_line);
      bool had_payload = false;
      for (const auto &candidate : candidates) {
        if (candidate.payload.empty()) {
          continue;
        }
        had_payload = true;
        ESP_LOGD(TAG, "debug_command_rx_decoded_candidate cmd=\"%s\" table=%s line=\"%s\"",
                 escape_control_text_for_log(this->debug_command_cmd_).c_str(), candidate.table_name,
                 transport_payload_log_text(candidate.payload).c_str());
        if (this->handle_debug_command_line_(candidate.payload, candidate.table_name, now)) {
          return;
        }
      }
      if (!had_payload) {
        ESP_LOGD(TAG, "debug_command_rx_noise cmd=\"%s\" reason=inner_decode_empty",
                 escape_control_text_for_log(this->debug_command_cmd_).c_str());
      }
    } else if (this->is_printable_status_text_(raw_line)) {
      if (this->handle_debug_command_line_(raw_line, "ascii", now)) {
        return;
      }
    } else {
      ESP_LOGD(TAG, "debug_command_rx_noise cmd=\"%s\" reason=non_printable hex=\"%s\"",
               escape_control_text_for_log(this->debug_command_cmd_).c_str(),
               compact_hex_string(raw_line, raw_line.size()).c_str());
    }
  }

  if (time_reached(now, this->debug_command_deadline_ms_)) {
    ESP_LOGD(TAG, "debug_command_timeout cmd=\"%s\" frames=%u",
             escape_control_text_for_log(this->debug_command_cmd_).c_str(),
             static_cast<unsigned>(this->debug_command_frames_));
    this->finish_debug_command_(now, "timeout");
  }
}

void JuraComponent::finish_debug_command_(uint32_t now, const char *result) {
  const char *safe_result = result != nullptr ? result : "done";
  const std::string command = this->debug_command_cmd_;
  const uint16_t frames = this->debug_command_frames_;
  ESP_LOGD(TAG, "debug_command_done cmd=\"%s\" result=%s frames=%u",
           command.empty() ? "(none)" : escape_control_text_for_log(command).c_str(), safe_result,
           static_cast<unsigned>(frames));
  if (this->db_transaction_owner_ == DbTransactionOwner::DEBUG_COMMAND) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
  }
  this->post_gate_tx_ready_event_ = true;
  this->debug_command_state_ = DebugCommandState::IDLE;
  this->debug_command_cmd_.clear();
  this->debug_command_transport_.clear();
  this->debug_command_deadline_ms_ = 0;
  this->publish_debug_command_last_response_((command.empty() ? std::string{} : command + " -> ") + safe_result +
                                             " frames=" + std::to_string(frames));
  (void) now;
}

void JuraComponent::process_machine_data_query() {
  uint32_t now = esphome::millis();
  if (!this->enable_machine_xml_poll_) {
    return;
  }
  if (this->machine_data_sensor_ == nullptr && this->machine_status_sensor_ == nullptr) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  if (this->is_busy()) {
    return;
  }
  bool xml_poll_active = this->xml_inflight_ || this->db_transaction_owner_ != DbTransactionOwner::NONE ||
                         this->xml_command_probe_ ||
                         this->xml_session_probe_ ||
                         (this->enable_xml_poll_ && this->xml_transport_selftest_) ||
                         (this->enable_xml_poll_ && this->xml_state_ != XmlPollState::IDLE &&
                          this->xml_state_ != XmlPollState::SLEEP) ||
                         (this->enable_xml_poll_ && this->xml_dongle_startup_ && !this->stats_session_ready_) ||
                         (this->enable_xml_poll_ && this->xml_run_tablet_start_sequence_ &&
                          !this->xml_tablet_start_sequence_done_);
  if (xml_poll_active) {
    if (this->machine_xml_busy_backoff_until_ != 0 && !time_reached(now, this->machine_xml_busy_backoff_until_)) {
      return;
    }
    ESP_LOGD(TAG, "Machine-XML query skipped while XML DB polling is active");
    this->machine_xml_busy_backoff_until_ = now + MACHINE_XML_BUSY_BACKOFF_MS;
    this->machine_data_query_next_ = this->machine_xml_busy_backoff_until_;
    return;
  }

  if (this->machine_data_query_next_ != 0 && !time_reached(now, this->machine_data_query_next_)) {
    return;
  }
  this->machine_data_query_next_ = now + MACHINE_DATA_QUERY_INTERVAL_MS;
    std::string xml;
    if (!this->request_machine_xml_(xml)) {
      ESP_LOGW(TAG, "Machine-XML konnte nicht abgefragt werden.");
      return;
    }

    if (!xml.empty()) {
      this->handle_machine_xml_(xml);
    }
  //std::string xml;
  //if (!this->request_machine_xml_(xml)) {
  //  ESP_LOGW(TAG, "Machine-XML konnte nicht abgefragt werden.");
  //  return;
  //}

  //this->handle_machine_xml_(xml);
}

void JuraComponent::publish_machine_data_(const std::string &response) {
  std::string sanitized = response;
  sanitized.erase(std::remove_if(sanitized.begin(), sanitized.end(),
                                 [](unsigned char c) { return c == '\r' || c == '\n'; }),
                  sanitized.end());
  ESP_LOGD(TAG, "Machine data response: %s", sanitize_text_for_api(sanitized).c_str());
  if (!this->is_printable_status_text_(sanitized)) {
    ESP_LOGW(TAG, "Machine data publish skipped (binary/non-printable payload)");
    return;
  }
  if (this->machine_data_sensor_ != nullptr) {
    std::string safe = sanitize_text_for_api(sanitized);
    this->machine_data_sensor_->publish_state(safe);
  }
}

bool JuraComponent::request_machine_xml_(std::string &xml) {
  xml.clear();
  if (!this->enable_machine_xml_poll_) {
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  bool xml_poll_active = this->xml_inflight_ || this->db_transaction_owner_ == DbTransactionOwner::XML_POLL ||
                         this->xml_command_probe_ ||
                         this->xml_session_probe_ ||
                         (this->enable_xml_poll_ && this->xml_transport_selftest_) ||
                         (this->enable_xml_poll_ && this->xml_state_ != XmlPollState::IDLE &&
                          this->xml_state_ != XmlPollState::SLEEP) ||
                         (this->enable_xml_poll_ && this->xml_dongle_startup_ && !this->stats_session_ready_) ||
                         (this->enable_xml_poll_ && this->xml_run_tablet_start_sequence_ &&
                          !this->xml_tablet_start_sequence_done_);
  if (xml_poll_active) {
    uint32_t now = esphome::millis();
    this->machine_xml_busy_backoff_until_ = now + MACHINE_XML_BUSY_BACKOFF_MS;
    ESP_LOGD(TAG, "Machine-XML query skipped while XML DB polling is active");
    return false;
  }

  auto *connection = this->coffee_maker_->connection.get();

  auto send_and_receive = [&](const char *command, std::string &out) -> bool {
    if (command == nullptr || command[0] == '\0') {
      return false;
    }
    if (this->db_transaction_owner_ != DbTransactionOwner::NONE) {
      uint32_t now = esphome::millis();
      this->machine_xml_busy_backoff_until_ = now + MACHINE_XML_BUSY_BACKOFF_MS;
      ESP_LOGD(TAG, "Machine-XML command %s skipped; DB transaction already active", command);
      return false;
    }
    this->db_transaction_owner_ = DbTransactionOwner::MACHINE_XML;
    connection->reset_db_rx_buffer();
    if (!connection->write_decoded(command)) {
      ESP_LOGW(TAG, "Machine-XML Befehl %s konnte nicht gesendet werden.", command);
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
    std::vector<uint8_t> decoded;
    bool had_crlf = false;
    size_t decoded_len = 0;
    if (!connection->read_db_frame(decoded, MACHINE_XML_TIMEOUT_MS, &had_crlf, &decoded_len)) {
      ESP_LOGW(TAG, "Machine-XML timeout for %s", command);
      connection->reset_db_rx_buffer();
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
    if (decoded.empty()) {
      this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
      return false;
    }
      out.assign(decoded.begin(), decoded.end());
      if (!this->is_printable_status_text_(out)) {
        ESP_LOGD(TAG, "Machine-XML received binary status response; handled by DB frame decoder");
        connection->reset_db_rx_buffer();
        out.clear();
        this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
        return true;
      }
    //out.assign(decoded.begin(), decoded.end());
    //if (!this->is_printable_status_text_(out)) {
    //  ESP_LOGW(TAG, "Machine-XML ignored binary response");
    //  connection->reset_db_rx_buffer();
    //  out.clear();
    //  this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
    //  return false;
    //}
    out.erase(std::remove(out.begin(), out.end(), '\r'), out.end());
    trim_in_place(out);
    bool ok = !out.empty();
    this->clear_db_transaction_(DbTransactionOwner::MACHINE_XML);
    return ok;
  };
    std::string primary;
    if (send_and_receive(MACHINE_XML_PRIMARY_COMMAND, primary)) {
      if (primary.empty()) {
        return true;  // binäre Statusantwort wurde bereits über db_frame verarbeitet
      }
      if (primary.size() >= MACHINE_XML_MIN_LENGTH) {
        xml.swap(primary);
        return true;
      }
      ESP_LOGW(TAG, "Machine-XML Antwort zu kurz (%u Byte) – versuche Fallback.",
               static_cast<unsigned>(primary.size()));
    } else {
      ESP_LOGW(TAG, "Machine-XML Primärkommando ohne Antwort – versuche Fallback.");
    }
  //std::string primary;
  //if (send_and_receive(MACHINE_XML_PRIMARY_COMMAND, primary)) {
  //  if (primary.size() >= MACHINE_XML_MIN_LENGTH) {
  //    xml.swap(primary);
  //    return true;
  //  }
  //  ESP_LOGW(TAG, "Machine-XML Antwort zu kurz (%u Byte) – versuche Fallback.",
  //           static_cast<unsigned>(primary.size()));
  //} else {
  //  ESP_LOGW(TAG, "Machine-XML Primärkommando ohne Antwort – versuche Fallback.");
  //}

  std::string fallback;
  if (send_and_receive(MACHINE_XML_FALLBACK_COMMAND, fallback)) {
    xml.swap(fallback);
    return true;
  }

  if (!primary.empty()) {
    xml.swap(primary);
    return true;
  }

  return false;
}

void JuraComponent::handle_machine_xml_(const std::string &xml) {
  if (!this->is_printable_status_text_(xml)) {
    ESP_LOGW(TAG, "Machine-XML ignored binary response");
    return;
  }
  if (xml.find('<') == std::string::npos || xml.find('>') == std::string::npos) {
    ESP_LOGW(TAG, "Machine-XML ignored non-XML response");
    return;
  }
  uint32_t now = esphome::millis();
  std::string normalized = xml;
  normalized.erase(std::remove(normalized.begin(), normalized.end(), '\r'), normalized.end());
  this->machine_xml_cache_ = normalized;
  this->machine_xml_timestamp_ = now;

  std::string summary = this->format_machine_status_summary_(normalized);
  if (this->is_printable_status_text_(summary)) {
    this->publish_machine_status_(summary);
    this->publish_machine_data_(summary);
  }
  this->update_settings_from_xml_(normalized);
  this->update_errors_from_xml_(normalized);
}

bool JuraComponent::ensure_machine_xml_(uint32_t max_age_ms, std::string &xml_out) {
  uint32_t now = esphome::millis();
  if (!this->machine_xml_cache_.empty()) {
    bool fresh = max_age_ms == 0;
    if (!fresh && this->machine_xml_timestamp_ != 0) {
      fresh = static_cast<int32_t>(now - this->machine_xml_timestamp_) <= static_cast<int32_t>(max_age_ms);
    }
    if (fresh) {
      xml_out = this->machine_xml_cache_;
      return true;
    }
  }

  std::string fetched;
  if (!this->request_machine_xml_(fetched)) {
    return false;
  }
  this->handle_machine_xml_(fetched);
  xml_out = this->machine_xml_cache_;
  return !xml_out.empty();
}

std::string JuraComponent::format_machine_status_summary_(const std::string &xml) const {
  std::vector<std::pair<std::string, std::string>> fields;
  std::string value;
  if (this->xml_get_value_(xml, "Machine/Status/State", value) && !value.empty()) {
    fields.emplace_back("State", value);
  }
  if (this->xml_get_value_(xml, "Machine/Status/WaterLevel", value) && !value.empty()) {
    fields.emplace_back("Water", value);
  }
  if (this->xml_get_value_(xml, "Machine/Status/BeanLevel", value) && !value.empty()) {
    fields.emplace_back("Beans", value);
  }
  if (this->xml_get_value_(xml, "Machine/Status/ErrorCode", value) && !value.empty()) {
    fields.emplace_back("Error", value);
  }

  if (fields.empty()) {
    std::string collapsed = collapse_whitespace(xml);
    if (collapsed.size() > 160) {
      collapsed.resize(157);
      collapsed.append("...");
    }
    return collapsed;
  }

  std::string summary;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i > 0) {
      summary.append(", ");
    }
    summary.append(fields[i].first);
    summary.push_back('=');
    summary.append(fields[i].second);
  }
  return summary;
}

void JuraComponent::update_settings_from_xml_(const std::string &xml) {
  this->ensure_setting_entities_created_();
  if (this->setting_descs_.empty()) {
    return;
  }

  for (const auto &entry : this->setting_descs_) {
    const auto &desc = entry.second;
    if (!desc.source_cmd.empty()) {
      continue;
    }
    std::string path = this->determine_setting_path_(desc);
    if (path.empty()) {
      continue;
    }
    std::string raw_value;
    if (!this->xml_get_value_(xml, path, raw_value)) {
      continue;
    }
    if (desc.type == SettingValueType::String) {
      this->publish_setting_value_(desc, 0.0f, raw_value);
      continue;
    }
    if (raw_value.empty()) {
      continue;
    }

    double numeric = 0.0;
    bool parsed = false;
    {
      char *end = nullptr;
      numeric = std::strtod(raw_value.c_str(), &end);
      parsed = end != nullptr && end != raw_value.c_str();
      if (!parsed) {
        long long as_int = std::strtoll(raw_value.c_str(), &end, 0);
        if (end != nullptr && end != raw_value.c_str()) {
          numeric = static_cast<double>(as_int);
          parsed = true;
        }
      }
    }
    if (!parsed) {
      std::string lowered = raw_value;
      std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (desc.type == SettingValueType::Bool) {
        bool value_bool = (lowered == "true" || lowered == "on" || lowered == "1" || lowered == "yes");
        this->publish_setting_value_(desc, value_bool ? 1.0f : 0.0f, value_bool ? "on" : "off");
      }
      continue;
    }

    double scaled = numeric * static_cast<double>(desc.scale);
    std::string text_value;
    float publish_value = static_cast<float>(scaled);
    switch (desc.type) {
      case SettingValueType::Bool: {
        bool value_bool = scaled != 0.0;
        publish_value = value_bool ? 1.0f : 0.0f;
        text_value = value_bool ? "on" : "off";
        break;
      }
      case SettingValueType::String:
        text_value = raw_value;
        break;
      case SettingValueType::Enum:
      case SettingValueType::U8:
      case SettingValueType::U16:
      case SettingValueType::U32:
        text_value = format_numeric_text(scaled);
        break;
    }
    this->publish_setting_value_(desc, publish_value, text_value);
  }
}

void JuraComponent::update_errors_from_xml_(const std::string &xml) {
  std::string path = error_source_path();
  if (path.empty()) {
    path = "Machine/Status/ErrorCode";
  }
  if (path.empty()) {
    return;
  }
  std::string raw;
  if (!this->xml_get_value_(xml, path, raw)) {
    return;
  }
  if (raw.empty()) {
    if (this->errors_entities_created_ && this->last_error_code_ != 0) {
      this->publish_error_state_(0);
    }
    return;
  }
  char *end = nullptr;
  uint32_t code = static_cast<uint32_t>(std::strtoul(raw.c_str(), &end, 0));
  if (end == raw.c_str()) {
    return;
  }
  if (!this->errors_entities_created_ || code != this->last_error_code_) {
    this->publish_error_state_(code);
  }
}

bool JuraComponent::xml_get_value_(const std::string &xml, const std::string &path, std::string &out) const {
  return xml_get_value_simple(xml, path, out);
}

std::string JuraComponent::determine_setting_path_(const SettingDesc &desc) const {
  if (!desc.path.empty()) {
    return desc.path;
  }
  if (desc.id.empty()) {
    return {};
  }
  std::string tag = to_pascal_case(desc.id);
  if (tag.empty()) {
    tag = desc.id;
  }
  return "Machine/Settings/" + tag;
}

bool JuraComponent::decode_field_value_(const std::vector<uint8_t> &decoded, const XmlField &field,
                                        bool little_endian, std::uint64_t &out) const {
  if (field.offset + field.size > decoded.size()) {
    ESP_LOGW(TAG, "XML @TG:C0 Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", field.name.c_str(),
             static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
             static_cast<unsigned>(decoded.size()));
    return false;
  }
  out = 0;
  if (little_endian) {
    for (std::size_t i = 0; i < field.size; ++i) {
      out |= static_cast<std::uint64_t>(decoded[field.offset + i]) << (8U * i);
    }
  } else {
    for (std::size_t i = 0; i < field.size; ++i) {
      out = (out << 8U) | static_cast<std::uint64_t>(decoded[field.offset + i]);
    }
  }
  return true;
}

void JuraComponent::start_new_xml_cycle_(uint32_t now) {
  this->xml_cycle_started_ms_ = now;
  this->xml_stats_.clear();
  this->xml_rx_buffer_.clear();
  this->xml_rx_line_.clear();
  this->xml_inflight_ = false;
  this->xml_last_command_.clear();
  this->xml_expected_prefix_.clear();
  this->xml_tr32_page_ = 0;
  this->xml_binary_probe_prev_tr32_payload_.clear();
  this->xml_binary_probe_prev_tr32_page_ = 0;
  this->xml_binary_probe_has_prev_tr32_ = false;
  this->xml_stats_locked_ = false;
  this->xml_cycle_failed_ = false;
  this->xml_stats_consecutive_failures_ = 0;
  this->xml_tr32_pages_ok_ = 0;
  this->xml_tg43_ok_ = false;
  this->xml_tgc0_ok_ = false;
  this->xml_stats_reject_reason_.clear();
  this->xml_stats_reject_decoded_.clear();
  this->xml_stats_binary_response_ = false;
  this->xml_command_started_ms_ = 0;
  this->xml_command_frames_ = 0;
  this->xml_command_noise_frames_ = 0;
  this->xml_retry_count_.fill(0);
  this->xml_invalid_len_seen_.fill(false);
  this->xml_last_invalid_len_.fill(0);
  this->transport_selftest_state_ = TransportSelftestState::IDLE;
  this->transport_selftest_rx_buffer_.clear();
  this->transport_selftest_current_cmd_.clear();
  this->transport_selftest_deadline_ms_ = 0;
  this->xml_command_probe_state_ = XmlCommandProbeState::IDLE;
  this->xml_command_probe_index_ = 0;
  this->xml_command_probe_rx_buffer_.clear();
  this->xml_command_probe_current_cmd_.clear();
  this->xml_command_probe_deadline_ms_ = 0;
  this->xml_command_probe_next_ms_ = 0;
  this->xml_command_probe_last_wait_reason_.clear();
  this->xml_session_probe_state_ = XmlSessionProbeState::IDLE;
  this->xml_session_probe_index_ = 0;
  this->xml_session_probe_rx_buffer_.clear();
  this->xml_session_probe_current_cmd_.clear();
  this->xml_session_probe_deadline_ms_ = 0;
  this->xml_session_probe_next_ms_ = 0;
  this->xml_session_probe_last_wait_reason_.clear();
  this->xml_session_probe_timeouts_ = 0;
}

size_t JuraComponent::xml_command_index_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return 0;
    case XmlPollState::SEND_TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::PARSE_TG43:
      return 1;
    case XmlPollState::SEND_TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::PARSE_TGC0:
      return 2;
    default:
      break;
  }
  return 0;
}

const char *JuraComponent::db_transaction_owner_name_(DbTransactionOwner owner) const {
  switch (owner) {
    case DbTransactionOwner::NONE:
      return "none";
    case DbTransactionOwner::XML_POLL:
      return "xml_poll";
    case DbTransactionOwner::MACHINE_XML:
      return "machine_xml";
  }
  return "unknown";
}

bool JuraComponent::begin_xml_transaction_(const char *command, uint32_t now) {
  (void) now;
  const char *cmd = command != nullptr ? command : "";
  if (this->db_transaction_owner_ == DbTransactionOwner::XML_POLL && !this->xml_inflight_) {
    ESP_LOGD(TAG, "xml_tx_end cmd=%s reason=stale_before_tx",
             this->xml_transaction_cmd_.empty() ? this->xml_last_command_.c_str() : this->xml_transaction_cmd_.c_str());
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
    this->xml_transaction_cmd_.clear();
  }
  if (this->db_transaction_owner_ != DbTransactionOwner::NONE) {
    ESP_LOGD(TAG, "xml_tx_skip cmd=%s reason=busy busy_owner=%s", cmd,
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    return false;
  }
  if (this->xml_inflight_) {
    ESP_LOGD(TAG, "xml_tx_skip cmd=%s reason=inflight busy_owner=%s", cmd,
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    return false;
  }
  this->db_transaction_owner_ = DbTransactionOwner::XML_POLL;
  this->xml_transaction_cmd_ = cmd;
  ESP_LOGD(TAG, "xml_tx_begin cmd=%s", cmd);
  return true;
}

void JuraComponent::end_xml_transaction_(const char *reason) {
  const char *end_reason = reason != nullptr && reason[0] != '\0' ? reason : "done";
  if (this->db_transaction_owner_ == DbTransactionOwner::XML_POLL || !this->xml_transaction_cmd_.empty()) {
    const char *cmd = !this->xml_transaction_cmd_.empty() ? this->xml_transaction_cmd_.c_str()
                                                           : this->xml_last_command_.c_str();
    ESP_LOGD(TAG, "xml_tx_end cmd=%s reason=%s", cmd, end_reason);
  }
  if (this->db_transaction_owner_ == DbTransactionOwner::XML_POLL) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
  }
  if (!this->xml_transaction_cmd_.empty() && this->xml_last_command_ == this->xml_transaction_cmd_) {
    this->xml_last_command_.clear();
  }
  this->xml_transaction_cmd_.clear();
}

void JuraComponent::clear_db_transaction_(DbTransactionOwner owner) {
  if (owner == DbTransactionOwner::NONE || this->db_transaction_owner_ == owner) {
    this->db_transaction_owner_ = DbTransactionOwner::NONE;
    if (owner == DbTransactionOwner::NONE || owner == DbTransactionOwner::XML_POLL) {
      this->xml_transaction_cmd_.clear();
    }
  }
}

void JuraComponent::flush_xml_rx_(bool flush_serial) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  this->coffee_maker_->connection->reset_db_rx_buffer();
  if (flush_serial) {
    this->coffee_maker_->connection->drain_serial_input_nonblocking();
  }
}

bool JuraComponent::validate_xml_frame_(XmlPollState state, const std::vector<uint8_t> &decoded, bool had_crlf,
                                        size_t decoded_len, std::vector<uint8_t> &payload,
                                        size_t &expected_min_len, uint8_t &head0, std::string &reason) const {
  (void) had_crlf;
  reason.clear();
  const char *expected_command = this->xml_state_command_(state);
  if (expected_command == nullptr || expected_command[0] == '\0') {
    reason = "missing_expected_command";
    ESP_LOGW(TAG, "XML validate: kein erwarteter Befehl für Zustand %d", static_cast<int>(state));
    return false;
  }
  if (this->xml_last_command_ != expected_command) {
    reason = "pending_command_mismatch";
    ESP_LOGW(TAG, "XML %s verworfen: pending command mismatch (pending=%s expected=%s)",
             this->xml_state_label_(state), this->xml_last_command_.c_str(), expected_command);
    return false;
  }

  const XmlCommandMapping *mapping = nullptr;
  std::size_t minimum = 0;
  switch (state) {
    case XmlPollState::WAIT_TR32:
      mapping = &this->xml_mapping_.tr32;
      minimum = kTR32MinFrameLength;
      break;
    case XmlPollState::WAIT_TG43:
      mapping = &this->xml_mapping_.tg43;
      minimum = kTG43MinFrameLength;
      break;
    case XmlPollState::WAIT_TGC0:
      mapping = &this->xml_mapping_.tgc0;
      minimum = kTGC0MinFrameLength;
      break;
    default:
      reason = "unexpected_state";
      ESP_LOGW(TAG, "XML validate: unerwarteter Zustand %d", static_cast<int>(state));
      return false;
  }

  if (mapping == nullptr || mapping->empty()) {
    reason = "mapping_empty";
    ESP_LOGW(TAG, "XML %s: kein Mapping aktiv", this->xml_state_label_(state));
    return false;
  }

  std::size_t expected_len = 0;
  for (const auto &field : mapping->fields) {
    expected_len = std::max(expected_len, field.offset + field.size);
  }
  expected_min_len = std::max(expected_len, minimum);

  int start_index = -1;
  for (std::size_t i = 0; i < decoded.size(); ++i) {
    if (decoded[i] == 0x26) {
      start_index = static_cast<int>(i);
      break;
    }
  }
  if (start_index < 0) {
    reason = "invalid_start_byte";
    ESP_LOGW(TAG, "XML %s: Startbyte 0x26 nicht gefunden (decoded_len=%u)", this->xml_state_label_(state),
             static_cast<unsigned>(decoded.size()));
    return false;
  }
  if (start_index != 0) {
    reason = "invalid_start_byte";
    ESP_LOGW(TAG, "XML %s verworfen: Startbyte 0x26 erst bei Index %d (decoded_len=%u)",
             this->xml_state_label_(state), start_index, static_cast<unsigned>(decoded.size()));
    return false;
  }

  payload.assign(decoded.begin() + start_index, decoded.end());
  head0 = payload.empty() ? 0x00 : payload.front();

  std::size_t avail = payload.size();
  if (avail != expected_min_len) {
    reason = "invalid_length";
    ESP_LOGW(TAG, "XML %s verworfen: decoded_len (%u) != expected_len (%u)",
             this->xml_state_label_(state), static_cast<unsigned>(avail), static_cast<unsigned>(expected_min_len));
    return false;
  }
  if (head0 != 0x26) {
    reason = "invalid_start_byte";
    ESP_LOGW(TAG, "XML %s: erstes Payload-Byte 0x%02X statt 0x26", this->xml_state_label_(state),
             static_cast<unsigned>(head0));
    return false;
  }

  ESP_LOGD(TAG, "XML RX cmd=%s decoded_len=%u expected_len=%u head0=0x%02X", expected_command,
           static_cast<unsigned>(decoded_len), static_cast<unsigned>(expected_min_len),
           static_cast<unsigned>(head0));
  return true;
}

bool JuraComponent::validate_counter_frame_(XmlPollState state, const XmlCommandMapping &mapping,
                                            const std::vector<uint8_t> &frame, const char *command_label,
                                            std::string &reason) const {
  if (mapping.empty()) {
    reason = "mapping_empty";
    return false;
  }
  if (frame.empty() || frame.front() != 0x26) {
    reason = "missing_frame_header_0x26";
    return false;
  }
  for (const auto &field : mapping.fields) {
    if (field.offset + field.size > frame.size()) {
      reason = "field_overflow:" + field.name;
      return false;
    }
    if (field.size != 1 && field.size != 2 && field.size != 4) {
      reason = "unsupported_field_size:" + field.name;
      return false;
    }
    std::uint64_t raw = 0;
    if (field.little_endian) {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw |= static_cast<std::uint64_t>(frame[field.offset + i]) << (8U * i);
      }
    } else {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw = (raw << 8U) | static_cast<std::uint64_t>(frame[field.offset + i]);
      }
    }
    double value = static_cast<double>(raw) * field.scale;
    if (field.has_add) {
      value += field.add;
    }
    if (!std::isfinite(value)) {
      reason = "non_finite_value:" + field.name;
      return false;
    }
    if (value < 0.0 || value > static_cast<double>(this->xml_counter_max_)) {
      reason = "counter_out_of_range:" + std::string(command_label != nullptr ? command_label : "?") + ":" +
               field.name + "=" + format_numeric_text(value) + " max=" + std::to_string(this->xml_counter_max_);
      return false;
    }
  }
  (void) state;
  reason.clear();
  return true;
}

bool JuraComponent::counter_frame_is_stable_(XmlPollState state, const std::vector<uint8_t> &frame,
                                             const char *command_label, std::string &reason) {
  if (this->xml_publish_unstable_) {
    reason.clear();
    return true;
  }
  size_t index = this->xml_command_index_(state);
  if (index >= this->xml_counter_candidate_frame_.size()) {
    reason = "invalid_command_index";
    return false;
  }
  if (this->xml_counter_candidate_frame_[index] == frame) {
    if (this->xml_counter_candidate_count_[index] < std::numeric_limits<uint8_t>::max()) {
      this->xml_counter_candidate_count_[index] += 1;
    }
  } else {
    this->xml_counter_candidate_frame_[index] = frame;
    this->xml_counter_candidate_count_[index] = 1;
  }
  if (this->xml_counter_candidate_count_[index] < 2) {
    reason = "unstable_frame_waiting_for_repeat:" + std::string(command_label != nullptr ? command_label : "?");
    return false;
  }
  reason.clear();
  return true;
}

bool JuraComponent::stage_counter_frame_(const XmlCommandMapping &mapping, const std::vector<uint8_t> &frame,
                                         const char *command_label) {
  bool any = false;
  for (const auto &field : mapping.fields) {
    if (field.offset + field.size > frame.size()) {
      ESP_LOGW(TAG, "XML %s Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", command_label,
               field.name.c_str(), static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               static_cast<unsigned>(frame.size()));
      continue;
    }
    std::uint64_t raw = 0;
    if (field.little_endian) {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw |= static_cast<std::uint64_t>(frame[field.offset + i]) << (8U * i);
      }
    } else {
      for (std::size_t i = 0; i < field.size; ++i) {
        raw = (raw << 8U) | static_cast<std::uint64_t>(frame[field.offset + i]);
      }
    }
    double value = static_cast<double>(raw) * field.scale;
    if (field.has_add) {
      value += field.add;
    }
    ESP_LOGD(TAG, "XML field %s offset=%u size=%u endian=%s value=%.3f", field.name.c_str(),
             static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
             field.little_endian ? "LE" : "BE", static_cast<double>(value));
    this->xml_stats_.set_value(field.name, value, field.label);
    any = true;
  }
  return any;
}

bool JuraComponent::process_valid_tgc0_frame_(const std::vector<uint8_t> &frame, bool stage_values) {
  const auto &mapping = this->xml_mapping_.tgc0;
  if (mapping.empty()) {
    ESP_LOGW(TAG, "XML @TG:C0: kein Mapping aktiv");
    return true;
  }
  bool any_value = false;
  for (const auto &field : mapping.fields) {
    if (field.size < 4) {
      ESP_LOGW(TAG, "XML @TG:C0 Feld %s ignoriert (nicht unterstützte Größe %u)", field.name.c_str(),
               static_cast<unsigned>(field.size));
      continue;
    }
    if (field.offset + field.size > frame.size()) {
      ESP_LOGW(TAG, "XML @TG:C0 Feld %s überläuft Frame (Offset=%u, Bytes=%u, Frame=%u)", field.name.c_str(),
               static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               static_cast<unsigned>(frame.size()));
      continue;
    }

    uint16_t header_value = frame[field.offset];
    uint16_t encoded_value = frame[field.offset + 1];
    XmlField raw_field = field;
    raw_field.offset = field.offset + field.size - 2;
    raw_field.size = 2;
    bool little_endian = raw_field.has_endian ? raw_field.little_endian : TGC0_TRY_LITTLE_ENDIAN_FIRST;
    std::uint64_t raw_value = 0;
    if (!this->decode_field_value_(frame, raw_field, little_endian, raw_value)) {
      continue;
    }
    double percent = static_cast<double>(raw_value) * field.scale;
    if (field.has_add) {
      percent += field.add;
    }
    if (!std::isfinite(percent)) {
      auto &state = this->tgc0_filters_[field.name];
      state.window.clear();
      state.consecutive_valid = 0;
      continue;
    }
    if (percent < 0.0 || percent > 100.0) {
      ESP_LOGW(TAG, "XML @TG:C0 ungültig: %s raw=%llu percent=%.2f", field.name.c_str(),
               static_cast<unsigned long long>(raw_value), percent);
      return false;
    }
    if (stage_values) {
      ESP_LOGD(TAG, "XML field %s offset=%u size=%u endian=%s value=%.2f", field.name.c_str(),
               static_cast<unsigned>(field.offset), static_cast<unsigned>(field.size),
               little_endian ? "LE" : "BE", percent);
      if (this->stage_tgc0_value_(field.name, field.label, static_cast<float>(percent), header_value, encoded_value,
                                  static_cast<uint16_t>(raw_value & 0xFFFFu))) {
        any_value = true;
      }
    }
  }
  if (stage_values && !any_value) {
    ESP_LOGW(TAG, "XML TGC0: keine gültigen Werte im Frame");
  }
  return true;
}

bool JuraComponent::should_retry_current_(XmlPollState wait_state, uint32_t now) {
  size_t index = this->xml_command_index_(wait_state);
  if (this->xml_retry_count_[index] >= 1) {
    return false;
  }
  this->xml_retry_count_[index] += 1;
  XmlPollState resend_state = wait_state == XmlPollState::WAIT_TR32
                                  ? XmlPollState::SEND_TR32
                                  : (wait_state == XmlPollState::WAIT_TG43 ? XmlPollState::SEND_TG43
                                                                           : XmlPollState::SEND_TGC0);
  ESP_LOGW(TAG, "XML %s: erneuter Versuch", this->xml_state_label_(wait_state));
  ESP_LOGD(TAG, "xml_retry_scheduled cmd=%s delay_ms=%u", this->xml_state_command_(wait_state),
           static_cast<unsigned>(kInterCmdGapMs));
  this->transition_to_state_(resend_state, now, kInterCmdGapMs);
  return true;
}

void JuraComponent::handle_xml_failure_(XmlPollState wait_state, bool is_timeout, size_t decoded_len, uint32_t now,
                                        const char *reason) {
  size_t index = this->xml_command_index_(wait_state);
  const char *failure_reason = reason != nullptr && reason[0] != '\0' ? reason : (is_timeout ? "timeout" : "invalid_frame");
  if (index < this->xml_counter_candidate_frame_.size()) {
    this->xml_counter_candidate_frame_[index].clear();
    this->xml_counter_candidate_count_[index] = 0;
  }
  if (wait_state == XmlPollState::WAIT_TGC0 && is_timeout) {
    if (this->xml_tgc0_timeout_streak_ < std::numeric_limits<uint8_t>::max()) {
      this->xml_tgc0_timeout_streak_ += 1;
    }
    if (this->xml_tgc0_timeout_streak_ >= 3) {
      ESP_LOGW(TAG, "XML @TG:C0 drei Timeouts – überspringe nächsten Versuch");
      this->xml_skip_tgc0_ = true;
      this->xml_tgc0_timeout_streak_ = 0;
    }
  }

  if (!is_timeout) {
    if (!this->xml_invalid_len_seen_[index]) {
      this->xml_invalid_len_seen_[index] = true;
      this->xml_last_invalid_len_[index] = decoded_len;
    } else if (decoded_len != 0 && decoded_len != this->xml_last_invalid_len_[index]) {
      ESP_LOGW(TAG, "XML %s: unterschiedliche decoded_len (%u vs %u) – überspringe", this->xml_state_label_(wait_state),
               static_cast<unsigned>(this->xml_last_invalid_len_[index]), static_cast<unsigned>(decoded_len));
      this->xml_retry_count_[index] = 1;
      this->xml_invalid_len_seen_[index] = false;
    }
  } else {
    this->xml_invalid_len_seen_[index] = false;
    this->xml_last_invalid_len_[index] = 0;
  }
  this->flush_xml_rx_(is_timeout);
  this->end_xml_transaction_(failure_reason);

  this->xml_inflight_ = false;
  this->xml_deadline_ms_ = 0;
  this->xml_rx_buffer_.clear();

  if (this->should_retry_current_(wait_state, now)) {
    return;
  }
  this->xml_retry_count_[index] = 0;
  ESP_LOGD(TAG, "xml_tx_end cmd=%s reason=retry_exhausted", this->xml_state_command_(wait_state));
  XmlPollState next_state = wait_state == XmlPollState::WAIT_TR32
                                ? XmlPollState::SEND_TG43
                                : (wait_state == XmlPollState::WAIT_TG43 ? XmlPollState::SEND_TGC0
                                                                          : XmlPollState::SLEEP);
  if (next_state == XmlPollState::SLEEP) {
    uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
    uint32_t deadline = now + sleep;
    this->xml_deadline_ms_ = deadline;
    this->xml_next_poll_ = deadline;
    this->transition_to_state_(XmlPollState::SLEEP, now);
  } else {
    this->transition_to_state_(next_state, now, kInterCmdGapMs);
  }
}

void JuraComponent::complete_command_success_(XmlPollState wait_state) {
  size_t index = this->xml_command_index_(wait_state);
  this->xml_retry_count_[index] = 0;
  this->xml_invalid_len_seen_[index] = false;
  this->xml_last_invalid_len_[index] = 0;
  if (wait_state == XmlPollState::WAIT_TGC0) {
    this->xml_tgc0_timeout_streak_ = 0;
  }
}

bool JuraComponent::stage_tgc0_value_(const std::string &name, const std::string &label, float raw_percent,
                                      uint16_t header_value, uint16_t encoded_value, uint16_t raw_value) {
  auto &state = this->tgc0_filters_[name];
  (void) header_value;
  (void) encoded_value;
  (void) raw_value;
  state.window.clear();
  state.consecutive_valid = 0;
  this->xml_stats_.set_value(name, raw_percent, label);
  return true;
}

void JuraComponent::add_configured_xml_sensor(const std::string &field, sensor::Sensor *sensor) {
  if (field.empty() || sensor == nullptr) {
    return;
  }
  this->xml_sensors_[field] = sensor;
  this->xml_unconfigured_sensor_logged_.erase(field);
  if (this->xml_sensor_meta_.find(field) != this->xml_sensor_meta_.end()) {
    this->apply_sensor_metadata_(field, sensor);
  }
}

void JuraComponent::register_setting_sensor(const std::string &id, sensor::Sensor *sensor) {
  if (id.empty() || sensor == nullptr) {
    return;
  }
  this->setting_sensors_[id] = sensor;
  this->settings_entities_created_ = false;
}

void JuraComponent::register_setting_text_sensor(const std::string &id, text_sensor::TextSensor *sensor) {
  if (id.empty() || sensor == nullptr) {
    return;
  }
  this->setting_text_sensors_[id] = sensor;
  this->settings_entities_created_ = false;
}

void JuraComponent::publish_xml_stats_() {
  if (!this->enable_xml_poll_) {
    return;
  }
  if (this->xml_stats_.empty()) {
    return;
  }

  const auto &stats = this->xml_stats_.values();
  for (const auto &entry : stats) {
    this->publish_single_stat_(entry.first, entry.second.value, entry.second.label);
  }
}

void JuraComponent::publish_single_stat_(const std::string &name, double value, const std::string &label) {
  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it == this->xml_sensor_meta_.end() || !meta_it->second.configured) {
    ESP_LOGD(TAG, "XML Feld %s hat keine Sensor-Metadaten – Wert wird verworfen", name.c_str());
    return;
  }
  auto &meta = meta_it->second;
  auto *sensor = this->find_configured_sensor_(name);
  if (sensor == nullptr) {
    auto logged_it = this->xml_unconfigured_sensor_logged_.find(name);
    bool already_logged = logged_it != this->xml_unconfigured_sensor_logged_.end() && logged_it->second;
    if (!already_logged) {
      ESP_LOGD(TAG, "XML Feld %s ist nicht in der YAML als Sensor verlinkt – Wert wird verworfen", name.c_str());
      this->xml_unconfigured_sensor_logged_[name] = true;
    }
    return;
  }

  uint32_t now = esphome::millis();
  float publish_value = static_cast<float>(value);
  if (meta.kind == XmlSensorKind::Counter) {
    if (publish_value < 0.0f) {
      ESP_LOGD(TAG, "XML counter negativ verworfen: %s=%.3f", name.c_str(), static_cast<double>(publish_value));
      return;
    }
    publish_value = static_cast<float>(std::round(static_cast<double>(publish_value)));
  }

  if (!std::isfinite(publish_value)) {
    ESP_LOGW(TAG, "XML publish_state unterdrückt: %s ist nicht endlich", name.c_str());
    return;
  }

  float tolerance = (meta.kind == XmlSensorKind::Counter) ? XML_COUNTER_TOLERANCE : XML_MEASUREMENT_TOLERANCE;
  if (meta.has_last_value && std::fabs(publish_value - meta.last_value) < tolerance) {
    return;
  }

  sensor->publish_state(publish_value);
  meta.last_value = publish_value;
  meta.has_last_value = true;
  meta.last_update_ms = now;
  if (meta.kind == XmlSensorKind::Counter) {
    ESP_LOGD(TAG, "XML publish_state: %s=%u", name.c_str(),
             static_cast<unsigned>(std::lround(static_cast<double>(publish_value))));
  } else {
    ESP_LOGD(TAG, "XML publish_state: %s=%.3f", name.c_str(), static_cast<double>(publish_value));
  }
}

void JuraComponent::register_xml_sensor_(const XmlField &field, XmlSensorKind kind, const char *command_label) {
  auto &meta = this->xml_sensor_meta_[field.name];
  meta.kind = kind;
  meta.min_value = (kind == XmlSensorKind::Counter) ? XML_COUNTER_MIN : XML_MEASUREMENT_MIN;
  meta.max_value = (kind == XmlSensorKind::Counter) ? XML_COUNTER_MAX : XML_MEASUREMENT_MAX;
  meta.accuracy_decimals = determine_accuracy(kind, field.scale);
  meta.is_tgc0 = false;
  meta.is_percent = false;
  meta.has_unit = false;
  meta.unit_of_measurement.clear();
  meta.has_icon = false;
  meta.icon.clear();

  std::string command = command_label != nullptr ? command_label : "";
  meta.command_label = command;
  std::string lower_command = command;
  std::transform(lower_command.begin(), lower_command.end(), lower_command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  bool is_tgc0 = lower_command == "@tg:c0";
  if (kind == XmlSensorKind::Measurement && is_tgc0) {
    meta.min_value = 0.0;
    meta.max_value = 150.0;
    meta.is_percent = true;
    if (meta.accuracy_decimals < 1) {
      meta.accuracy_decimals = 1;
    }
    meta.has_unit = true;
    meta.unit_of_measurement = "%";
    meta.has_icon = true;
    meta.icon = "mdi:percent";
    meta.is_tgc0 = true;
  }

  meta.configured = true;

  sensor::Sensor *sensor = this->find_configured_sensor_(field.name);
  if (sensor != nullptr) {
    this->apply_sensor_metadata_(field.name, sensor);
  }
}

sensor::Sensor *JuraComponent::find_configured_sensor_(const std::string &name) const {
  auto it = this->xml_sensors_.find(name);
  if (it == this->xml_sensors_.end()) {
    return nullptr;
  }
  return it->second;
}

void JuraComponent::apply_sensor_metadata_(const std::string &name, sensor::Sensor *sensor) {
  if (sensor == nullptr) {
    return;
  }
  auto meta_it = this->xml_sensor_meta_.find(name);
  if (meta_it == this->xml_sensor_meta_.end()) {
    return;
  }
  const auto &meta = meta_it->second;
  sensor->set_accuracy_decimals(meta.accuracy_decimals);
  sensor::StateClass state_class = meta.kind == XmlSensorKind::Counter
                                       ? sensor::StateClass::STATE_CLASS_TOTAL_INCREASING
                                       : sensor::StateClass::STATE_CLASS_MEASUREMENT;
  sensor->set_state_class(state_class);
  set_sensor_entity_category_if_supported(sensor, EntityCategory::ENTITY_CATEGORY_DIAGNOSTIC);
  if (meta.has_unit) {
    set_sensor_unit_if_supported(sensor, meta.unit_of_measurement.c_str());
  }
  if (meta.has_icon) {
    set_sensor_icon_if_supported(sensor, meta.icon.c_str());
  }
}

void JuraComponent::ensure_xml_sensors_created_() {
  if (!this->enable_xml_poll_) {
    return;
  }
  auto ensure_block = [&](const XmlCommandMapping &mapping, XmlSensorKind kind, const char *command) {
    if (mapping.empty()) {
      return;
    }
    for (const auto &field : mapping.fields) {
      this->register_xml_sensor_(field, kind, command);
    }
  };
  ensure_block(this->xml_mapping_.tr32, XmlSensorKind::Counter, "@TR:32");
  ensure_block(this->xml_mapping_.tg43, XmlSensorKind::Counter, "@TG:43");
  ensure_block(this->xml_mapping_.tgc0, XmlSensorKind::Measurement, "@TG:C0");

  for (const auto &entry : this->xml_sensors_) {
    if (this->xml_sensor_meta_.find(entry.first) != this->xml_sensor_meta_.end()) {
      continue;
    }
    if (this->xml_missing_sensor_logged_[entry.first]) {
      continue;
    }
    ESP_LOGW(TAG, "XML Sensor %s ist im aktuellen Mapping nicht vorhanden", entry.first.c_str());
    this->xml_missing_sensor_logged_[entry.first] = true;
  }
}

bool JuraComponent::ensure_xml_mapping_loaded_() {
  if (this->xml_mapping_loaded_) {
    return this->xml_mapping_.valid;
  }

  if (this->xml_mapping_data_ == nullptr || this->xml_mapping_length_ == 0) {
    ESP_LOGW(TAG, "Kein XML-Mapping verfügbar (Quelle: %s)",
             this->xml_mapping_path_.c_str());
    this->xml_mapping_loaded_ = true;
    this->xml_mapping_ = {};
    this->xml_stats_.clear();
    this->xml_missing_sensor_logged_.clear();
    this->log_xml_mapping_status_();
    return false;
  }

  std::string xml_source(this->xml_mapping_data_, this->xml_mapping_length_);
  bool valid = load_mapping_from_string(xml_source);
  load_settings_from_xml(xml_source);
  load_errors_from_xml(xml_source);
  this->xml_mapping_ = get_xml_mapping();
  this->xml_mapping_loaded_ = true;
  this->xml_mapping_logged_ = false;
  this->xml_stats_.clear();
  this->xml_sensor_meta_.clear();
  this->tgc0_filters_.clear();
  for (auto &candidate : this->xml_counter_candidate_frame_) {
    candidate.clear();
  }
  this->xml_counter_candidate_count_.fill(0);
  this->xml_missing_sensor_logged_.clear();
  this->settings_entities_created_ = false;
  this->settings_boot_polled_ = false;
  this->last_error_code_ = 0;
  this->errors_entities_created_ = false;
  this->log_xml_mapping_status_();
  if (this->xml_mapping_.valid) {
    this->ensure_xml_sensors_created_();
  }
  return this->xml_mapping_.valid;
}

void JuraComponent::log_xml_mapping_status_(bool force) {
  if (this->xml_mapping_logged_ && !force) {
    return;
  }
  ESP_LOGCONFIG(TAG,
                "  XML mapping Status: geladen=%s, gültig=%s, TR32=%s, TG43=%s, TGC0=%s",
                YESNO(this->xml_mapping_loaded_), YESNO(this->xml_mapping_.valid),
                YESNO(!this->xml_mapping_.tr32.empty()), YESNO(!this->xml_mapping_.tg43.empty()),
                YESNO(!this->xml_mapping_.tgc0.empty()));
  this->xml_mapping_logged_ = true;
}

void JuraComponent::reset_xml_poll_state_() {
  this->xml_state_ = XmlPollState::IDLE;
  this->xml_deadline_ms_ = 0;
  this->xml_next_action_ms_ = 0;
  this->xml_inflight_ = false;
  this->clear_db_transaction_(DbTransactionOwner::NONE);
  this->xml_last_command_.clear();
  this->xml_expected_prefix_.clear();
  this->xml_rx_line_.clear();
  this->xml_stats_capture_start_ms_ = 0;
  this->xml_stats_reject_reason_.clear();
  this->xml_stats_reject_decoded_.clear();
  this->xml_command_started_ms_ = 0;
  this->xml_command_frames_ = 0;
  this->xml_command_noise_frames_ = 0;
  this->xml_stats_rx_logged_ = false;
  this->xml_stats_binary_response_ = false;
  this->xml_tr32_page_ = 0;
  this->xml_binary_probe_prev_tr32_payload_.clear();
  this->xml_binary_probe_prev_tr32_page_ = 0;
  this->xml_binary_probe_has_prev_tr32_ = false;
  this->xml_stats_locked_ = false;
  this->xml_cycle_failed_ = false;
  this->xml_tablet_start_sequence_done_ = false;
  this->tablet_seq_state_ = TabletSeqState::IDLE;
  this->tablet_seq_rx_buffer_.clear();
  this->tablet_seq_current_cmd_.clear();
  this->tablet_seq_deadline_ms_ = 0;
  this->tablet_seq_tx_failed_ = false;
  this->xml_stats_consecutive_failures_ = 0;
  this->xml_tr32_pages_ok_ = 0;
  this->xml_tg43_ok_ = false;
  this->xml_tgc0_ok_ = false;
  this->xml_rx_buffer_.clear();
  this->xml_stats_.clear();
  this->xml_next_poll_ = esphome::millis() + this->xml_startup_delay_ms_;
  this->xml_retry_count_.fill(0);
  this->xml_invalid_len_seen_.fill(false);
  this->xml_last_invalid_len_.fill(0);
  for (auto &candidate : this->xml_counter_candidate_frame_) {
    candidate.clear();
  }
  this->xml_counter_candidate_count_.fill(0);
  this->xml_tgc0_timeout_streak_ = 0;
  this->xml_skip_tgc0_ = false;
  this->xml_command_probe_state_ = XmlCommandProbeState::IDLE;
  this->xml_command_probe_index_ = 0;
  this->xml_command_probe_rx_buffer_.clear();
  this->xml_command_probe_current_cmd_.clear();
  this->xml_command_probe_deadline_ms_ = 0;
  this->xml_command_probe_next_ms_ = 0;
  this->xml_command_probe_last_wait_reason_.clear();
  this->stats_session_ready_ = !this->xml_dongle_startup_;
  this->stats_inner_tx_required_ = false;
  this->dongle_startup_state_ = DongleStartupState::IDLE;
  this->dongle_startup_rx_buffer_.clear();
      this->dongle_startup_deadline_ms_ = 0;
      this->dongle_startup_next_action_ms_ = 0;
      this->dongle_startup_next_retry_ms_ = 0;
      this->dongle_startup_quiet_start_ms_ = 0;
      this->dongle_startup_probe_attempt_ = 0;
      this->dongle_startup_t1_attempt_ = 0;
      this->dongle_startup_tr37_attempt_ = 0;
      this->dongle_startup_t3_seen_during_quiet_ = false;
      this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
      this->dongle_startup_quiet_then_prep_tr37_ = true;
      this->dongle_tr_payload_.clear();
}

void JuraComponent::process_xml_polling() {
  uint32_t now = esphome::millis();
  if (this->xml_command_probe_) {
    this->process_xml_command_probe_scheduler_(now);
    return;
  }
  if (this->xml_session_probe_) {
    this->process_xml_session_probe_scheduler_(now);
    return;
  }
  if (!this->enable_xml_poll_) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  bool mapping_loaded = this->ensure_xml_mapping_loaded_();
  if (!mapping_loaded && !this->xml_transport_selftest_ && !this->xml_command_probe_) {
    return;
  }
  if (mapping_loaded) {
    this->ensure_xml_sensors_created_();
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  if (this->db_transaction_owner_ == DbTransactionOwner::MACHINE_XML) {
    ESP_LOGD(TAG, "XML DB polling skipped while Machine-XML transaction is active");
    return;
  }
  if (this->xml_dongle_startup_ && !this->stats_session_ready_) {
    this->process_dongle_startup_(now);
    return;
  }

  this->handle_xml_state_machine_(now);
}

void JuraComponent::log_xml_command_probe_wait_(const char *reason, const char *owner) {
  std::string key = reason != nullptr ? reason : "unknown";
  if (owner != nullptr && owner[0] != '\0') {
    key.append(":");
    key.append(owner);
  }
  if (key == this->xml_command_probe_last_wait_reason_) {
    return;
  }
  this->xml_command_probe_last_wait_reason_ = key;
  if (owner != nullptr && owner[0] != '\0') {
    ESP_LOGD(TAG, "xml_command_probe_wait reason=%s owner=%s", reason, owner);
  } else {
    ESP_LOGD(TAG, "xml_command_probe_wait reason=%s", reason);
  }
}

void JuraComponent::process_xml_command_probe_scheduler_(uint32_t now) {
  if (this->handshake_stage_ != HandshakeStage::DONE) {
    this->log_xml_command_probe_wait_("handshake_not_done");
    return;
  }
  if (!this->is_ready()) {
    this->log_xml_command_probe_wait_("machine_not_ready");
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->log_xml_command_probe_wait_("controller_not_ready");
    return;
  }
  if (this->is_busy()) {
    this->log_xml_command_probe_wait_("uart_busy", "coffee_maker");
    return;
  }
  if (this->db_transaction_owner_ != DbTransactionOwner::NONE) {
    this->log_xml_command_probe_wait_("uart_busy", this->db_transaction_owner_name_(this->db_transaction_owner_));
    return;
  }
  if (this->xml_command_probe_next_ms_ != 0 && !time_reached(now, this->xml_command_probe_next_ms_)) {
    return;
  }
  this->xml_command_probe_last_wait_reason_.clear();
  this->process_xml_command_probe_(now);
}

const char *JuraComponent::transport_selftest_state_name_(TransportSelftestState state) const {
  switch (state) {
    case TransportSelftestState::IDLE:
      return "idle";
    case TransportSelftestState::STATS_SEND_TY:
      return "stats_send_ty";
    case TransportSelftestState::STATS_WAIT_TY:
      return "stats_wait_ty";
    case TransportSelftestState::NORMAL_SEND_TY:
      return "normal_send_ty";
    case TransportSelftestState::NORMAL_WAIT_TY:
      return "normal_wait_ty";
    case TransportSelftestState::DONE:
      return "done";
  }
  return "unknown";
}

void JuraComponent::start_transport_selftest_(uint32_t now) {
  this->transport_selftest_state_ = TransportSelftestState::STATS_SEND_TY;
  this->transport_selftest_rx_buffer_.clear();
  this->transport_selftest_current_cmd_.clear();
  this->transport_selftest_deadline_ms_ = 0;
  ESP_LOGD(TAG, "transport_selftest_start");
  ESP_LOGD(TAG, "transport_selftest_skip cmd=@T1 reason=session_unsafe");
  (void) now;
}

void JuraComponent::send_transport_selftest_command_(const char *path, const std::string &command, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->transport_selftest_state_ = TransportSelftestState::DONE;
    return;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_response_line_buffer();
  connection->reset_db_rx_buffer();
  connection->drain_serial_input_nonblocking();

  std::string wire_command = command + "\r\n";
  std::vector<uint8_t> encoded = jutta_proto::JuttaConnection::encode_decoded_bytes(wire_command);
  std::string encoded_bytes(encoded.begin(), encoded.end());
  ESP_LOGD(TAG, "transport_selftest_tx_encoded path=%s cmd=%s hex=\"%s\"", path, command.c_str(),
           compact_hex_string(encoded_bytes, encoded_bytes.size()).c_str());
  ESP_LOGD(TAG, "transport_selftest_tx path=%s cmd=%s", path, command.c_str());

  this->transport_selftest_rx_buffer_.clear();
  this->transport_selftest_current_cmd_ = command;
  if (!connection->write_decoded(wire_command)) {
    ESP_LOGD(TAG, "transport_selftest_result path=%s cmd=%s expected=\"ty:\" ok=NO reason=tx_failed", path,
             command.c_str());
    this->transport_selftest_state_ = TransportSelftestState::DONE;
    return;
  }
  this->transport_selftest_deadline_ms_ = now + kStatsRxCaptureWindowMs;
}

void JuraComponent::finish_transport_selftest_step_(const char *path, const std::string &command, const char *expected,
                                                    bool timeout, uint32_t now) {
  std::string sanitized = sanitize_text_for_api(this->transport_selftest_rx_buffer_);
  ESP_LOGD(TAG, "transport_selftest_rx path=%s raw_hex=\"%s\" ascii=\"%s\"", path,
           compact_hex_string(this->transport_selftest_rx_buffer_, this->transport_selftest_rx_buffer_.size()).c_str(),
           sanitized.c_str());

  std::string lower = to_lower_copy(this->transport_selftest_rx_buffer_);
  trim_in_place(lower);
  std::string expected_lower = to_lower_copy(expected != nullptr ? expected : "");
  bool ok = !timeout && !expected_lower.empty() && lower.rfind(expected_lower, 0) == 0;
  ESP_LOGD(TAG, "transport_selftest_result path=%s cmd=%s expected=\"%s\" ok=%s", path, command.c_str(),
           expected != nullptr ? expected : "", YESNO(ok));
  this->transport_selftest_rx_buffer_.clear();
  this->transport_selftest_current_cmd_.clear();
  this->transport_selftest_deadline_ms_ = 0;
  (void) now;
}

bool JuraComponent::process_transport_selftest_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return true;
  }
  auto *connection = this->coffee_maker_->connection.get();
  if (this->transport_selftest_state_ == TransportSelftestState::IDLE) {
    this->start_transport_selftest_(now);
    return true;
  }

  switch (this->transport_selftest_state_) {
    case TransportSelftestState::STATS_SEND_TY:
      this->send_transport_selftest_command_("stats", "TY:", now);
      if (this->transport_selftest_state_ != TransportSelftestState::DONE) {
        this->transport_selftest_state_ = TransportSelftestState::STATS_WAIT_TY;
      }
      return true;
    case TransportSelftestState::STATS_WAIT_TY: {
      std::vector<uint8_t> buffer;
      if (connection->read_decoded(buffer) && !buffer.empty()) {
        size_t current_size = this->transport_selftest_rx_buffer_.size();
        size_t remaining = current_size < kTabletSeqMaxRxBytes ? kTabletSeqMaxRxBytes - current_size : 0;
        size_t count = std::min(remaining, buffer.size());
        if (count > 0) {
          this->transport_selftest_rx_buffer_.append(reinterpret_cast<const char *>(buffer.data()), count);
        }
        if (count < buffer.size()) {
          ESP_LOGD(TAG, "transport_selftest_rx_truncated path=stats max_bytes=%u",
                   static_cast<unsigned>(kTabletSeqMaxRxBytes));
        }
      }
      if (this->transport_selftest_rx_buffer_.find("\r\n") != std::string::npos ||
          (this->transport_selftest_deadline_ms_ != 0 && time_reached(now, this->transport_selftest_deadline_ms_))) {
        bool timeout = this->transport_selftest_rx_buffer_.empty() ||
                       (this->transport_selftest_deadline_ms_ != 0 &&
                        time_reached(now, this->transport_selftest_deadline_ms_) &&
                        this->transport_selftest_rx_buffer_.find("\r\n") == std::string::npos);
        this->finish_transport_selftest_step_("stats", "TY:", "ty:", timeout, now);
        this->transport_selftest_state_ = TransportSelftestState::NORMAL_SEND_TY;
      }
      return true;
    }
    case TransportSelftestState::NORMAL_SEND_TY:
      this->send_transport_selftest_command_("normal", "TY:", now);
      if (this->transport_selftest_state_ != TransportSelftestState::DONE) {
        this->transport_selftest_state_ = TransportSelftestState::NORMAL_WAIT_TY;
      }
      return true;
    case TransportSelftestState::NORMAL_WAIT_TY: {
      std::string line;
      if (connection->read_line_until(line)) {
        this->transport_selftest_rx_buffer_ = line;
        this->finish_transport_selftest_step_("normal", "TY:", "ty:", false, now);
        this->transport_selftest_state_ = TransportSelftestState::DONE;
        return true;
      }
      if (this->transport_selftest_deadline_ms_ != 0 && time_reached(now, this->transport_selftest_deadline_ms_)) {
        this->finish_transport_selftest_step_("normal", "TY:", "ty:", true, now);
        this->transport_selftest_state_ = TransportSelftestState::DONE;
      }
      return true;
    }
    case TransportSelftestState::DONE:
      ESP_LOGD(TAG, "transport_selftest_done next_retry_ms=%u", static_cast<unsigned>(this->xml_poll_interval_ms_));
      this->transport_selftest_state_ = TransportSelftestState::IDLE;
      this->xml_next_poll_ = now + this->xml_poll_interval_ms_;
      this->xml_state_ = XmlPollState::IDLE;
      this->xml_inflight_ = false;
      this->xml_last_command_.clear();
      return true;
    case TransportSelftestState::IDLE:
    default:
      return true;
  }
}

void JuraComponent::start_xml_command_probe_(uint32_t now) {
  this->xml_command_probe_state_ = XmlCommandProbeState::SEND;
  this->xml_command_probe_index_ = 0;
  this->xml_command_probe_rx_buffer_.clear();
  this->xml_command_probe_current_cmd_.clear();
  this->xml_command_probe_deadline_ms_ = 0;
  ESP_LOGD(TAG, "xml_probe_start with_ts_lock=%s", YESNO(this->xml_command_probe_with_ts_lock_));
  (void) now;
}

size_t JuraComponent::xml_command_probe_command_count_() const {
  return this->xml_command_probe_with_ts_lock_ ? 7U : 5U;
}

const char *JuraComponent::xml_command_probe_command_(size_t index) const {
  static constexpr const char *LOCKED_COMMANDS[] = {"@TS:01", "@TR:37", "@TR:32,00", "@TR:32,01",
                                                    "@TG:43", "@TG:C0", "@TS:00"};
  static constexpr const char *UNLOCKED_COMMANDS[] = {"@TR:37", "@TR:32,00", "@TR:32,01",
                                                      "@TG:43", "@TG:C0"};
  if (this->xml_command_probe_with_ts_lock_) {
    return index < (sizeof(LOCKED_COMMANDS) / sizeof(LOCKED_COMMANDS[0])) ? LOCKED_COMMANDS[index] : nullptr;
  }
  return index < (sizeof(UNLOCKED_COMMANDS) / sizeof(UNLOCKED_COMMANDS[0])) ? UNLOCKED_COMMANDS[index] : nullptr;
}

const char *JuraComponent::classify_xml_probe_response_(const std::string &response, bool line_complete) const {
  if (response.empty()) {
    return line_complete ? "crlf_only" : "timeout";
  }
  uint8_t first = static_cast<uint8_t>(response.front());
  std::string lower = to_lower_copy(response);
  trim_in_place(lower);
  if (lower.rfind("@", 0) == 0) {
    return "ascii_at";
  }
  if (lower.rfind("ty:", 0) == 0) {
    return "ascii_ty";
  }
  if (first == 0x26) {
    if (!line_complete && !has_unescaped_inner_transport_cr(response)) {
      return "binary_26_incomplete";
    }
    return "binary_26";
  }
  return "unknown";
}

void JuraComponent::send_xml_command_probe_command_(const std::string &command, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->xml_command_probe_state_ = XmlCommandProbeState::DONE;
    return;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_response_line_buffer();
  connection->reset_db_rx_buffer();
  connection->drain_serial_input_nonblocking();

  this->xml_command_probe_rx_buffer_.clear();
  this->xml_command_probe_current_cmd_ = command;
  ESP_LOGD(TAG, "xml_probe_tx cmd=%s", command.c_str());
  if (!connection->write_decoded(command + "\r\n")) {
    ESP_LOGD(TAG, "xml_probe_rx cmd=%s class=timeout len=0 reason=tx_failed hex=\"\"", command.c_str());
    this->xml_command_probe_state_ = XmlCommandProbeState::DONE;
    return;
  }
  this->xml_command_probe_deadline_ms_ = now + kStatsRxCaptureWindowMs;
  this->xml_command_probe_state_ = XmlCommandProbeState::WAIT;
}

void JuraComponent::finish_xml_command_probe_step_(const std::string &command, uint32_t now) {
  std::vector<ProbeRxLine> lines = split_probe_rx_lines(this->xml_command_probe_rx_buffer_);
  bool saw_useful = false;
  bool saw_tf = false;
  bool saw_unmatched = false;
  for (size_t i = 0; i < lines.size(); ++i) {
    const auto &rx_line = lines[i];
    const char *response_class = classify_xml_probe_response_(rx_line.data, rx_line.complete);
    ESP_LOGD(TAG, "xml_probe_rx_line cmd=%s idx=%u class=%s hex=\"%s\"", command.c_str(),
             static_cast<unsigned>(i), response_class,
             compact_hex_string(rx_line.data, rx_line.data.size()).c_str());

    bool matched = false;
    std::string decoded_or_ascii;
    if (!rx_line.data.empty() && this->is_printable_status_text_(rx_line.data)) {
      decoded_or_ascii = sanitize_text_for_api(rx_line.data);
      this->update_dongle_events_from_line_(rx_line.data);
      ESP_LOGD(TAG, "xml_probe_ascii cmd=%s idx=%u ascii=\"%s\"", command.c_str(), static_cast<unsigned>(i),
               decoded_or_ascii.c_str());
      matched = this->xml_session_probe_expected_match_(command, rx_line.data);
    }

    if (!rx_line.data.empty() && static_cast<uint8_t>(rx_line.data.front()) == 0x26 &&
        this->xml_decode_inner_transport_) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(rx_line.data);
      if (!candidates.empty() && !candidates.front().payload.empty()) {
        const auto &decoded = candidates.front();
        decoded_or_ascii = transport_payload_log_text(decoded.payload);
        const char *decoded_class = classify_decoded_inner_response(decoded.payload);
        if (rx_line.complete) {
          ESP_LOGD(TAG, "xml_probe_inner cmd=%s idx=%u decoded=\"%s\"", command.c_str(), static_cast<unsigned>(i),
                   decoded_or_ascii.c_str());
          ESP_LOGD(TAG, "xml_probe_class cmd=%s idx=%u decoded_class=%s", command.c_str(),
                   static_cast<unsigned>(i), decoded_class);
          this->update_dongle_events_from_line_(decoded.payload);
          if (std::strcmp(decoded_class, "tf_status") == 0) {
            saw_tf = true;
          }
          matched = this->xml_session_probe_expected_match_(command, decoded.payload);
        } else {
          ESP_LOGD(TAG, "xml_probe_inner_partial cmd=%s idx=%u decoded=\"%s\"", command.c_str(),
                   static_cast<unsigned>(i), decoded_or_ascii.c_str());
        }
      }
    }

    if (matched) {
      saw_useful = true;
    } else if (std::strcmp(response_class, "crlf_only") != 0) {
      saw_unmatched = true;
      ESP_LOGD(TAG, "xml_probe_rx_unmatched cmd=%s idx=%u decoded_or_ascii=\"%s\"", command.c_str(),
               static_cast<unsigned>(i), decoded_or_ascii.empty() ? "" : decoded_or_ascii.c_str());
    }
  }
  const char *result = saw_useful ? "useful_response"
                                  : (saw_tf ? "tf_status" : (lines.empty() ? "timeout" : "unmatched_only"));
  ESP_LOGD(TAG, "xml_probe_result cmd=%s result=%s lines=%u unmatched=%s", command.c_str(), result,
           static_cast<unsigned>(lines.size()), YESNO(saw_unmatched));
  this->xml_command_probe_rx_buffer_.clear();
  this->xml_command_probe_current_cmd_.clear();
  this->xml_command_probe_deadline_ms_ = 0;
  ++this->xml_command_probe_index_;
  this->xml_command_probe_state_ = this->xml_command_probe_index_ < this->xml_command_probe_command_count_()
                                       ? XmlCommandProbeState::SEND
                                       : XmlCommandProbeState::DONE;
  (void) now;
}

bool JuraComponent::process_xml_command_probe_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return true;
  }
  auto *connection = this->coffee_maker_->connection.get();
  if (this->xml_command_probe_state_ == XmlCommandProbeState::IDLE) {
    this->start_xml_command_probe_(now);
    return true;
  }

  switch (this->xml_command_probe_state_) {
    case XmlCommandProbeState::SEND: {
      const char *command = this->xml_command_probe_command_(this->xml_command_probe_index_);
      if (command == nullptr) {
        this->xml_command_probe_state_ = XmlCommandProbeState::DONE;
        return true;
      }
      this->send_xml_command_probe_command_(command, now);
      return true;
    }
    case XmlCommandProbeState::WAIT: {
      std::vector<uint8_t> buffer;
      if (connection->read_decoded(buffer) && !buffer.empty()) {
        size_t current_size = this->xml_command_probe_rx_buffer_.size();
        size_t remaining = current_size < kTabletSeqMaxRxBytes ? kTabletSeqMaxRxBytes - current_size : 0;
        size_t count = std::min(remaining, buffer.size());
        if (count > 0) {
          this->xml_command_probe_rx_buffer_.append(reinterpret_cast<const char *>(buffer.data()), count);
        }
        if (count < buffer.size()) {
          ESP_LOGD(TAG, "xml_probe_rx_truncated cmd=%s max_bytes=%u", this->xml_command_probe_current_cmd_.c_str(),
                   static_cast<unsigned>(kTabletSeqMaxRxBytes));
        }
      }
      if (this->xml_command_probe_deadline_ms_ != 0 && time_reached(now, this->xml_command_probe_deadline_ms_)) {
        this->finish_xml_command_probe_step_(this->xml_command_probe_current_cmd_, now);
      }
      return true;
    }
    case XmlCommandProbeState::DONE:
      ESP_LOGD(TAG, "xml_probe_done next_retry_ms=300000");
      this->xml_command_probe_state_ = XmlCommandProbeState::IDLE;
      this->xml_command_probe_index_ = 0;
      this->xml_command_probe_next_ms_ = now + 300000U;
      this->xml_state_ = XmlPollState::IDLE;
      this->xml_inflight_ = false;
      this->xml_last_command_.clear();
      return true;
    case XmlCommandProbeState::IDLE:
    default:
      return true;
  }
}

void JuraComponent::log_xml_session_probe_wait_(const char *reason, const char *owner) {
  std::string key = reason != nullptr ? reason : "unknown";
  if (owner != nullptr && owner[0] != '\0') {
    key.append(":");
    key.append(owner);
  }
  if (key == this->xml_session_probe_last_wait_reason_) {
    return;
  }
  this->xml_session_probe_last_wait_reason_ = key;
  if (owner != nullptr && owner[0] != '\0') {
    ESP_LOGD(TAG, "xml_session_probe_wait reason=%s owner=%s", reason, owner);
  } else {
    ESP_LOGD(TAG, "xml_session_probe_wait reason=%s", reason);
  }
}

void JuraComponent::process_xml_session_probe_scheduler_(uint32_t now) {
  if (this->handshake_stage_ != HandshakeStage::DONE) {
    this->log_xml_session_probe_wait_("handshake_not_done");
    return;
  }
  if (!this->is_ready()) {
    this->log_xml_session_probe_wait_("machine_not_ready");
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->log_xml_session_probe_wait_("controller_not_ready");
    return;
  }
  if (this->is_busy()) {
    this->log_xml_session_probe_wait_("uart_busy", "coffee_maker");
    return;
  }
  if (this->db_transaction_owner_ != DbTransactionOwner::NONE) {
    this->log_xml_session_probe_wait_("uart_busy", this->db_transaction_owner_name_(this->db_transaction_owner_));
    return;
  }
  if (this->xml_session_probe_next_ms_ != 0 && !time_reached(now, this->xml_session_probe_next_ms_)) {
    return;
  }
  this->xml_session_probe_last_wait_reason_.clear();
  this->process_xml_session_probe_(now);
}

void JuraComponent::start_xml_session_probe_(uint32_t now) {
  this->xml_session_probe_state_ = XmlSessionProbeState::SEND;
  this->xml_session_probe_index_ = 0;
  this->xml_session_probe_rx_buffer_.clear();
  this->xml_session_probe_current_cmd_.clear();
  this->xml_session_probe_deadline_ms_ = 0;
  this->xml_session_probe_timeouts_ = 0;
  ESP_LOGD(TAG, "xml_session_start variant=%s", this->xml_session_probe_variant_.c_str());
  if (this->xml_session_probe_variant_ == "dongle_full") {
    ESP_LOGD(TAG, "xml_session_variant variant=dongle_full experimental=true");
  }
  (void) now;
}

size_t JuraComponent::xml_session_probe_command_count_() const {
  if (this->xml_session_probe_variant_ == "dongle_full") {
    return 8U;
  }
  if (this->xml_session_probe_variant_ == "no_d1") {
    return 3U;
  }
  return 4U;
}

const char *JuraComponent::xml_session_probe_command_(size_t index) const {
  static constexpr const char *MINIMAL_COMMANDS[] = {"@D1", "@TR:37", "@TR:32,00", "@TG:C0"};
  static constexpr const char *DONGLE_FULL_COMMANDS[] = {"@D1", "TY:", "@T1", "@t2:818811%04X0000",
                                                         "@t3", "@TR:37", "@TR:32,00", "@TG:C0"};
  static constexpr const char *NO_D1_COMMANDS[] = {"@TR:37", "@TR:32,00", "@TG:C0"};

  if (this->xml_session_probe_variant_ == "dongle_full") {
    return index < (sizeof(DONGLE_FULL_COMMANDS) / sizeof(DONGLE_FULL_COMMANDS[0])) ? DONGLE_FULL_COMMANDS[index]
                                                                                    : nullptr;
  }
  if (this->xml_session_probe_variant_ == "no_d1") {
    return index < (sizeof(NO_D1_COMMANDS) / sizeof(NO_D1_COMMANDS[0])) ? NO_D1_COMMANDS[index] : nullptr;
  }
  return index < (sizeof(MINIMAL_COMMANDS) / sizeof(MINIMAL_COMMANDS[0])) ? MINIMAL_COMMANDS[index] : nullptr;
}

std::string JuraComponent::xml_session_probe_format_command_(const char *command) const {
  if (command == nullptr) {
    return {};
  }
  std::string text(command);
  if (text.find("%04X") == std::string::npos) {
    return text;
  }
  uint16_t t2_word = 0xB228;
  parse_t2_word_from_response(this->handshake_t2_response_, t2_word);
  char buffer[48];
  std::snprintf(buffer, sizeof(buffer), command, static_cast<unsigned>(t2_word));
  return std::string(buffer);
}

const char *JuraComponent::classify_xml_session_decoded_response_(const std::string &response) const {
  std::string lower = lower_trimmed_transport_payload(response);
  if (lower.rfind("@tf", 0) == 0) {
    return "tf_status";
  }
  if (lower.rfind("@tr", 0) == 0) {
    return "stats_response";
  }
  if (lower.rfind("@tg", 0) == 0) {
    return "maintenance_response";
  }
  if (lower.rfind("@t1", 0) == 0 || lower.rfind("@t2", 0) == 0 || lower.rfind("@t3", 0) == 0 ||
      lower.rfind("ty:", 0) == 0) {
    return "handshake_response";
  }
  if (lower.rfind("@ts", 0) == 0 || lower.rfind("ok", 0) == 0) {
    return "control_response";
  }
  return "unknown";
}

bool JuraComponent::xml_session_probe_expected_match_(const std::string &command,
                                                      const std::string &response) const {
  std::string cmd = lower_trimmed_transport_payload(command);
  if (cmd == "ty:") {
    return payload_starts_with_ci(response, "ty:");
  }
  if (cmd == "@t1") {
    return payload_starts_with_ci(response, "@t1");
  }
  if (cmd == "@t3") {
    return payload_starts_with_ci(response, "@t3");
  }
  if (cmd == "@ts:00" || cmd == "@ts:01") {
    return payload_starts_with_ci(response, "@ts") || payload_starts_with_ci(response, "ok");
  }
  if (cmd == "@tr:37") {
    return payload_starts_with_ci(response, "@tr:37");
  }
  if (cmd.rfind("@tr:32,", 0) == 0) {
    std::string expected = "@tr:32," + cmd.substr(std::strlen("@tr:32,"));
    return payload_starts_with_ci(response, expected);
  }
  if (cmd.rfind("@tg:", 0) == 0) {
    std::string expected = "@tg:" + cmd.substr(std::strlen("@tg:"));
    return payload_starts_with_ci(response, expected);
  }
  return false;
}

void JuraComponent::send_xml_session_probe_command_(const std::string &command, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->finish_xml_session_probe_cycle_(now, "failed", "controller_not_ready");
    return;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_response_line_buffer();
  connection->reset_db_rx_buffer();
  connection->drain_serial_input_nonblocking();

  this->xml_session_probe_rx_buffer_.clear();
  this->xml_session_probe_current_cmd_ = command;
  ESP_LOGD(TAG, "xml_session_tx variant=%s cmd=%s", this->xml_session_probe_variant_.c_str(), command.c_str());
  if (!connection->write_decoded(command + "\r\n")) {
    ESP_LOGD(TAG, "xml_session_rx variant=%s cmd=%s class=timeout len=0 reason=tx_failed hex=\"\"",
             this->xml_session_probe_variant_.c_str(), command.c_str());
    this->finish_xml_session_probe_cycle_(now, "failed", "tx_failed");
    return;
  }
  this->xml_session_probe_deadline_ms_ = now + kStatsRxCaptureWindowMs;
  this->xml_session_probe_state_ = XmlSessionProbeState::WAIT;
}

void JuraComponent::finish_xml_session_probe_step_(const std::string &command, uint32_t now) {
  std::vector<ProbeRxLine> lines = split_probe_rx_lines(this->xml_session_probe_rx_buffer_);
  const bool timed_out = lines.empty();
  bool saw_useful = false;
  bool saw_tf = false;
  bool saw_unmatched = false;
  bool abort_unexpected_handshake = false;

  for (size_t i = 0; i < lines.size(); ++i) {
    const auto &rx_line = lines[i];
    const char *response_class = classify_xml_probe_response_(rx_line.data, rx_line.complete);
    ESP_LOGD(TAG, "xml_session_rx_line variant=%s cmd=%s idx=%u class=%s hex=\"%s\"",
             this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i), response_class,
             compact_hex_string(rx_line.data, rx_line.data.size()).c_str());

    bool matched = false;
    std::string decoded_or_ascii;
    const char *decoded_class = "unknown";
    if (!rx_line.data.empty() && this->is_printable_status_text_(rx_line.data)) {
      decoded_or_ascii = sanitize_text_for_api(rx_line.data);
      decoded_class = this->classify_xml_session_decoded_response_(rx_line.data);
      this->update_dongle_events_from_line_(rx_line.data);
      ESP_LOGD(TAG, "xml_session_ascii variant=%s cmd=%s idx=%u ascii=\"%s\"",
               this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i),
               decoded_or_ascii.c_str());
      ESP_LOGD(TAG, "xml_session_class variant=%s cmd=%s idx=%u decoded_class=%s",
               this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i), decoded_class);
      matched = this->xml_session_probe_expected_match_(command, rx_line.data);
    }

    if (!rx_line.data.empty() && static_cast<uint8_t>(rx_line.data.front()) == 0x26 &&
        this->xml_decode_inner_transport_) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(rx_line.data);
      if (!candidates.empty() && !candidates.front().payload.empty()) {
        const auto &decoded = candidates.front();
        decoded_or_ascii = transport_payload_log_text(decoded.payload);
        decoded_class = this->classify_xml_session_decoded_response_(decoded.payload);
        if (rx_line.complete) {
          ESP_LOGD(TAG, "xml_session_inner variant=%s cmd=%s idx=%u decoded=\"%s\"",
                   this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i),
                   decoded_or_ascii.c_str());
          ESP_LOGD(TAG, "xml_session_class variant=%s cmd=%s idx=%u decoded_class=%s",
                   this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i),
                   decoded_class);
          this->update_dongle_events_from_line_(decoded.payload);
          if (std::strcmp(decoded_class, "tf_status") == 0) {
            saw_tf = true;
          }
          matched = this->xml_session_probe_expected_match_(command, decoded.payload);
        } else {
          ESP_LOGD(TAG, "xml_session_inner_partial variant=%s cmd=%s idx=%u decoded=\"%s\"",
                   this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i),
                   decoded_or_ascii.c_str());
        }
      }
    }

    if (matched) {
      saw_useful = true;
    } else if (std::strcmp(response_class, "crlf_only") != 0) {
      saw_unmatched = true;
      ESP_LOGD(TAG, "xml_session_rx_unmatched variant=%s cmd=%s idx=%u decoded_or_ascii=\"%s\"",
               this->xml_session_probe_variant_.c_str(), command.c_str(), static_cast<unsigned>(i),
               decoded_or_ascii.empty() ? "" : decoded_or_ascii.c_str());
      if (std::strcmp(decoded_class, "handshake_response") == 0 &&
          !this->xml_session_probe_expected_match_(command, decoded_or_ascii)) {
        abort_unexpected_handshake = true;
      }
    }
  }

  const char *result = saw_useful ? "useful_response"
                                  : (saw_tf ? "tf_status" : (timed_out ? "timeout" : "unmatched_only"));
  ESP_LOGD(TAG, "xml_session_result variant=%s cmd=%s result=%s lines=%u unmatched=%s",
           this->xml_session_probe_variant_.c_str(), command.c_str(), result, static_cast<unsigned>(lines.size()),
           YESNO(saw_unmatched));

  if (timed_out) {
    ++this->xml_session_probe_timeouts_;
  } else {
    this->xml_session_probe_timeouts_ = 0;
  }

  if (abort_unexpected_handshake) {
    ESP_LOGD(TAG, "xml_session_abort reason=unexpected_handshake_response variant=%s",
             this->xml_session_probe_variant_.c_str());
    this->finish_xml_session_probe_cycle_(now, "failed", "unexpected_handshake_response");
    return;
  }

  this->xml_session_probe_rx_buffer_.clear();
  this->xml_session_probe_current_cmd_.clear();
  this->xml_session_probe_deadline_ms_ = 0;
  ++this->xml_session_probe_index_;

  if (this->xml_session_probe_timeouts_ >= 2U) {
    this->finish_xml_session_probe_cycle_(now, "failed", "repeated_timeouts");
    return;
  }

  this->xml_session_probe_state_ = this->xml_session_probe_index_ < this->xml_session_probe_command_count_()
                                       ? XmlSessionProbeState::SEND
                                       : XmlSessionProbeState::DONE;
}

void JuraComponent::finish_xml_session_probe_cycle_(uint32_t now, const char *result, const char *reason) {
  if (reason != nullptr && reason[0] != '\0') {
    ESP_LOGD(TAG, "xml_session_done variant=%s result=%s reason=%s next_retry_ms=300000",
             this->xml_session_probe_variant_.c_str(), result, reason);
  } else {
    ESP_LOGD(TAG, "xml_session_done variant=%s result=%s next_retry_ms=300000",
             this->xml_session_probe_variant_.c_str(), result);
  }
  this->xml_session_probe_state_ = XmlSessionProbeState::IDLE;
  this->xml_session_probe_index_ = 0;
  this->xml_session_probe_rx_buffer_.clear();
  this->xml_session_probe_current_cmd_.clear();
  this->xml_session_probe_deadline_ms_ = 0;
  this->xml_session_probe_next_ms_ = now + 300000U;
  this->xml_session_probe_timeouts_ = 0;
  this->xml_state_ = XmlPollState::IDLE;
  this->xml_inflight_ = false;
  this->xml_last_command_.clear();
}

const char *JuraComponent::dongle_startup_state_name_(DongleStartupState state) const {
  switch (state) {
    case DongleStartupState::IDLE:
      return "idle";
    case DongleStartupState::START_CLEAR:
      return "start_clear";
    case DongleStartupState::PROBE_D1:
      return "probe_d1";
    case DongleStartupState::PROBE_TY:
      return "probe_ty";
    case DongleStartupState::SEND_T1:
      return "send_t1";
    case DongleStartupState::WAIT_T2:
      return "wait_t2";
    case DongleStartupState::SEND_T2:
      return "send_t2";
    case DongleStartupState::WAIT_T3:
      return "wait_t3";
    case DongleStartupState::SEND_T3:
      return "send_t3";
    case DongleStartupState::WAIT_T0_AFTER_T3:
      return "wait_t0_after_t3";
    case DongleStartupState::WAIT_AFTER_T3:
      return "wait_after_t3";
    case DongleStartupState::PREP_TR37:
      return "prep_tr37";
    case DongleStartupState::SEND_TR37:
      return "send_tr37";
    case DongleStartupState::WAIT_TR37:
      return "wait_tr37";
    case DongleStartupState::READY:
      return "ready";
    case DongleStartupState::FAILED:
      return "failed";
  }
  return "unknown";
}

void JuraComponent::transition_dongle_startup_(DongleStartupState state, uint32_t now) {
  if (this->dongle_startup_state_ != state || this->xml_dongle_startup_debug_) {
    ESP_LOGD(TAG, "dongle_startup_state old=%s new=%s events=0x%02X",
             this->dongle_startup_state_name_(this->dongle_startup_state_),
             this->dongle_startup_state_name_(state), static_cast<unsigned>(this->dongle_events_));
  }
  this->dongle_startup_state_ = state;
  this->dongle_startup_deadline_ms_ = 0;
  this->dongle_startup_next_action_ms_ = now;
}

bool JuraComponent::send_dongle_startup_command_(const std::string &command, uint32_t now, bool inner_uart0) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (this->xml_dongle_startup_mode_ == "full" && command == "@TR:37" && !inner_uart0) {
    ESP_LOGE(TAG, "dongle_startup_error reason=tr37_plaintext_not_allowed_in_full_mode");
    return false;
  }
  if (this->xml_dongle_startup_debug_) {
    this->coffee_maker_->connection->reset_response_line_buffer();
  }
  this->coffee_maker_->connection->reset_db_rx_buffer();
  std::string framed = command;
  if (framed.size() < 2 || framed.substr(framed.size() - 2) != "\r\n") {
    framed.append("\r\n");
  }
  if (inner_uart0) {
    ESP_LOGD(TAG, "dongle_startup_tx cmd=%s mode=inner_uart0", command.c_str());
    return this->write_inner_uart0_command_(command, now);
  }

  ESP_LOGD(TAG, "dongle_startup_tx cmd=%s mode=plaintext", command.c_str());
  return this->coffee_maker_->connection->write_decoded(framed);
}

bool JuraComponent::write_inner_uart0_command_(const std::string &command, uint32_t now, bool no_rx_flush) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  std::string framed = command;
  if (framed.size() < 2 || framed.substr(framed.size() - 2) != "\r\n") {
    framed.append("\r\n");
  }

  std::vector<uint8_t> encoded;
  uint8_t key = static_cast<uint8_t>((now >> 4) ^ (now >> 12) ^ this->dongle_inner_tx_key_counter_ ^
                                     static_cast<uint8_t>(command.size() * 31U));
  this->dongle_inner_tx_key_counter_ = static_cast<uint8_t>(this->dongle_inner_tx_key_counter_ + 0x37U);
  if (!encode_inner_transport_uart_mode0(framed, key, encoded)) {
    ESP_LOGE(TAG, "inner_tx_error reason=inner_uart0_encode_failed cmd=%s", command.c_str());
    return false;
  }

  std::string encoded_text(encoded.begin(), encoded.end());
  if (this->xml_dongle_inner_tx_debug_) {
    ESP_LOGD(TAG, "inner_tx_plain cmd=\"%s\" plain_hex=\"%s\"", command.c_str(),
             compact_hex_string(framed, framed.size()).c_str());
    ESP_LOGD(TAG, "inner_tx_encoded cmd=\"%s\" hex=\"%s\"", command.c_str(),
             compact_hex_string(encoded_text, encoded_text.size()).c_str());
    InnerTransportDecodeResult roundtrip =
        decode_inner_transport_with_tables(encoded_text, INNER_UART_MODE0_A, INNER_UART_MODE0_B, "uart_mode0");
    bool roundtrip_ok = roundtrip.payload == framed;
    ESP_LOGD(TAG, "inner_tx_roundtrip cmd=\"%s\" decoded=\"%s\" ok=%s", command.c_str(),
             escape_control_text_for_log(roundtrip.payload).c_str(), YESNO(roundtrip_ok));
  }

  if (no_rx_flush) {
    return this->coffee_maker_->connection->write_decoded_no_flush(encoded);
  }
  return this->coffee_maker_->connection->write_decoded(encoded);
}

void JuraComponent::fail_dongle_startup_(uint32_t now, const char *reason) {
  this->stats_session_ready_ = false;
  this->stats_inner_tx_required_ = false;
  this->post_gate_tx_ready_event_ = true;
  this->dongle_startup_next_retry_ms_ = now + this->xml_poll_interval_ms_;
  ESP_LOGD(TAG, "dongle_startup_failed reason=%s events=0x%02X next_retry_ms=%u",
           reason != nullptr ? reason : "unknown", static_cast<unsigned>(this->dongle_events_),
           static_cast<unsigned>(this->xml_poll_interval_ms_));
  this->transition_dongle_startup_(DongleStartupState::FAILED, now);
}

void JuraComponent::update_dongle_events_from_line_(const std::string &line) {
  if (line.empty()) {
    return;
  }
  std::string trimmed = line;
  trim_in_place(trimmed);
  if (trimmed.empty()) {
    return;
  }

  uint32_t bit = 0;
  std::string lower = to_lower_copy(trimmed);
  if (lower.rfind("ty:", 0) == 0) {
    bit = DONGLE_EVENT_TY;
  } else if (lower.rfind("@t0", 0) == 0) {
    bit = DONGLE_EVENT_T0;
    if (this->status_debug_) {
      ESP_LOGD(TAG, "passive_status_frame type=t0 line=\"%s\" event=0x02",
               sanitize_text_for_api(trimmed).c_str());
    }
  } else if (lower.rfind("@t1", 0) == 0) {
    bit = DONGLE_EVENT_T1;
  } else if (trimmed.rfind("@T2", 0) == 0) {
    bit = DONGLE_EVENT_T2;
    if (this->status_debug_) {
      ESP_LOGD(TAG, "passive_status_frame type=t2 line=\"%s\"",
               sanitize_text_for_api(trimmed).c_str());
      this->handle_t2_status_debug_(trimmed);
    }
    uint16_t parsed_word = 0;
    if (parse_t2_word_from_response(trimmed, parsed_word)) {
      this->startup_t2_word_ = parsed_word;
    }
  } else if (trimmed.rfind("@T3", 0) == 0) {
    bit = DONGLE_EVENT_T3;
    if (this->status_debug_) {
      ESP_LOGD(TAG, "passive_status_frame type=t3 line=\"%s\"",
               sanitize_text_for_api(trimmed).c_str());
    }
    this->dongle_machine_identity_ = trimmed;
    if (this->dongle_startup_state_ == DongleStartupState::WAIT_AFTER_T3) {
      this->dongle_startup_t3_seen_during_quiet_ = true;
    } else if (this->dongle_startup_state_ == DongleStartupState::WAIT_TR37) {
      this->dongle_startup_t3_seen_while_waiting_tr37_ = true;
      ESP_LOGD(TAG, "dongle_startup_rx_unmatched cmd=@TR:37 decoded_or_ascii=\"%s\"",
               sanitize_text_for_api(trimmed).c_str());
    }
  } else if (lower.rfind("@tr", 0) == 0) {
    bit = DONGLE_EVENT_TR;
    this->dongle_tr_payload_ = trimmed;
  } else if (lower.rfind("@tf:", 0) == 0) {
    bit = DONGLE_EVENT_TF;
    if (this->status_debug_) {
      ESP_LOGD(TAG, "passive_status_frame type=tf line=\"%s\"",
               sanitize_text_for_api(trimmed).c_str());
    }
    if (this->dongle_startup_state_ == DongleStartupState::WAIT_TR37) {
      ESP_LOGD(TAG, "dongle_startup_not_stats class=tf_status decoded=\"%s\"",
               sanitize_text_for_api(trimmed).c_str());
    }
  } else if (lower.rfind("@tv:", 0) == 0) {
    if (this->status_debug_) {
      ESP_LOGD(TAG, "passive_status_frame type=tv line=\"%s\"",
               sanitize_text_for_api(trimmed).c_str());
    }
  } else if (lower.rfind("@tg", 0) == 0 && this->xml_dongle_startup_debug_) {
    ESP_LOGD(TAG, "dongle_event line=\"%s\" set=0x00 events=0x%02X class=maintenance_response",
             sanitize_text_for_api(trimmed).c_str(), static_cast<unsigned>(this->dongle_events_));
  }

  if (bit == 0) {
    return;
  }

  bool newly_set = (this->dongle_events_ & bit) == 0;
  this->dongle_events_ |= bit;
  if (this->xml_dongle_startup_ && (newly_set || this->xml_dongle_startup_debug_)) {
    ESP_LOGD(TAG, "dongle_event line=\"%s\" set=0x%02X events=0x%02X",
             sanitize_text_for_api(trimmed).c_str(), static_cast<unsigned>(bit),
             static_cast<unsigned>(this->dongle_events_));
  }
}

void JuraComponent::process_dongle_startup_rx_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  std::vector<uint8_t> buffer;
  while (this->coffee_maker_->connection->read_decoded(buffer) && !buffer.empty()) {
    size_t current_size = this->dongle_startup_rx_buffer_.size();
    size_t remaining = current_size < kDongleStartupMaxRxBytes ? kDongleStartupMaxRxBytes - current_size : 0;
    size_t count = std::min(remaining, buffer.size());
    if (count > 0) {
      this->dongle_startup_rx_buffer_.append(reinterpret_cast<const char *>(buffer.data()), count);
    }
    if (count < buffer.size()) {
      ESP_LOGD(TAG, "dongle_startup_rx_truncated max_bytes=%u", static_cast<unsigned>(kDongleStartupMaxRxBytes));
    }
    buffer.clear();
  }

  if (this->dongle_startup_rx_buffer_.empty()) {
    (void) now;
    return;
  }

  std::vector<ProbeRxLine> lines = split_probe_rx_lines(this->dongle_startup_rx_buffer_);
  std::string remainder;
  for (const auto &rx_line : lines) {
    if (!rx_line.complete) {
      remainder = rx_line.data;
      continue;
    }
    if (rx_line.data.empty()) {
      continue;
    }
    ESP_LOGD(TAG, "dongle_startup_rx class=%s len=%u hex=\"%s\"",
             classify_xml_probe_response_(rx_line.data, true), static_cast<unsigned>(rx_line.data.size()),
             compact_hex_string(rx_line.data, rx_line.data.size()).c_str());
    if (is_inner_transport_start(static_cast<uint8_t>(rx_line.data.front())) && this->xml_decode_inner_transport_) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(rx_line.data);
      if (!candidates.empty() && !candidates.front().payload.empty()) {
        const auto &decoded = candidates.front();
        ESP_LOGD(TAG, "dongle_startup_inner decoded=\"%s\" class=%s",
                 transport_payload_log_text(decoded.payload).c_str(),
                 classify_decoded_inner_response(decoded.payload));
        if (to_lower_copy(decoded.payload).rfind("@tf:", 0) == 0) {
          ESP_LOGD(TAG, "dongle_startup_not_stats class=tf_status decoded=\"%s\"",
                   transport_payload_log_text(decoded.payload).c_str());
          this->publish_tf_status_(decoded.payload);
        }
        this->update_dongle_events_from_line_(decoded.payload);
      }
      continue;
    }
    if (this->is_printable_status_text_(rx_line.data)) {
      this->update_dongle_events_from_line_(rx_line.data);
    }
  }
  if (remainder == "@t0") {
    ESP_LOGD(TAG, "dongle_startup_rx class=ascii_at len=%u hex=\"%s\"",
             static_cast<unsigned>(remainder.size()), compact_hex_string(remainder, remainder.size()).c_str());
    this->update_dongle_events_from_line_(remainder);
    remainder.clear();
  }
  this->dongle_startup_rx_buffer_ = remainder;
}

bool JuraComponent::process_dongle_startup_(uint32_t now) {
  if (!this->xml_dongle_startup_) {
    this->stats_session_ready_ = true;
    this->stats_inner_tx_required_ = false;
    return true;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (this->dongle_startup_state_ != DongleStartupState::IDLE &&
      this->dongle_startup_state_ != DongleStartupState::READY &&
      this->dongle_startup_state_ != DongleStartupState::FAILED) {
    this->process_dongle_startup_rx_(now);
  }

  switch (this->dongle_startup_state_) {
    case DongleStartupState::IDLE:
      if (this->dongle_startup_next_retry_ms_ != 0 && !time_reached(now, this->dongle_startup_next_retry_ms_)) {
        return false;
      }
      this->transition_dongle_startup_(DongleStartupState::START_CLEAR, now);
      return false;

    case DongleStartupState::START_CLEAR: {
      this->stats_session_ready_ = false;
      this->stats_inner_tx_required_ = false;
      this->dongle_events_ &= ~DONGLE_STARTUP_CLEAR_MASK;
      this->dongle_startup_rx_buffer_.clear();
      this->dongle_startup_probe_attempt_ = 0;
      this->dongle_startup_t1_attempt_ = 0;
      this->dongle_startup_tr37_attempt_ = 0;
      this->startup_t2_word_ = 0;
      this->update_dongle_events_from_line_(this->handshake_t2_response_);
      this->update_dongle_events_from_line_(this->handshake_t3_response_);
      if (this->coffee_maker_ != nullptr && this->coffee_maker_->connection != nullptr) {
        this->coffee_maker_->connection->reset_response_line_buffer();
        this->coffee_maker_->connection->reset_db_rx_buffer();
        this->coffee_maker_->connection->drain_serial_input_nonblocking();
      }
      if (this->xml_dongle_startup_mode_ == "gate_only") {
        ESP_LOGD(TAG, "dongle_startup_mode gate_only");
        this->dongle_startup_quiet_then_prep_tr37_ = true;
        this->dongle_startup_t3_seen_during_quiet_ = false;
        this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
        this->transition_dongle_startup_(DongleStartupState::WAIT_AFTER_T3, now);
        this->dongle_startup_quiet_start_ms_ = now;
        this->dongle_startup_next_action_ms_ = now + kDongleStartupGateOnlyQuietMs;
        this->dongle_startup_deadline_ms_ = now + kDongleStartupMaxWaitAfterT3Ms;
        ESP_LOGD(TAG, "dongle_startup_gate_only_wait_after_handshake ms=%u",
                 static_cast<unsigned>(kDongleStartupGateOnlyQuietMs));
        return false;
      }
      this->transition_dongle_startup_(DongleStartupState::PROBE_D1, now);
      return false;
    }

    case DongleStartupState::PROBE_D1:
      if (!time_reached(now, this->dongle_startup_next_action_ms_)) {
        return false;
      }
      if (!this->send_dongle_startup_command_("@D1", now)) {
        this->fail_dongle_startup_(now, "send_d1_failed");
        return false;
      }
      this->transition_dongle_startup_(DongleStartupState::PROBE_TY, now);
      this->dongle_startup_next_action_ms_ = now + kDongleStartupProbeDelayMs;
      return false;

    case DongleStartupState::PROBE_TY:
      if ((this->dongle_events_ & DONGLE_EVENT_TY) != 0) {
        this->transition_dongle_startup_(DongleStartupState::SEND_T1, now);
        return false;
      }
      if (this->dongle_startup_deadline_ms_ == 0) {
        if (!time_reached(now, this->dongle_startup_next_action_ms_)) {
          return false;
        }
        if (this->dongle_startup_probe_attempt_ >= kDongleStartupMaxProbeAttempts) {
          this->fail_dongle_startup_(now, "ty_timeout");
          return false;
        }
        ++this->dongle_startup_probe_attempt_;
        if (!this->send_dongle_startup_command_("TY:", now)) {
          this->fail_dongle_startup_(now, "send_ty_failed");
          return false;
        }
        this->dongle_startup_deadline_ms_ = now + kDongleStartupTimeoutMs;
        ESP_LOGD(TAG, "dongle_startup_wait event=0x%02X timeout_ms=%u",
                 static_cast<unsigned>(DONGLE_EVENT_TY), static_cast<unsigned>(kDongleStartupTimeoutMs));
        return false;
      }
      if (time_reached(now, this->dongle_startup_deadline_ms_)) {
        ESP_LOGD(TAG, "dongle_startup_retry state=PROBE_TY attempt=%u",
                 static_cast<unsigned>(this->dongle_startup_probe_attempt_));
        this->transition_dongle_startup_(DongleStartupState::PROBE_D1, now);
      }
      return false;

    case DongleStartupState::SEND_T1:
      if ((this->dongle_events_ & DONGLE_EVENT_T1) != 0) {
        this->transition_dongle_startup_(DongleStartupState::WAIT_T2, now);
        return false;
      }
      if (this->dongle_startup_deadline_ms_ == 0) {
        if (this->dongle_startup_t1_attempt_ >= kDongleStartupMaxT1Attempts) {
          this->fail_dongle_startup_(now, "t1_timeout");
          return false;
        }
        ++this->dongle_startup_t1_attempt_;
        if (!this->send_dongle_startup_command_("@T1", now)) {
          this->fail_dongle_startup_(now, "send_t1_failed");
          return false;
        }
        this->dongle_startup_deadline_ms_ = now + kDongleStartupTimeoutMs;
        ESP_LOGD(TAG, "dongle_startup_wait event=0x%02X timeout_ms=%u",
                 static_cast<unsigned>(DONGLE_EVENT_T1), static_cast<unsigned>(kDongleStartupTimeoutMs));
        return false;
      }
      if (time_reached(now, this->dongle_startup_deadline_ms_)) {
        ESP_LOGD(TAG, "dongle_startup_retry state=SEND_T1 attempt=%u",
                 static_cast<unsigned>(this->dongle_startup_t1_attempt_));
        this->dongle_startup_deadline_ms_ = 0;
      }
      return false;

    case DongleStartupState::WAIT_T2:
      if ((this->dongle_events_ & DONGLE_EVENT_T2) != 0) {
        this->transition_dongle_startup_(DongleStartupState::SEND_T2, now);
        return false;
      }
      if (this->dongle_startup_deadline_ms_ == 0) {
        this->dongle_startup_deadline_ms_ = now + kDongleStartupTimeoutMs;
        ESP_LOGD(TAG, "dongle_startup_wait event=0x%02X timeout_ms=%u",
                 static_cast<unsigned>(DONGLE_EVENT_T2), static_cast<unsigned>(kDongleStartupTimeoutMs));
      } else if (time_reached(now, this->dongle_startup_deadline_ms_)) {
        this->fail_dongle_startup_(now, "t2_timeout");
      }
      return false;

    case DongleStartupState::SEND_T2: {
      if (this->startup_t2_word_ == 0) {
        uint16_t parsed_word = 0;
        if (parse_t2_word_from_response(this->handshake_t2_response_, parsed_word)) {
          this->startup_t2_word_ = parsed_word;
        }
      }
      if (this->startup_t2_word_ == 0) {
        this->fail_dongle_startup_(now, "missing_t2_word");
        return false;
      }
      char command[24];
      std::snprintf(command, sizeof(command), "@t2:818811%04X0000", static_cast<unsigned>(this->startup_t2_word_));
      if (!this->send_dongle_startup_command_(command, now)) {
        this->fail_dongle_startup_(now, "send_t2_failed");
        return false;
      }
      this->transition_dongle_startup_(DongleStartupState::WAIT_T3, now);
      return false;
    }

    case DongleStartupState::WAIT_T3:
      if ((this->dongle_events_ & DONGLE_EVENT_T3) != 0) {
        this->transition_dongle_startup_(DongleStartupState::SEND_T3, now);
        return false;
      }
      if (this->dongle_startup_deadline_ms_ == 0) {
        this->dongle_startup_deadline_ms_ = now + kDongleStartupTimeoutMs;
        ESP_LOGD(TAG, "dongle_startup_wait event=0x%02X timeout_ms=%u",
                 static_cast<unsigned>(DONGLE_EVENT_T3), static_cast<unsigned>(kDongleStartupTimeoutMs));
      } else if (time_reached(now, this->dongle_startup_deadline_ms_)) {
        this->fail_dongle_startup_(now, "t3_timeout");
      }
      return false;

    case DongleStartupState::SEND_T3:
      if (!this->send_dongle_startup_command_("@t3", now, this->xml_dongle_startup_mode_ == "full")) {
        this->fail_dongle_startup_(now, "send_t3_failed");
        return false;
      }
      this->dongle_startup_t3_seen_during_quiet_ = false;
      this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
      this->dongle_startup_quiet_then_prep_tr37_ = true;
      if (!this->xml_dongle_wait_t0_after_t3_) {
        this->transition_dongle_startup_(DongleStartupState::WAIT_AFTER_T3, now);
        this->dongle_startup_quiet_start_ms_ = now;
        this->dongle_startup_next_action_ms_ = now + kDongleStartupT3QuietMs;
        this->dongle_startup_deadline_ms_ = now + kDongleStartupMaxWaitAfterT3Ms;
        ESP_LOGD(TAG, "dongle_startup_wait_t3_quiet start events=0x%02X",
                 static_cast<unsigned>(this->dongle_events_));
        return false;
      }
      this->transition_dongle_startup_(DongleStartupState::WAIT_T0_AFTER_T3, now);
      this->dongle_startup_deadline_ms_ = now + kDongleStartupT0AfterT3TimeoutMs;
      ESP_LOGD(TAG, "dongle_startup_wait event=0x%02X timeout_ms=%u",
               static_cast<unsigned>(DONGLE_EVENT_T0), static_cast<unsigned>(kDongleStartupT0AfterT3TimeoutMs));
      return false;

    case DongleStartupState::WAIT_T0_AFTER_T3:
      if ((this->dongle_events_ & DONGLE_EVENT_T0) != 0) {
        ESP_LOGD(TAG, "dongle_startup_t0_after_t3_ok events=0x%02X",
                 static_cast<unsigned>(this->dongle_events_));
        this->dongle_startup_t3_seen_during_quiet_ = false;
        this->dongle_startup_quiet_then_prep_tr37_ = true;
        this->transition_dongle_startup_(DongleStartupState::WAIT_AFTER_T3, now);
        this->dongle_startup_quiet_start_ms_ = now;
        this->dongle_startup_next_action_ms_ = now + kDongleStartupT3QuietMs;
        this->dongle_startup_deadline_ms_ = now + kDongleStartupMaxWaitAfterT3Ms;
        ESP_LOGD(TAG, "dongle_startup_wait_t3_quiet start events=0x%02X",
                 static_cast<unsigned>(this->dongle_events_));
        return false;
      }
      if (this->dongle_startup_deadline_ms_ != 0 && time_reached(now, this->dongle_startup_deadline_ms_)) {
        this->fail_dongle_startup_(now, "t0_after_t3_timeout");
      }
      return false;

    case DongleStartupState::WAIT_AFTER_T3:
      if (this->dongle_startup_t3_seen_during_quiet_) {
        this->dongle_startup_t3_seen_during_quiet_ = false;
        this->dongle_startup_next_action_ms_ = now + kDongleStartupT3QuietMs;
        ESP_LOGD(TAG, "dongle_startup_t3_seen_during_quiet restart_quiet_timer");
      }
      if (time_reached(now, this->dongle_startup_next_action_ms_)) {
        if (this->coffee_maker_ != nullptr && this->coffee_maker_->connection != nullptr) {
          this->coffee_maker_->connection->reset_response_line_buffer();
          this->coffee_maker_->connection->reset_db_rx_buffer();
          this->coffee_maker_->connection->drain_serial_input_nonblocking();
        }
        this->dongle_startup_rx_buffer_.clear();
        ESP_LOGD(TAG, "dongle_startup_t3_quiet_ok elapsed_ms=%u",
                 static_cast<unsigned>(now - this->dongle_startup_quiet_start_ms_));
        if (this->dongle_startup_quiet_then_prep_tr37_) {
          this->transition_dongle_startup_(DongleStartupState::PREP_TR37, now);
        } else {
          this->transition_dongle_startup_(DongleStartupState::SEND_TR37, now);
        }
        return false;
      }
      if (this->dongle_startup_deadline_ms_ != 0 && time_reached(now, this->dongle_startup_deadline_ms_)) {
        this->fail_dongle_startup_(now, "t3_quiet_timeout");
      }
      return false;

    case DongleStartupState::PREP_TR37:
      if (!time_reached(now, this->dongle_startup_next_action_ms_)) {
        return false;
      }
      this->dongle_events_ &= ~DONGLE_EVENT_TR;
      this->dongle_tr_payload_.clear();
      this->dongle_startup_tr37_attempt_ = 0;
      this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
      this->transition_dongle_startup_(DongleStartupState::SEND_TR37, now);
      return false;

    case DongleStartupState::SEND_TR37:
      if (this->dongle_startup_tr37_attempt_ >= kDongleStartupMaxTr37Attempts) {
        this->fail_dongle_startup_(now, this->xml_dongle_startup_mode_ == "gate_only" ? "gate_only_tr37_timeout"
                                                                                       : "tr37_timeout");
        return false;
      }
      ++this->dongle_startup_tr37_attempt_;
      this->dongle_startup_rx_buffer_.clear();
      if (!this->send_dongle_startup_command_("@TR:37", now, this->xml_dongle_startup_mode_ == "full")) {
        this->fail_dongle_startup_(now, "send_tr37_failed");
        return false;
      }
      this->dongle_startup_deadline_ms_ = now + kDongleStartupTr37TimeoutMs;
      ESP_LOGD(TAG, "dongle_startup_wait event=0x%02X timeout_ms=%u",
               static_cast<unsigned>(DONGLE_EVENT_TR), static_cast<unsigned>(kDongleStartupTr37TimeoutMs));
      this->transition_dongle_startup_(DongleStartupState::WAIT_TR37, now);
      this->dongle_startup_deadline_ms_ = now + kDongleStartupTr37TimeoutMs;
      return false;

    case DongleStartupState::WAIT_TR37:
      if ((this->dongle_events_ & DONGLE_EVENT_TR) != 0) {
        if ((this->dongle_events_ & DONGLE_STARTUP_READY_MASK) == DONGLE_STARTUP_READY_MASK) {
          this->transition_dongle_startup_(DongleStartupState::READY, now);
          return false;
        }
        this->fail_dongle_startup_(now, "tr37_without_ready_bits");
        return false;
      }
      if (time_reached(now, this->dongle_startup_deadline_ms_)) {
        if (this->dongle_startup_tr37_attempt_ < kDongleStartupMaxTr37Attempts) {
          ESP_LOGD(TAG, "dongle_startup_retry state=SEND_TR37 attempt=%u",
                   static_cast<unsigned>(this->dongle_startup_tr37_attempt_));
          if (this->xml_dongle_startup_mode_ == "gate_only") {
            this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
            this->dongle_startup_t3_seen_during_quiet_ = false;
            this->dongle_startup_quiet_then_prep_tr37_ = false;
            this->transition_dongle_startup_(DongleStartupState::WAIT_AFTER_T3, now);
            this->dongle_startup_quiet_start_ms_ = now;
            this->dongle_startup_next_action_ms_ = now + kDongleStartupGateOnlyQuietMs;
            this->dongle_startup_deadline_ms_ = now + kDongleStartupMaxWaitAfterT3Ms;
            ESP_LOGD(TAG, "dongle_startup_gate_only_wait_after_handshake ms=%u",
                     static_cast<unsigned>(kDongleStartupGateOnlyQuietMs));
          } else if (this->dongle_startup_t3_seen_while_waiting_tr37_) {
            this->dongle_startup_t3_seen_while_waiting_tr37_ = false;
            this->dongle_startup_t3_seen_during_quiet_ = false;
            this->dongle_startup_quiet_then_prep_tr37_ = false;
            this->transition_dongle_startup_(DongleStartupState::WAIT_AFTER_T3, now);
            this->dongle_startup_quiet_start_ms_ = now;
            this->dongle_startup_next_action_ms_ = now + kDongleStartupT3QuietMs;
            this->dongle_startup_deadline_ms_ = now + kDongleStartupMaxWaitAfterT3Ms;
            ESP_LOGD(TAG, "dongle_startup_wait_t3_quiet start events=0x%02X",
                     static_cast<unsigned>(this->dongle_events_));
          } else {
            this->transition_dongle_startup_(DongleStartupState::SEND_TR37, now);
          }
        } else {
          this->fail_dongle_startup_(now, this->xml_dongle_startup_mode_ == "gate_only" ? "gate_only_tr37_timeout"
                                                                                        : "tr37_timeout");
        }
      }
      return false;

    case DongleStartupState::READY:
      if (!this->stats_session_ready_) {
        this->stats_session_ready_ = true;
        this->stats_inner_tx_required_ = true;
        this->post_gate_tx_ready_event_ = true;
        ESP_LOGD(TAG, "dongle_startup_ready events=0x%02X all_events=0x%02X",
                 static_cast<unsigned>(this->dongle_events_ & DONGLE_STARTUP_READY_MASK),
                 static_cast<unsigned>(this->dongle_events_));
        ESP_LOGD(TAG, "post_gate_transport_enabled inner_tx=YES flags_equiv=0x90");
      }
      return true;

    case DongleStartupState::FAILED:
      if (this->dongle_startup_next_retry_ms_ != 0 && time_reached(now, this->dongle_startup_next_retry_ms_)) {
        this->transition_dongle_startup_(DongleStartupState::IDLE, now);
      }
      return false;
  }
  return false;
}

bool JuraComponent::process_xml_session_probe_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->finish_xml_session_probe_cycle_(now, "failed", "controller_not_ready");
    return true;
  }
  if (this->xml_session_probe_state_ != XmlSessionProbeState::IDLE &&
      this->handshake_stage_ != HandshakeStage::DONE) {
    this->finish_xml_session_probe_cycle_(now, "failed", "handshake_restarted");
    return true;
  }

  auto *connection = this->coffee_maker_->connection.get();
  if (this->xml_session_probe_state_ == XmlSessionProbeState::IDLE) {
    this->start_xml_session_probe_(now);
    return true;
  }

  switch (this->xml_session_probe_state_) {
    case XmlSessionProbeState::SEND: {
      const char *raw_command = this->xml_session_probe_command_(this->xml_session_probe_index_);
      if (raw_command == nullptr) {
        this->finish_xml_session_probe_cycle_(now, "failed", "invalid_variant");
        return true;
      }
      std::string command = this->xml_session_probe_format_command_(raw_command);
      this->send_xml_session_probe_command_(command, now);
      return true;
    }
    case XmlSessionProbeState::WAIT: {
      std::vector<uint8_t> buffer;
      if (connection->read_decoded(buffer) && !buffer.empty()) {
        size_t current_size = this->xml_session_probe_rx_buffer_.size();
        size_t remaining = current_size < kTabletSeqMaxRxBytes ? kTabletSeqMaxRxBytes - current_size : 0;
        size_t count = std::min(remaining, buffer.size());
        if (count > 0) {
          this->xml_session_probe_rx_buffer_.append(reinterpret_cast<const char *>(buffer.data()), count);
        }
        if (count < buffer.size()) {
          ESP_LOGD(TAG, "xml_session_rx_truncated variant=%s cmd=%s max_bytes=%u",
                   this->xml_session_probe_variant_.c_str(), this->xml_session_probe_current_cmd_.c_str(),
                   static_cast<unsigned>(kTabletSeqMaxRxBytes));
        }
      }
      if (this->xml_session_probe_deadline_ms_ != 0 && time_reached(now, this->xml_session_probe_deadline_ms_)) {
        this->finish_xml_session_probe_step_(this->xml_session_probe_current_cmd_, now);
      }
      return true;
    }
    case XmlSessionProbeState::DONE:
      this->finish_xml_session_probe_cycle_(now, "success");
      return true;
    case XmlSessionProbeState::FAILED:
      this->finish_xml_session_probe_cycle_(now, "failed");
      return true;
    case XmlSessionProbeState::IDLE:
    default:
      return true;
  }
}


void JuraComponent::handle_xml_state_machine_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }

  if (this->xml_next_action_ms_ != 0 && static_cast<int32_t>(now - this->xml_next_action_ms_) < 0) {
    return;
  }

  auto *connection = this->coffee_maker_->connection.get();

  if (this->xml_state_ == XmlPollState::IDLE) {
    if (this->xml_next_poll_ != 0 && !time_reached(now, this->xml_next_poll_)) {
      return;
    }
    if (this->xml_command_probe_) {
      if (this->xml_run_tablet_start_sequence_ && !this->xml_tablet_start_sequence_done_) {
        this->process_tablet_start_sequence_(now);
        return;
      }
      this->process_xml_command_probe_(now);
      return;
    }
    if (this->xml_transport_selftest_) {
      this->process_transport_selftest_(now);
      return;
    }
    bool has_mapping = this->xml_state_has_mapping_(XmlPollState::TR32_PAGE) ||
                       this->xml_state_has_mapping_(XmlPollState::TG43) ||
                       this->xml_state_has_mapping_(XmlPollState::TGC0);
    if (!has_mapping) {
      ESP_LOGW(TAG, "XML Polling übersprungen - kein Mapping aktiv");
      uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
      this->xml_deadline_ms_ = now + sleep;
      this->xml_next_poll_ = this->xml_deadline_ms_;
      this->transition_to_state_(XmlPollState::SLEEP, now);
      return;
    }
    if (this->xml_run_tablet_start_sequence_ && !this->xml_tablet_start_sequence_done_) {
      this->process_tablet_start_sequence_(now);
      return;
    }
    this->start_new_xml_cycle_(now);
    ESP_LOGD(TAG, "stats_cycle_start mode=%s inner_tx=%s ts_lock=%s",
             this->stats_session_ready_ && this->stats_inner_tx_required_ ? "post_gate" : "legacy",
             YESNO(this->stats_inner_tx_required_), YESNO(this->xml_stats_use_ts_lock_));
    this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_LOCK : XmlPollState::TR32_PAGE, now);
  }

  if (this->xml_next_action_ms_ != 0 && static_cast<int32_t>(now - this->xml_next_action_ms_) < 0) {
    return;
  }

  switch (this->xml_state_) {
    case XmlPollState::TS_LOCK:
      if (!this->xml_inflight_) {
        if (this->stats_inner_tx_required_ || this->xml_wait_for_ts_ack_) {
          this->send_stats_ascii_command_("@TS:01", XmlPollState::WAIT_TS_LOCK, now);
        } else {
          this->send_stats_fire_and_forget_("@TS:01", XmlPollState::TR32_PAGE, now, kInterCmdGapMs);
        }
      }
      return;
    case XmlPollState::TR32_PAGE:
      if (this->xml_inflight_) {
        return;
      }
      if (!this->xml_state_has_mapping_(XmlPollState::TR32_PAGE) || this->xml_tr32_page_ >= kTr32PageCount) {
        this->transition_to_state_(XmlPollState::TG43, now, kInterCmdGapMs);
        return;
      }
      {
        char command[16];
        std::snprintf(command, sizeof(command), "@TR:32,%02X", static_cast<unsigned>(this->xml_tr32_page_));
        ESP_LOGV(TAG, "stats_next page=%02X cmd=%s", static_cast<unsigned>(this->xml_tr32_page_), command);
        this->send_stats_ascii_command_(command, XmlPollState::WAIT_TR32_PAGE, now);
      }
      return;
    case XmlPollState::TG43:
      if (!this->xml_inflight_) {
        if (!this->xml_state_has_mapping_(XmlPollState::TG43)) {
          this->transition_to_state_(XmlPollState::TGC0, now, kInterCmdGapMs);
        } else {
          ESP_LOGV(TAG, "stats_next cmd=@TG:43");
          this->send_stats_ascii_command_("@TG:43", XmlPollState::WAIT_TG43, now);
        }
      }
      return;
    case XmlPollState::TGC0:
      if (this->xml_inflight_) {
        return;
      }
      if (this->xml_skip_tgc0_) {
        ESP_LOGW(TAG, "XML @TG:C0 übersprungen (Timeout-Streak)");
        this->xml_skip_tgc0_ = false;
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
        return;
      }
      if (!this->xml_state_has_mapping_(XmlPollState::TGC0)) {
        this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_UNLOCK : XmlPollState::DONE, now,
                                   kInterCmdGapMs);
      } else {
        ESP_LOGV(TAG, "stats_next cmd=@TG:C0");
        this->send_stats_ascii_command_("@TG:C0", XmlPollState::WAIT_TGC0, now);
      }
      return;
    case XmlPollState::TS_UNLOCK:
      if (!this->xml_inflight_) {
        if (this->stats_inner_tx_required_ || this->xml_wait_for_ts_ack_) {
          this->send_stats_ascii_command_("@TS:00", XmlPollState::WAIT_TS_UNLOCK, now);
        } else {
          this->send_stats_fire_and_forget_("@TS:00", XmlPollState::DONE, now, kInterCmdGapMs);
        }
      }
      return;
    case XmlPollState::WAIT_TS_LOCK:
    case XmlPollState::WAIT_TR32_PAGE:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::WAIT_TS_UNLOCK: {
      std::string line;
      if (this->read_stats_line_(line)) {
        this->handle_stats_line_(line, now);
        return;
      }
      if (this->handle_stats_binary_response_(now)) {
        return;
      }
      if (this->xml_deadline_ms_ != 0 && static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        if (this->stats_inner_tx_required_) {
          this->post_gate_tx_ready_event_ = true;
          uint32_t waited_ms = (this->xml_last_command_.rfind("@TR:32", 0) == 0 ||
                                this->xml_last_command_.rfind("@TG:", 0) == 0)
                                   ? kPostGateStatsTimeoutMs
                                   : kPostGateControlTimeoutMs;
          ESP_LOGD(TAG, "forward_post_gate_timeout cmd=%s waited_ms=%u", this->xml_last_command_.c_str(),
                   static_cast<unsigned>(waited_ms));
          ESP_LOGD(TAG, "post_gate_timeout cmd=%s waited_ms=%u", this->xml_last_command_.c_str(),
                   static_cast<unsigned>(waited_ms));
        }
        ESP_LOGD(TAG, "stats_timeout cmd=%s", this->xml_last_command_.c_str());
        ESP_LOGD(TAG, "stats_command_perf cmd=%s result=timeout duration_ms=%u frames=%u noise=%u",
                 this->xml_last_command_.c_str(), static_cast<unsigned>(now - this->xml_command_started_ms_),
                 static_cast<unsigned>(this->xml_command_frames_),
                 static_cast<unsigned>(this->xml_command_noise_frames_));
        if (this->xml_state_ == XmlPollState::WAIT_TGC0) {
          if (this->xml_tgc0_timeout_streak_ < std::numeric_limits<uint8_t>::max()) {
            this->xml_tgc0_timeout_streak_ += 1;
          }
          if (this->xml_tgc0_timeout_streak_ >= 3) {
            ESP_LOGW(TAG, "XML @TG:C0 drei Timeouts – überspringe nächsten Versuch");
            this->xml_skip_tgc0_ = true;
            this->xml_tgc0_timeout_streak_ = 0;
          }
        }
        this->xml_cycle_failed_ = true;
        this->xml_inflight_ = false;
        this->xml_deadline_ms_ = 0;
        this->xml_rx_line_.clear();
        this->xml_stats_capture_start_ms_ = 0;
        if (!this->stats_inner_tx_required_) {
          connection->reset_response_line_buffer();
          connection->reset_db_rx_buffer();
          connection->drain_serial_input_nonblocking();
        }
        this->advance_after_stats_timeout_(now);
      }
      return;
    }
    case XmlPollState::DONE:
      this->finish_stats_cycle_(now, this->xml_cycle_failed_ ? "done_with_errors" : "done");
      return;
    case XmlPollState::SLEEP:
      if (this->xml_deadline_ms_ == 0 || static_cast<int32_t>(now - this->xml_deadline_ms_) >= 0) {
        this->xml_deadline_ms_ = 0;
        this->xml_next_action_ms_ = 0;
        this->xml_state_ = XmlPollState::IDLE;
        this->xml_next_poll_ = now;
        this->xml_inflight_ = false;
        this->xml_last_command_.clear();
        this->clear_db_transaction_(DbTransactionOwner::XML_POLL);
      }
      return;
    case XmlPollState::IDLE:
    default:
      return;
  }
}

bool JuraComponent::xml_state_has_mapping_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::TR32_PAGE:
    case XmlPollState::WAIT_TR32_PAGE:
      return !this->xml_mapping_.tr32.empty() || !this->xml_mapping_.products.empty();
    case XmlPollState::TG43:
    case XmlPollState::WAIT_TG43:
      return !this->xml_mapping_.tg43.empty();
    case XmlPollState::TGC0:
    case XmlPollState::WAIT_TGC0:
      return !this->xml_mapping_.tgc0.empty();
    case XmlPollState::TS_LOCK:
    case XmlPollState::WAIT_TS_LOCK:
    case XmlPollState::TS_UNLOCK:
    case XmlPollState::WAIT_TS_UNLOCK:
    case XmlPollState::DONE:
      return true;
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return !this->xml_mapping_.tr32.empty();
    case XmlPollState::SEND_TG43:
    case XmlPollState::PARSE_TG43:
      return !this->xml_mapping_.tg43.empty();
    case XmlPollState::SEND_TGC0:
    case XmlPollState::PARSE_TGC0:
      return !this->xml_mapping_.tgc0.empty();
    case XmlPollState::IDLE:
    case XmlPollState::SLEEP:
    default:
      return true;
  }
}

const char *JuraComponent::xml_state_command_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::TS_LOCK:
    case XmlPollState::WAIT_TS_LOCK:
      return "@TS:01";
    case XmlPollState::TR32_PAGE:
    case XmlPollState::WAIT_TR32_PAGE:
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return "@TR:32";
    case XmlPollState::TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::SEND_TG43:
    case XmlPollState::PARSE_TG43:
      return "@TG:43";
    case XmlPollState::TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::SEND_TGC0:
    case XmlPollState::PARSE_TGC0:
      return "@TG:C0";
    case XmlPollState::TS_UNLOCK:
    case XmlPollState::WAIT_TS_UNLOCK:
      return "@TS:00";
    default:
      break;
  }
  return "";
}

const char *JuraComponent::xml_state_label_(XmlPollState state) const {
  switch (state) {
    case XmlPollState::TS_LOCK:
    case XmlPollState::WAIT_TS_LOCK:
      return "TS_LOCK";
    case XmlPollState::TR32_PAGE:
    case XmlPollState::WAIT_TR32_PAGE:
      return "TR32_PAGE";
    case XmlPollState::TG43:
    case XmlPollState::WAIT_TG43:
    case XmlPollState::SEND_TG43:
    case XmlPollState::PARSE_TG43:
      return "TG43";
    case XmlPollState::TGC0:
    case XmlPollState::WAIT_TGC0:
    case XmlPollState::SEND_TGC0:
    case XmlPollState::PARSE_TGC0:
      return "TGC0";
    case XmlPollState::TS_UNLOCK:
    case XmlPollState::WAIT_TS_UNLOCK:
      return "TS_UNLOCK";
    case XmlPollState::SEND_TR32:
    case XmlPollState::WAIT_TR32:
    case XmlPollState::PARSE_TR32:
      return "TR32";
    case XmlPollState::DONE:
      return "DONE";
    case XmlPollState::IDLE:
      return "IDLE";
    case XmlPollState::SLEEP:
      return "SLEEP";
    default:
      break;
  }
  return "?";
}

void JuraComponent::transition_to_state_(XmlPollState state, uint32_t now, uint32_t delay_ms) {
  this->xml_state_ = state;
  this->xml_next_action_ms_ = delay_ms > 0 ? now + delay_ms : 0;
  if (state != XmlPollState::WAIT_TS_LOCK && state != XmlPollState::WAIT_TR32_PAGE &&
      state != XmlPollState::WAIT_TG43 && state != XmlPollState::WAIT_TGC0 &&
      state != XmlPollState::WAIT_TS_UNLOCK && state != XmlPollState::WAIT_TR32) {
    this->xml_deadline_ms_ = 0;
  }
}

bool JuraComponent::write_stats_command_(const std::string &command, uint32_t now, bool fire_and_forget) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }

  if (this->stats_inner_tx_required_) {
    ESP_LOGD(TAG, "stats_tx cmd=%s mode=inner_uart0%s", command.c_str(),
             fire_and_forget ? " fire_and_forget=true" : "");
    if (!this->write_inner_uart0_command_(command, now)) {
      ESP_LOGE(TAG, "stats_error reason=inner_uart0_send_failed cmd=%s", command.c_str());
      return false;
    }
    return true;
  }

  bool stats_family = command.rfind("@TR", 0) == 0 || command.rfind("@TG", 0) == 0 || command.rfind("@TS", 0) == 0;
  if (this->xml_dongle_startup_ && this->stats_session_ready_ && stats_family) {
    ESP_LOGE(TAG, "stats_error reason=plaintext_stats_not_allowed_after_dongle_ready cmd=%s", command.c_str());
    return false;
  }

  ESP_LOGD(TAG, "stats_tx cmd=%s mode=plaintext%s", command.c_str(), fire_and_forget ? " fire_and_forget=true" : "");
  return this->coffee_maker_->connection->write_decoded(command + "\r\n");
}

bool JuraComponent::forward_post_gate_app_command_(const std::string &command, const std::string &expected_prefix,
                                                   uint32_t timeout_ms, XmlPollState wait_state, uint32_t now) {
  if (command.empty()) {
    this->transition_to_state_(wait_state, now);
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (!this->stats_session_ready_ || !this->stats_inner_tx_required_) {
    ESP_LOGE(TAG, "forward_post_gate_error reason=session_not_ready cmd=%s", command.c_str());
    return false;
  }
  if (this->xml_inflight_) {
    ESP_LOGD(TAG, "forward_post_gate_wait_tx_ready event_0x200=WAIT cmd=%s reason=inflight", command.c_str());
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (!this->post_gate_tx_ready_event_) {
    ESP_LOGD(TAG, "forward_post_gate_wait_tx_ready event_0x200=WAIT cmd=%s", command.c_str());
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (this->db_transaction_owner_ != DbTransactionOwner::NONE &&
      this->db_transaction_owner_ != DbTransactionOwner::XML_POLL) {
    ESP_LOGD(TAG, "forward_post_gate_wait_tx_ready event_0x200=WAIT cmd=%s reason=busy busy_owner=%s", command.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (this->db_transaction_owner_ == DbTransactionOwner::NONE && !this->begin_xml_transaction_(command.c_str(), now)) {
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }

  auto *connection = this->coffee_maker_->connection.get();
  if (command == "@TS:01") {
    connection->reset_response_line_buffer();
    connection->reset_db_rx_buffer();
    this->xml_rx_line_.clear();
    ESP_LOGD(TAG, "stats_rx_flush reason=start_cycle");
  } else {
    ESP_LOGD(TAG, "stats_rx_flush_skipped reason=normal_command_sequence");
  }
  this->xml_rx_buffer_.clear();
  this->xml_stats_capture_start_ms_ = 0;
  this->xml_stats_rx_logged_ = false;
  this->xml_stats_binary_response_ = false;
  this->xml_stats_reject_reason_.clear();
  this->xml_stats_reject_decoded_.clear();
  this->xml_command_started_ms_ = now;
  this->xml_command_frames_ = 0;
  this->xml_command_noise_frames_ = 0;
  this->xml_last_command_ = command;
  this->xml_expected_prefix_ = expected_prefix;

  ESP_LOGD(TAG, "forward_post_gate_begin cmd=%s flags_equiv=0x90 param2=0", command.c_str());
  ESP_LOGD(TAG, "forward_post_gate_wait_tx_ready event_0x200=SET cmd=%s", command.c_str());
  this->post_gate_tx_ready_event_ = false;
  ESP_LOGD(TAG, "forward_post_gate_clear_event event=0x200");
  ESP_LOGD(TAG, "post_gate_tx_begin owner=stats cmd=%s expected=%s timeout_ms=%u mode=inner_uart0",
           command.c_str(), expected_prefix.empty() ? "none" : expected_prefix.c_str(),
           static_cast<unsigned>(timeout_ms));
  ESP_LOGD(TAG, "forward_post_gate_tx cmd=%s mode=inner_uart0", command.c_str());
  ESP_LOGD(TAG, "stats_tx cmd=%s mode=inner_uart0", command.c_str());

  if (!this->write_inner_uart0_command_(command, now, true)) {
    ESP_LOGE(TAG, "stats_error reason=inner_uart0_send_failed cmd=%s", command.c_str());
    this->post_gate_tx_ready_event_ = true;
    this->end_xml_transaction_("stats_tx_failed");
    this->xml_last_command_.clear();
    this->xml_expected_prefix_.clear();
    return false;
  }
  ESP_LOGD(TAG, "forward_post_gate_no_post_tx_flush cmd=%s", command.c_str());

  this->xml_state_ = wait_state;
  this->xml_inflight_ = true;
  this->xml_deadline_ms_ = now + timeout_ms;
  this->xml_next_action_ms_ = now;
  return true;
}

bool JuraComponent::send_stats_ascii_command_(const std::string &command, XmlPollState wait_state, uint32_t now) {
  if (command.empty()) {
    this->transition_to_state_(wait_state, now);
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (this->stats_session_ready_ && this->stats_inner_tx_required_) {
    const std::string expected_prefix = expected_stats_prefix_for_command(command);
    uint32_t timeout_ms = (command.rfind("@TR:32", 0) == 0 || command.rfind("@TG:", 0) == 0)
                              ? kPostGateStatsTimeoutMs
                              : kPostGateControlTimeoutMs;
    return this->forward_post_gate_app_command_(command, expected_prefix, timeout_ms, wait_state, now);
  }
  if (this->db_transaction_owner_ != DbTransactionOwner::NONE &&
      this->db_transaction_owner_ != DbTransactionOwner::XML_POLL) {
    ESP_LOGD(TAG, "xml_tx_skip cmd=%s reason=busy busy_owner=%s", command.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (this->xml_inflight_) {
    ESP_LOGD(TAG, "xml_tx_skip cmd=%s reason=inflight busy_owner=%s", command.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (this->db_transaction_owner_ == DbTransactionOwner::NONE && !this->begin_xml_transaction_(command.c_str(), now)) {
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_response_line_buffer();
  connection->reset_db_rx_buffer();
  this->xml_rx_buffer_.clear();
  this->xml_rx_line_.clear();
  this->xml_stats_capture_start_ms_ = 0;
  this->xml_stats_rx_logged_ = false;
  this->xml_stats_binary_response_ = false;
  this->xml_stats_reject_reason_.clear();
  this->xml_stats_reject_decoded_.clear();
  if (command == "@TS:00") {
    ESP_LOGD(TAG, "stats_unlock_sent fire_and_forget=false");
  } else if (command == "@TS:01") {
    ESP_LOGD(TAG, "stats_lock_sent fire_and_forget=false");
  }
  if (!this->write_stats_command_(command, now, false)) {
    this->end_xml_transaction_("stats_tx_failed");
    return false;
  }
  this->xml_inflight_ = true;
  this->xml_last_command_ = command;
  this->xml_deadline_ms_ = now + kXmlRxTimeoutMs;
  this->xml_next_action_ms_ = now + kXmlQuietMs;
  this->xml_state_ = wait_state;
  return true;
}

bool JuraComponent::send_stats_fire_and_forget_(const std::string &command, XmlPollState next_state, uint32_t now,
                                                uint32_t settle_delay_ms) {
  if (command.empty()) {
    this->transition_to_state_(next_state, now, settle_delay_ms);
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  if (this->stats_session_ready_ && this->stats_inner_tx_required_) {
    ESP_LOGE(TAG, "forward_post_gate_error reason=fire_and_forget_not_allowed cmd=%s", command.c_str());
    return false;
  }
  if (this->db_transaction_owner_ != DbTransactionOwner::NONE &&
      this->db_transaction_owner_ != DbTransactionOwner::XML_POLL) {
    ESP_LOGD(TAG, "xml_tx_skip cmd=%s reason=busy busy_owner=%s", command.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (this->xml_inflight_) {
    ESP_LOGD(TAG, "xml_tx_skip cmd=%s reason=inflight busy_owner=%s", command.c_str(),
             this->db_transaction_owner_name_(this->db_transaction_owner_));
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }
  if (this->db_transaction_owner_ == DbTransactionOwner::NONE && !this->begin_xml_transaction_(command.c_str(), now)) {
    this->xml_next_action_ms_ = now + kInterCmdGapMs;
    return false;
  }

  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_response_line_buffer();
  connection->reset_db_rx_buffer();
  this->xml_rx_line_.clear();
  this->xml_rx_buffer_.clear();
  this->xml_stats_capture_start_ms_ = 0;
  this->xml_stats_rx_logged_ = false;
  this->xml_stats_binary_response_ = false;
  this->xml_stats_reject_reason_.clear();
  this->xml_stats_reject_decoded_.clear();

  if (command == "@TS:01") {
    this->xml_stats_locked_ = true;
    ESP_LOGD(TAG, "stats_lock_sent fire_and_forget=true");
    ESP_LOGV(TAG, "stats_next page=%02X cmd=@TR:32,%02X", 0U, 0U);
  } else if (command == "@TS:00") {
    this->xml_stats_locked_ = false;
    ESP_LOGD(TAG, "stats_unlock_sent fire_and_forget=true");
  }

  if (!this->write_stats_command_(command, now, true)) {
    this->end_xml_transaction_("stats_tx_failed");
    return false;
  }
  this->xml_inflight_ = false;
  this->xml_last_command_ = command;
  this->xml_deadline_ms_ = 0;
  this->transition_to_state_(next_state, now, settle_delay_ms);
  return true;
}

const char *JuraComponent::tablet_sequence_state_name_(TabletSeqState state) const {
  switch (state) {
    case TabletSeqState::IDLE:
      return "idle";
    case TabletSeqState::SEND_D1:
      return "send_d1";
    case TabletSeqState::WAIT_D1:
      return "wait_d1";
    case TabletSeqState::SEND_TY:
      return "send_ty";
    case TabletSeqState::WAIT_TY:
      return "wait_ty";
    case TabletSeqState::SEND_T1:
      return "send_t1";
    case TabletSeqState::WAIT_T1:
      return "wait_t1";
    case TabletSeqState::SEND_T2:
      return "send_t2";
    case TabletSeqState::WAIT_T2:
      return "wait_t2";
    case TabletSeqState::SEND_T3:
      return "send_t3";
    case TabletSeqState::WAIT_T3:
      return "wait_t3";
    case TabletSeqState::SEND_TR37:
      return "send_tr37";
    case TabletSeqState::WAIT_TR37:
      return "wait_tr37";
    case TabletSeqState::DONE:
      return "done";
    case TabletSeqState::FAILED:
      return "failed";
  }
  return "unknown";
}

void JuraComponent::start_tablet_start_sequence_(uint32_t now) {
  uint16_t t2_word = 0;
  bool t2_word_found = parse_t2_word_from_response(this->handshake_t2_response_, t2_word);

  this->tablet_seq_state_ = TabletSeqState::SEND_D1;
  this->tablet_seq_rx_buffer_.clear();
  this->tablet_seq_current_cmd_.clear();
  this->tablet_seq_deadline_ms_ = 0;
  this->tablet_seq_tx_failed_ = false;
  ESP_LOGD(TAG, "tablet_seq_start mode=%s t2_word=0x%04X source=%s", this->xml_tablet_sequence_mode_.c_str(),
           static_cast<unsigned>(t2_word), t2_word_found ? "handshake_t2" : "default");
  (void) now;
}

void JuraComponent::send_tablet_sequence_command_(const std::string &command, TabletSeqState wait_state, uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->tablet_seq_state_ = TabletSeqState::FAILED;
    this->xml_tablet_start_sequence_done_ = true;
    ESP_LOGD(TAG, "tablet_seq_done result=failed");
    return;
  }

  auto *connection = this->coffee_maker_->connection.get();
  connection->reset_response_line_buffer();
  connection->reset_db_rx_buffer();
  connection->drain_serial_input_nonblocking();

  this->tablet_seq_rx_buffer_.clear();
  this->tablet_seq_current_cmd_ = command;
  ESP_LOGD(TAG, "tablet_seq_state state=%s", this->tablet_sequence_state_name_(this->tablet_seq_state_));
  ESP_LOGD(TAG, "tablet_seq_tx cmd=%s", command.c_str());
  if (!connection->write_decoded(command + "\r\n")) {
    ESP_LOGD(TAG, "tablet_seq_rx cmd=%s result=tx_failed", command.c_str());
    this->tablet_seq_tx_failed_ = true;
    this->tablet_seq_state_ = wait_state;
    this->finish_tablet_sequence_command_(now, false);
    return;
  }

  this->tablet_seq_deadline_ms_ = now + kTabletSeqRxWindowMs;
  this->tablet_seq_state_ = wait_state;
}

void JuraComponent::finish_tablet_sequence_command_(uint32_t now, bool timeout) {
  uint32_t start = this->tablet_seq_deadline_ms_ >= kTabletSeqRxWindowMs
                       ? this->tablet_seq_deadline_ms_ - kTabletSeqRxWindowMs
                       : now;
  uint32_t duration = now - start;
  if (!this->tablet_seq_rx_buffer_.empty()) {
    ESP_LOGD(TAG, "tablet_seq_rx cmd=%s bytes=%u duration_ms=%u hex=\"%s\" ascii=\"%s\"",
             this->tablet_seq_current_cmd_.c_str(), static_cast<unsigned>(this->tablet_seq_rx_buffer_.size()),
             static_cast<unsigned>(duration),
             compact_hex_string(this->tablet_seq_rx_buffer_, this->tablet_seq_rx_buffer_.size()).c_str(),
             printable_or_dot_ascii(this->tablet_seq_rx_buffer_, 80).c_str());
  } else if (timeout) {
    ESP_LOGD(TAG, "tablet_seq_timeout cmd=%s", this->tablet_seq_current_cmd_.c_str());
  }

  switch (this->tablet_seq_state_) {
    case TabletSeqState::WAIT_D1:
      if (this->xml_tablet_sequence_mode_ == "minimal_tr37") {
        this->tablet_seq_state_ = TabletSeqState::SEND_TR37;
      } else {
        this->tablet_seq_state_ = TabletSeqState::DONE;
        this->xml_tablet_start_sequence_done_ = true;
        ESP_LOGD(TAG, "tablet_seq_done result=%s", this->tablet_seq_tx_failed_ ? "partial" : "success");
      }
      break;
    case TabletSeqState::WAIT_TY:
    case TabletSeqState::WAIT_T1:
    case TabletSeqState::WAIT_T2:
    case TabletSeqState::WAIT_T3:
      ESP_LOGD(TAG, "tablet_seq_done result=failed reason=unsupported_mode_state state=%s mode=%s",
               this->tablet_sequence_state_name_(this->tablet_seq_state_), this->xml_tablet_sequence_mode_.c_str());
      this->tablet_seq_state_ = TabletSeqState::FAILED;
      this->xml_tablet_start_sequence_done_ = true;
      break;
    case TabletSeqState::WAIT_TR37:
      this->tablet_seq_state_ = TabletSeqState::DONE;
      this->xml_tablet_start_sequence_done_ = true;
      ESP_LOGD(TAG, "tablet_seq_done result=%s", this->tablet_seq_tx_failed_ ? "partial" : "success");
      break;
    default:
      this->tablet_seq_state_ = TabletSeqState::FAILED;
      this->xml_tablet_start_sequence_done_ = true;
      ESP_LOGD(TAG, "tablet_seq_done result=failed");
      break;
  }

  this->tablet_seq_rx_buffer_.clear();
  this->tablet_seq_current_cmd_.clear();
  this->tablet_seq_deadline_ms_ = 0;
}

bool JuraComponent::process_tablet_start_sequence_(uint32_t now) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    this->tablet_seq_state_ = TabletSeqState::FAILED;
    this->xml_tablet_start_sequence_done_ = true;
    ESP_LOGD(TAG, "tablet_seq_done result=failed");
    return true;
  }
  if (this->tablet_seq_state_ == TabletSeqState::IDLE) {
    this->start_tablet_start_sequence_(now);
    return true;
  }

  auto *connection = this->coffee_maker_->connection.get();
  switch (this->tablet_seq_state_) {
    case TabletSeqState::SEND_D1:
      this->send_tablet_sequence_command_("@D1", TabletSeqState::WAIT_D1, now);
      return true;
    case TabletSeqState::SEND_TY:
    case TabletSeqState::SEND_T1:
    case TabletSeqState::SEND_T2:
    case TabletSeqState::SEND_T3: {
      TabletSeqState blocked_state = this->tablet_seq_state_;
      this->tablet_seq_state_ = TabletSeqState::FAILED;
      this->xml_tablet_start_sequence_done_ = true;
      ESP_LOGD(TAG, "tablet_seq_done result=failed reason=unsupported_mode_state state=%s mode=%s",
               this->tablet_sequence_state_name_(blocked_state), this->xml_tablet_sequence_mode_.c_str());
      return true;
    }
    case TabletSeqState::SEND_TR37:
      this->send_tablet_sequence_command_("@TR:37", TabletSeqState::WAIT_TR37, now);
      return true;
    case TabletSeqState::WAIT_D1:
    case TabletSeqState::WAIT_TR37: {
      std::vector<uint8_t> buffer;
      if (connection->read_decoded(buffer) && !buffer.empty()) {
        size_t current_size = this->tablet_seq_rx_buffer_.size();
        size_t remaining = current_size < kTabletSeqMaxRxBytes ? kTabletSeqMaxRxBytes - current_size : 0;
        size_t count = std::min(remaining, buffer.size());
        if (count > 0) {
          this->tablet_seq_rx_buffer_.append(reinterpret_cast<const char *>(buffer.data()), count);
        }
        if (count < buffer.size()) {
          ESP_LOGD(TAG, "tablet_seq_rx_truncated cmd=%s max_bytes=%u", this->tablet_seq_current_cmd_.c_str(),
                   static_cast<unsigned>(kTabletSeqMaxRxBytes));
        }
      }
      if (this->tablet_seq_deadline_ms_ != 0 && time_reached(now, this->tablet_seq_deadline_ms_)) {
        this->finish_tablet_sequence_command_(now, true);
      }
      return true;
    }
    case TabletSeqState::WAIT_TY:
    case TabletSeqState::WAIT_T1:
    case TabletSeqState::WAIT_T2:
    case TabletSeqState::WAIT_T3: {
      TabletSeqState blocked_state = this->tablet_seq_state_;
      this->tablet_seq_state_ = TabletSeqState::FAILED;
      this->xml_tablet_start_sequence_done_ = true;
      ESP_LOGD(TAG, "tablet_seq_done result=failed reason=unsupported_mode_state state=%s mode=%s",
               this->tablet_sequence_state_name_(blocked_state), this->xml_tablet_sequence_mode_.c_str());
      return true;
    }
    case TabletSeqState::DONE:
    case TabletSeqState::FAILED:
      this->xml_tablet_start_sequence_done_ = true;
      return true;
    case TabletSeqState::IDLE:
    default:
      return true;
  }
}

bool JuraComponent::read_stats_line_(std::string &line) {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  uint32_t now = esphome::millis();

  if (extract_crlf_line(this->xml_rx_line_, line)) {
    if (this->xml_command_frames_ < std::numeric_limits<uint16_t>::max()) {
      ++this->xml_command_frames_;
    }
    if (!this->xml_stats_rx_logged_ || !this->xml_debug_compact_) {
      uint8_t first = line.empty() ? 0 : static_cast<uint8_t>(line.front());
      if (this->xml_decode_inner_transport_ && is_inner_transport_start(first)) {
        ESP_LOGD(TAG, "stats_rx_frame cmd=%s len=%u first=0x%02X hex=\"%s\"", this->xml_last_command_.c_str(),
                 static_cast<unsigned>(line.size()), static_cast<unsigned>(first), compact_hex_string(line).c_str());
      } else {
        ESP_LOGD(TAG, "stats_rx cmd=%s len=%u first=0x%02X hex=\"%s\"", this->xml_last_command_.c_str(),
                 static_cast<unsigned>(line.size()), static_cast<unsigned>(first), compact_hex_string(line).c_str());
      }
      if (this->stats_inner_tx_required_) {
        ESP_LOGD(TAG, "forward_post_gate_rx_raw hex=\"%s\"", compact_hex_string(line).c_str());
        ESP_LOGD(TAG, "post_gate_rx raw_hex=\"%s\"", compact_hex_string(line).c_str());
      }
      this->xml_stats_rx_logged_ = true;
    }
    bool line_is_inner = this->xml_decode_inner_transport_ && !line.empty() &&
                         is_inner_transport_start(static_cast<uint8_t>(line.front()));
    std::string decoded_line;
    if (line_is_inner && this->decode_stats_inner_transport_line_(line, decoded_line)) {
      line = decoded_line;
      this->xml_stats_reject_reason_.clear();
      this->xml_stats_reject_decoded_.clear();
    } else if (line_is_inner && !this->xml_stats_reject_reason_.empty()) {
      if (this->xml_command_noise_frames_ < std::numeric_limits<uint16_t>::max()) {
        ++this->xml_command_noise_frames_;
      }
      uint32_t remaining_ms =
          this->xml_deadline_ms_ > now ? static_cast<uint32_t>(this->xml_deadline_ms_ - now) : 0;
      ESP_LOGD(TAG, "stats_rx_noise cmd=%s reason=%s action=ignore_until_deadline remaining_ms=%u",
               this->xml_last_command_.c_str(), this->xml_stats_reject_reason_.c_str(),
               static_cast<unsigned>(remaining_ms));
      return false;
    }
    if (!line_is_inner) {
      this->xml_stats_reject_reason_.clear();
      this->xml_stats_reject_decoded_.clear();
    }
    return true;
  }

  std::vector<uint8_t> buffer;
  if (!this->coffee_maker_->connection->read_decoded(buffer) || buffer.empty()) {
    return false;
  }

  std::string incoming(buffer.begin(), buffer.end());
  this->xml_rx_line_.append(incoming);
  ESP_LOGV(TAG, "stats_rx_chunk cmd=%s len=%u hex=\"%s\"", this->xml_last_command_.c_str(),
           static_cast<unsigned>(incoming.size()), compact_hex_string(incoming).c_str());

  if (extract_crlf_line(this->xml_rx_line_, line)) {
    if (this->xml_command_frames_ < std::numeric_limits<uint16_t>::max()) {
      ++this->xml_command_frames_;
    }
    if (!this->xml_stats_rx_logged_ || !this->xml_debug_compact_) {
      uint8_t first = line.empty() ? 0 : static_cast<uint8_t>(line.front());
      if (this->xml_decode_inner_transport_ && is_inner_transport_start(first)) {
        ESP_LOGD(TAG, "stats_rx_frame cmd=%s len=%u first=0x%02X hex=\"%s\"", this->xml_last_command_.c_str(),
                 static_cast<unsigned>(line.size()), static_cast<unsigned>(first), compact_hex_string(line).c_str());
      } else {
        ESP_LOGD(TAG, "stats_rx cmd=%s len=%u first=0x%02X hex=\"%s\"", this->xml_last_command_.c_str(),
                 static_cast<unsigned>(line.size()), static_cast<unsigned>(first), compact_hex_string(line).c_str());
      }
      if (this->stats_inner_tx_required_) {
        ESP_LOGD(TAG, "forward_post_gate_rx_raw hex=\"%s\"", compact_hex_string(line).c_str());
        ESP_LOGD(TAG, "post_gate_rx raw_hex=\"%s\"", compact_hex_string(line).c_str());
      }
      this->xml_stats_rx_logged_ = true;
    }
    bool line_is_inner = this->xml_decode_inner_transport_ && !line.empty() &&
                         is_inner_transport_start(static_cast<uint8_t>(line.front()));
    std::string decoded_line;
    if (line_is_inner && this->decode_stats_inner_transport_line_(line, decoded_line)) {
      line = decoded_line;
      this->xml_stats_reject_reason_.clear();
      this->xml_stats_reject_decoded_.clear();
    } else if (line_is_inner && !this->xml_stats_reject_reason_.empty()) {
      if (this->xml_command_noise_frames_ < std::numeric_limits<uint16_t>::max()) {
        ++this->xml_command_noise_frames_;
      }
      uint32_t remaining_ms =
          this->xml_deadline_ms_ > now ? static_cast<uint32_t>(this->xml_deadline_ms_ - now) : 0;
      ESP_LOGD(TAG, "stats_rx_noise cmd=%s reason=%s action=ignore_until_deadline remaining_ms=%u",
               this->xml_last_command_.c_str(), this->xml_stats_reject_reason_.c_str(),
               static_cast<unsigned>(remaining_ms));
      return false;
    }
    if (!line_is_inner) {
      this->xml_stats_reject_reason_.clear();
      this->xml_stats_reject_decoded_.clear();
    }
    return true;
  }

  bool starts_inner_transport = !this->xml_rx_line_.empty() && this->xml_decode_inner_transport_ &&
                                is_inner_transport_start(static_cast<uint8_t>(this->xml_rx_line_.front()));
  if (!this->xml_rx_line_.empty() && !starts_inner_transport &&
      (has_binary_bytes(this->xml_rx_line_) || this->xml_rx_line_.front() != '@')) {
    if (!this->xml_stats_rx_logged_ || !this->xml_debug_compact_) {
      uint8_t first = static_cast<uint8_t>(this->xml_rx_line_.front());
      ESP_LOGD(TAG, "stats_rx cmd=%s len=%u first=0x%02X hex=\"%s\"", this->xml_last_command_.c_str(),
               static_cast<unsigned>(this->xml_rx_line_.size()), static_cast<unsigned>(first),
               compact_hex_string(this->xml_rx_line_).c_str());
      this->xml_stats_rx_logged_ = true;
    }
    this->publish_raw_rx_(this->xml_rx_line_, "stats_binary");
    this->xml_stats_reject_reason_ = "unexpected_binary";
    if (this->xml_command_noise_frames_ < std::numeric_limits<uint16_t>::max()) {
      ++this->xml_command_noise_frames_;
    }
    uint32_t remaining_ms =
        this->xml_deadline_ms_ > now ? static_cast<uint32_t>(this->xml_deadline_ms_ - now) : 0;
    ESP_LOGD(TAG, "stats_rx_noise cmd=%s reason=unexpected_binary action=ignore_until_deadline remaining_ms=%u",
             this->xml_last_command_.c_str(), static_cast<unsigned>(remaining_ms));
    this->xml_rx_line_.clear();
  }
  return false;
}

bool JuraComponent::finish_stats_rx_capture_(std::string &line, uint32_t now) {
  line.clear();
  uint32_t duration = this->xml_stats_capture_start_ms_ == 0 ? 0 : now - this->xml_stats_capture_start_ms_;
  size_t line_based_len = first_raw_crlf_len(this->xml_rx_line_);
  bool capture_has_complete_inner_frame = has_unescaped_inner_transport_cr(this->xml_rx_line_);
  ESP_LOGD(TAG, "stats_rx_capture cmd=%s bytes=%u duration_ms=%u line_based_len=%u hex=\"%s\"",
           this->xml_last_command_.c_str(), static_cast<unsigned>(this->xml_rx_line_.size()),
           static_cast<unsigned>(duration), static_cast<unsigned>(line_based_len),
           compact_hex_string(this->xml_rx_line_, this->xml_rx_line_.size()).c_str());
  if (this->stats_inner_tx_required_) {
    ESP_LOGD(TAG, "forward_post_gate_rx_raw hex=\"%s\"",
             compact_hex_string(this->xml_rx_line_, this->xml_rx_line_.size()).c_str());
    ESP_LOGD(TAG, "post_gate_rx raw_hex=\"%s\"",
             compact_hex_string(this->xml_rx_line_, this->xml_rx_line_.size()).c_str());
  }

  std::vector<std::string> frames = split_inner_transport_frames(this->xml_rx_line_);
  size_t frame0_len = frames.size() > 0 ? frames[0].size() : 0;
  size_t frame1_len = frames.size() > 1 ? frames[1].size() : 0;
  size_t frame2_len = frames.size() > 2 ? frames[2].size() : 0;
  ESP_LOGD(TAG, "stats_rx_frame_split cmd=%s frames=%u frame0_len=%u frame1_len=%u frame2_len=%u",
           this->xml_last_command_.c_str(), static_cast<unsigned>(frames.size()), static_cast<unsigned>(frame0_len),
           static_cast<unsigned>(frame1_len), static_cast<unsigned>(frame2_len));

  bool saw_inner_frame = false;
  for (size_t i = 0; i < frames.size(); ++i) {
    const std::string &frame = frames[i];
    if (frame.empty()) {
      continue;
    }
    ESP_LOGD(TAG, "stats_rx_frame cmd=%s index=%u len=%u first=0x%02X hex=\"%s\"",
             this->xml_last_command_.c_str(), static_cast<unsigned>(i), static_cast<unsigned>(frame.size()),
             static_cast<unsigned>(static_cast<uint8_t>(frame.front())),
             compact_hex_string(frame, frame.size()).c_str());
    if (!is_inner_transport_start(static_cast<uint8_t>(frame.front()))) {
      continue;
    }
    if (this->xml_last_command_.rfind("@TR:32", 0) == 0) {
      ESP_LOGD(TAG, "stats_frame_changed cmd=%s changed=%s", this->xml_last_command_.c_str(),
               YESNO(!matches_known_repeated_tr32_frame(frame)));
    }
    saw_inner_frame = true;
    this->log_stats_binary_probe_(frame);
    if (this->xml_deep_debug_) {
      std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(frame);
      for (const auto &candidate : candidates) {
        bool plausible_tr = payload_starts_with_tr(candidate.payload) && candidate.payload.size() >= 16;
        bool plausible_tg = payload_starts_with_tg(candidate.payload) && candidate.payload.size() >= 8;
        ESP_LOGD(TAG, "stats_rx_capture_candidate cmd=%s frame=%u table=%s len=%u plausible_tr=%s plausible_tg=%s "
                      "printable=%u%% first_ascii=\"%s\"",
                 this->xml_last_command_.c_str(), static_cast<unsigned>(i), candidate.table_name,
                 static_cast<unsigned>(candidate.payload.size()), YESNO(plausible_tr), YESNO(plausible_tg),
                 static_cast<unsigned>(candidate.printable_ratio), printable_preview(candidate.payload, 32).c_str());
      }
    }

    std::string decoded_line;
    if (this->decode_stats_inner_transport_line_(frame, decoded_line, capture_has_complete_inner_frame)) {
      line = decoded_line;
      this->xml_rx_line_.clear();
      this->xml_stats_capture_start_ms_ = 0;
      return true;
    }
    if (this->xml_stats_binary_response_) {
      return false;
    }
  }

  if (!this->xml_stats_binary_response_) {
    this->publish_raw_rx_(this->xml_rx_line_, saw_inner_frame ? "stats_inner_transport_capture" : "stats_binary_capture");
    this->xml_stats_reject_reason_ = saw_inner_frame ? "inner_decode_not_ascii" : "unexpected_binary";
    this->xml_stats_binary_response_ = true;
  }
  return false;
}

void JuraComponent::log_stats_binary_probe_(const std::string &frame) {
  if (!this->xml_binary_probe_) {
    return;
  }
  bool is_tr32 = this->xml_last_command_.rfind("@TR:32", 0) == 0;
  bool is_tg43 = this->xml_last_command_.rfind("@TG:43", 0) == 0;
  bool is_tgc0 = this->xml_last_command_.rfind("@TG:C0", 0) == 0;
  if (!is_tr32 && !is_tg43 && !is_tgc0) {
    return;
  }

  InnerBinaryProbePayload probe = extract_current_inner_binary_payload(frame);
  if (!probe.reason.empty()) {
    ESP_LOGD(TAG, "stats_binary_probe cmd=%s raw_len=%u reason=%s raw_hex=\"%s\"",
             this->xml_last_command_.c_str(), static_cast<unsigned>(frame.size()), probe.reason.c_str(),
             compact_hex_string(frame, frame.size()).c_str());
    return;
  }

  ESP_LOGD(TAG,
           "stats_binary_probe cmd=%s raw_len=%u key_byte=0x%02X key_escaped=%s payload_len=%u esc_count=%u "
           "raw_hex=\"%s\" raw_payload=\"%s\" payload_unesc=\"%s\" u16be=\"%s\" u16le=\"%s\" bytes=\"%s\"",
           this->xml_last_command_.c_str(), static_cast<unsigned>(frame.size()), static_cast<unsigned>(probe.key),
           YESNO(probe.key_escaped), static_cast<unsigned>(probe.unescaped_payload.size()),
           static_cast<unsigned>(probe.esc_count),
           compact_hex_string(frame, frame.size()).c_str(),
           compact_hex_string(probe.raw_payload, probe.raw_payload.size()).c_str(),
           compact_hex_string(probe.unescaped_payload, probe.unescaped_payload.size()).c_str(),
           format_u16_pairs(probe.unescaped_payload, true).c_str(),
           format_u16_pairs(probe.unescaped_payload, false).c_str(),
           format_uint8_list(probe.unescaped_payload).c_str());
  ESP_LOGD(TAG, "stats_binary_nibbles cmd=%s bcd=\"%s\" nibbles=\"%s\"", this->xml_last_command_.c_str(),
           format_bcd_like(probe.unescaped_payload).c_str(), format_nibble_dump(probe.unescaped_payload).c_str());

  if (is_tr32) {
    uint8_t page = this->xml_tr32_page_;
    if (this->xml_binary_probe_has_prev_tr32_ && page <= 0x03) {
      const std::string &previous = this->xml_binary_probe_prev_tr32_payload_;
      size_t max_len = std::max(previous.size(), probe.unescaped_payload.size());
      size_t min_len = std::min(previous.size(), probe.unescaped_payload.size());
      size_t changed = max_len - min_len;
      size_t same_prefix = 0;
      for (size_t i = 0; i < min_len; ++i) {
        if (previous[i] != probe.unescaped_payload[i]) {
          ++changed;
        } else if (same_prefix == i) {
          ++same_prefix;
        }
      }
      ESP_LOGD(TAG, "stats_binary_page_diff page=%02X prev_page=%02X changed_bytes=%u same_prefix=%u",
               static_cast<unsigned>(page), static_cast<unsigned>(this->xml_binary_probe_prev_tr32_page_),
               static_cast<unsigned>(changed), static_cast<unsigned>(same_prefix));
    }
    this->xml_binary_probe_prev_tr32_payload_ = probe.unescaped_payload;
    this->xml_binary_probe_prev_tr32_page_ = page;
    this->xml_binary_probe_has_prev_tr32_ = true;
  }
}

bool JuraComponent::probe_stats_inner_key_variants_(const std::string &frame, std::string &decoded_line) {
  decoded_line.clear();
  if (!this->xml_key_probe_ && !this->xml_deep_debug_) {
    return false;
  }

  InnerBinaryProbePayload probe = extract_current_inner_binary_payload(frame);
  if (!probe.reason.empty()) {
    ESP_LOGD(TAG, "inner_key_variant cmd=%s variant=extract_failed table=unknown reason=%s",
             this->xml_last_command_.c_str(), probe.reason.c_str());
    return false;
  }

  uint16_t t2_word = 0;
  bool has_t2_word = parse_t2_word_from_response(this->handshake_t2_response_, t2_word);
  std::vector<InnerTransportDecodeResult> candidates =
      decode_inner_transport_key_variant_candidates(probe, has_t2_word, t2_word);
  for (const auto &candidate : candidates) {
    ESP_LOGD(TAG, "inner_key_variant cmd=%s variant=%s table=%s len=%u printable=%u%% starts_at=%s "
                  "starts_tr=%s starts_tg=%s starts_ts=%s starts_ok=%s first_ascii=\"%s\"",
             this->xml_last_command_.c_str(), candidate.key_variant_name, candidate.table_name,
             static_cast<unsigned>(candidate.payload.size()), static_cast<unsigned>(candidate.printable_ratio),
             YESNO(payload_starts_with_at(candidate.payload)), YESNO(payload_starts_with_tr(candidate.payload)),
             YESNO(payload_starts_with_tg(candidate.payload)), YESNO(payload_starts_with_ts(candidate.payload)),
             YESNO(payload_starts_with_ok(candidate.payload)), printable_preview(candidate.payload, 32).c_str());
    bool parser_response = payload_starts_with_tr(candidate.payload) || payload_starts_with_tg(candidate.payload) ||
                           payload_starts_with_ts(candidate.payload) || payload_starts_with_ok(candidate.payload);
    if (candidate.ok && parser_response) {
      decoded_line = candidate.payload;
      ESP_LOGD(TAG, "stats_inner_decode cmd=%s ok=true variant=%s table=%s len=%u publish=false ascii=\"%s\"",
               this->xml_last_command_.c_str(), candidate.key_variant_name, candidate.table_name,
               static_cast<unsigned>(decoded_line.size()), sanitize_text_for_api(decoded_line).c_str());
      return true;
    }
  }
  return false;
}

bool JuraComponent::decode_stats_inner_transport_line_(const std::string &raw_line, std::string &decoded_line,
                                                       bool frame_complete) {
  decoded_line.clear();
  if (raw_line.empty() || !this->xml_decode_inner_transport_ ||
      !is_inner_transport_start(static_cast<uint8_t>(raw_line.front()))) {
    return false;
  }

  uint8_t start = static_cast<uint8_t>(raw_line.front());
  std::vector<InnerTransportDecodeResult> candidates = decode_inner_transport_candidates(raw_line);
  const InnerTransportDecodeResult *frame_info = candidates.empty() ? nullptr : &candidates.front();
  if (this->xml_inner_decode_trace_ && frame_info != nullptr) {
    ESP_LOGD(TAG, "stats_rx_transport cmd=%s start=0x%02X len=%u key_byte=0x%02X key_escaped=%s payload_len=%u "
                  "esc_count=%u hex=\"%s\"",
             this->xml_last_command_.c_str(), static_cast<unsigned>(start), static_cast<unsigned>(raw_line.size()),
             static_cast<unsigned>(frame_info->key), YESNO(frame_info->key_escaped),
             static_cast<unsigned>(frame_info->payload_len), static_cast<unsigned>(frame_info->esc_count),
             compact_hex_string(raw_line).c_str());
  } else {
    ESP_LOGD(TAG, "stats_rx_transport cmd=%s start=0x%02X len=%u hex=\"%s\"", this->xml_last_command_.c_str(),
             static_cast<unsigned>(start), static_cast<unsigned>(raw_line.size()), compact_hex_string(raw_line).c_str());
  }
  this->publish_raw_rx_(raw_line, "stats_inner_transport");

  if (this->probe_stats_inner_key_variants_(raw_line, decoded_line)) {
    this->xml_stats_reject_reason_ = "key_probe_match_not_published";
    return false;
  }

  if (this->xml_deep_debug_) {
    const InnerEscapeVariant variants[] = {InnerEscapeVariant::CURRENT, InnerEscapeVariant::RAW,
                                           InnerEscapeVariant::MASK_7F, InnerEscapeVariant::XOR_20,
                                           InnerEscapeVariant::PASSTHROUGH};
    for (const auto variant : variants) {
      std::vector<InnerTransportDecodeResult> variant_candidates =
          decode_inner_transport_variant_candidates(raw_line, variant);
      for (const auto &candidate : variant_candidates) {
        ESP_LOGD(TAG, "inner_variant cmd=%s esc=%s table=%s len=%u printable=%u%% starts_at=%s starts_tr=%s "
                      "starts_tg=%s starts_ts=%s starts_ok=%s first_ascii=\"%s\"",
                 this->xml_last_command_.c_str(), candidate.escape_name, candidate.table_name,
                 static_cast<unsigned>(candidate.payload.size()), static_cast<unsigned>(candidate.printable_ratio),
                 YESNO(payload_starts_with_at(candidate.payload)), YESNO(payload_starts_with_tr(candidate.payload)),
                 YESNO(payload_starts_with_tg(candidate.payload)), YESNO(payload_starts_with_ts(candidate.payload)),
                 YESNO(payload_starts_with_ok(candidate.payload)), printable_preview(candidate.payload, 32).c_str());
      }
    }
  }

  InnerTransportDecodeResult decoded;
  decoded.reason = "no_candidate_matched";
  for (const auto &candidate : candidates) {
    if (this->xml_deep_debug_) {
      ESP_LOGD(TAG, "stats_inner_candidate cmd=%s esc=%s table=%s ok=%s key=0x%02X key_escaped=%s payload_len=%u "
                    "esc_count=%u pre_hex=\"%s\" decoded_hex=\"%s\" len=%u printable=%u%% starts_at=%s "
                    "starts_tr=%s starts_tg=%s starts_ts=%s starts_ok=%s first_hex=\"%s\" first_ascii=\"%s\" "
                    "reason=%s",
               this->xml_last_command_.c_str(), candidate.escape_name, candidate.table_name, YESNO(candidate.ok),
               static_cast<unsigned>(candidate.key), YESNO(candidate.key_escaped),
               static_cast<unsigned>(candidate.payload_len), static_cast<unsigned>(candidate.esc_count),
               compact_hex_string(candidate.encoded_payload, 32).c_str(),
               compact_hex_string(candidate.payload, 32).c_str(), static_cast<unsigned>(candidate.payload.size()),
               static_cast<unsigned>(candidate.printable_ratio), YESNO(payload_starts_with_at(candidate.payload)),
               YESNO(payload_starts_with_tr(candidate.payload)), YESNO(payload_starts_with_tg(candidate.payload)),
               YESNO(payload_starts_with_ts(candidate.payload)), YESNO(payload_starts_with_ok(candidate.payload)),
               compact_hex_string(candidate.payload, 12).c_str(), printable_preview(candidate.payload, 32).c_str(),
               candidate.reason.empty() ? "none" : candidate.reason.c_str());
    }
    if (candidate.ok) {
      decoded = candidate;
      break;
    }
    if (decoded.reason == "no_candidate_matched" || candidate.payload.size() > decoded.payload.size()) {
      decoded = candidate;
    }
  }

  if (!decoded.ok) {
    std::string reason = decoded.reason.empty() ? "inner_decode_failed" : decoded.reason;
    ESP_LOGD(TAG, "stats_inner_decode cmd=%s ok=false reason=%s table=%s", this->xml_last_command_.c_str(),
             reason.c_str(), decoded.table_name);
    this->xml_stats_reject_reason_ = reason;
    return false;
  }

  if (!frame_complete) {
    ESP_LOGD(TAG, "stats_inner_decoded_partial cmd=%s decoded=\"%s\"", this->xml_last_command_.c_str(),
             transport_payload_log_text(decoded.payload).c_str());
    this->xml_stats_reject_reason_ = "binary_26_incomplete";
    this->xml_stats_reject_decoded_ = transport_payload_log_text(decoded.payload);
    return false;
  }

  if (!is_printable_transport_payload(decoded.payload) || !is_stats_ascii_response(decoded.payload)) {
    ESP_LOGD(TAG, "stats_inner_decode cmd=%s ok=false reason=inner_decode_not_ascii table=%s len=%u hex=\"%s\"",
             this->xml_last_command_.c_str(), decoded.table_name, static_cast<unsigned>(decoded.payload.size()),
             compact_hex_string(decoded.payload).c_str());
    this->xml_stats_reject_reason_ = "inner_decode_not_ascii";
    return false;
  }

  const char *decoded_class = classify_decoded_inner_response(decoded.payload);
  ESP_LOGD(TAG, "stats_inner_decoded cmd=%s decoded=\"%s\"", this->xml_last_command_.c_str(),
           transport_payload_log_text(decoded.payload).c_str());
  this->update_dongle_events_from_line_(decoded.payload);
  if (std::strcmp(decoded_class, "tf_status") == 0) {
    this->publish_tf_status_(decoded.payload);
    ESP_LOGD(TAG, "stats_reject cmd=%s reason=not_stats decoded_class=tf_status decoded=\"%s\"",
             this->xml_last_command_.c_str(), transport_payload_log_text(decoded.payload).c_str());
  } else if (std::strcmp(decoded_class, "tv_progress") == 0) {
    this->handle_tv_progress_(decoded.payload);
    ESP_LOGD(TAG, "stats_reject cmd=%s reason=not_stats decoded_class=tv_progress decoded=\"%s\"",
             this->xml_last_command_.c_str(), transport_payload_log_text(decoded.payload).c_str());
  }

  decoded_line = decoded.payload;
  ESP_LOGD(TAG, "stats_inner_decode cmd=%s ok=true table=%s len=%u ascii=\"%s\"", this->xml_last_command_.c_str(),
           decoded.table_name, static_cast<unsigned>(decoded_line.size()), sanitize_text_for_api(decoded_line).c_str());
  return true;
}

bool JuraComponent::handle_stats_line_(const std::string &line, uint32_t now) {
  std::string lower = to_lower_copy(line);
  trim_in_place(lower);
  this->update_dongle_events_from_line_(line);
  bool parsed = false;
  bool post_gate = this->stats_session_ready_ && this->stats_inner_tx_required_;

  if (post_gate) {
    ESP_LOGD(TAG, "forward_post_gate_rx_decoded line=\"%s\"", sanitize_text_for_api(line).c_str());
    ESP_LOGD(TAG, "post_gate_rx_decoded line=\"%s\"", sanitize_text_for_api(line).c_str());
    if (!this->xml_expected_prefix_.empty()) {
      std::string expected = to_lower_copy(this->xml_expected_prefix_);
      bool expected_match = lower.rfind(expected, 0) == 0;
      bool tr32_no_data_match = expected == "@tr:32" && lower.rfind("@tr:00", 0) == 0;
      if (!expected_match && !tr32_no_data_match) {
        if (lower.rfind("@tf", 0) == 0) {
          this->publish_tf_status_(line);
          ESP_LOGD(TAG, "post_gate_rx_unmatched cmd=%s decoded_or_ascii=\"%s\" class=tf_status",
                   this->xml_last_command_.c_str(), sanitize_text_for_api(line).c_str());
        } else if (lower.rfind("@tv", 0) == 0) {
          this->handle_tv_progress_(line);
          ESP_LOGD(TAG, "post_gate_rx_unmatched cmd=%s decoded_or_ascii=\"%s\" class=tv_progress",
                   this->xml_last_command_.c_str(), sanitize_text_for_api(line).c_str());
        } else {
          ESP_LOGD(TAG, "post_gate_rx_unmatched cmd=%s decoded_or_ascii=\"%s\"",
                   this->xml_last_command_.c_str(), sanitize_text_for_api(line).c_str());
        }
        return false;
      }
      ESP_LOGD(TAG, "forward_post_gate_match cmd=%s response=\"%s\"", this->xml_last_command_.c_str(),
               sanitize_text_for_api(line).c_str());
      ESP_LOGD(TAG, "post_gate_match cmd=%s matched=%s immediate=YES", this->xml_last_command_.c_str(),
               tr32_no_data_match ? "@tr:00" : this->xml_expected_prefix_.c_str());
    }
  }

  if (lower.rfind("@t3", 0) == 0 || lower.rfind("@t0", 0) == 0) {
    if (post_gate) {
      ESP_LOGD(TAG, "post_gate_rx_unmatched cmd=%s decoded_or_ascii=\"%s\"",
               this->xml_last_command_.c_str(), sanitize_text_for_api(line).c_str());
      return false;
    }
    ESP_LOGD(TAG, "stats_reject cmd=%s reason=session_desync raw='%s'", this->xml_last_command_.c_str(),
             sanitize_text_for_api(line).c_str());
    this->xml_stats_reject_reason_ = "session_desync";
    this->xml_stats_binary_response_ = true;
    return false;
  }

  switch (this->xml_state_) {
    case XmlPollState::WAIT_TS_LOCK:
      if (lower.rfind("@ts", 0) == 0 || lower == "ok") {
        ESP_LOGV(TAG, "stats_parse cmd=@TS:01 payload=lock_ack");
        this->xml_stats_locked_ = true;
        this->xml_inflight_ = false;
        if (post_gate) {
          this->post_gate_tx_ready_event_ = true;
        }
        this->xml_deadline_ms_ = 0;
        ESP_LOGD(TAG, "stats_command_perf cmd=@TS:01 result=ok duration_ms=%u frames=%u noise=%u",
                 static_cast<unsigned>(now - this->xml_command_started_ms_),
                 static_cast<unsigned>(this->xml_command_frames_),
                 static_cast<unsigned>(this->xml_command_noise_frames_));
        ESP_LOGD(TAG, "stats_next_command cmd=@TR:32,00 delay_ms=%u",
                 static_cast<unsigned>(kStatsNextCommandDelayMs));
        this->transition_to_state_(XmlPollState::TR32_PAGE, now, kStatsNextCommandDelayMs);
        return true;
      }
      ESP_LOGD(TAG, "stats_reject cmd=@TS:01 reason=unexpected_response raw='%s'",
               sanitize_text_for_api(line).c_str());
      return false;

    case XmlPollState::WAIT_TR32_PAGE:
      if (lower.rfind("@tr:32", 0) == 0 || lower.rfind("@tr:00", 0) == 0) {
        parsed = this->parse_tr32_page_line_(line, this->xml_tr32_page_);
        if (parsed) {
          this->xml_stats_consecutive_failures_ = 0;
          if (this->xml_tr32_pages_ok_ < std::numeric_limits<uint8_t>::max()) {
            ++this->xml_tr32_pages_ok_;
          }
          this->publish_xml_stats_();
          this->xml_stats_.clear();
        } else {
          this->xml_cycle_failed_ = true;
          if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
            ++this->xml_stats_consecutive_failures_;
          }
        }
        this->xml_inflight_ = false;
        if (post_gate) {
          this->post_gate_tx_ready_event_ = true;
        }
        this->xml_deadline_ms_ = 0;
        if (!post_gate && this->xml_stats_consecutive_failures_ >= kStatsMaxConsecutiveFailures) {
          ESP_LOGD(TAG, "stats_reject cmd=@TR:32 reason=too_many_consecutive_failures count=%u",
                   static_cast<unsigned>(this->xml_stats_consecutive_failures_));
          this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
          return parsed;
        }
        ++this->xml_tr32_page_;
        char next_command[16];
        if (this->xml_tr32_page_ < kTr32PageCount) {
          std::snprintf(next_command, sizeof(next_command), "@TR:32,%02X",
                        static_cast<unsigned>(this->xml_tr32_page_));
        } else {
          std::snprintf(next_command, sizeof(next_command), "@TG:43");
        }
        ESP_LOGD(TAG, "stats_command_perf cmd=%s result=%s duration_ms=%u frames=%u noise=%u",
                 this->xml_last_command_.c_str(), parsed ? "ok" : "parse_failed",
                 static_cast<unsigned>(now - this->xml_command_started_ms_),
                 static_cast<unsigned>(this->xml_command_frames_),
                 static_cast<unsigned>(this->xml_command_noise_frames_));
        ESP_LOGD(TAG, "stats_next_command cmd=%s delay_ms=%u", next_command,
                 static_cast<unsigned>(kStatsNextCommandDelayMs));
        this->transition_to_state_(this->xml_tr32_page_ < kTr32PageCount ? XmlPollState::TR32_PAGE
                                                                         : XmlPollState::TG43,
                                   now, kStatsNextCommandDelayMs);
        return parsed;
      }
      ESP_LOGD(TAG, "stats_reject cmd=%s reason=unexpected_response raw='%s'", this->xml_last_command_.c_str(),
               sanitize_text_for_api(line).c_str());
      return false;

    case XmlPollState::WAIT_TG43:
      if (lower.rfind("@tg:43", 0) == 0) {
        this->xml_stats_.clear();
        parsed = this->parse_tg43_line_(line);
        if (parsed) {
          this->xml_stats_consecutive_failures_ = 0;
          this->xml_tg43_ok_ = true;
          this->publish_xml_stats_();
        } else {
          this->xml_cycle_failed_ = true;
          if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
            ++this->xml_stats_consecutive_failures_;
          }
        }
        this->xml_stats_.clear();
        this->xml_inflight_ = false;
        if (post_gate) {
          this->post_gate_tx_ready_event_ = true;
        }
        this->xml_deadline_ms_ = 0;
        ESP_LOGD(TAG, "stats_command_perf cmd=@TG:43 result=%s duration_ms=%u frames=%u noise=%u",
                 parsed ? "ok" : "parse_failed", static_cast<unsigned>(now - this->xml_command_started_ms_),
                 static_cast<unsigned>(this->xml_command_frames_),
                 static_cast<unsigned>(this->xml_command_noise_frames_));
        ESP_LOGD(TAG, "stats_next_command cmd=@TG:C0 delay_ms=%u",
                 static_cast<unsigned>(kStatsNextCommandDelayMs));
        this->transition_to_state_(XmlPollState::TGC0, now, kStatsNextCommandDelayMs);
        return parsed;
      }
      ESP_LOGD(TAG, "stats_reject cmd=@TG:43 reason=unexpected_response raw='%s'",
               sanitize_text_for_api(line).c_str());
      return false;

    case XmlPollState::WAIT_TGC0:
      if (lower.rfind("@tg:c0", 0) == 0) {
        this->xml_stats_.clear();
        parsed = this->parse_tgc0_line_(line);
        if (parsed) {
          this->xml_stats_consecutive_failures_ = 0;
          this->xml_tgc0_timeout_streak_ = 0;
          this->xml_tgc0_ok_ = true;
          this->publish_xml_stats_();
        } else {
          this->xml_cycle_failed_ = true;
          if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
            ++this->xml_stats_consecutive_failures_;
          }
        }
        this->xml_stats_.clear();
        this->xml_inflight_ = false;
        if (post_gate) {
          this->post_gate_tx_ready_event_ = true;
        }
        this->xml_deadline_ms_ = 0;
        ESP_LOGD(TAG, "stats_command_perf cmd=@TG:C0 result=%s duration_ms=%u frames=%u noise=%u",
                 parsed ? "ok" : "parse_failed", static_cast<unsigned>(now - this->xml_command_started_ms_),
                 static_cast<unsigned>(this->xml_command_frames_),
                 static_cast<unsigned>(this->xml_command_noise_frames_));
        ESP_LOGD(TAG, "stats_next_command cmd=%s delay_ms=%u",
                 this->xml_stats_use_ts_lock_ ? "@TS:00" : "DONE", static_cast<unsigned>(kStatsNextCommandDelayMs));
        this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_UNLOCK : XmlPollState::DONE, now,
                                   kStatsNextCommandDelayMs);
        return parsed;
      }
      ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 reason=unexpected_response raw='%s'",
               sanitize_text_for_api(line).c_str());
      return false;

    case XmlPollState::WAIT_TS_UNLOCK:
      if (lower.rfind("@ts", 0) == 0 || lower == "ok") {
        ESP_LOGV(TAG, "stats_parse cmd=@TS:00 payload=unlock_ack raw='%s'", sanitize_text_for_api(line).c_str());
        this->xml_stats_locked_ = false;
        this->xml_inflight_ = false;
        if (post_gate) {
          this->post_gate_tx_ready_event_ = true;
        }
        this->xml_deadline_ms_ = 0;
        ESP_LOGD(TAG, "stats_command_perf cmd=@TS:00 result=ok duration_ms=%u frames=%u noise=%u",
                 static_cast<unsigned>(now - this->xml_command_started_ms_),
                 static_cast<unsigned>(this->xml_command_frames_),
                 static_cast<unsigned>(this->xml_command_noise_frames_));
        this->transition_to_state_(XmlPollState::DONE, now);
        return true;
      }
      ESP_LOGD(TAG, "stats_reject cmd=@TS:00 reason=unexpected_response raw='%s'",
               sanitize_text_for_api(line).c_str());
      return false;

    default:
      ESP_LOGD(TAG, "stats_reject cmd=%s reason=unexpected_state state=%s", this->xml_last_command_.c_str(),
               this->xml_state_label_(this->xml_state_));
      return false;
  }
}

bool JuraComponent::handle_stats_binary_response_(uint32_t now) {
  if (!this->xml_stats_binary_response_) {
    return false;
  }

  const char *reason = this->xml_stats_reject_reason_.empty() ? "unexpected_binary" : this->xml_stats_reject_reason_.c_str();
  if (this->xml_deadline_ms_ != 0 && static_cast<int32_t>(now - this->xml_deadline_ms_) < 0) {
    uint32_t remaining_ms = static_cast<uint32_t>(this->xml_deadline_ms_ - now);
    if (this->xml_command_noise_frames_ < std::numeric_limits<uint16_t>::max()) {
      ++this->xml_command_noise_frames_;
    }
    ESP_LOGD(TAG, "stats_rx_noise cmd=%s reason=%s action=ignore_until_deadline remaining_ms=%u",
             this->xml_last_command_.c_str(), reason, static_cast<unsigned>(remaining_ms));
    this->xml_stats_binary_response_ = false;
    this->xml_stats_reject_reason_.clear();
    this->xml_stats_reject_decoded_.clear();
    this->xml_stats_capture_start_ms_ = 0;
    return false;
  }

  if (!this->xml_stats_reject_decoded_.empty()) {
    ESP_LOGD(TAG, "stats_reject cmd=%s reason=%s decoded=\"%s\"", this->xml_last_command_.c_str(), reason,
             this->xml_stats_reject_decoded_.c_str());
  } else {
    ESP_LOGD(TAG, "stats_reject cmd=%s reason=%s", this->xml_last_command_.c_str(), reason);
  }
  this->xml_stats_binary_response_ = false;
  this->xml_stats_reject_reason_.clear();
  this->xml_stats_reject_decoded_.clear();
  this->xml_cycle_failed_ = true;
  this->xml_inflight_ = false;
  if (this->stats_inner_tx_required_) {
    this->post_gate_tx_ready_event_ = true;
  }
  this->xml_deadline_ms_ = 0;
  this->xml_rx_line_.clear();
  this->xml_stats_capture_start_ms_ = 0;
  if (!this->stats_inner_tx_required_ && this->coffee_maker_ != nullptr && this->coffee_maker_->connection != nullptr) {
    auto *connection = this->coffee_maker_->connection.get();
    connection->reset_response_line_buffer();
    connection->reset_db_rx_buffer();
    connection->drain_serial_input_nonblocking();
  }
  this->advance_after_stats_reject_(now);
  return true;
}

void JuraComponent::advance_after_stats_reject_(uint32_t now) {
  switch (this->xml_state_) {
    case XmlPollState::WAIT_TS_LOCK:
      if (this->stats_inner_tx_required_ || this->xml_wait_for_ts_ack_) {
        this->xml_cycle_failed_ = true;
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
      } else {
        this->xml_stats_locked_ = true;
        this->transition_to_state_(XmlPollState::TR32_PAGE, now, kInterCmdGapMs);
      }
      return;

    case XmlPollState::WAIT_TR32_PAGE:
      if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
        ++this->xml_stats_consecutive_failures_;
      }
      if (!this->stats_inner_tx_required_ && this->xml_stats_consecutive_failures_ >= kStatsMaxConsecutiveFailures) {
        ESP_LOGD(TAG, "stats_reject cmd=@TR:32 reason=too_many_consecutive_failures count=%u",
                 static_cast<unsigned>(this->xml_stats_consecutive_failures_));
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
        return;
      }
      ++this->xml_tr32_page_;
      this->transition_to_state_(this->xml_tr32_page_ < kTr32PageCount ? XmlPollState::TR32_PAGE
                                                                       : XmlPollState::TG43,
                                 now, kStatsNextCommandDelayMs);
      return;

    case XmlPollState::WAIT_TG43:
      if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
        ++this->xml_stats_consecutive_failures_;
      }
      if (!this->stats_inner_tx_required_ && this->xml_stats_consecutive_failures_ >= kStatsMaxConsecutiveFailures) {
        ESP_LOGD(TAG, "stats_reject cmd=@TG:43 reason=too_many_consecutive_failures count=%u",
                 static_cast<unsigned>(this->xml_stats_consecutive_failures_));
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
      } else {
        this->transition_to_state_(XmlPollState::TGC0, now, kStatsNextCommandDelayMs);
      }
      return;

    case XmlPollState::WAIT_TGC0:
      if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
        ++this->xml_stats_consecutive_failures_;
      }
      this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_UNLOCK : XmlPollState::DONE, now,
                                 kStatsNextCommandDelayMs);
      return;

    case XmlPollState::WAIT_TS_UNLOCK:
      this->xml_stats_locked_ = false;
      this->finish_stats_cycle_(now, "unlock_rejected");
      return;

    default:
      this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_UNLOCK : XmlPollState::DONE, now,
                                 kInterCmdGapMs);
      return;
  }
}

void JuraComponent::advance_after_stats_timeout_(uint32_t now) {
  switch (this->xml_state_) {
    case XmlPollState::WAIT_TS_LOCK:
      if (this->stats_inner_tx_required_ || this->xml_wait_for_ts_ack_) {
        this->xml_cycle_failed_ = true;
        ESP_LOGD(TAG, "xml_retry_scheduled cmd=@TS:00 delay_ms=%u", static_cast<unsigned>(kInterCmdGapMs));
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
      } else {
        this->xml_stats_locked_ = true;
        this->transition_to_state_(XmlPollState::TR32_PAGE, now, kInterCmdGapMs);
      }
      return;

    case XmlPollState::WAIT_TR32_PAGE:
      ESP_LOGD(TAG, "stats_timeout cmd=@TR:32 page=%02X",
               static_cast<unsigned>(this->xml_tr32_page_));
      if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
        ++this->xml_stats_consecutive_failures_;
      }
      if (!this->stats_inner_tx_required_ && this->xml_stats_consecutive_failures_ >= kStatsMaxConsecutiveFailures) {
        ESP_LOGD(TAG, "stats_reject cmd=@TR:32 reason=too_many_consecutive_failures count=%u",
                 static_cast<unsigned>(this->xml_stats_consecutive_failures_));
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
        return;
      }
      ++this->xml_tr32_page_;
      this->transition_to_state_(this->xml_tr32_page_ < kTr32PageCount ? XmlPollState::TR32_PAGE
                                                                       : XmlPollState::TG43,
                                 now, kStatsNextCommandDelayMs);
      return;

    case XmlPollState::WAIT_TG43:
      if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
        ++this->xml_stats_consecutive_failures_;
      }
      if (!this->stats_inner_tx_required_ && this->xml_stats_consecutive_failures_ >= kStatsMaxConsecutiveFailures) {
        ESP_LOGD(TAG, "stats_reject cmd=@TG:43 reason=too_many_consecutive_failures count=%u",
                 static_cast<unsigned>(this->xml_stats_consecutive_failures_));
        this->transition_to_state_(XmlPollState::TS_UNLOCK, now, kInterCmdGapMs);
      } else {
        this->transition_to_state_(XmlPollState::TGC0, now, kStatsNextCommandDelayMs);
      }
      return;

    case XmlPollState::WAIT_TGC0:
      if (this->xml_stats_consecutive_failures_ < std::numeric_limits<uint8_t>::max()) {
        ++this->xml_stats_consecutive_failures_;
      }
      this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_UNLOCK : XmlPollState::DONE, now,
                                 kStatsNextCommandDelayMs);
      return;

    case XmlPollState::WAIT_TS_UNLOCK:
      this->xml_stats_locked_ = false;
      this->finish_stats_cycle_(now, "unlock_timeout");
      return;

    default:
      this->transition_to_state_(this->xml_stats_use_ts_lock_ ? XmlPollState::TS_UNLOCK : XmlPollState::DONE, now,
                                 kInterCmdGapMs);
      return;
  }
}

bool JuraComponent::extract_stats_hex_payload_(const std::string &line, const std::string &prefix,
                                               std::string &payload) const {
  std::string lower_line = to_lower_copy(line);
  std::string lower_prefix = to_lower_copy(prefix);
  trim_in_place(lower_line);
  if (lower_line.rfind(lower_prefix, 0) != 0) {
    payload.clear();
    return false;
  }
  std::string trimmed = line;
  trim_in_place(trimmed);
  payload = trimmed.substr(prefix.size());
  if (!payload.empty() && payload.front() == ',') {
    payload.erase(payload.begin());
  }
  trim_in_place(payload);
  return true;
}

bool JuraComponent::parse_hex_bytes_(const std::string &hex, std::vector<uint8_t> &bytes, std::string &reason) const {
  std::string payload = hex;
  trim_in_place(payload);
  bytes.clear();
  if (payload.empty()) {
    reason = "empty_payload";
    return false;
  }
  if ((payload.size() % 2) != 0) {
    reason = "odd_hex_length";
    return false;
  }
  if (!is_hex_text(payload)) {
    reason = "non_hex_payload";
    return false;
  }
  bytes.reserve(payload.size() / 2);
  for (size_t i = 0; i < payload.size(); i += 2) {
    char tmp[3] = {payload[i], payload[i + 1], '\0'};
    bytes.push_back(static_cast<uint8_t>(std::strtoul(tmp, nullptr, 16)));
  }
  reason.clear();
  return true;
}

std::string JuraComponent::raw_product_counter_field_name_(uint8_t page, uint8_t slot) const {
  char name[24];
  std::snprintf(name, sizeof(name), "tr32_page%02u_slot%u", static_cast<unsigned>(page), static_cast<unsigned>(slot));
  return name;
}

std::string JuraComponent::product_counter_field_name_(uint8_t product_index, std::string &label) const {
  for (const auto &product : this->xml_mapping_.products) {
    if (product.code != product_index) {
      continue;
    }
    label = product.name.empty() ? ("Product " + std::to_string(product_index)) : product.name;
    std::string id = sanitize_identifier(label);
    if (id.empty()) {
      id = "product_" + std::to_string(product_index);
    }
    return "tr32_" + id;
  }
  uint8_t page = product_index / kTr32ProductsPerPage;
  uint8_t slot = product_index % kTr32ProductsPerPage;
  std::string field = this->raw_product_counter_field_name_(page, slot);
  label = field;
  return field;
}

bool JuraComponent::stage_xml_stat_value_(const std::string &name, const std::string &label, double value,
                                          XmlSensorKind kind, const char *command_label) {
  auto &meta = this->xml_sensor_meta_[name];
  if (!meta.configured) {
    XmlField field;
    field.name = name;
    field.label = label;
    field.scale = 1.0;
    this->register_xml_sensor_(field, kind, command_label);
  }
  this->xml_stats_.set_value(name, value, label);
  ESP_LOGV(TAG, "stats_publish field=%s value=%s", name.c_str(), format_numeric_text(value).c_str());
  return true;
}

bool JuraComponent::parse_tr32_page_line_(const std::string &line, uint8_t expected_page) {
  std::string lower = to_lower_copy(line);
  trim_in_place(lower);
  if (lower.rfind("@tr:00", 0) == 0) {
    ESP_LOGD(TAG, "stats_tr32_page_empty page=%02X reason=machine_returned_tr00 action=ok",
             static_cast<unsigned>(expected_page));
    return true;
  }
  constexpr const char *kPrefix = "@tr:32,";
  if (lower.rfind(kPrefix, 0) != 0) {
    ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X reason=prefix_mismatch raw='%s'",
             static_cast<unsigned>(expected_page), sanitize_text_for_api(line).c_str());
    return false;
  }
  std::string trimmed = line;
  trim_in_place(trimmed);
  size_t pos = std::char_traits<char>::length(kPrefix);
  if (trimmed.size() < pos + 3 || trimmed[pos + 2] != ',') {
    ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X reason=missing_page_or_payload raw='%s'",
             static_cast<unsigned>(expected_page), sanitize_text_for_api(line).c_str());
    return false;
  }
  std::string page_text = trimmed.substr(pos, 2);
  if (!is_hex_text(page_text)) {
    ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X reason=invalid_page page_text='%s'",
             static_cast<unsigned>(expected_page), page_text.c_str());
    return false;
  }
  uint8_t page = static_cast<uint8_t>(std::strtoul(page_text.c_str(), nullptr, 16));
  if (page != expected_page) {
    ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X reason=page_mismatch got=%02X",
             static_cast<unsigned>(expected_page), static_cast<unsigned>(page));
    return false;
  }
  std::string payload = trimmed.substr(pos + 3);
  trim_in_place(payload);
  constexpr size_t expected_hex_len = kTr32ProductsPerPage * kTr32BytesPerProduct * 2;
  if (payload.size() < expected_hex_len) {
    ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X reason=short_payload hex_len=%u expected=%u",
             static_cast<unsigned>(page), static_cast<unsigned>(payload.size()),
             static_cast<unsigned>(expected_hex_len));
    return false;
  }
  payload = payload.substr(0, expected_hex_len);
  std::vector<uint8_t> bytes;
  std::string reason;
  if (!this->parse_hex_bytes_(payload, bytes, reason)) {
    ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X reason=%s payload='%s'", static_cast<unsigned>(page),
             reason.c_str(), payload.c_str());
    return false;
  }
  ESP_LOGV(TAG, "stats_parse cmd=@TR:32 page=%02X payload=%s bytes=%s", static_cast<unsigned>(page),
           payload.c_str(), format_hex_string(bytes).c_str());
  bool any = false;
  for (uint8_t slot = 0; slot < kTr32ProductsPerPage; ++slot) {
    size_t byte_offset = slot * kTr32BytesPerProduct;
    uint16_t value = (static_cast<uint16_t>(bytes[byte_offset]) << 8U) | bytes[byte_offset + 1];
    uint8_t product_index = page * kTr32ProductsPerPage + slot;
    if (value == 0xFFFF) {
      ESP_LOGV(TAG, "stats_parse cmd=@TR:32 page=%02X slot=%u value=FFFF action=empty",
               static_cast<unsigned>(page), static_cast<unsigned>(slot));
      continue;
    }
    if (value > this->xml_counter_max_) {
      ESP_LOGD(TAG, "stats_reject cmd=@TR:32 page=%02X slot=%u reason=counter_out_of_range value=%u max=%u",
               static_cast<unsigned>(page), static_cast<unsigned>(slot), static_cast<unsigned>(value),
               static_cast<unsigned>(this->xml_counter_max_));
      continue;
    }
    std::string label;
    std::string field = this->product_counter_field_name_(product_index, label);
    any = this->stage_xml_stat_value_(field, label, static_cast<double>(value), XmlSensorKind::Counter, "@TR:32") || any;
  }
  if (!any) {
    ESP_LOGD(TAG, "stats_tr32_page_empty page=%02X action=ok", static_cast<unsigned>(page));
  }
  return true;
}

bool JuraComponent::parse_tg43_line_(const std::string &line) {
  if (this->xml_mapping_.tg43.empty()) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:43 reason=mapping_empty");
    return false;
  }
  std::string payload;
  if (!this->extract_stats_hex_payload_(line, "@tg:43", payload)) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:43 reason=prefix_mismatch raw='%s'",
             sanitize_text_for_api(line).c_str());
    return false;
  }
  std::vector<uint8_t> bytes;
  std::string reason;
  if (!this->parse_hex_bytes_(payload, bytes, reason)) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:43 reason=%s payload='%s'", reason.c_str(), payload.c_str());
    return false;
  }
  ESP_LOGV(TAG, "stats_parse cmd=@TG:43 payload=%s bytes=%s", payload.c_str(), format_hex_string(bytes).c_str());
  bool any = false;
  for (size_t index = 0; index < this->xml_mapping_.tg43.fields.size(); ++index) {
    size_t byte_offset = index * 2;
    if (byte_offset + 1 >= bytes.size()) {
      ESP_LOGD(TAG, "stats_reject cmd=@TG:43 field=%s reason=field_overflow index=%u bytes=%u",
               this->xml_mapping_.tg43.fields[index].name.c_str(), static_cast<unsigned>(index),
               static_cast<unsigned>(bytes.size()));
      continue;
    }
    const auto &field = this->xml_mapping_.tg43.fields[index];
    uint16_t raw = (static_cast<uint16_t>(bytes[byte_offset]) << 8U) | bytes[byte_offset + 1];
    double value = static_cast<double>(raw) * field.scale;
    if (field.has_add) {
      value += field.add;
    }
    if (value < 0.0 || value > static_cast<double>(this->xml_counter_max_)) {
      ESP_LOGD(TAG, "stats_reject cmd=@TG:43 field=%s byte_index=%u reason=counter_out_of_range value=%s max=%u",
               field.name.c_str(), static_cast<unsigned>(byte_offset), format_numeric_text(value).c_str(),
               static_cast<unsigned>(this->xml_counter_max_));
      continue;
    }
    ESP_LOGV(TAG, "stats_parse cmd=@TG:43 field=%s byte_index=%u raw=0x%02X%02X value=%s", field.name.c_str(),
             static_cast<unsigned>(byte_offset), static_cast<unsigned>(bytes[byte_offset]),
             static_cast<unsigned>(bytes[byte_offset + 1]), format_numeric_text(value).c_str());
    any = this->stage_xml_stat_value_(field.name, field.label, value, XmlSensorKind::Counter, "@TG:43") || any;
  }
  return any;
}

bool JuraComponent::parse_tgc0_line_(const std::string &line) {
  if (this->xml_mapping_.tgc0.empty()) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 reason=mapping_empty");
    return false;
  }
  std::string trimmed = line;
  trim_in_place(trimmed);
  std::string lower = to_lower_copy(trimmed);
  constexpr const char *kPrefix = "@tg:c0";
  if (lower.rfind(kPrefix, 0) != 0) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 reason=prefix_mismatch raw='%s'",
             sanitize_text_for_api(line).c_str());
    ESP_LOGD(TAG, "maintenance_publish source=@TG:C0 valid=NO reason=prefix_mismatch publish=NO");
    return false;
  }
  std::string payload = trimmed.substr(std::char_traits<char>::length(kPrefix));
  trim_in_place(payload);
  if (payload.size() != 6) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 reason=invalid_hex_length hex_len=%u expected=6",
             static_cast<unsigned>(payload.size()));
    ESP_LOGD(TAG, "maintenance_publish source=@TG:C0 valid=NO reason=invalid_hex_length publish=NO");
    return false;
  }
  std::vector<uint8_t> bytes;
  std::string reason;
  if (!this->parse_hex_bytes_(payload, bytes, reason)) {
    ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 reason=%s payload='%s'", reason.c_str(), payload.c_str());
    ESP_LOGD(TAG, "maintenance_publish source=@TG:C0 valid=NO reason=%s publish=NO", reason.c_str());
    return false;
  }
  ESP_LOGV(TAG, "stats_parse cmd=@TG:C0 payload=%s bytes=%s", payload.c_str(), format_hex_string(bytes).c_str());
  bool any = false;
  size_t count = std::min(this->xml_mapping_.tgc0.fields.size(), bytes.size());
  for (size_t index = 0; index < count; ++index) {
    const auto &field = this->xml_mapping_.tgc0.fields[index];
    uint8_t raw = bytes[index];
    if (raw == 0xFF) {
      ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 field=%s byte_index=%u reason=not_available raw=0xFF",
               field.name.c_str(), static_cast<unsigned>(index));
      ESP_LOGD(TAG, "maintenance_publish source=@TG:C0 valid=YES field=%s raw=0xFF publish=NO reason=not_available",
               field.name.c_str());
      continue;
    }
    if (raw > 100) {
      ESP_LOGD(TAG, "stats_reject cmd=@TG:C0 field=%s byte_index=%u reason=percent_out_of_range value=%u",
               field.name.c_str(), static_cast<unsigned>(index), static_cast<unsigned>(raw));
      ESP_LOGD(TAG, "maintenance_publish source=@TG:C0 valid=NO field=%s cleaning_percent=%u publish=NO reason=out_of_range",
               field.name.c_str(), static_cast<unsigned>(raw));
      continue;
    }
    ESP_LOGV(TAG, "stats_parse cmd=@TG:C0 field=%s byte_index=%u raw=0x%02X value=%u", field.name.c_str(),
             static_cast<unsigned>(index), static_cast<unsigned>(raw), static_cast<unsigned>(raw));
    ESP_LOGD(TAG, "maintenance_publish source=@TG:C0 valid=YES field=%s cleaning_percent=%u publish=YES",
             field.name.c_str(), static_cast<unsigned>(raw));
    any = this->stage_xml_stat_value_(field.name, field.label, static_cast<double>(raw),
                                      XmlSensorKind::Measurement, "@TG:C0") || any;
  }
  return any;
}

void JuraComponent::finish_stats_cycle_(uint32_t now, const char *reason) {
  const char *end_reason = reason != nullptr && reason[0] != '\0' ? reason : "done";
  bool tr32_expected = this->xml_state_has_mapping_(XmlPollState::TR32_PAGE);
  bool tg43_expected = this->xml_state_has_mapping_(XmlPollState::TG43);
  bool tgc0_expected = this->xml_state_has_mapping_(XmlPollState::TGC0);
  bool any_ok = this->xml_tr32_pages_ok_ > 0 || this->xml_tg43_ok_ || this->xml_tgc0_ok_;
  bool all_ok = (!tr32_expected || this->xml_tr32_pages_ok_ == kTr32PageCount) &&
                (!tg43_expected || this->xml_tg43_ok_) &&
                (!tgc0_expected || this->xml_tgc0_ok_);
  const char *result = all_ok ? "success" : (any_ok ? "partial" : "failed");
  this->end_xml_transaction_(end_reason);
  this->xml_inflight_ = false;
  this->post_gate_tx_ready_event_ = true;
  this->xml_stats_locked_ = false;
  this->xml_last_command_.clear();
  this->xml_expected_prefix_.clear();
  this->xml_rx_line_.clear();
  this->xml_stats_capture_start_ms_ = 0;
  this->xml_rx_buffer_.clear();
  this->xml_stats_.clear();
  uint32_t sleep = std::max(this->xml_poll_interval_ms_, kCycleSleepMs);
  this->transition_to_state_(XmlPollState::SLEEP, now);
  this->xml_deadline_ms_ = now + sleep;
  this->xml_next_poll_ = this->xml_deadline_ms_;
  if (!any_ok) {
    ESP_LOGD(TAG, "stats_cycle_end result=failed reason=no_valid_tr32_pages next_retry_ms=%u",
             static_cast<unsigned>(sleep));
  } else {
    ESP_LOGD(TAG, "stats_cycle_end result=%s pages_ok=%u tg43_ok=%s tgc0_ok=%s next_retry_ms=%u", result,
             static_cast<unsigned>(this->xml_tr32_pages_ok_), YESNO(this->xml_tg43_ok_), YESNO(this->xml_tgc0_ok_),
             static_cast<unsigned>(sleep));
  }
  ESP_LOGD(TAG, "stats_cycle_perf duration_ms=%u pages_ok=%u tg43_ok=%s tgc0_ok=%s",
           static_cast<unsigned>(now - this->xml_cycle_started_ms_),
           static_cast<unsigned>(this->xml_tr32_pages_ok_), YESNO(this->xml_tg43_ok_), YESNO(this->xml_tgc0_ok_));
}

void JuraComponent::handle_xml_timeout_(XmlPollState next_state, const char *label, uint32_t now) {
  (void) next_state;
  (void) label;
  if (!this->xml_inflight_) {
    return;
  }
  ESP_LOGW(TAG, "RX_DB timeout %s", this->xml_state_label_(this->xml_state_));
  this->handle_xml_failure_(this->xml_state_, true, 0, now);
}

void JuraComponent::prepare_tgc0_request_() {
  // Keine zusätzlichen Flush-Operationen während des XML-Pollings.
}

void JuraComponent::ensure_setting_entities_created_() {
  if (this->settings_entities_created_) {
    return;
  }
  this->setting_descs_.clear();
  const auto &settings = get_settings();
  for (const auto &desc : settings) {
    if (!desc.id.empty()) {
      this->setting_descs_[desc.id] = desc;
    }
  }
  this->settings_entities_created_ = true;
}

bool JuraComponent::query_setting_command_(const std::string &command, std::vector<uint8_t> &decoded) {
  decoded.clear();
  if (command.empty()) {
    return false;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return false;
  }
  std::string cmd = command;
  if (cmd.find("\r\n") == std::string::npos) {
    cmd.append("\r\n");
  }
  auto response = this->coffee_maker_->connection->write_decoded_with_response(
      cmd, std::chrono::milliseconds{kCommandTimeoutMs});
  if (response == nullptr) {
    ESP_LOGW(TAG, "Einstellungspoll: keine Antwort für %s", command.c_str());
    this->publish_last_command_result_(std::string("timeout: ") + command);
    return false;
  }
  std::string payload = *response;
  payload.erase(std::remove(payload.begin(), payload.end(), '\r'), payload.end());
  payload.erase(std::remove(payload.begin(), payload.end(), '\n'), payload.end());
  std::string filtered;
  filtered.reserve(payload.size());
  for (unsigned char c : payload) {
    if (!std::isspace(c)) {
      filtered.push_back(static_cast<char>(c));
    }
  }
  for (std::size_t i = 0; i + 1 < filtered.size(); i += 2) {
    std::string byte_hex = filtered.substr(i, 2);
    char *end = nullptr;
    auto value = std::strtoul(byte_hex.c_str(), &end, 16);
    if (end == byte_hex.c_str()) {
      decoded.clear();
      break;
    }
    decoded.push_back(static_cast<uint8_t>(value));
  }
  if (decoded.empty() && !filtered.empty()) {
    decoded.assign(filtered.begin(), filtered.end());
  }
  this->publish_last_command_result_(std::string("response: ") + command);
  return !decoded.empty();
}

bool JuraComponent::query_error_command_(const std::string &command, std::vector<uint8_t> &decoded) {
  return this->query_setting_command_(command, decoded);
}

void JuraComponent::publish_setting_value_(const SettingDesc &desc, float value, const std::string &raw_text) {
  if (desc.type != SettingValueType::String) {
    auto it = this->setting_sensors_.find(desc.id);
    if (it != this->setting_sensors_.end() && it->second != nullptr) {
      it->second->publish_state(value);
    }
  }
  auto text_it = this->setting_text_sensors_.find(desc.id);
  if (text_it != this->setting_text_sensors_.end() && text_it->second != nullptr) {
    text_it->second->publish_state(sanitize_text_for_api(raw_text));
  }
}

void JuraComponent::poll_settings_refresh_() {
  this->ensure_setting_entities_created_();
  if (this->setting_descs_.empty()) {
    return;
  }
  bool has_command_settings = false;
  bool has_xml_settings = false;
  for (const auto &entry : this->setting_descs_) {
    if (!entry.second.source_cmd.empty()) {
      has_command_settings = true;
    } else {
      has_xml_settings = true;
    }
  }

  if (has_command_settings) {
    std::unordered_map<std::string, std::vector<uint8_t>> command_cache;
    std::unordered_set<std::string> failed_commands;

    auto get_command_payload = [&](const std::string &command) -> const std::vector<uint8_t> * {
      if (command.empty()) {
        return nullptr;
      }
      if (failed_commands.find(command) != failed_commands.end()) {
        return nullptr;
      }
      auto cache_it = command_cache.find(command);
      if (cache_it != command_cache.end()) {
        return &cache_it->second;
      }
      std::vector<uint8_t> decoded;
      if (!this->query_setting_command_(command, decoded)) {
        failed_commands.insert(command);
        return nullptr;
      }
      auto inserted = command_cache.emplace(command, std::move(decoded));
      return &inserted.first->second;
    };

    for (const auto &entry : this->setting_descs_) {
      const auto &desc = entry.second;
      if (desc.source_cmd.empty()) {
        continue;
      }
      const auto *decoded_ptr = get_command_payload(desc.source_cmd);
      if (decoded_ptr == nullptr) {
        continue;
      }
      const auto &decoded = *decoded_ptr;
      if (desc.offset + desc.width > decoded.size()) {
        ESP_LOGW(TAG, "Einstellung %s: Antwort zu kurz (offset=%u width=%u size=%u)", desc.id.c_str(),
                 static_cast<unsigned>(desc.offset), static_cast<unsigned>(desc.width),
                 static_cast<unsigned>(decoded.size()));
        continue;
      }
      const uint8_t *ptr = decoded.data() + desc.offset;
      std::uint64_t raw = 0;
      for (std::size_t i = 0; i < desc.width; ++i) {
        raw = (raw << 8U) | static_cast<std::uint64_t>(ptr[i]);
      }
      float scaled = static_cast<float>(raw) * desc.scale;
      std::string text_value;
      switch (desc.type) {
        case SettingValueType::Bool:
          text_value = raw ? "on" : "off";
          scaled = raw ? 1.0f : 0.0f;
          break;
        case SettingValueType::Enum:
        case SettingValueType::U8:
        case SettingValueType::U16:
        case SettingValueType::U32:
          text_value = format_numeric_text(static_cast<double>(scaled));
          break;
        case SettingValueType::String:
          text_value.assign(reinterpret_cast<const char *>(ptr), desc.width);
          if (auto zero_pos = text_value.find('\0'); zero_pos != std::string::npos) {
            text_value.resize(zero_pos);
          }
          this->publish_setting_value_(desc, 0.0f, text_value);
          continue;
      }
      this->publish_setting_value_(desc, scaled, text_value);
    }
  }

  if (has_xml_settings) {
    uint32_t previous_timestamp = this->machine_xml_timestamp_;
    std::string xml;
    if (this->ensure_machine_xml_(kSettingsRefreshMs, xml)) {
      if (previous_timestamp == this->machine_xml_timestamp_) {
        this->update_settings_from_xml_(xml);
      }
    }
  }
}

void JuraComponent::poll_settings_once_() {
  if (!this->is_ready()) {
    return;
  }
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  uint32_t now = esphome::millis();
  if (!this->settings_boot_polled_) {
    this->poll_settings_refresh_();
    this->settings_boot_polled_ = true;
    this->settings_next_refresh_ = now + kSettingsRefreshMs;
    return;
  }
  if (this->settings_next_refresh_ != 0 && !time_reached(now, this->settings_next_refresh_)) {
    return;
  }
  this->poll_settings_refresh_();
  this->settings_next_refresh_ = now + kSettingsRefreshMs;
}

void JuraComponent::publish_error_state_(uint32_t code) {
  if (this->error_code_sensor_ != nullptr) {
    this->error_code_sensor_->publish_state(static_cast<float>(code));
  }
  bool has_error = code != 0;
  if (this->error_active_sensor_ != nullptr) {
    this->error_active_sensor_->publish_state(has_error);
  }
  const ErrorDesc *desc = find_error(code);
  if (desc != nullptr) {
    if (this->error_text_sensor_ != nullptr) {
      this->error_text_sensor_->publish_state(desc->text);
    }
    if (this->error_severity_sensor_ != nullptr) {
      this->error_severity_sensor_->publish_state(desc->severity);
    }
  } else {
    if (this->error_text_sensor_ != nullptr) {
      this->error_text_sensor_->publish_state(has_error ? "unbekannt" : "kein Fehler");
    }
    if (this->error_severity_sensor_ != nullptr) {
      this->error_severity_sensor_->publish_state(has_error ? "unknown" : "none");
    }
  }
  this->publish_last_command_result_(has_error ? "error_active" : "no_error");
  this->last_error_code_ = code;
  this->errors_entities_created_ = true;
}

void JuraComponent::poll_error_cycle_() {
  if (this->coffee_maker_ == nullptr || this->coffee_maker_->connection == nullptr) {
    return;
  }
  if (!this->is_ready()) {
    return;
  }
  uint32_t now = esphome::millis();
  if (this->errors_next_poll_ != 0 && !time_reached(now, this->errors_next_poll_)) {
    return;
  }
  this->errors_next_poll_ = now + kErrorPollIntervalMs;
  std::string command = error_source_command();
  if (!command.empty()) {
    std::vector<uint8_t> decoded;
    if (!this->query_error_command_(command, decoded)) {
      return;
    }
    if (decoded.empty()) {
      return;
    }
    uint32_t code = 0;
    for (uint8_t byte : decoded) {
      code = (code << 8U) | byte;
    }
    if (!this->errors_entities_created_ || code != this->last_error_code_) {
      this->publish_error_state_(code);
    }
    return;
  }

  uint32_t previous_timestamp = this->machine_xml_timestamp_;
  std::string xml;
  if (this->ensure_machine_xml_(kErrorPollIntervalMs, xml)) {
    if (previous_timestamp == this->machine_xml_timestamp_) {
      this->update_errors_from_xml_(xml);
    }
  }
}

void JuraComponent::start_brew(::jutta_proto::CoffeeMaker::coffee_t coffee) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot start brew - component not ready.");
    return;
  }
  this->coffee_maker_->brew_coffee(coffee);
}

void JuraComponent::start_custom_brew(uint32_t grind_duration_ms, uint32_t water_duration_ms) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot brew custom coffee - component not ready.");
    return;
  }
  this->custom_cancel_flag_ = false;
  this->coffee_maker_->brew_custom_coffee(&this->custom_cancel_flag_, std::chrono::milliseconds{grind_duration_ms},
                                          std::chrono::milliseconds{water_duration_ms});
}

void JuraComponent::cancel_custom_brew() {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot cancel custom brew - component not ready.");
    return;
  }
  if (!this->custom_cancel_flag_) {
    ESP_LOGI(TAG, "Cancelling custom brew.");
  }
  this->custom_cancel_flag_ = true;
}

void JuraComponent::switch_page(uint32_t page) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot switch page - component not ready.");
    return;
  }
  this->coffee_maker_->switch_page(page);
}

void JuraComponent::run_sequence(const std::vector<::jutta_proto::CoffeeMaker::SequenceStep> &steps) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot run sequence - component not ready.");
    return;
  }
  if (this->coffee_maker_->is_locked()) {
    ESP_LOGW(TAG, "Cannot run sequence - coffee maker busy.");
    return;
  }
  this->coffee_maker_->run_sequence(steps);
}

bool JuraComponent::is_busy() const {
  if (this->coffee_maker_ == nullptr) {
    return false;
  }
  return this->coffee_maker_->is_locked();
}

}  // namespace jutta_component
}  // namespace esphome
