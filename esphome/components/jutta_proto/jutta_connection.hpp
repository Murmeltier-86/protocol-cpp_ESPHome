#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "esphome/components/uart/uart.h"

#include "serial_connection.hpp"

namespace jutta_proto {

class JuttaConnection {
 public:
  enum class WaitResult { Success, Timeout, Error };

  explicit JuttaConnection(esphome::uart::UARTComponent *parent);

  void init();

  bool write_decoded(const std::string &command);

  WaitResult wait_for_ok(const std::chrono::milliseconds &timeout);

  WaitResult write_decoded_wait_for(const std::string &command, const std::string &expected_response,
                                    const std::chrono::milliseconds &timeout);

  std::shared_ptr<std::string> write_xml_with_response(const std::string &command,
                                                       const std::chrono::milliseconds &timeout);

  bool poll_response_line(std::string &line);

  void reset_response_line_buffer();

  void flush_serial_input();

 private:
  bool send_line_cmd(const std::string &command);
  bool send_db_cmd(const std::string &command);

  bool read_line_until(std::string &line, uint32_t timeout_ms);
  bool read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms);
  bool extract_line_from_buffer(std::string &line);

  bool transact_line(const std::string &command, std::string *response_line, bool need_ok,
                     uint32_t timeout_ms);
  bool transact_db(const std::string &command, std::vector<uint8_t> *decoded, uint32_t timeout_ms);

  void flush_and_gap();
  void drain_uart();

  bool write_bytes(const uint8_t *data, size_t length);
  size_t available() const;
  bool read_byte(uint8_t *byte);

  static std::string sanitize_line_for_log(const std::string &line);

  serial::SerialConnection serial_;
  std::string line_buffer_;

  static constexpr uint32_t LINE_TIMEOUT_MS = 700;
  static constexpr uint32_t DB_TIMEOUT_MS = 900;
  static constexpr uint32_t PRE_SEND_GAP_MS = 35;
};

}  // namespace jutta_proto
