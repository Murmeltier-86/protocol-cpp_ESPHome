#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "serial_connection.hpp"

namespace jutta_proto {

class JuttaConnection {
 public:
  enum class WaitResult { Pending, Success, Timeout, Error };

  explicit JuttaConnection(esphome::uart::UARTComponent *parent);

  void init();

  bool write_decoded(const std::string &command);

  WaitResult write_decoded_wait_for(const std::string &command, const std::string &expected_response,
                                    const std::chrono::milliseconds &timeout = std::chrono::milliseconds{5000});

  std::shared_ptr<std::string> transact_db(const std::string &command,
                                           const std::chrono::milliseconds &timeout = std::chrono::milliseconds{1000});

  WaitResult wait_for_ok(const std::chrono::milliseconds &timeout = std::chrono::milliseconds{5000});

  bool poll_response_line(std::string &line);

  void reset_response_line_buffer();

  void flush_serial_input();

 private:
  serial::SerialConnection serial_;

  std::string line_rx_buffer_;
  std::deque<std::string> pending_lines_;
  std::vector<uint8_t> db_rx_buffer_;

  struct OkWaitContext {
    bool active{false};
    std::chrono::milliseconds timeout{0};
    uint32_t start_time{0};
  } ok_wait_context_;

  void flush_and_gap();

  static std::string trim_command(const std::string &command);
  static bool command_requires_ok(const std::string &command);

  void send_line_cmd(const std::string &line);
  void send_db_cmd(const std::string &command);

  bool read_line_until(std::string &out, uint32_t timeout_ms);
  bool read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms);

  void poll_serial_lines(uint32_t timeout_ms);
  bool wait_for_line(const std::string &expected, const std::chrono::milliseconds &timeout);

  std::shared_ptr<std::string> transact_db_internal(const std::string &command,
                                                    const std::chrono::milliseconds &timeout, bool *ok);
};

}  // namespace jutta_proto
