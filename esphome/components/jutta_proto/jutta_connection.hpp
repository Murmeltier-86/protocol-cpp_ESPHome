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

  void drain_plain_serial(uint32_t duration_ms);

  void send_plain_line(const std::string &line);

  bool read_line_until(std::string &out, uint32_t timeout_ms);

 private:
  serial::SerialConnection serial_;

  struct XmlFrame {
    std::vector<uint8_t> payload;
    size_t encoded_size{0};
  };

  enum class ActivePipeline { Idle, Plain, Xml };

  std::vector<uint8_t> encoded_rx_buffer_;
  std::string plain_rx_buffer_;
  std::deque<std::string> pending_lines_;
  std::deque<XmlFrame> pending_xml_frames_;
  ActivePipeline active_pipeline_{ActivePipeline::Idle};

  struct OkWaitContext {
    bool active{false};
    std::chrono::milliseconds timeout{0};
    uint32_t start_time{0};
  } ok_wait_context_;

  void flush_and_gap();
  void drain_encoded(const std::chrono::milliseconds &duration);
  bool pump_serial(uint32_t timeout_ms);
  bool process_encoded_frames();
  bool decode_encoded_frame(const std::vector<uint8_t> &encoded, std::vector<uint8_t> &decoded) const;
  bool is_plain_text_frame(const std::vector<uint8_t> &decoded) const;
  void route_decoded_frame(std::vector<uint8_t> decoded, size_t encoded_length);
  static std::string format_hex(const std::vector<uint8_t> &buffer, size_t max_bytes = 64);
  void set_active_pipeline(ActivePipeline pipeline);

  static std::string trim_command(const std::string &command);
  static bool command_requires_ok(const std::string &command);

  void send_line_cmd(const std::string &line);
  void send_db_cmd(const std::string &command);
  bool read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms);

  void poll_serial_lines(uint32_t timeout_ms);
  bool wait_for_line(const std::string &expected, const std::chrono::milliseconds &timeout);

  std::shared_ptr<std::string> transact_db_internal(const std::string &command,
                                                    const std::chrono::milliseconds &timeout, bool *ok);

  std::string line_read_buffer_;
};

}  // namespace jutta_proto
