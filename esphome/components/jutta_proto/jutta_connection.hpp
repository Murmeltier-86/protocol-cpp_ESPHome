#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "esphome/components/uart/uart.h"

#include "serial_connection.hpp"

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------
class JuttaConnection {
 public:
    enum class WaitResult { Pending, Success, Timeout, Error };

    explicit JuttaConnection(esphome::uart::UARTComponent* parent);

    void init();

    bool write_decoded(const std::string& ascii_line);

    WaitResult write_decoded_wait_for(const std::string& command, const std::string& expected_response,
                                      const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});

    WaitResult wait_for_ok(const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});

    std::shared_ptr<std::string> write_xml_with_response(
        const std::string& command, const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});

    bool poll_response_line(std::string& line);

    void reset_response_line_buffer();

    void flush_serial_input() const;

 private:
    struct LineWaitContext {
        bool active{false};
        std::string expected{};
        bool expect_ok{false};
        std::chrono::milliseconds timeout{std::chrono::milliseconds{5000}};
        uint32_t start_time{0};
    };

    struct DbWaitContext {
        bool active{false};
        std::chrono::milliseconds timeout{std::chrono::milliseconds{5000}};
        uint32_t start_time{0};
    };

    serial::SerialConnection serial_;

    mutable std::string line_rx_buffer_{};
    mutable std::vector<uint8_t> db_rx_buffer_{};
    mutable LineWaitContext line_wait_context_{};
    mutable DbWaitContext db_wait_context_{};

    bool send_line_cmd(const std::string& ascii_line);
    bool send_db_cmd(const std::string& ascii_command);

    bool read_line_until(std::string& out, uint32_t timeout_ms) const;
    bool read_db_frame(std::vector<uint8_t>& decoded, uint32_t timeout_ms) const;

    WaitResult poll_line_wait(const std::string& expected, bool expect_ok,
                              const std::chrono::milliseconds& timeout) const;

    void prepare_for_send() const;
    void drain_uart() const;

    static bool time_reached(uint32_t now, uint32_t target);
};
//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
