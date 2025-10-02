#include "jutta_connection.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <iterator>
#include <utility>

#include "esphome/core/log.h"
#include "esphome/core/time.h"

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------

namespace {
constexpr const char *TAG = "jutta_connection";
constexpr uint32_t PRE_SEND_GAP_MS = 35;
constexpr std::array<uint8_t, 8> DB_TRAILER = {0xDF, 0xFF, 0xDB, 0xDB, 0xFB, 0xFB, 0xDB, 0xDB};

bool extract_line(std::string &buffer, std::string &line) {
    auto terminator = buffer.find("\r\n");
    if (terminator == std::string::npos) {
        return false;
    }
    line = buffer.substr(0, terminator);
    buffer.erase(0, terminator + 2);
    return true;
}

bool ends_with_crlf(const std::string &value) {
    return value.size() >= 2 && value[value.size() - 2] == '\r' && value[value.size() - 1] == '\n';
}

std::string sanitize_for_log(const std::string &value) {
    std::string result;
    result.reserve(value.size());
    for (unsigned char c : value) {
        switch (c) {
            case '\r':
                result.append("\\r");
                break;
            case '\n':
                result.append("\\n");
                break;
            case '\t':
                result.append("\\t");
                break;
            default:
                if (std::isprint(c) != 0) {
                    result.push_back(static_cast<char>(c));
                } else {
                    char buffer[5];
                    snprintf(buffer, sizeof(buffer), "\\x%02X", c);
                    result.append(buffer);
                }
                break;
        }
    }
    return result;
}

std::string trim_crlf(const std::string &value) {
    std::string trimmed = value;
    while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n')) {
        trimmed.pop_back();
    }
    return trimmed;
}

void append_chunk(std::string &target, const std::array<uint8_t, 4> &chunk, size_t count) {
    target.append(reinterpret_cast<const char *>(chunk.data()), count);
}

void append_chunk(std::vector<uint8_t> &target, const std::array<uint8_t, 4> &chunk, size_t count) {
    target.insert(target.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(count));
}

bool decode_db_payload(const std::vector<uint8_t> &raw, std::vector<uint8_t> &decoded) {
    decoded.clear();
    decoded.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
        uint8_t byte = raw[i];
        if (byte == 0xDB) {
            if (i + 1 >= raw.size()) {
                return false;
            }
            uint8_t escaped = static_cast<uint8_t>(raw[i + 1] ^ 0x20);
            decoded.push_back(escaped);
            ++i;
        } else {
            decoded.push_back(byte);
        }
    }
    return true;
}

std::string bytes_to_string(const std::vector<uint8_t> &data) {
    return std::string(reinterpret_cast<const char *>(data.data()), data.size());
}

}  // namespace

JuttaConnection::JuttaConnection(esphome::uart::UARTComponent *parent) : serial_(parent) {}

void JuttaConnection::init() { serial_.init(); }

bool JuttaConnection::write_decoded(const std::string &ascii_line) {
    line_wait_context_.active = false;
    db_wait_context_.active = false;
    return send_line_cmd(ascii_line);
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(const std::string &command,
                                                                    const std::string &expected_response,
                                                                    const std::chrono::milliseconds &timeout) {
    if (!line_wait_context_.active) {
        if (!send_line_cmd(command)) {
            return WaitResult::Error;
        }
        line_wait_context_.active = true;
        line_wait_context_.expected = trim_crlf(expected_response);
        line_wait_context_.expect_ok = false;
        line_wait_context_.timeout = timeout;
        line_wait_context_.start_time = esphome::millis();
    }
    return poll_line_wait(line_wait_context_.expected, false, timeout);
}

JuttaConnection::WaitResult JuttaConnection::wait_for_ok(const std::chrono::milliseconds &timeout) {
    return poll_line_wait("ok:", true, timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_xml_with_response(const std::string &command,
                                                                      const std::chrono::milliseconds &timeout) {
    if (command.empty()) {
        ESP_LOGW(TAG, "write_xml_with_response called with empty command");
        return nullptr;
    }

    if (!db_wait_context_.active) {
        if (!send_db_cmd(command)) {
            return nullptr;
        }
        db_wait_context_.active = true;
        db_wait_context_.timeout = timeout;
        db_wait_context_.start_time = esphome::millis();
    }

    std::vector<uint8_t> decoded;
    if (read_db_frame(decoded, 0)) {
        db_wait_context_.active = false;
        return std::make_shared<std::string>(bytes_to_string(decoded));
    }

    if (timeout.count() > 0) {
        uint32_t now = esphome::millis();
        uint32_t target = db_wait_context_.start_time + static_cast<uint32_t>(timeout.count());
        if (time_reached(now, target)) {
            ESP_LOGW(TAG, "Timeout while waiting for DB response frame");
            db_wait_context_.active = false;
            db_rx_buffer_.clear();
        }
    }
    return nullptr;
}

bool JuttaConnection::poll_response_line(std::string &line) {
    return read_line_until(line, 0);
}

void JuttaConnection::reset_response_line_buffer() {
    if (!line_rx_buffer_.empty()) {
        ESP_LOGD(TAG, "Resetting line RX buffer (%zu bytes discarded)", line_rx_buffer_.size());
        line_rx_buffer_.clear();
    }
}

void JuttaConnection::flush_serial_input() const {
    drain_uart();
    line_rx_buffer_.clear();
    db_rx_buffer_.clear();
    line_wait_context_ = {};
    db_wait_context_ = {};
}

bool JuttaConnection::send_line_cmd(const std::string &ascii_line) {
    prepare_for_send();

    std::string payload = ascii_line;
    if (!ends_with_crlf(payload)) {
        payload.append("\r\n");
    }

    ESP_LOGD(TAG, "TX_LINE \"%s\"", sanitize_for_log(payload).c_str());

    const auto *bytes = reinterpret_cast<const uint8_t *>(payload.data());
    if (!serial_.write_serial_buffer(bytes, payload.size())) {
        ESP_LOGE(TAG, "Failed to send line command over UART");
        return false;
    }
    serial_.flush();
    return true;
}

bool JuttaConnection::send_db_cmd(const std::string &ascii_command) {
    prepare_for_send();

    std::vector<uint8_t> payload(ascii_command.begin(), ascii_command.end());
    std::vector<uint8_t> encoded;
    encoded.reserve(payload.size() * 2 + DB_TRAILER.size());

    for (uint8_t byte : payload) {
        switch (byte) {
            case 0xDB:
            case 0xDF:
            case 0xFB:
            case 0xFF:
                encoded.push_back(0xDB);
                encoded.push_back(static_cast<uint8_t>(byte ^ 0x20));
                break;
            default:
                encoded.push_back(byte);
                break;
        }
    }

    encoded.insert(encoded.end(), DB_TRAILER.begin(), DB_TRAILER.end());

    ESP_LOGD(TAG, "TX_DB len=%zu", encoded.size());

    if (!serial_.write_serial_buffer(encoded)) {
        ESP_LOGE(TAG, "Failed to send DB command over UART");
        return false;
    }
    serial_.flush();
    return true;
}

bool JuttaConnection::read_line_until(std::string &out, uint32_t timeout_ms) const {
    if (extract_line(line_rx_buffer_, out)) {
        ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_for_log(out).c_str());
        return true;
    }

    uint32_t start = esphome::millis();
    uint32_t deadline = timeout_ms > 0 ? start + timeout_ms : 0;

    while (true) {
        std::array<uint8_t, 4> chunk{};
        size_t read = serial_.read_serial(chunk);
        if (read > 0) {
            append_chunk(line_rx_buffer_, chunk, read);
            if (extract_line(line_rx_buffer_, out)) {
                ESP_LOGD(TAG, "RX_LINE \"%s\"", sanitize_for_log(out).c_str());
                return true;
            }
            continue;
        }

        if (timeout_ms == 0) {
            return false;
        }

        if (time_reached(esphome::millis(), deadline)) {
            return false;
        }

        esphome::delay(1);
    }
}

bool JuttaConnection::read_db_frame(std::vector<uint8_t> &decoded, uint32_t timeout_ms) const {
    auto try_extract = [&]() -> bool {
        if (db_rx_buffer_.size() < DB_TRAILER.size()) {
            return false;
        }

        auto trailer_pos = std::search(db_rx_buffer_.begin(), db_rx_buffer_.end(), DB_TRAILER.begin(), DB_TRAILER.end());
        if (trailer_pos == db_rx_buffer_.end()) {
            return false;
        }

        std::vector<uint8_t> raw(db_rx_buffer_.begin(), trailer_pos);
        db_rx_buffer_.erase(db_rx_buffer_.begin(), trailer_pos + static_cast<std::ptrdiff_t>(DB_TRAILER.size()));

        std::vector<uint8_t> unescaped;
        if (!decode_db_payload(raw, unescaped)) {
            ESP_LOGW(TAG, "Discarding DB frame with invalid escape sequence (raw_len=%zu)", raw.size());
            return false;
        }

        ESP_LOGD(TAG, "RX_DB raw_len=%zu decoded_len=%zu trailer_ok=1", raw.size(), unescaped.size());
        decoded = std::move(unescaped);
        return true;
    };

    if (try_extract()) {
        return true;
    }

    uint32_t start = esphome::millis();
    uint32_t deadline = timeout_ms > 0 ? start + timeout_ms : 0;

    while (true) {
        std::array<uint8_t, 4> chunk{};
        size_t read = serial_.read_serial(chunk);
        if (read > 0) {
            append_chunk(db_rx_buffer_, chunk, read);
            if (try_extract()) {
                return true;
            }
            continue;
        }

        if (timeout_ms == 0) {
            return false;
        }

        if (time_reached(esphome::millis(), deadline)) {
            return false;
        }

        esphome::delay(1);
    }
}

JuttaConnection::WaitResult JuttaConnection::poll_line_wait(const std::string &expected, bool expect_ok,
                                                            const std::chrono::milliseconds &timeout) const {
    if (!line_wait_context_.active) {
        line_wait_context_.active = true;
        line_wait_context_.expected = expected;
        line_wait_context_.expect_ok = expect_ok;
        line_wait_context_.timeout = timeout;
        line_wait_context_.start_time = esphome::millis();
    }

    std::string line;
    while (read_line_until(line, 0)) {
        if ((expect_ok && line == "ok:") || (!expect_ok && line == expected)) {
            if (expect_ok) {
                ESP_LOGD(TAG, "OK=1");
            }
            line_wait_context_ = {};
            return WaitResult::Success;
        }

        ESP_LOGD(TAG, "RX_LINE \"%s\" (unexpected)", sanitize_for_log(line).c_str());
        std::string reinject = line;
        reinject.append("\r\n");
        line_rx_buffer_.insert(0, reinject);
        break;
    }

    if (timeout.count() > 0) {
        uint32_t now = esphome::millis();
        uint32_t target = line_wait_context_.start_time + static_cast<uint32_t>(timeout.count());
        if (time_reached(now, target)) {
            if (expect_ok) {
                ESP_LOGD(TAG, "OK=0 (timeout)");
            }
            line_wait_context_ = {};
            return WaitResult::Timeout;
        }
    }

    return WaitResult::Pending;
}

void JuttaConnection::prepare_for_send() const {
    flush_serial_input();
    if (PRE_SEND_GAP_MS > 0) {
        esphome::delay(PRE_SEND_GAP_MS);
    }
}

void JuttaConnection::drain_uart() const {
    std::array<uint8_t, 4> chunk{};
    while (serial_.read_serial(chunk) > 0) {
        // discard
    }
}

bool JuttaConnection::time_reached(uint32_t now, uint32_t target) {
    if (target == 0) {
        return false;
    }
    return static_cast<int32_t>(target - now) <= 0;
}

//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
