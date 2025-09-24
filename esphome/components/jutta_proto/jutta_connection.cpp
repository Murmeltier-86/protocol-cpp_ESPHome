#include "jutta_connection.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <sstream>
#include <string>
#include <utility>
#include "esphome/core/log.h"
#include "esphome/core/time.h"

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------
static const char* TAG = "jutta_connection";

namespace {
constexpr uint32_t JUTTA_SERIAL_GAP_MS = 8;

inline void wait_for_jutta_gap() {
    const uint32_t start = esphome::millis();
    while (esphome::millis() - start < JUTTA_SERIAL_GAP_MS) {
        // Busy-wait to preserve the required 8 ms spacing between JUTTA bytes.
    }
}
}  // namespace
JuttaConnection::JuttaConnection(esphome::uart::UARTComponent* parent) : serial(parent) {}

void JuttaConnection::init() {
    serial.init();
}

bool JuttaConnection::read_decoded(std::vector<uint8_t>& data) {
    return read_decoded_unsafe(data);
}

bool JuttaConnection::read_decoded(uint8_t* byte) {
    return read_decoded_unsafe(byte);
}

bool JuttaConnection::read_decoded_unsafe(uint8_t* byte) const {
    if (!this->decoded_rx_buffer_.empty()) {
        *byte = this->decoded_rx_buffer_.front();
        this->decoded_rx_buffer_.pop_front();
        return true;
    }
    std::array<uint8_t, 4> buffer{};
    if (!read_encoded_unsafe(buffer)) {
        return false;
    }
    *byte = decode(buffer);
    return true;
}

bool JuttaConnection::read_decoded_unsafe(std::vector<uint8_t>& data) const {
    bool got_any = false;
    if (!this->decoded_rx_buffer_.empty()) {
        data.insert(data.end(), this->decoded_rx_buffer_.begin(), this->decoded_rx_buffer_.end());
        this->decoded_rx_buffer_.clear();
        got_any = true;
    }

    // Read encoded data:
    std::vector<std::array<uint8_t, 4>> dataBuffer;
    if (read_encoded_unsafe(dataBuffer) <= 0) {
        if (got_any && !data.empty()) {
            std::string decoded = vec_to_string(data);
            ESP_LOGD(TAG, "Read: %s", decoded.c_str());
        }
        return got_any;
    }

    // Decode all:
    for (const std::array<uint8_t, 4>& buffer : dataBuffer) {
        data.push_back(decode(buffer));
    }
    got_any = true;
    std::string decoded = vec_to_string(data);
    ESP_LOGD(TAG, "Read: %s", decoded.c_str());
    return got_any;
}

bool JuttaConnection::write_decoded_unsafe(const uint8_t& byte) const {
    return write_encoded_unsafe(encode(byte));
}

bool JuttaConnection::write_decoded_unsafe(const std::vector<uint8_t>& data) const {
    // Bad compiler support:
    // return std::ranges::all_of(data.begin(), data.end(), [this](uint8_t byte) { return write_decoded_unsafe(byte); });
    // So we use this until it gets better:
    bool result = true;
    for (uint8_t byte : data) {
        if (!write_decoded_unsafe(byte)) {
            result = false;
        }
    }
    return result;
}

bool JuttaConnection::write_decoded_unsafe(const std::string& data) const {
    // Bad compiler support:
    // return std::ranges::all_of(data.begin(), data.end(), [this](char c) { return write_decoded_unsafe(static_cast<uint8_t>(c)); });
    // So we use this until it gets better:
    bool result = true;
    for (char c : data) {
        if (!write_decoded_unsafe(static_cast<uint8_t>(c))) {
            result = false;
        }
    }
    return result;
}

bool JuttaConnection::write_decoded(const uint8_t& byte) {
    return write_decoded_unsafe(byte);
}

bool JuttaConnection::write_decoded(const std::vector<uint8_t>& data) {
    return write_decoded_unsafe(data);
}

bool JuttaConnection::write_decoded(const std::string& data) {
    return write_decoded_unsafe(data);
}

void JuttaConnection::print_byte(const uint8_t& byte) {
    for (size_t i = 0; i < 8; i++) {
        ESP_LOGI(TAG, "%d ", ((byte >> (7 - i)) & 0b00000001));
    }
    // printf("-> %d\t%02x\t%c", byte, byte, byte);
    printf("-> %d\t%02x", byte, byte);
}

void JuttaConnection::print_bytes(const std::vector<uint8_t>& data) {
    for (const uint8_t& byte : data) {
        print_byte(byte);
    }
}

void JuttaConnection::run_encode_decode_test() {
    bool success = true;

    for (uint16_t i = 0b00000000; i <= 0b11111111; i++) {
        if (i != decode(encode(i))) {
            success = false;
            ESP_LOGE(TAG, "data:");
            print_byte(i);

            std::array<uint8_t, 4> dataEnc = encode(i);
            for (size_t i = 0; i < 4; i++) {
                ESP_LOGE(TAG, "dataEnc[%zu]", i);
                print_byte(dataEnc.at(i));
            }

            uint8_t dataDec = decode(dataEnc);
            ESP_LOGE(TAG, "dataDec:");
            print_byte(dataDec);
        }
    }
    // Flush the stdout to ensure the result gets printed when assert(success) fails:
    ESP_LOGI(TAG, "Encode decode test: %s", success ? "true" : "false");
    assert(success);
}

std::array<uint8_t, 4> JuttaConnection::encode(const uint8_t& decData) {
    // 1111 0000 -> 0000 1111:
    uint8_t tmp = ((decData & 0xF0) >> 4) | ((decData & 0x0F) << 4);

    // 1100 1100 -> 0011 0011:
    tmp = ((tmp & 0xC0) >> 2) | ((tmp & 0x30) << 2) | ((tmp & 0x0C) >> 2) | ((tmp & 0x03) << 2);

    // The base bit layout for all send bytes:
    constexpr uint8_t BASE = 0b01011011;

    std::array<uint8_t, 4> encData{};
    encData[0] = BASE | ((tmp & 0b10000000) >> 2);
    encData[0] |= ((tmp & 0b01000000) >> 4);

    encData[1] = BASE | (tmp & 0b00100000);
    encData[1] |= ((tmp & 0b00010000) >> 2);

    encData[2] = BASE | ((tmp & 0b00001000) << 2);
    encData[2] |= (tmp & 0b00000100);

    encData[3] = BASE | ((tmp & 0b00000010) << 4);
    encData[3] |= ((tmp & 0b00000001) << 2);

    return encData;
}

uint8_t JuttaConnection::decode(const std::array<uint8_t, 4>& encData) {
    // Bit mask for the 2. bit from the left:
    constexpr uint8_t B2_MASK = (0b10000000 >> 2);
    // Bit mask for the 5. bit from the left:
    constexpr uint8_t B5_MASK = (0b10000000 >> 5);

    uint8_t decData = 0;
    decData |= (encData[0] & B2_MASK) << 2;
    decData |= (encData[0] & B5_MASK) << 4;

    decData |= (encData[1] & B2_MASK);
    decData |= (encData[1] & B5_MASK) << 2;

    decData |= (encData[2] & B2_MASK) >> 2;
    decData |= (encData[2] & B5_MASK);

    decData |= (encData[3] & B2_MASK) >> 4;
    decData |= (encData[3] & B5_MASK) >> 2;

    // 1111 0000 -> 0000 1111:
    decData = ((decData & 0xF0) >> 4) | ((decData & 0x0F) << 4);

    // 1100 1100 -> 0011 0011:
    decData = ((decData & 0xC0) >> 2) | ((decData & 0x30) << 2) | ((decData & 0x0C) >> 2) | ((decData & 0x03) << 2);

    return decData;
}

bool JuttaConnection::write_encoded_unsafe(const std::array<uint8_t, 4>& encData) const {
    bool result = serial.write_serial(encData);
    serial.flush();
    wait_for_jutta_gap();
    return result;
}

bool JuttaConnection::read_encoded_unsafe(std::array<uint8_t, 4>& buffer) const {
    if (!this->encoded_rx_buffer_.empty() && (this->encoded_rx_buffer_.size() % buffer.size()) != 0) {
        ESP_LOGW(TAG, "Discarding %zu stray encoded bytes.", this->encoded_rx_buffer_.size());
        flush_serial_input();
    }

    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        wait_for_jutta_gap();
        std::array<uint8_t, 4> chunk{};
        size_t size = serial.read_serial(chunk);
        if (size > chunk.size()) {
            ESP_LOGW(TAG, "Invalid amount of UART data found (%zu byte) - ignoring.", size);
            size = chunk.size();
        }

        if (size == 0) {
            if (this->encoded_rx_buffer_.empty()) {
                ESP_LOGV(TAG, "No serial data found.");
            }
            return false;
        }

        if (size < chunk.size()) {
            ESP_LOGW(TAG, "Invalid amount of UART data found (%zu byte) - flushing.", size);
            flush_serial_input();
            return false;
        }

        this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.end(), chunk.begin(), chunk.begin() + size);
    }

    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        return false;
    }

    std::copy_n(this->encoded_rx_buffer_.begin(), buffer.size(), buffer.begin());
    this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(),
                                   this->encoded_rx_buffer_.begin() + buffer.size());
    ESP_LOGV(TAG, "Read 4 encoded bytes.");
    return true;
}

size_t JuttaConnection::read_encoded_unsafe(std::vector<std::array<uint8_t, 4>>& data) const {
    size_t count = 0;
    while (true) {
        std::array<uint8_t, 4> buffer{};
        if (!read_encoded_unsafe(buffer)) {
            break;
        }
        data.push_back(buffer);
        ++count;
    }
    return count;
}

void JuttaConnection::flush_serial_input() const {
    this->encoded_rx_buffer_.clear();
    std::array<uint8_t, 4> discard{};
    while (true) {
        size_t read = serial.read_serial(discard);
        if (read == 0) {
            break;
        }
        if (read > discard.size()) {
            ESP_LOGW(TAG, "Invalid amount of UART data found while flushing (%zu byte).", read);
        }
        wait_for_jutta_gap();
    }
}

void JuttaConnection::reinject_decoded_front(const std::string& data) const {
    if (data.empty()) {
        return;
    }

    std::vector<uint8_t> encoded;
    encoded.reserve(data.size() * 4);
    for (char c : data) {
        auto enc = encode(static_cast<uint8_t>(static_cast<unsigned char>(c)));
        encoded.insert(encoded.end(), enc.begin(), enc.end());
    }

    this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.begin(), encoded.begin(), encoded.end());
    ESP_LOGV(TAG, "Re-injected %zu decoded bytes.", data.size());
}

JuttaConnection::WaitResult JuttaConnection::wait_for_ok(const std::chrono::milliseconds& timeout) {
    return wait_for_response_unsafe("ok:\r\n", timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_decoded_with_response(const std::vector<uint8_t>& data,
                                                                         const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        if (!write_decoded_unsafe(data)) {
            return nullptr;
        }
    }
    return wait_for_str_unsafe(timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_decoded_with_response(const std::string& data,
                                                                         const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        if (!write_decoded_unsafe(data)) {
            return nullptr;
        }
    }
    return wait_for_str_unsafe(timeout);
}

std::shared_ptr<std::string> JuttaConnection::wait_for_str_unsafe(const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        this->wait_string_context_.active = true;
        this->wait_string_context_.timeout = timeout;
        this->wait_string_context_.start_time = esphome::millis();
        this->wait_string_context_.buffer.clear();
    }

    std::vector<uint8_t> buffer;
    if (read_decoded_unsafe(buffer) && !buffer.empty()) {
        this->wait_string_context_.buffer.append(buffer.begin(), buffer.end());
    }

    auto newline_pos = this->wait_string_context_.buffer.find("\r\n");
    if (newline_pos != std::string::npos) {
        std::string result = this->wait_string_context_.buffer.substr(0, newline_pos + 2);
        std::string remainder = this->wait_string_context_.buffer.substr(newline_pos + 2);
        if (!remainder.empty()) {
            this->decoded_rx_buffer_.insert(this->decoded_rx_buffer_.end(), remainder.begin(), remainder.end());
        }
        this->wait_string_context_.buffer.clear();
        this->wait_string_context_.active = false;
        return std::make_shared<std::string>(std::move(result));
    }

    if (timeout.count() > 0) {
        uint32_t now = esphome::millis();
        uint32_t elapsed = now - this->wait_string_context_.start_time;
        if (elapsed >= static_cast<uint32_t>(timeout.count())) {
            if (!this->wait_string_context_.buffer.empty()) {
                this->decoded_rx_buffer_.insert(this->decoded_rx_buffer_.end(),
                                                this->wait_string_context_.buffer.begin(),
                                                this->wait_string_context_.buffer.end());
                this->wait_string_context_.buffer.clear();
            }
            this->wait_string_context_.active = false;
        }
    }

    return nullptr;
}

JuttaConnection::WaitResult JuttaConnection::wait_for_response_unsafe(const std::string& response,
                                                                      const std::chrono::milliseconds& timeout) {
    if (!this->wait_context_.active || this->wait_context_.expected != response) {
        this->wait_context_.active = true;
        this->wait_context_.expected = response;
        this->wait_context_.recent.clear();
        this->wait_context_.timeout = timeout;
        this->wait_context_.start_time = esphome::millis();
    }

    if (response.empty()) {
        this->wait_context_.active = false;
        this->wait_context_.recent.clear();
        return WaitResult::Success;
    }

    if (timeout.count() > 0) {
        uint32_t now = esphome::millis();
        uint32_t elapsed = now - this->wait_context_.start_time;
        if (elapsed >= static_cast<uint32_t>(timeout.count())) {
            this->wait_context_.active = false;
            this->wait_context_.recent.clear();
            return WaitResult::Timeout;
        }
    }

    std::vector<uint8_t> buffer;
    if (read_decoded_unsafe(buffer) && !buffer.empty()) {
        std::string incoming(buffer.begin(), buffer.end());
        this->wait_context_.recent.append(incoming);
        if (this->wait_context_.recent.find(response) != std::string::npos) {
            this->wait_context_.active = false;
            this->wait_context_.recent.clear();
            return WaitResult::Success;
        }
        if (this->wait_context_.recent.size() > response.size()) {
            this->wait_context_.recent.erase(0, this->wait_context_.recent.size() - response.size());
        }
    }

    return WaitResult::Pending;
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(const std::vector<uint8_t>& data,
                                                                    const std::string& response,
                                                                    const std::chrono::milliseconds& timeout) {
    if (!this->wait_context_.active || this->wait_context_.expected != response) {
        if (!write_decoded_unsafe(data)) {
            return WaitResult::Error;
        }
    }
    return wait_for_response_unsafe(response, timeout);
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(const std::string& data, const std::string& response,
                                                                    const std::chrono::milliseconds& timeout) {
    if (!this->wait_context_.active || this->wait_context_.expected != response) {
        if (!write_decoded_unsafe(data)) {
            return WaitResult::Error;
        }
    }
    return wait_for_response_unsafe(response, timeout);
}

std::string JuttaConnection::vec_to_string(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        return "";
    }

    std::ostringstream sstream;
    for (unsigned char i : data) {
        sstream << static_cast<char>(i);
    }
    return sstream.str();
}

//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
