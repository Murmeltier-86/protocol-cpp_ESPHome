#include "jutta_connection.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include "esphome/core/log.h"
#include "esphome/core/time.h"

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------
static const char* TAG = "jutta_connection";

namespace {
constexpr uint32_t JUTTA_SERIAL_GAP_MS = 8;
constexpr uint8_t JUTTA_BYTE_MASK = 0x7F;

std::string format_bytes(const uint8_t* data, size_t size) {
    if (size == 0) {
        return "<empty>";
    }

    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (size_t i = 0; i < size; ++i) {
        if (i != 0) {
            stream << ' ';
        }
        stream << "0x" << std::setw(2) << static_cast<int>(data[i]);
    }
    return stream.str();
}

std::string format_bytes(const std::vector<uint8_t>& data) {
    return format_bytes(data.data(), data.size());
}

template <size_t N>
std::string format_bytes(const std::array<uint8_t, N>& data) {
    return format_bytes(data.data(), data.size());
}

inline void wait_for_jutta_gap() {
    const uint32_t start = esphome::millis();
    while (esphome::millis() - start < JUTTA_SERIAL_GAP_MS) {
        // Busy-wait to preserve the required 8 ms spacing between JUTTA bytes.
    }
}

inline uint8_t normalize_encoded_byte(uint8_t byte) {
    return byte & JUTTA_BYTE_MASK;
}

inline bool is_possible_encoded_byte(uint8_t byte) {
    switch (normalize_encoded_byte(byte)) {
        case 0x5B:
        case 0x5F:
        case 0x7B:
        case 0x7F:
            return true;
        default:
            return false;
    }
}

inline bool frames_equivalent(const std::array<uint8_t, 4>& lhs, const std::array<uint8_t, 4>& rhs) {
    for (size_t i = 0; i < lhs.size(); ++i) {
        if (normalize_encoded_byte(lhs[i]) != normalize_encoded_byte(rhs[i])) {
            return false;
        }
    }
    return true;
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
    std::array<uint8_t, 4> buffer{};
    if (!read_encoded_unsafe(buffer)) {
        return false;
    }
    *byte = decode(buffer);
    return true;
}

bool JuttaConnection::read_decoded_unsafe(std::vector<uint8_t>& data) const {
    // Read encoded data:
    std::vector<std::array<uint8_t, 4>> dataBuffer;
    if (read_encoded_unsafe(dataBuffer) <= 0) {
        ESP_LOGV(TAG, "No encoded frames available when attempting to read decoded data.");
        return false;
    }

    // Decode all:
    for (const std::array<uint8_t, 4>& buffer : dataBuffer) {
        data.push_back(decode(buffer));
    }
    std::string decoded = vec_to_string(data);
    ESP_LOGD(TAG, "Decoded %zu byte(s): %s", data.size(), decoded.c_str());
    return true;
}

bool JuttaConnection::write_decoded_unsafe(const uint8_t& byte) const {
    ESP_LOGD(TAG, "Writing single decoded byte: 0x%02x", byte);
    return write_encoded_unsafe(encode(byte));
}

bool JuttaConnection::write_decoded_unsafe(const std::vector<uint8_t>& data) const {
    ESP_LOGD(TAG, "Writing %zu decoded byte(s).", data.size());
    ESP_LOGVV(TAG, "Decoded payload bytes: %s", format_bytes(data).c_str());
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
    ESP_LOGD(TAG, "Writing decoded string (%zu byte).", data.size());
    ESP_LOGVV(TAG, "Decoded string payload: %s", data.c_str());
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

    bool result = true;
    ESP_LOGV(TAG, "Sending encoded frame: %s", format_bytes(encData).c_str());
    for (uint8_t byte : encData) {
        if (!serial.write_serial_byte(byte)) {
            result = false;
            break;
        }
        ESP_LOGVV(TAG, "Serial byte 0x%02x written, flushing UART.", byte);
        serial.flush();
        ESP_LOGVV(TAG, "UART flush complete, waiting for bus gap.");
        wait_for_jutta_gap();
    }

    return result;
}

bool JuttaConnection::read_encoded_unsafe(std::array<uint8_t, 4>& buffer) const {

    ESP_LOGVV(TAG, "Starting encoded frame read. buffered=%zu", this->encoded_rx_buffer_.size());
    if (!align_encoded_rx_buffer()) {
        if (this->encoded_rx_buffer_.size() < buffer.size()) {
            wait_for_jutta_gap();
            std::array<uint8_t, 4> chunk{};
            size_t size = serial.read_serial(chunk);
            if (size > chunk.size()) {
                ESP_LOGW(TAG, "Invalid amount of UART data found (%zu byte) - ignoring.", size);
                size = chunk.size();
            }

            if (size > 0) {
                this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.end(), chunk.begin(), chunk.begin() + size);
                ESP_LOGVV(TAG, "Read %zu raw byte(s) from UART: %s", size,
                         format_bytes(chunk.data(), size).c_str());
            } else if (this->encoded_rx_buffer_.empty()) {
                ESP_LOGV(TAG, "No serial data found.");
                return false;
            }
        }

        if (!align_encoded_rx_buffer()) {
            return false;
        }
    }

    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        wait_for_jutta_gap();
        std::array<uint8_t, 4> chunk{};
        size_t read = serial.read_serial(chunk);
        if (read > chunk.size()) {
            ESP_LOGW(TAG, "Invalid amount of UART data found (%zu byte) - ignoring.", read);
            read = chunk.size();
        }

        if (read == 0) {
            if (this->encoded_rx_buffer_.empty()) {
                ESP_LOGV(TAG, "No serial data found.");
            }
            return false;
        }

        this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.end(), chunk.begin(), chunk.begin() + read);
        ESP_LOGVV(TAG, "Buffered %zu additional byte(s): %s", read,
                 format_bytes(chunk.data(), read).c_str());
    }

    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        ESP_LOGW(TAG, "Invalid amount of UART data found while forming encoded frame (%zu byte).",
                 this->encoded_rx_buffer_.size());
        return false;
    }

    std::copy_n(this->encoded_rx_buffer_.begin(), buffer.size(), buffer.begin());
    this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(),
                                   this->encoded_rx_buffer_.begin() + buffer.size());

    ESP_LOGV(TAG, "Read 4 encoded bytes.");
    ESP_LOGVV(TAG, "Encoded frame contents: %s", format_bytes(buffer).c_str());
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

bool JuttaConnection::align_encoded_rx_buffer() const {
    size_t skipped = 0;

    auto log_skipped = [&]() {
        if (skipped > 0) {
            ESP_LOGW(TAG, "Discarded %zu stray encoded byte%s while seeking JUTTA frame boundary.", skipped,
                     skipped == 1 ? "" : "s");
            skipped = 0;
        }
    };

    while (true) {
        ESP_LOGVV(TAG, "Aligning buffer. current buffered=%zu", this->encoded_rx_buffer_.size());
        while (!this->encoded_rx_buffer_.empty() && !is_possible_encoded_byte(this->encoded_rx_buffer_.front())) {
            this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin());
            ++skipped;
        }

        if (this->encoded_rx_buffer_.size() < 4) {
            log_skipped();
            return false;
        }

        std::array<uint8_t, 4> candidate{};
        std::copy_n(this->encoded_rx_buffer_.begin(), candidate.size(), candidate.begin());

        bool candidate_valid = true;
        for (uint8_t byte : candidate) {
            if (!is_possible_encoded_byte(byte)) {
                candidate_valid = false;
                break;
            }
        }

        if (!candidate_valid) {
            this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin());
            ++skipped;
            continue;
        }

        uint8_t decoded = decode(candidate);
        auto reencoded = encode(decoded);
        if (frames_equivalent(candidate, reencoded)) {
            log_skipped();
            return true;
        }

        this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin());
        ++skipped;
    }
}

void JuttaConnection::flush_serial_input() const {
    ESP_LOGD(TAG, "Flushing serial input buffers.");
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
        ESP_LOGVV(TAG, "Discarded %zu byte(s) during flush: %s", read,
                 format_bytes(discard.data(), std::min(read, discard.size())).c_str());
        wait_for_jutta_gap();
    }
}

void JuttaConnection::reinject_decoded_front(const std::string& data) const {
    if (data.empty()) {
        return;
    }

    ESP_LOGD(TAG, "Reinjecting %zu decoded byte(s) into RX buffer.", data.size());
    std::vector<uint8_t> encoded;
    encoded.reserve(data.size() * 4);
    for (char c : data) {
        auto enc = encode(static_cast<uint8_t>(static_cast<unsigned char>(c)));
        encoded.insert(encoded.end(), enc.begin(), enc.end());
    }

    this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.begin(), encoded.begin(), encoded.end());
    ESP_LOGV(TAG, "Re-injected %zu decoded bytes.", data.size());
    ESP_LOGVV(TAG, "Encoded reinjection data: %s", format_bytes(encoded).c_str());
}


JuttaConnection::WaitResult JuttaConnection::wait_for_ok(const std::chrono::milliseconds& timeout) {
    return wait_for_response_unsafe("ok:\r\n", timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_decoded_with_response(const std::vector<uint8_t>& data,
                                                                         const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        ESP_LOGD(TAG, "write_decoded_with_response starting. timeout=%lu ms", static_cast<unsigned long>(timeout.count()));
        if (!write_decoded_unsafe(data)) {
            return nullptr;
        }
        ESP_LOGVV(TAG, "Awaiting response after sending decoded data: %s", format_bytes(data).c_str());
    }
    return wait_for_str_unsafe(timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_decoded_with_response(const std::string& data,
                                                                         const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        ESP_LOGD(TAG, "write_decoded_with_response (string) starting. timeout=%lu ms",
                 static_cast<unsigned long>(timeout.count()));
        if (!write_decoded_unsafe(data)) {
            return nullptr;
        }
        ESP_LOGVV(TAG, "Awaiting response after sending decoded string: %s", data.c_str());
    }
    return wait_for_str_unsafe(timeout);
}

std::shared_ptr<std::string> JuttaConnection::wait_for_str_unsafe(const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        this->wait_string_context_.active = true;
        this->wait_string_context_.timeout = timeout;
        this->wait_string_context_.start_time = esphome::millis();
    }

    std::vector<uint8_t> buffer;
    if (read_decoded_unsafe(buffer) && !buffer.empty()) {
        this->wait_string_context_.active = false;
        ESP_LOGD(TAG, "Received response string (%zu byte).", buffer.size());
        ESP_LOGVV(TAG, "Response payload bytes: %s", format_bytes(buffer).c_str());
        return std::make_shared<std::string>(vec_to_string(buffer));
    }

    if (timeout.count() > 0) {
        uint32_t now = esphome::millis();
        uint32_t elapsed = now - this->wait_string_context_.start_time;
        if (elapsed >= static_cast<uint32_t>(timeout.count())) {
            this->wait_string_context_.active = false;
            ESP_LOGW(TAG, "Timed out waiting for response string after %lu ms.",
                     static_cast<unsigned long>(timeout.count()));
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
        ESP_LOGD(TAG, "Waiting for response '%s' (timeout=%lu ms)", response.c_str(),
                 static_cast<unsigned long>(timeout.count()));
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
            ESP_LOGW(TAG, "Timeout waiting for response '%s' after %lu ms", response.c_str(),
                     static_cast<unsigned long>(timeout.count()));
            return WaitResult::Timeout;
        }
    }

    std::vector<uint8_t> buffer;
    if (read_decoded_unsafe(buffer) && !buffer.empty()) {
        std::string incoming(buffer.begin(), buffer.end());
        this->wait_context_.recent.append(incoming);
        ESP_LOGD(TAG, "Received %zu byte(s) while waiting for '%s'.", buffer.size(), response.c_str());
        ESP_LOGVV(TAG, "Incoming bytes: %s", format_bytes(buffer).c_str());
        if (this->wait_context_.recent.find(response) != std::string::npos) {
            this->wait_context_.active = false;
            this->wait_context_.recent.clear();
            ESP_LOGD(TAG, "Received expected response '%s'.", response.c_str());
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
        ESP_LOGD(TAG, "Sent decoded bytes while waiting for '%s'.", response.c_str());
        ESP_LOGVV(TAG, "Sent bytes: %s", format_bytes(data).c_str());
    }
    return wait_for_response_unsafe(response, timeout);
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(const std::string& data, const std::string& response,
                                                                    const std::chrono::milliseconds& timeout) {
    if (!this->wait_context_.active || this->wait_context_.expected != response) {
        if (!write_decoded_unsafe(data)) {
            return WaitResult::Error;
        }
        ESP_LOGD(TAG, "Sent decoded string while waiting for '%s'.", response.c_str());
        ESP_LOGVV(TAG, "Sent string: %s", data.c_str());
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
