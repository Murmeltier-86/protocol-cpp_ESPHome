#include "jutta_connection.hpp"

#include <algorithm>
#include <cassert>
#include <cctype>
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
constexpr uint8_t JUTTA_ENCODE_BASE = 0xFF;
constexpr uint8_t JUTTA_BIT0_MASK = static_cast<uint8_t>(1u << 2);
constexpr uint8_t JUTTA_BIT1_MASK = static_cast<uint8_t>(1u << 5);
std::string format_hex(const uint8_t* data, size_t length) {
    if (length == 0) {
        return "[]";
    }
    std::ostringstream stream;
    stream << "[";
    for (size_t i = 0; i < length; ++i) {
        if (i > 0) {
            stream << ' ';
        }
        stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
               << static_cast<int>(data[i]);
    }
    stream << "]";
    return stream.str();
}

template <size_t N>
std::string format_hex(const std::array<uint8_t, N>& data) {
    return format_hex(data.data(), data.size());
}

std::string format_hex(const std::vector<uint8_t>& data) {
    return format_hex(data.data(), data.size());
}

std::string format_hex(uint8_t byte) {
    return format_hex(&byte, 1);
}

std::string format_printable(const uint8_t* data, size_t length) {
    if (length == 0) {
        return "";
    }

    std::ostringstream stream;
    for (size_t i = 0; i < length; ++i) {
        const unsigned char c = data[i];
        switch (c) {
            case '\r':
                stream << "\\r";
                break;
            case '\n':
                stream << "\\n";
                break;
            case '\t':
                stream << "\\t";
                break;
            default:
                if (std::isprint(c) != 0) {
                    stream << static_cast<char>(c);
                } else {
                    stream << "\\x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
                           << static_cast<int>(c);
                }
                break;
        }
    }
    return stream.str();
}

template <size_t N>
std::string format_printable(const std::array<uint8_t, N>& data) {
    return format_printable(data.data(), data.size());
}

std::string format_printable(const std::vector<uint8_t>& data) {
    return format_printable(data.data(), data.size());
}

std::string format_printable(const std::string& data) {
    return format_printable(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

std::string format_printable(uint8_t byte) {
    return format_printable(&byte, 1);
}

bool try_extract_line(std::string& buffer, std::string& line) {
    auto terminator = buffer.find("\r\n");
    if (terminator == std::string::npos) {
        return false;
    }

    line = buffer.substr(0, terminator);
    buffer.erase(0, terminator + 2);
    return true;
}

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
    ESP_LOGVV(TAG, "Attempting to read single decoded byte (encoded buffer size=%zu, decoded buffer size=%zu).",
              this->encoded_rx_buffer_.size(), this->decoded_rx_buffer_.size());
    if (!this->decoded_rx_buffer_.empty()) {
        *byte = this->decoded_rx_buffer_.front();
        this->decoded_rx_buffer_.pop_front();
        ESP_LOGD(TAG, "Decoded byte from buffer: '%s' (%s)", format_printable(*byte).c_str(),
                 format_hex(*byte).c_str());
        return true;
    }
    std::array<uint8_t, 4> buffer{};
    if (!read_encoded_unsafe(buffer)) {
        ESP_LOGVV(TAG, "Unable to read encoded frame for single byte - waiting for more data.");
        return false;
    }
    *byte = decode(buffer);
    ESP_LOGD(TAG, "Decoded byte: '%s' (%s)", format_printable(*byte).c_str(), format_hex(*byte).c_str());
    return true;
}

bool JuttaConnection::read_decoded_unsafe(std::vector<uint8_t>& data) const {
    ESP_LOGVV(TAG, "Attempting to read decoded bytes (encoded buffer size=%zu, decoded buffer size=%zu).",
              this->encoded_rx_buffer_.size(), this->decoded_rx_buffer_.size());
    bool any_data = false;

    if (!this->decoded_rx_buffer_.empty()) {
        while (!this->decoded_rx_buffer_.empty()) {
            uint8_t buffered_byte = this->decoded_rx_buffer_.front();
            this->decoded_rx_buffer_.pop_front();
            data.push_back(buffered_byte);
        }
        any_data = true;
    }

    std::vector<std::array<uint8_t, 4>> dataBuffer;
    size_t frames_read = read_encoded_unsafe(dataBuffer);
    if (frames_read == 0) {
        if (any_data) {
            ESP_LOGD(TAG, "Read decoded payload from buffer (%zu byte%s).", data.size(), data.size() == 1 ? "" : "s");
            return true;
        }
        ESP_LOGVV(TAG, "No complete encoded frames available to decode yet.");
        return false;
    }

    size_t index = 0;
    std::vector<uint8_t> newly_decoded;
    newly_decoded.reserve(dataBuffer.size());
    for (const std::array<uint8_t, 4>& buffer : dataBuffer) {
        uint8_t decoded_byte = decode(buffer);
        data.push_back(decoded_byte);
        newly_decoded.push_back(decoded_byte);
        ESP_LOGVV(TAG, "Decoded frame %zu: %s -> '%s' (%s)", index, format_hex(buffer).c_str(),
                  format_printable(decoded_byte).c_str(), format_hex(decoded_byte).c_str());
        ++index;
    }
    std::string decoded = vec_to_string(newly_decoded);
    ESP_LOGD(TAG, "Read decoded payload (%zu byte%s): '%s' (hex %s)", dataBuffer.size(),
             dataBuffer.size() == 1 ? "" : "s", format_printable(decoded).c_str(), format_hex(newly_decoded).c_str());
    return true;
}

bool JuttaConnection::write_decoded_unsafe(const uint8_t& byte) const {
    ESP_LOGD(TAG, "Queueing single decoded byte for transmission: '%s' (%s)",
             format_printable(byte).c_str(), format_hex(byte).c_str());
    auto encoded = encode(byte);
    ESP_LOGVV(TAG, "Encoded representation: %s", format_hex(encoded).c_str());
    bool result = write_encoded_unsafe(encoded);
    ESP_LOGVV(TAG, "Transmission of decoded byte %s", result ? "succeeded" : "failed");
    return result;
}

bool JuttaConnection::write_decoded_unsafe(const std::vector<uint8_t>& data) const {
    // Bad compiler support:
    // return std::ranges::all_of(data.begin(), data.end(), [this](uint8_t byte) { return write_decoded_unsafe(byte); });
    // So we use this until it gets better:
    if (!data.empty()) {
        ESP_LOGD(TAG, "Queueing %zu decoded byte%s for transmission: '%s' (hex %s)", data.size(),
                 data.size() == 1 ? "" : "s", format_printable(data).c_str(), format_hex(data).c_str());
    } else {
        ESP_LOGVV(TAG, "Requested to write an empty decoded payload.");
    }
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
    std::vector<uint8_t> bytes(data.begin(), data.end());
    return write_decoded_unsafe(bytes);
}

bool JuttaConnection::write_decoded(const uint8_t& byte) {
    if (!this->wait_context_.active && !this->wait_string_context_.active) {
        flush_serial_input();
    }
    return write_decoded_unsafe(byte);
}

bool JuttaConnection::write_decoded(const std::vector<uint8_t>& data) {
    if (!this->wait_context_.active && !this->wait_string_context_.active) {
        flush_serial_input();
    }
    return write_decoded_unsafe(data);
}

bool JuttaConnection::write_decoded(const std::string& data) {
    if (!this->wait_context_.active && !this->wait_string_context_.active) {
        flush_serial_input();
    }
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
    std::array<uint8_t, 4> encData{};
    for (int group = 0; group < 4; ++group) {
        uint8_t encoded = JUTTA_ENCODE_BASE;
        uint8_t bit0 = (decData >> (group * 2)) & 0x1;
        uint8_t bit1 = (decData >> (group * 2 + 1)) & 0x1;
        if (bit0 == 0) {
            encoded = static_cast<uint8_t>(encoded - JUTTA_BIT0_MASK);
        }
        if (bit1 == 0) {
            encoded = static_cast<uint8_t>(encoded - JUTTA_BIT1_MASK);
        }
        encData[group] = encoded;
    }
    return encData;
}

uint8_t JuttaConnection::decode(const std::array<uint8_t, 4>& encData) {
    uint8_t decData = 0;
    for (int group = 0; group < 4; ++group) {
        uint8_t encoded = encData[group];
        uint8_t bit0 = (encoded >> 2) & 0x1;
        uint8_t bit1 = (encoded >> 5) & 0x1;
        decData |= static_cast<uint8_t>(bit0 << (group * 2));
        decData |= static_cast<uint8_t>(bit1 << (group * 2 + 1));
    }
    return decData;
}

bool JuttaConnection::write_encoded_unsafe(const std::array<uint8_t, 4>& encData) const {
    ESP_LOGVV(TAG, "Writing encoded frame: %s", format_hex(encData).c_str());

    bool result = true;
    size_t index = 0;
    for (uint8_t byte : encData) {
        ESP_LOGVV(TAG, " -> Writing encoded byte %zu/%zu: 0x%02X", index + 1, encData.size(), byte);
        if (!serial.write_serial_byte(byte)) {
            ESP_LOGE(TAG, "Failed to write encoded byte %zu (0x%02X) to UART.", index, byte);
            result = false;
            break;
        }
        ESP_LOGVV(TAG, " -> Flushing UART TX buffer after encoded byte %zu", index + 1);
        serial.flush();
        ESP_LOGVV(TAG, " -> Waiting %u ms for inter-byte gap", JUTTA_SERIAL_GAP_MS);
        wait_for_jutta_gap();
        ++index;
    }

    if (result) {
        ESP_LOGVV(TAG, "Encoded frame transmitted successfully.");
    }
    return result;
}

bool JuttaConnection::read_encoded_unsafe(std::array<uint8_t, 4>& buffer) const {
    ESP_LOGVV(TAG, "Attempting to read encoded frame (buffered bytes=%zu).", this->encoded_rx_buffer_.size());
    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        wait_for_jutta_gap();
        std::array<uint8_t, 4> chunk{};
        size_t read = serial.read_serial(chunk);
        if (read > chunk.size()) {
            ESP_LOGW(TAG, "Invalid amount of UART data found (%zu byte) - ignoring.", read);
            read = chunk.size();
        }

        if (read > 0) {
            this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.end(), chunk.begin(), chunk.begin() + read);
            std::vector<uint8_t> chunk_vec(chunk.begin(), chunk.begin() + read);
            ESP_LOGVV(TAG, "Read %zu encoded byte%s from UART: %s (buffer now %zu bytes)", read,
                      read == 1 ? "" : "s", format_hex(chunk_vec).c_str(), this->encoded_rx_buffer_.size());
        } else if (this->encoded_rx_buffer_.empty()) {
            ESP_LOGV(TAG, "No serial data found.");
            return false;
        }
    }

    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        ESP_LOGVV(TAG, "Not enough encoded bytes buffered yet (size=%zu).", this->encoded_rx_buffer_.size());
        return false;
    }

    if (this->encoded_rx_buffer_.size() < buffer.size()) {
        ESP_LOGW(TAG, "Invalid amount of UART data found while forming encoded frame (%zu byte).",
                 this->encoded_rx_buffer_.size());
        return false;
    }

    std::copy_n(this->encoded_rx_buffer_.begin(), buffer.size(), buffer.begin());
    this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(),
                                   this->encoded_rx_buffer_.begin() + buffer.size());

    ESP_LOGV(TAG, "Read encoded frame: %s (buffer remaining %zu bytes)", format_hex(buffer).c_str(),
             this->encoded_rx_buffer_.size());
    return true;
}

size_t JuttaConnection::read_encoded_unsafe(std::vector<std::array<uint8_t, 4>>& data) const {
    ESP_LOGVV(TAG, "Attempting to read sequence of encoded frames.");
    size_t count = 0;
    while (true) {
        std::array<uint8_t, 4> buffer{};
        if (!read_encoded_unsafe(buffer)) {
            ESP_LOGVV(TAG, "Stopping encoded frame read loop after %zu frame%s.", count, count == 1 ? "" : "s");
            break;
        }
        data.push_back(buffer);
        ++count;
        ESP_LOGVV(TAG, "Buffered encoded frame %zu: %s", count, format_hex(buffer).c_str());
    }
    return count;
}

void JuttaConnection::flush_serial_input() const {
    ESP_LOGD(TAG, "Flushing serial input (discarding %zu buffered encoded bytes).",
             this->encoded_rx_buffer_.size());
    this->encoded_rx_buffer_.clear();
    if (!this->decoded_rx_buffer_.empty()) {
        ESP_LOGD(TAG, "Discarding %zu buffered decoded byte%s.", this->decoded_rx_buffer_.size(),
                 this->decoded_rx_buffer_.size() == 1 ? "" : "s");
        this->decoded_rx_buffer_.clear();
    }
    if (!this->response_line_buffer_.empty()) {
        ESP_LOGD(TAG, "Discarding %zu byte%s of buffered response line fragments while flushing.",
                 this->response_line_buffer_.size(),
                 this->response_line_buffer_.size() == 1 ? "" : "s");
        this->response_line_buffer_.clear();
    }
    std::array<uint8_t, 4> discard{};
    while (true) {
        size_t read = serial.read_serial(discard);
        if (read == 0) {
            break;
        }
        if (read > discard.size()) {
            ESP_LOGW(TAG, "Invalid amount of UART data found while flushing (%zu byte).", read);
        }
        std::vector<uint8_t> discard_vec(discard.begin(), discard.begin() + std::min(read, discard.size()));
        ESP_LOGVV(TAG, "Flushed %zu encoded byte%s from UART: %s", read, read == 1 ? "" : "s",
                  format_hex(discard_vec).c_str());
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
    ESP_LOGV(TAG, "Re-injected %zu decoded byte%s (encoded %zu bytes) to front of buffer: '%s' (hex %s)", data.size(),
             data.size() == 1 ? "" : "s", encoded.size(), format_printable(data).c_str(), format_hex(encoded).c_str());
}

bool JuttaConnection::poll_response_line(std::string& line) {
    if (try_extract_line(this->response_line_buffer_, line)) {
        ESP_LOGD(TAG, "Polled buffered response line: '%s'", format_printable(line).c_str());
        return true;
    }

    std::vector<uint8_t> buffer;
    if (!read_decoded_unsafe(buffer) || buffer.empty()) {
        return false;
    }

    std::string incoming = vec_to_string(buffer);
    this->response_line_buffer_.append(incoming);
    ESP_LOGD(TAG, "Received chunk while polling for response line: '%s' (hex %s) -> buffer '%s'",
             format_printable(incoming).c_str(), format_hex(buffer).c_str(),
             format_printable(this->response_line_buffer_).c_str());

    if (try_extract_line(this->response_line_buffer_, line)) {
        ESP_LOGD(TAG, "Polled response line: '%s'", format_printable(line).c_str());
        return true;
    }

    return false;
}

void JuttaConnection::reset_response_line_buffer() {
    if (!this->response_line_buffer_.empty()) {
        ESP_LOGD(TAG, "Clearing %zu byte%s of buffered response line fragments.",
                 this->response_line_buffer_.size(),
                 this->response_line_buffer_.size() == 1 ? "" : "s");
        this->response_line_buffer_.clear();
    }
}


JuttaConnection::WaitResult JuttaConnection::wait_for_ok(const std::chrono::milliseconds& timeout) {
    return wait_for_response_unsafe("ok:\r\n", timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_decoded_with_response(const std::vector<uint8_t>& data,
                                                                         const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        flush_serial_input();
        if (!write_decoded_unsafe(data)) {
            return nullptr;
        }
    }
    ESP_LOGD(TAG, "Waiting for response after writing decoded payload (timeout=%lld ms).",
             static_cast<long long>(timeout.count()));
    return wait_for_str_unsafe(timeout);
}

std::shared_ptr<std::string> JuttaConnection::write_decoded_with_response(const std::string& data,
                                                                         const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        flush_serial_input();
        if (!write_decoded_unsafe(data)) {
            return nullptr;
        }
    }
    ESP_LOGD(TAG, "Waiting for response after writing string payload (timeout=%lld ms).",
             static_cast<long long>(timeout.count()));
    return wait_for_str_unsafe(timeout);
}

std::shared_ptr<std::string> JuttaConnection::wait_for_str_unsafe(const std::chrono::milliseconds& timeout) {
    if (!this->wait_string_context_.active) {
        this->wait_string_context_.active = true;
        this->wait_string_context_.timeout = timeout;
        this->wait_string_context_.start_time = esphome::millis();
        this->wait_string_context_.buffer.clear();
        ESP_LOGD(TAG, "Waiting for any response (timeout=%lld ms).", static_cast<long long>(timeout.count()));
    }

    auto try_complete = [&]() -> std::shared_ptr<std::string> {
        auto terminator = this->wait_string_context_.buffer.find("\r\n");
        if (terminator == std::string::npos) {
            return nullptr;
        }

        std::string response = this->wait_string_context_.buffer.substr(0, terminator);
        std::string remainder = this->wait_string_context_.buffer.substr(terminator + 2);
        this->wait_string_context_.buffer.clear();

        if (!remainder.empty()) {
            for (auto it = remainder.rbegin(); it != remainder.rend(); ++it) {
                this->decoded_rx_buffer_.push_front(static_cast<uint8_t>(static_cast<unsigned char>(*it)));
            }
            ESP_LOGV(TAG, "Re-queued %zu byte%s of trailing response data for later processing.", remainder.size(),
                     remainder.size() == 1 ? "" : "s");
        }

        this->wait_string_context_.active = false;
        auto shared_response = std::make_shared<std::string>(response);
        ESP_LOGD(TAG, "Received response line: '%s'", format_printable(*shared_response).c_str());
        return shared_response;
    };

    if (auto ready = try_complete(); ready != nullptr) {
        return ready;
    }

    std::vector<uint8_t> buffer;
    if (read_decoded_unsafe(buffer) && !buffer.empty()) {
        std::string incoming = vec_to_string(buffer);
        this->wait_string_context_.buffer.append(incoming);
        ESP_LOGD(TAG, "Received chunk while waiting for response: '%s' (hex %s) -> buffer '%s'",
                 format_printable(incoming).c_str(), format_hex(buffer).c_str(),
                 format_printable(this->wait_string_context_.buffer).c_str());

        if (auto ready = try_complete(); ready != nullptr) {
            return ready;
        }
    }

    if (timeout.count() > 0) {
        uint32_t now = esphome::millis();
        uint32_t elapsed = now - this->wait_string_context_.start_time;
        if (elapsed >= static_cast<uint32_t>(timeout.count())) {
            this->wait_string_context_.active = false;
            if (!this->wait_string_context_.buffer.empty()) {
                for (auto it = this->wait_string_context_.buffer.rbegin();
                     it != this->wait_string_context_.buffer.rend(); ++it) {
                    this->decoded_rx_buffer_.push_front(static_cast<uint8_t>(static_cast<unsigned char>(*it)));
                }
                ESP_LOGV(TAG, "Timeout while waiting for generic response - re-queued %zu buffered byte%s.",
                         this->wait_string_context_.buffer.size(),
                         this->wait_string_context_.buffer.size() == 1 ? "" : "s");
                this->wait_string_context_.buffer.clear();
            }
            ESP_LOGW(TAG, "Timeout while waiting for generic response after %u ms.", elapsed);
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
        ESP_LOGD(TAG, "Waiting for response '%s' (timeout=%lld ms).", format_printable(response).c_str(),
                 static_cast<long long>(timeout.count()));
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
            ESP_LOGW(TAG, "Timeout while waiting for response '%s' after %u ms.", format_printable(response).c_str(), elapsed);
            return WaitResult::Timeout;
        }
    }

    std::vector<uint8_t> buffer;
    if (read_decoded_unsafe(buffer) && !buffer.empty()) {
        std::string incoming(buffer.begin(), buffer.end());
        this->wait_context_.recent.append(incoming);
        ESP_LOGD(TAG, "Received chunk while waiting for '%s': '%s' (hex %s) -> recent buffer '%s'",
                 format_printable(response).c_str(), format_printable(incoming).c_str(), format_hex(buffer).c_str(),
                 format_printable(this->wait_context_.recent).c_str());
        if (this->wait_context_.recent.find(response) != std::string::npos) {
            this->wait_context_.active = false;
            this->wait_context_.recent.clear();
            ESP_LOGD(TAG, "Response '%s' detected.", format_printable(response).c_str());
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
        flush_serial_input();
        if (!write_decoded_unsafe(data)) {
            return WaitResult::Error;
        }
    }
    return wait_for_response_unsafe(response, timeout);
}

JuttaConnection::WaitResult JuttaConnection::write_decoded_wait_for(const std::string& data, const std::string& response,
                                                                    const std::chrono::milliseconds& timeout) {
    if (!this->wait_context_.active || this->wait_context_.expected != response) {
        flush_serial_input();
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
