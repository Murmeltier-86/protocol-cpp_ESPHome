#include "jutta_connection.hpp"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include "esphome/core/helpers.h"
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
constexpr uint8_t DB_SYMBOLS[4] = {0xFF, 0xDF, 0xFB, 0xDB};
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

inline void wait_for_jutta_gap() {
    const uint32_t start = esphome::millis();
    while (esphome::millis() - start < JUTTA_SERIAL_GAP_MS) {
        // Busy-wait to preserve the required 8 ms spacing between JUTTA bytes.
    }
}

inline bool is_possible_encoded_byte(uint8_t byte) {
    switch (byte) {
        case JUTTA_ENCODE_BASE:
        case static_cast<uint8_t>(JUTTA_ENCODE_BASE - JUTTA_BIT0_MASK):
        case static_cast<uint8_t>(JUTTA_ENCODE_BASE - JUTTA_BIT1_MASK):
        case static_cast<uint8_t>(JUTTA_ENCODE_BASE - JUTTA_BIT0_MASK - JUTTA_BIT1_MASK):
            return true;
        default:
            return false;
    }
}

inline bool frames_equivalent(const std::array<uint8_t, 4>& lhs, const std::array<uint8_t, 4>& rhs) {
    for (size_t i = 0; i < lhs.size(); ++i) {
        if (lhs[i] != rhs[i]) {
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
    std::array<uint8_t, 4> encData{};
    for (int group = 0; group < 4; ++group) {
        uint8_t encoded = JUTTA_ENCODE_BASE;
        uint8_t bit0 = static_cast<uint8_t>((decData >> (group * 2)) & 0x1);
        uint8_t bit1 = static_cast<uint8_t>((decData >> (group * 2 + 1)) & 0x1);
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
        uint8_t bit0 = static_cast<uint8_t>((encoded >> 2) & 0x1);
        uint8_t bit1 = static_cast<uint8_t>((encoded >> 5) & 0x1);
        decData |= static_cast<uint8_t>(bit0 << (group * 2));
        decData |= static_cast<uint8_t>(bit1 << (group * 2 + 1));
    }
    return decData;
}

bool JuttaConnection::write_encoded_unsafe(const std::array<uint8_t, 4>& encData) const {
    ESP_LOGVV(TAG, "Writing encoded frame: %s", format_hex(encData).c_str());

    if (!serial.write_serial(encData)) {
        ESP_LOGE(TAG, "Failed to write encoded frame to UART.");
        return false;
    }

    ESP_LOGVV(TAG, " -> Flushing UART TX buffer after encoded frame");
    serial.flush();
    ESP_LOGVV(TAG, " -> Waiting %u ms for inter-frame gap", JUTTA_SERIAL_GAP_MS);
    wait_for_jutta_gap();
    ESP_LOGVV(TAG, "Encoded frame transmitted successfully.");
    return true;
}

bool JuttaConnection::read_encoded_unsafe(std::array<uint8_t, 4>& buffer) const {
    ESP_LOGVV(TAG, "Attempting to read encoded frame (buffered bytes=%zu).", this->encoded_rx_buffer_.size());
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
                std::vector<uint8_t> chunk_vec(chunk.begin(), chunk.begin() + size);
                ESP_LOGVV(TAG, "Read %zu encoded byte%s from UART: %s (buffer now %zu bytes)", size,
                          size == 1 ? "" : "s", format_hex(chunk_vec).c_str(), this->encoded_rx_buffer_.size());
            } else if (this->encoded_rx_buffer_.empty()) {
                ESP_LOGV(TAG, "No serial data found.");
                return false;
            }
        }

        if (!align_encoded_rx_buffer()) {
            ESP_LOGVV(TAG, "Encoded buffer could not be aligned after UART read (size=%zu).",
                      this->encoded_rx_buffer_.size());
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
        std::vector<uint8_t> chunk_vec(chunk.begin(), chunk.begin() + read);
        ESP_LOGVV(TAG, "Buffered additional %zu encoded byte%s: %s (buffer now %zu bytes)", read,
                  read == 1 ? "" : "s", format_hex(chunk_vec).c_str(), this->encoded_rx_buffer_.size());
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

bool JuttaConnection::align_encoded_rx_buffer() const {
    ESP_LOGVV(TAG, "Aligning encoded RX buffer (current size=%zu).", this->encoded_rx_buffer_.size());
    size_t skipped = 0;

    auto log_skipped = [&]() {
        if (skipped > 0) {
            ESP_LOGW(TAG, "Discarded %zu stray encoded byte%s while seeking JUTTA frame boundary.", skipped,
                     skipped == 1 ? "" : "s");
            skipped = 0;
        }
    };

    while (true) {
        while (!this->encoded_rx_buffer_.empty() && !is_possible_encoded_byte(this->encoded_rx_buffer_.front())) {
            this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin());
            ++skipped;
        }

        if (this->encoded_rx_buffer_.size() < 4) {
            log_skipped();
            ESP_LOGVV(TAG, "Insufficient encoded bytes to form a frame during alignment (size=%zu).",
                      this->encoded_rx_buffer_.size());
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
            ESP_LOGVV(TAG, "Discarded byte while scanning for frame boundary - candidate contained invalid values.");
            continue;
        }

        uint8_t decoded = decode(candidate);
        auto reencoded = encode(decoded);
        if (frames_equivalent(candidate, reencoded)) {
            log_skipped();
            ESP_LOGVV(TAG, "Found aligned encoded frame candidate: %s -> '%s' (%s)", format_hex(candidate).c_str(),
                      format_printable(decoded).c_str(), format_hex(decoded).c_str());
            return true;
        }

        this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin());
        ++skipped;
        ESP_LOGVV(TAG, "Discarded leading byte after mismatch with re-encoded candidate.");
    }
}

void JuttaConnection::flush_serial_input() const {
    ESP_LOGD(TAG, "Flushing serial input (discarding %zu buffered encoded bytes).",
             this->encoded_rx_buffer_.size());
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


std::shared_ptr<std::string> JuttaConnection::write_db_with_response(
    const std::string& data, const std::chrono::milliseconds& timeout) {
    this->flush_serial_input();
    this->encoded_rx_buffer_.clear();
    this->decoded_rx_buffer_.clear();

    if (!this->write_db_ascii(data)) {
        ESP_LOGW(TAG, "DB: Schreiben der ASCII-Zeile fehlgeschlagen.");
        return nullptr;
    }

    auto line = this->wait_for_db_line(timeout);
    if (line == nullptr) {
        ESP_LOGW(TAG, "DB: Keine Antwort innerhalb des Zeitlimits erhalten.");
    }
    return line;
}

bool JuttaConnection::write_db_ascii(const std::string& ascii) const {
    if (ascii.empty()) {
        return false;
    }
    std::vector<uint8_t> payload(ascii.begin(), ascii.end());
    std::vector<uint8_t> encoded;
    auto mode = this->db_mode_;
    if (mode == DbMode::Auto || mode == DbMode::None) {
        mode = DbMode::FourToOne;
    }
    db_encode(mode, payload, encoded);
    bool ok = serial.write_serial(encoded);
    serial.flush();
    return ok;
}

std::shared_ptr<std::string> JuttaConnection::wait_for_db_line(const std::chrono::milliseconds& timeout) const {
    std::vector<uint8_t> raw_buffer;
    std::vector<uint8_t> decoded;
    raw_buffer.reserve(256);
    decoded.reserve(256);

    auto mode = this->db_mode_;
    uint32_t start = esphome::millis();

    while (true) {
        std::array<uint8_t, 4> chunk{};
        size_t read = serial.read_serial(chunk);
        if (read > chunk.size()) {
            read = chunk.size();
        }
        if (read > 0) {
            raw_buffer.insert(raw_buffer.end(), chunk.begin(), chunk.begin() + read);
        } else {
            esphome::delay(1);
        }

        if (!raw_buffer.empty()) {
            auto temp_mode = mode;
            size_t decoded_len = db_decode(temp_mode, raw_buffer, decoded);
            if (decoded_len > 0) {
                for (size_t i = 1; i < decoded.size(); ++i) {
                    if (decoded[i - 1] == '\r' && decoded[i] == '\n') {
                        std::string line(decoded.begin(), decoded.begin() + i + 1);
                        this->db_mode_ = temp_mode;
                        return std::make_shared<std::string>(std::move(line));
                    }
                }
            }
        }

        if (timeout.count() == 0) {
            continue;
        }
        uint32_t now = esphome::millis();
        if (static_cast<int32_t>(now - start) >= static_cast<int32_t>(timeout.count())) {
            break;
        }
    }

    return nullptr;
}

size_t JuttaConnection::db_encode(DbMode mode, const std::vector<uint8_t>& input, std::vector<uint8_t>& output) {
    output.clear();

    auto encode_none = [&]() {
        output.insert(output.end(), input.begin(), input.end());
        return output.size();
    };

    auto encode_esc = [&]() {
        for (uint8_t byte : input) {
            if (byte == 0xDB) {
                output.push_back(0xDB);
                output.push_back(0xDC);
            } else if (byte == 0x00) {
                output.push_back(0xDB);
                output.push_back(0xDD);
            } else {
                output.push_back(byte);
            }
        }
        return output.size();
    };

    auto encode_pack4 = [&]() {
        output.push_back(0xDB);
        for (uint8_t value : input) {
            uint8_t q0 = static_cast<uint8_t>((value >> 6) & 0x03);
            uint8_t q1 = static_cast<uint8_t>((value >> 4) & 0x03);
            uint8_t q2 = static_cast<uint8_t>((value >> 2) & 0x03);
            uint8_t q3 = static_cast<uint8_t>(value & 0x03);
            output.push_back(DB_SYMBOLS[q0]);
            output.push_back(DB_SYMBOLS[q1]);
            output.push_back(DB_SYMBOLS[q2]);
            output.push_back(DB_SYMBOLS[q3]);
        }
        return output.size();
    };

    switch (mode) {
        case DbMode::Esc:
            return encode_esc();
        case DbMode::FourToOne:
        case DbMode::Auto:
            return encode_pack4();
        case DbMode::None:
        default:
            return encode_none();
    }
}

size_t JuttaConnection::db_decode(DbMode& mode, const std::vector<uint8_t>& input, std::vector<uint8_t>& output) {
    output.clear();
    if (input.empty()) {
        return 0;
    }

    auto guess_mode = [](const std::vector<uint8_t>& data) {
        if (data.size() >= 2 && data.front() == 0xDB) {
            size_t small = 0;
            for (size_t i = 1; i < data.size(); ++i) {
                if ((data[i] & 0xFC) == 0) {
                    ++small;
                }
            }
            if (small > data.size() / 2) {
                return DbMode::FourToOne;
            }
            return DbMode::Esc;
        }
        return DbMode::None;
    };

    auto decode_esc = [&](const std::vector<uint8_t>& data) {
        for (size_t i = 0; i < data.size(); ++i) {
            uint8_t byte = data[i];
            if (byte == 0xDB && i + 1 < data.size()) {
                uint8_t esc = data[++i];
                if (esc == 0xDC) {
                    output.push_back(0xDB);
                } else if (esc == 0xDD) {
                    output.push_back(0x00);
                } else {
                    output.push_back(esc);
                }
            } else {
                output.push_back(byte);
            }
        }
        return output.size();
    };

    auto symbol_to_bits = [](uint8_t symbol) -> int {
        for (int idx = 0; idx < 4; ++idx) {
            if (symbol == DB_SYMBOLS[idx]) {
                return idx;
            }
        }
        return -1;
    };

    auto decode_pack4 = [&](const std::vector<uint8_t>& data) {
        if (data.size() < 2 || data.front() != 0xDB) {
            return static_cast<size_t>(0);
        }
        size_t index = 1;
        while (index + 3 < data.size()) {
            int a = symbol_to_bits(data[index + 0]);
            int b = symbol_to_bits(data[index + 1]);
            int c = symbol_to_bits(data[index + 2]);
            int d = symbol_to_bits(data[index + 3]);
            if ((a | b | c | d) < 0) {
                break;
            }
            uint8_t value = static_cast<uint8_t>((a << 6) | (b << 4) | (c << 2) | d);
            output.push_back(value);
            index += 4;
        }
        return output.size();
    };

    DbMode effective = mode;
    if (effective == DbMode::Auto) {
        effective = guess_mode(input);
    }

    size_t produced = 0;
    switch (effective) {
        case DbMode::Esc:
            produced = decode_esc(input);
            break;
        case DbMode::FourToOne:
            produced = decode_pack4(input);
            break;
        case DbMode::None:
        case DbMode::Auto:
        default:
            output.insert(output.end(), input.begin(), input.end());
            produced = output.size();
            break;
    }

    mode = effective;
    return produced;
}

}  // namespace jutta_proto
//---------------------------------------------------------------------------
