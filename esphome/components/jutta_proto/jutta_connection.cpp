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
    this->xml_pending_symbols_.clear();
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

std::vector<std::pair<bool, uint8_t>> JuttaConnection::build_codec_candidates() const {
    std::vector<std::pair<bool, uint8_t>> candidates;
    if (this->xml_codec_state_.valid) {
        candidates.emplace_back(this->xml_codec_state_.msb_first, this->xml_codec_state_.xor_key);
    }
    std::array<uint8_t, 3> keys{0x00, 0xFF, 0xA5};
    for (bool msb_first : {true, false}) {
        for (uint8_t key : keys) {
            if (this->xml_codec_state_.valid && msb_first == this->xml_codec_state_.msb_first &&
                key == this->xml_codec_state_.xor_key) {
                continue;
            }
            candidates.emplace_back(msb_first, key);
        }
    }
    return candidates;
}

void JuttaConnection::log_xml_attempt_failure(const char* context, const std::string& summary) const {
    ESP_LOGW(TAG, "%s: %s", context, summary.c_str());
}

bool JuttaConnection::write_xml_line(const std::string& ascii, bool msb_first, uint8_t xor_key) const {
    if (ascii.empty()) {
        return false;
    }
    std::vector<uint8_t> encoded;
    encoded.reserve(ascii.size() * 4);
    for (unsigned char ch : ascii) {
        uint8_t value = static_cast<uint8_t>(ch) ^ xor_key;
        uint8_t groups[4];
        if (msb_first) {
            groups[0] = static_cast<uint8_t>((value >> 6) & 0x03);
            groups[1] = static_cast<uint8_t>((value >> 4) & 0x03);
            groups[2] = static_cast<uint8_t>((value >> 2) & 0x03);
            groups[3] = static_cast<uint8_t>(value & 0x03);
        } else {
            groups[3] = static_cast<uint8_t>((value >> 6) & 0x03);
            groups[2] = static_cast<uint8_t>((value >> 4) & 0x03);
            groups[1] = static_cast<uint8_t>((value >> 2) & 0x03);
            groups[0] = static_cast<uint8_t>(value & 0x03);
        }
        for (uint8_t group : groups) {
            encoded.push_back(DB_SYMBOLS[group & 0x03]);
        }
    }
    bool ok = serial.write_serial(encoded);
    serial.flush();
    return ok;
}

JuttaConnection::XmlDecodeSummary JuttaConnection::decode_symbols(const std::vector<uint8_t>& symbols) const {
    XmlDecodeSummary best;
    if (symbols.size() < 4) {
        return best;
    }

    auto sym2bits = [](uint8_t symbol) -> int {
        for (size_t i = 0; i < 4; ++i) {
            if (symbol == DB_SYMBOLS[i]) {
                return static_cast<int>(i);
            }
        }
        return -1;
    };

    auto decode_try = [&](bool msb_first, int align, std::vector<uint8_t>& out, size_t& consumed) -> bool {
        out.clear();
        consumed = align;
        if (align >= static_cast<int>(symbols.size())) {
            return false;
        }
        size_t index = static_cast<size_t>(align);
        while (index + 3 < symbols.size()) {
            int q0 = sym2bits(symbols[index + 0]);
            int q1 = sym2bits(symbols[index + 1]);
            int q2 = sym2bits(symbols[index + 2]);
            int q3 = sym2bits(symbols[index + 3]);
            if ((q0 | q1 | q2 | q3) < 0) {
                return false;
            }
            uint8_t value = msb_first ? static_cast<uint8_t>((q0 << 6) | (q1 << 4) | (q2 << 2) | q3)
                                      : static_cast<uint8_t>((q3 << 6) | (q2 << 4) | (q1 << 2) | q0);
            out.push_back(value);
            index += 4;
            if (out.size() >= 2 && out[out.size() - 2] == 0x0D && out[out.size() - 1] == 0x0A) {
                consumed = index;
                return true;
            }
        }
        consumed = index;
        return false;
    };

    auto score_bytes = [](const std::vector<uint8_t>& bytes) -> int {
        if (bytes.empty()) {
            return -1;
        }
        int printable = 0;
        int crlf_bonus = 0;
        int head_bonus = 0;
        for (size_t i = 0; i < bytes.size(); ++i) {
            uint8_t c = bytes[i];
            if (c >= 0x20 && c <= 0x7E) {
                ++printable;
            }
            if (c == '\r' && i + 1 < bytes.size() && bytes[i + 1] == '\n') {
                crlf_bonus = 5;
            }
        }
        if (!bytes.empty() && (bytes[0] == '@' || bytes[0] == '&')) {
            head_bonus = 5;
        }
        return printable + crlf_bonus + head_bonus;
    };

    std::array<uint8_t, 3> keys{0x00, 0xFF, 0xA5};
    for (bool msb_first : {true, false}) {
        for (int align = 0; align < 4; ++align) {
            std::vector<uint8_t> decoded;
            size_t consumed = 0;
            if (!decode_try(msb_first, align, decoded, consumed)) {
                continue;
            }
            for (uint8_t key : keys) {
                std::vector<uint8_t> candidate(decoded.size());
                for (size_t i = 0; i < decoded.size(); ++i) {
                    candidate[i] = decoded[i] ^ key;
                }
                int score = score_bytes(candidate);
                if (score > best.score) {
                    best.score = score;
                    best.msb_first = msb_first;
                    best.xor_key = key;
                    best.align = align;
                    best.symbols_consumed = consumed;
                    best.line.assign(candidate.begin(), candidate.end());
                }
            }
        }
    }

    if (best.line.empty()) {
        best.symbols_consumed = 0;
        return best;
    }
    bool has_crlf = best.line.size() >= 2 && best.line[best.line.size() - 2] == '\r' && best.line.back() == '\n';
    if (!has_crlf) {
        best.line.clear();
        best.symbols_consumed = 0;
        return best;
    }
    int printable = 0;
    for (unsigned char c : best.line) {
        if (c >= 0x20 && c <= 0x7E) {
            ++printable;
        }
    }
    float ratio = static_cast<float>(printable) / static_cast<float>(best.line.size());
    if (ratio < 0.7f) {
        best.line.clear();
        best.symbols_consumed = 0;
        return best;
    }
    if (best.line.front() != '@' && best.line.front() != '&') {
        best.line.clear();
        best.symbols_consumed = 0;
        return best;
    }
    if (best.line.size() >= 2) {
        best.line.erase(best.line.size() - 2);
    }
    best.success = true;
    return best;
}

JuttaConnection::XmlDecodeSummary JuttaConnection::wait_for_xml_line(const std::chrono::milliseconds& timeout) const {
    XmlDecodeSummary summary;
    std::vector<uint8_t> buffer;
    if (!this->xml_pending_symbols_.empty()) {
        buffer = this->xml_pending_symbols_;
        this->xml_pending_symbols_.clear();
    }
    uint32_t start = esphome::millis();
    while (true) {
        if (!buffer.empty()) {
            auto decoded = this->decode_symbols(buffer);
            if (decoded.success) {
                if (decoded.symbols_consumed < buffer.size()) {
                    this->xml_pending_symbols_.assign(buffer.begin() + decoded.symbols_consumed, buffer.end());
                }
                return decoded;
            }
        }

        std::array<uint8_t, 4> chunk{};
        size_t read = serial.read_serial(chunk);
        if (read > chunk.size()) {
            read = chunk.size();
        }
        if (read > 0) {
            buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + read);
            continue;
        }

        if (timeout.count() == 0) {
            break;
        }
        uint32_t now = esphome::millis();
        if (static_cast<int32_t>(now - start) >= static_cast<int32_t>(timeout.count())) {
            break;
        }
    }

    if (!buffer.empty()) {
        this->xml_pending_symbols_.insert(this->xml_pending_symbols_.end(), buffer.begin(), buffer.end());
    }
    return summary;
}

JuttaConnection::XmlOperationResult JuttaConnection::request_xml_lines(const std::string& command,
                                                                       const std::chrono::milliseconds& timeout,
                                                                       std::vector<std::string>& lines) {
    XmlOperationResult result;
    lines.clear();
    if (command.empty()) {
        result.summary = "Leerer Befehl";
        return result;
    }

    this->flush_serial_input();

    auto candidates = this->build_codec_candidates();
    if (candidates.empty()) {
        candidates.emplace_back(true, static_cast<uint8_t>(0x00));
    }

    for (auto [msb_first, xor_key] : candidates) {
        ESP_LOGD(TAG, "Sende 2b4b-Zeile (msb_first=%s, xor=0x%02X)", msb_first ? "true" : "false",
                 static_cast<unsigned>(xor_key));
        if (!this->write_xml_line(command, msb_first, xor_key)) {
            this->log_xml_attempt_failure("XML", "Senden der 2b4b-Zeile fehlgeschlagen");
            continue;
        }
        auto response = this->wait_for_xml_line(timeout);
        if (!response.success) {
            this->log_xml_attempt_failure("XML", "Keine Antwort für 2b4b-Zeile, versuche alternative Kodierung.");
            continue;
        }

        this->xml_codec_state_.valid = true;
        this->xml_codec_state_.msb_first = response.msb_first;
        this->xml_codec_state_.xor_key = response.xor_key;
        this->xml_codec_state_.align = response.align;

        lines.push_back(response.line);
        while (true) {
            auto next = this->wait_for_xml_line(std::chrono::milliseconds{150});
            if (!next.success) {
                break;
            }
            lines.push_back(next.line);
        }

        std::ostringstream info;
        info << "msb_first=" << (response.msb_first ? "true" : "false") << ", xor=0x" << std::uppercase
             << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(response.xor_key);
        result.success = true;
        result.summary = info.str();
        return result;
    }

    result.success = false;
    result.summary = "Keine Antwort für 2b4b-Zeile";
    return result;
}

JuttaConnection::XmlOperationResult JuttaConnection::start_xml_handshake() {
    static const std::array<const char*, 5> PROBES = {"&WHO\r\n", "@TR:37\r\n", "@TR:32\r\n", "@t2:8188\r\n", "@TS:00\r\n"};
    XmlOperationResult result;
    this->flush_serial_input();
    for (const char* probe : PROBES) {
        std::vector<std::string> responses;
        auto op = this->request_xml_lines(probe, std::chrono::milliseconds{800}, responses);
        if (op.success) {
            result.success = true;
            if (!responses.empty()) {
                result.summary = std::string(probe, strlen(probe) - 2) + " -> " + responses.front();
            } else {
                result.summary = std::string(probe, strlen(probe) - 2) + " -> ok";
            }
            return result;
        }
    }
    result.success = false;
    result.summary = "Keine Antwort auf Handshake-Kommandos";
    return result;
}

//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
