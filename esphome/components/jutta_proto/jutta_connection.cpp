#include "jutta_connection.hpp"

#include <algorithm>
#include <cmath>
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
constexpr std::array<uint8_t, 4> JUTTA_DB_CODEWORDS = {0xFF, 0xDF, 0xFB, 0xDB};
constexpr std::array<uint8_t, 4> DEFAULT_PAIR_TO_CODEWORD = {0xDB, 0xDF, 0xFB, 0xFF};
constexpr float PRINTABLE_THRESHOLD = 0.7f;
constexpr float DETECTOR_SUCCESS_THRESHOLD = 0.9f;
constexpr float DETECTOR_LOG_THRESHOLD = 0.5f;
constexpr size_t DETECTOR_MIN_SYMBOLS = 16;
constexpr size_t DETECTOR_BUFFER_LIMIT = 512;

struct PrintableStats {
    size_t printable{0};
    size_t total{0};
    bool has_crlf{false};
    float ratio{0.0f};
};

PrintableStats analyze_printable(const std::string& text) {
    PrintableStats stats{};
    stats.total = text.size();
    stats.has_crlf = text.find("\r\n") != std::string::npos;
    for (unsigned char c : text) {
        if (std::isprint(c) != 0 || c == '\r' || c == '\n' || c == '\t') {
            ++stats.printable;
        }
    }
    if (stats.total > 0) {
        stats.ratio = static_cast<float>(stats.printable) / static_cast<float>(stats.total);
    }
    return stats;
}

int codeword_index(uint8_t symbol) {
    for (size_t i = 0; i < JUTTA_DB_CODEWORDS.size(); ++i) {
        if (JUTTA_DB_CODEWORDS[i] == symbol) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

const std::vector<std::array<uint8_t, 4>>& all_symbol_mappings() {
    static const std::vector<std::array<uint8_t, 4>> mappings = [] {
        std::vector<std::array<uint8_t, 4>> result;
        std::array<uint8_t, 4> values{0, 1, 2, 3};
        do {
            result.push_back(values);
        } while (std::next_permutation(values.begin(), values.end()));
        return result;
    }();
    return mappings;
}

std::string describe_mapping(const std::array<uint8_t, 4>& mapping) {
    std::ostringstream stream;
    for (size_t i = 0; i < mapping.size(); ++i) {
        if (i > 0) {
            stream << ", ";
        }
        stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
               << static_cast<int>(JUTTA_DB_CODEWORDS[i]) << "→" << std::dec << static_cast<int>(mapping[i]);
    }
    return stream.str();
}
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

bool is_mostly_printable(const std::string& text) {
    if (text.empty()) {
        return true;
    }

    size_t printable = 0;
    for (unsigned char c : text) {
        if (std::isprint(c) != 0 || c == '\r' || c == '\n' || c == '\t') {
            ++printable;
        }
    }

    float ratio = static_cast<float>(printable) / static_cast<float>(text.size());
    return ratio >= PRINTABLE_THRESHOLD;
}

inline void wait_for_jutta_gap() {
    if (JUTTA_SERIAL_GAP_MS > 0) {
        esphome::delay(JUTTA_SERIAL_GAP_MS);
    }
}

}  // namespace

JuttaConnection::JuttaConnection(esphome::uart::UARTComponent* parent) : serial(parent) {
    this->reset_codec_state();
}

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

    std::vector<uint8_t> chunk;
    if (!this->decode_buffer(chunk) || chunk.empty()) {
        ESP_LOGVV(TAG, "Decoder not ready for single byte yet.");
        return false;
    }

    for (uint8_t value : chunk) {
        this->decoded_rx_buffer_.push_back(value);
    }

    if (this->decoded_rx_buffer_.empty()) {
        return false;
    }

    *byte = this->decoded_rx_buffer_.front();
    this->decoded_rx_buffer_.pop_front();
    ESP_LOGD(TAG, "Decoded byte: '%s' (%s)", format_printable(*byte).c_str(), format_hex(*byte).c_str());
    return true;
}

bool JuttaConnection::read_decoded_unsafe(std::vector<uint8_t>& data) const {
    ESP_LOGVV(TAG, "Attempting to read decoded bytes (encoded buffer size=%zu, decoded buffer size=%zu).",
              this->encoded_rx_buffer_.size(), this->decoded_rx_buffer_.size());

    data.clear();
    bool any_data = false;

    if (!this->decoded_rx_buffer_.empty()) {
        data.insert(data.end(), this->decoded_rx_buffer_.begin(), this->decoded_rx_buffer_.end());
        this->decoded_rx_buffer_.clear();
        any_data = !data.empty();
    }

    std::vector<uint8_t> chunk;
    if (this->decode_buffer(chunk) && !chunk.empty()) {
        data.insert(data.end(), chunk.begin(), chunk.end());
        any_data = true;
    }

    if (any_data) {
        ESP_LOGD(TAG, "Read decoded payload (%zu byte%s): '%s' (hex %s)", data.size(),
                 data.size() == 1 ? "" : "s", format_printable(data).c_str(), format_hex(data).c_str());
    }

    return any_data;
}

bool JuttaConnection::write_decoded_unsafe(const uint8_t& byte) const {
    return write_decoded_unsafe(std::vector<uint8_t>{byte});
}

bool JuttaConnection::write_decoded_unsafe(const std::vector<uint8_t>& data) const {
    if (!data.empty()) {
        ESP_LOGD(TAG, "Queueing %zu decoded byte%s for transmission: '%s' (hex %s)", data.size(),
                 data.size() == 1 ? "" : "s", format_printable(data).c_str(), format_hex(data).c_str());
    } else {
        ESP_LOGVV(TAG, "Requested to write an empty decoded payload.");
        return true;
    }

    std::vector<uint8_t> encoded = this->encode_stream(data);
    ESP_LOGVV(TAG, "Encoded block: %s", format_hex(encoded).c_str());
    return this->write_encoded_unsafe(encoded);
}

bool JuttaConnection::write_decoded_unsafe(const std::string& data) const {
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

std::vector<uint8_t> JuttaConnection::encode_stream(const std::vector<uint8_t>& data) const {
    std::vector<uint8_t> encoded;
    encoded.reserve(data.size() * 4);

    const auto& pair_to_codeword = this->codec_state_.configured ? this->codec_state_.codeword_for_pair
                                                                  : DEFAULT_PAIR_TO_CODEWORD;
    bool msb_first = this->codec_state_.configured ? this->codec_state_.msb_first : false;

    for (uint8_t value : data) {
        for (int group = 0; group < 4; ++group) {
            size_t shift = msb_first ? static_cast<size_t>(3 - group) * 2 : static_cast<size_t>(group) * 2;
            uint8_t pair = static_cast<uint8_t>((value >> shift) & 0x03);
            encoded.push_back(pair_to_codeword[pair]);
        }
    }

    return encoded;
}

std::vector<uint8_t> JuttaConnection::encode_stream(const std::string& data) const {
    std::vector<uint8_t> bytes(data.begin(), data.end());
    return encode_stream(bytes);
}

bool JuttaConnection::write_encoded_unsafe(const std::array<uint8_t, 4>& encData) const {
    std::vector<uint8_t> block(encData.begin(), encData.end());
    return this->write_encoded_unsafe(block);
}

bool JuttaConnection::write_encoded_unsafe(const std::vector<uint8_t>& encData) const {
    if (encData.empty()) {
        ESP_LOGVV(TAG, "Requested to write empty encoded block.");
        return true;
    }

    ESP_LOGVV(TAG, "Writing encoded block (%zu Byte): %s", encData.size(), format_hex(encData).c_str());
    if (!serial.write_serial(encData)) {
        ESP_LOGE(TAG, "Failed to write encoded block to UART.");
        return false;
    }

    serial.flush();
    wait_for_jutta_gap();
    ESP_LOGVV(TAG, "Encoded block transmitted successfully.");
    return true;
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

void JuttaConnection::reset_codec_state() const {
    this->codec_state_ = DbCodecState{};
    this->codec_state_.codeword_for_pair = DEFAULT_PAIR_TO_CODEWORD;
    this->codec_detection_buffer_.clear();
    this->codec_alignment_applied_ = false;
}

void JuttaConnection::collect_detection_samples(const uint8_t* data, size_t length) const {
    if (data == nullptr || length == 0 || this->codec_state_.configured) {
        return;
    }
    this->codec_detection_buffer_.insert(this->codec_detection_buffer_.end(), data, data + length);
    if (this->codec_detection_buffer_.size() > DETECTOR_BUFFER_LIMIT) {
        size_t excess = this->codec_detection_buffer_.size() - DETECTOR_BUFFER_LIMIT;
        this->codec_detection_buffer_.erase(this->codec_detection_buffer_.begin(),
                                            this->codec_detection_buffer_.begin() + excess);
    }
}

bool JuttaConnection::decode_stream_into(std::vector<uint8_t>& output, const DbCodecState& state,
                                         const uint8_t* data, size_t symbol_count) const {
    if (data == nullptr || symbol_count == 0 || symbol_count % 4 != 0) {
        return false;
    }

    output.clear();
    size_t frames = symbol_count / 4;
    output.reserve(frames);
    for (size_t frame = 0; frame < frames; ++frame) {
        uint8_t decoded = 0;
        for (size_t group = 0; group < 4; ++group) {
            uint8_t symbol = data[frame * 4 + group];
            int index = codeword_index(symbol);
            if (index < 0) {
                ESP_LOGV(TAG, "Unbekanntes Codewort 0x%02X beim Dekodieren", symbol);
                return false;
            }
            uint8_t pair = state.pair_for_codeword[index];
            size_t shift = state.msb_first ? (3 - group) * 2 : group * 2;
            decoded |= static_cast<uint8_t>((pair & 0x3) << shift);
        }
        output.push_back(decoded);
    }
    return true;
}

bool JuttaConnection::ensure_codec_configured() const {
    if (this->codec_state_.configured) {
        return true;
    }

    if (this->codec_detection_buffer_.size() < DETECTOR_MIN_SYMBOLS) {
        return false;
    }

    struct Candidate {
        bool valid{false};
        float ratio{0.0f};
        size_t decoded_bytes{0};
        bool msb_first{false};
        uint8_t align{0};
        std::array<uint8_t, 4> mapping{{0, 1, 2, 3}};
        std::string preview;
    };

    Candidate best{};
    const auto& mappings = all_symbol_mappings();
    const uint8_t* symbols = this->codec_detection_buffer_.data();
    size_t symbol_count = this->codec_detection_buffer_.size();

    for (uint8_t align = 0; align < 4; ++align) {
        if (symbol_count <= align) {
            continue;
        }
        size_t available = symbol_count - align;
        size_t frames = available / 4;
        if (frames < 2) {
            continue;
        }
        size_t used_symbols = frames * 4;
        const uint8_t* start = symbols + align;

        for (const auto& mapping : mappings) {
            DbCodecState candidate_state{};
            candidate_state.pair_for_codeword = mapping;

            for (bool msb_first : {false, true}) {
                candidate_state.msb_first = msb_first;
                std::vector<uint8_t> decoded;
                if (!decode_stream_into(decoded, candidate_state, start, used_symbols)) {
                    continue;
                }
                std::string decoded_text(decoded.begin(), decoded.end());
                auto stats = analyze_printable(decoded_text);

                if (!best.valid || stats.ratio > best.ratio + 1e-3f ||
                    (std::abs(stats.ratio - best.ratio) < 1e-3f && decoded.size() > best.decoded_bytes)) {
                    best.valid = true;
                    best.ratio = stats.ratio;
                    best.decoded_bytes = decoded.size();
                    best.msb_first = msb_first;
                    best.align = align;
                    best.mapping = mapping;
                    best.preview = decoded_text;
                }
            }
        }
    }

    if (!best.valid) {
        return false;
    }

    std::string mapping_desc = describe_mapping(best.mapping);
    if (best.ratio < DETECTOR_SUCCESS_THRESHOLD) {
        bool should_log = !this->codec_last_candidate_.valid ||
                          std::abs(best.ratio - this->codec_last_candidate_.ratio) > 0.01f ||
                          best.msb_first != this->codec_last_candidate_.msb_first ||
                          best.align != this->codec_last_candidate_.align ||
                          best.mapping != this->codec_last_candidate_.mapping;
        if (should_log) {
            ESP_LOGD(TAG,
                     "Auto-Detektor Kandidat: mapping=%s, msb_first=%s, align=%u, druckbar=%.1f%% (%zu Byte)",
                     mapping_desc.c_str(), best.msb_first ? "true" : "false", best.align, best.ratio * 100.0f,
                     best.decoded_bytes);
            this->codec_last_candidate_.valid = true;
            this->codec_last_candidate_.ratio = best.ratio;
            this->codec_last_candidate_.msb_first = best.msb_first;
            this->codec_last_candidate_.align = best.align;
            this->codec_last_candidate_.mapping = best.mapping;
        }
        return false;
    }

    DbCodecState state{};
    state.configured = true;
    state.msb_first = best.msb_first;
    state.align = best.align;
    state.pair_for_codeword = best.mapping;
    state.codeword_for_pair = DEFAULT_PAIR_TO_CODEWORD;
    for (size_t i = 0; i < best.mapping.size(); ++i) {
        uint8_t pair = best.mapping[i];
        if (pair < state.codeword_for_pair.size()) {
            state.codeword_for_pair[pair] = JUTTA_DB_CODEWORDS[i];
        }
    }

    this->codec_state_ = state;
    this->codec_alignment_applied_ = false;
    this->codec_detection_buffer_.clear();
    this->codec_last_candidate_.valid = false;

    ESP_LOGI(TAG,
             "Auto-Detektor aktiviert: mapping=%s, msb_first=%s, align=%u, druckbar=%.1f%%, Beispiel='%s'",
             mapping_desc.c_str(), state.msb_first ? "true" : "false", state.align, best.ratio * 100.0f,
             format_printable(best.preview).c_str());
    return true;
}

bool JuttaConnection::decode_buffer(std::vector<uint8_t>& data) const {
    data.clear();

    while (true) {
        std::array<uint8_t, 4> chunk{};
        size_t read = serial.read_serial(chunk);
        if (read == 0) {
            break;
        }
        this->encoded_rx_buffer_.insert(this->encoded_rx_buffer_.end(), chunk.begin(), chunk.begin() + read);
        collect_detection_samples(chunk.data(), read);
    }

    if (!ensure_codec_configured()) {
        return false;
    }

    if (!this->codec_alignment_applied_ && this->codec_state_.align > 0) {
        size_t drop = std::min<size_t>(this->codec_state_.align, this->encoded_rx_buffer_.size());
        this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(), this->encoded_rx_buffer_.begin() + drop);
        this->codec_alignment_applied_ = true;
    }

    size_t usable_symbols = (this->encoded_rx_buffer_.size() / 4) * 4;
    if (usable_symbols == 0) {
        return false;
    }

    if (!decode_stream_into(data, this->codec_state_, this->encoded_rx_buffer_.data(), usable_symbols)) {
        ESP_LOGW(TAG, "Dekodierung von %zu Symbolen mit aktueller Zuordnung fehlgeschlagen.", usable_symbols);
        size_t drop = std::min<size_t>(usable_symbols, static_cast<size_t>(4));
        this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(), this->encoded_rx_buffer_.begin() + drop);
        return false;
    }

    this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(),
                                   this->encoded_rx_buffer_.begin() + usable_symbols);
    return !data.empty();
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

    this->reset_codec_state();

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

    std::vector<uint8_t> encoded = this->encode_stream(data);

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
    if (!is_mostly_printable(incoming)) {
        ESP_LOGW(TAG,
                 "Verwerfe Binärblock beim Zeilen-Polling (%zu Byte, <70%% druckbare Zeichen): hex %s",
                 incoming.size(), format_hex(buffer).c_str());
        return false;
    }
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
        if (!is_mostly_printable(incoming)) {
            ESP_LOGW(TAG,
                     "Verwerfe Binärblock während generischer Antwort (%zu Byte, <70%% druckbar): hex %s",
                     incoming.size(), format_hex(buffer).c_str());
        } else {
            this->wait_string_context_.buffer.append(incoming);
            ESP_LOGD(TAG, "Received chunk while waiting for response: '%s' (hex %s) -> buffer '%s'",
                     format_printable(incoming).c_str(), format_hex(buffer).c_str(),
                     format_printable(this->wait_string_context_.buffer).c_str());

            if (auto ready = try_complete(); ready != nullptr) {
                return ready;
            }
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
        if (!is_mostly_printable(incoming)) {
            ESP_LOGW(TAG,
                     "Verwerfe Binärblock beim Warten auf '%s' (%zu Byte, <70%% druckbar): hex %s",
                     format_printable(response).c_str(), incoming.size(), format_hex(buffer).c_str());
        } else {
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
