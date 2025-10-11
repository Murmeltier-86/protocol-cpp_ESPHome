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
constexpr std::array<uint8_t, 4> DEFAULT_PAIR_TO_CODEWORD = {0xDB, 0xDF, 0xFB, 0xFF};
constexpr float PRINTABLE_THRESHOLD = 0.7f;
constexpr int SCORE_INVALID = -1000;
constexpr size_t MAX_FRAME_BYTES = 1024;
constexpr std::array<uint8_t, 3> XML_XOR_KEYS = {0x00, 0xFF, 0xA5};

struct FrameScore {
    int score{SCORE_INVALID};
    float printable_ratio{0.0f};
    bool has_crlf{false};
};

FrameScore evaluate_ascii_buffer(const uint8_t* data, size_t length) {
    FrameScore result{};
    if (data == nullptr || length == 0) {
        result.score = SCORE_INVALID;
        return result;
    }

    size_t printable = 0;
    bool has_crlf = false;
    for (size_t i = 0; i < length; ++i) {
        unsigned char c = data[i];
        if (c == '\r' && i + 1 < length && data[i + 1] == '\n') {
            has_crlf = true;
        }
        if (std::isprint(c) != 0 || c == '\r' || c == '\n' || c == '\t') {
            ++printable;
        }
    }

    result.printable_ratio = static_cast<float>(printable) / static_cast<float>(length);
    result.has_crlf = has_crlf;

    int score = static_cast<int>(printable);
    if (has_crlf) {
        score += 5;
    }
    if (data[0] == '@' || data[0] == '&') {
        score += 5;
    }
    result.score = score;
    return result;
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
    this->xml_rx_buffer_.clear();
    this->reset_xml_codec_state();
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

    const auto& pair_to_codeword = DEFAULT_PAIR_TO_CODEWORD;
    bool msb_first = this->codec_state_.have_detection ? this->codec_state_.msb_first : false;
    uint8_t xor_key = this->codec_state_.have_detection ? this->codec_state_.xor_key : 0x00;

    for (uint8_t value : data) {
        uint8_t prepared = static_cast<uint8_t>(value ^ xor_key);
        for (int group = 0; group < 4; ++group) {
            size_t shift = msb_first ? static_cast<size_t>(3 - group) * 2 : static_cast<size_t>(group) * 2;
            uint8_t pair = static_cast<uint8_t>((prepared >> shift) & 0x03);
            encoded.push_back(pair_to_codeword[pair]);
        }
    }

    return encoded;
}

std::vector<uint8_t> JuttaConnection::encode_stream(const std::string& data) const {
    std::vector<uint8_t> bytes(data.begin(), data.end());
    return encode_stream(bytes);
}

void JuttaConnection::reset_xml_codec_state() const {
    this->xml_codec_.reset();
    this->xml_rx_buffer_.clear();
}

bool JuttaConnection::write_xml_encoded(const std::vector<uint8_t>& data) const {
    if (data.empty()) {
        return true;
    }
    ESP_LOGD(TAG, "Sende 2b4b-Block (%zu Byte): %s", data.size(), format_hex(data).c_str());
    if (!serial.write_serial(data)) {
        ESP_LOGE(TAG, "2b4b-Block konnte nicht über UART gesendet werden.");
        return false;
    }
    serial.flush();
    wait_for_jutta_gap();
    return true;
}

std::shared_ptr<std::string> JuttaConnection::wait_for_xml_line(const std::chrono::milliseconds& timeout) {
    uint32_t start = esphome::millis();
    while (true) {
        std::array<uint8_t, 4> chunk{};
        size_t read = serial.read_serial(chunk);
        if (read > 0) {
            this->xml_rx_buffer_.insert(this->xml_rx_buffer_.end(), chunk.begin(), chunk.begin() + read);
            DbCodec2B4B::DecodeResult result{};
            if (DbCodec2B4B::decode_best(this->xml_rx_buffer_, result) && result.success) {
                if (result.consumed > 0 && result.consumed <= this->xml_rx_buffer_.size()) {
                    this->xml_rx_buffer_.erase(this->xml_rx_buffer_.begin(), this->xml_rx_buffer_.begin() + result.consumed);
                }
                this->xml_codec_.update_detection(result.msb_first, result.xor_key);
                ESP_LOGD(TAG, "2b4b-Zeile dekodiert (msb_first=%s, xor=0x%02X, printable=%.2f)",
                         result.msb_first ? "true" : "false", result.xor_key, result.printable_ratio);
                return std::make_shared<std::string>(result.ascii);
            }
            if (this->xml_rx_buffer_.size() > 4096) {
                ESP_LOGW(TAG, "Verwerfe übergroßen 2b4b-Puffer (%zu Byte).", this->xml_rx_buffer_.size());
                this->xml_rx_buffer_.clear();
            }
        } else {
            esphome::delay(5);
        }

        if (timeout.count() > 0) {
            uint32_t now = esphome::millis();
            if (time_reached(now, start + static_cast<uint32_t>(timeout.count()))) {
                break;
            }
        }
    }
    return nullptr;
}

std::shared_ptr<std::string> JuttaConnection::write_xml_with_response(const std::string& data,
                                                                      const std::chrono::milliseconds& timeout) {
    if (data.empty()) {
        ESP_LOGW(TAG, "Leere 2b4b-Zeile wird nicht gesendet.");
        return nullptr;
    }

    this->flush_serial_input();
    this->xml_rx_buffer_.clear();

    struct Candidate {
        bool msb_first;
        uint8_t xor_key;
    };

    std::vector<Candidate> candidates;
    if (this->xml_codec_.has_detection()) {
        candidates.push_back({this->xml_codec_.msb_first(), this->xml_codec_.xor_key()});
    } else {
        for (bool msb_first : {true, false}) {
            for (uint8_t key : XML_XOR_KEYS) {
                candidates.push_back({msb_first, key});
            }
        }
    }

    for (size_t idx = 0; idx < candidates.size(); ++idx) {
        const Candidate& candidate = candidates[idx];
        auto encoded = DbCodec2B4B::encode(data, candidate.msb_first, candidate.xor_key);
        ESP_LOGD(TAG, "Sende 2b4b-Zeile (msb_first=%s, xor=0x%02X)", candidate.msb_first ? "true" : "false",
                 candidate.xor_key);
        if (!this->write_xml_encoded(encoded)) {
            return nullptr;
        }

        auto response = this->wait_for_xml_line(timeout);
        if (response != nullptr) {
            return response;
        }

        if (idx + 1 < candidates.size()) {
            ESP_LOGW(TAG, "Keine Antwort für 2b4b-Zeile, versuche alternative Kodierung.");
            this->flush_serial_input();
            this->xml_rx_buffer_.clear();
        }
    }

    return nullptr;
}

bool JuttaConnection::write_xml_payload(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        return true;
    }
    if (!this->xml_codec_.has_detection()) {
        ESP_LOGW(TAG, "2b4b-Kodierung noch nicht erkannt – Rohdaten werden nicht gesendet.");
        return false;
    }
    auto encoded = DbCodec2B4B::encode(data, this->xml_codec_.msb_first(), this->xml_codec_.xor_key());
    return this->write_xml_encoded(encoded);
}

bool JuttaConnection::write_xml_payload(const std::string& data) {
    std::vector<uint8_t> bytes(data.begin(), data.end());
    return this->write_xml_payload(bytes);
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
    this->codec_state_ = CodecRuntimeState{};
    this->codec_state_.have_detection = false;
    this->codec_state_.msb_first = false;
    this->codec_state_.xor_key = 0x00;
    this->last_logged_align_ = -1;
    this->last_logged_msb_first_ = false;
    this->last_logged_xor_ = 0x00;
}

JuttaConnection::FrameOutcome JuttaConnection::decode_best_ascii_frame(std::vector<uint8_t>& ascii,
                                                                      size_t& consumed_symbols,
                                                                      FrameDiagnostics& diagnostics) const {
    ascii.clear();
    consumed_symbols = 0;
    diagnostics = {};

    if (this->encoded_rx_buffer_.size() < 4) {
        return FrameOutcome::NeedMoreData;
    }

    const uint8_t* symbols = this->encoded_rx_buffer_.data();
    size_t symbol_count = this->encoded_rx_buffer_.size();

    auto sym2bits = [](uint8_t b) -> int {
        if (b == 0xDB) {
            return 0;
        }
        if (b == 0xDF) {
            return 1;
        }
        if (b == 0xFB) {
            return 2;
        }
        if (b == 0xFF) {
            return 3;
        }
        return -1;
    };

    struct Candidate {
        bool valid{false};
        bool ascii_complete{false};
        int score{SCORE_INVALID};
        int align{0};
        bool msb_first{true};
        uint8_t xor_key{0};
        size_t used_symbols{0};
        size_t length{0};
        float printable_ratio{0.0f};
        std::array<uint8_t, MAX_FRAME_BYTES> ascii{};
    };

    Candidate best_complete{};
    Candidate best_partial{};
    best_complete.score = SCORE_INVALID;
    best_partial.score = SCORE_INVALID;
    bool saw_invalid_symbol = false;

    const uint8_t xor_keys[] = {0x00, 0xFF, 0xA5};

    for (int align = 0; align < 4; ++align) {
        if (symbol_count <= static_cast<size_t>(align) + 3) {
            continue;
        }

        for (bool msb_first : {true, false}) {
            Candidate candidate{};
            candidate.align = align;
            candidate.msb_first = msb_first;

            std::array<uint8_t, MAX_FRAME_BYTES> raw{};
            size_t raw_len = 0;
            size_t used_symbols_local = 0;
            bool invalid = false;

            for (size_t idx = static_cast<size_t>(align); idx + 3 < symbol_count && raw_len < MAX_FRAME_BYTES;
                 idx += 4) {
                int q0 = sym2bits(symbols[idx + 0]);
                int q1 = sym2bits(symbols[idx + 1]);
                int q2 = sym2bits(symbols[idx + 2]);
                int q3 = sym2bits(symbols[idx + 3]);
                if ((q0 | q1 | q2 | q3) < 0) {
                    invalid = true;
                    break;
                }

                uint8_t decoded = 0;
                if (msb_first) {
                    decoded = static_cast<uint8_t>((q0 << 6) | (q1 << 4) | (q2 << 2) | q3);
                } else {
                    decoded = static_cast<uint8_t>((q3 << 6) | (q2 << 4) | (q1 << 2) | q0);
                }
                raw[raw_len++] = decoded;
                used_symbols_local = (idx + 4) - static_cast<size_t>(align);

                if (raw_len >= 2 && raw[raw_len - 2] == 0x0D && raw[raw_len - 1] == 0x0A) {
                    break;
                }
            }

            if (invalid) {
                saw_invalid_symbol = true;
                continue;
            }

            if (raw_len == 0) {
                continue;
            }

            candidate.length = raw_len;
            candidate.used_symbols = static_cast<size_t>(align) + used_symbols_local;
            if (candidate.used_symbols == 0) {
                candidate.used_symbols = raw_len * 4;  // fallback
            }

            std::array<uint8_t, MAX_FRAME_BYTES> tmp_ascii{};
            FrameScore best_score{};
            best_score.score = SCORE_INVALID;
            uint8_t best_key = 0x00;
            std::array<uint8_t, MAX_FRAME_BYTES> best_ascii{};

            for (uint8_t key : xor_keys) {
                for (size_t i = 0; i < raw_len; ++i) {
                    tmp_ascii[i] = static_cast<uint8_t>(raw[i] ^ key);
                }
                FrameScore score = evaluate_ascii_buffer(tmp_ascii.data(), raw_len);
                if (score.score > best_score.score) {
                    best_score = score;
                    best_key = key;
                    std::copy(tmp_ascii.begin(), tmp_ascii.begin() + raw_len, best_ascii.begin());
                }
            }

            if (best_score.score <= SCORE_INVALID) {
                continue;
            }

            candidate.valid = true;
            candidate.score = best_score.score;
            candidate.xor_key = best_key;
            candidate.printable_ratio = best_score.printable_ratio;
            candidate.ascii_complete = best_score.has_crlf;
            candidate.ascii = best_ascii;

            if (candidate.ascii_complete && candidate.printable_ratio >= PRINTABLE_THRESHOLD) {
                if (!best_complete.valid || candidate.score > best_complete.score) {
                    best_complete = candidate;
                }
            } else {
                if (!best_partial.valid || candidate.score > best_partial.score) {
                    best_partial = candidate;
                }
            }
        }
    }

    if (best_complete.valid) {
        ascii.assign(best_complete.ascii.begin(), best_complete.ascii.begin() + best_complete.length);
        consumed_symbols = best_complete.used_symbols;
        diagnostics.align = best_complete.align;
        diagnostics.msb_first = best_complete.msb_first;
        diagnostics.xor_key = best_complete.xor_key;
        diagnostics.printable_ratio = best_complete.printable_ratio;
        diagnostics.decoded_length = best_complete.length;
        diagnostics.score = best_complete.score;
        return FrameOutcome::Success;
    }

    if (best_partial.valid) {
        return FrameOutcome::NeedMoreData;
    }

    return saw_invalid_symbol ? FrameOutcome::Failure : FrameOutcome::NeedMoreData;
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
    }

    bool produced = false;

    while (!this->encoded_rx_buffer_.empty()) {
        std::vector<uint8_t> frame_ascii;
        size_t consumed = 0;
        FrameDiagnostics diag{};
        FrameOutcome outcome = this->decode_best_ascii_frame(frame_ascii, consumed, diag);

        if (outcome == FrameOutcome::Success) {
            if (consumed == 0 || consumed > this->encoded_rx_buffer_.size()) {
                this->encoded_rx_buffer_.clear();
            } else {
                this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin(),
                                               this->encoded_rx_buffer_.begin() + consumed);
            }

            if (!frame_ascii.empty()) {
                data.insert(data.end(), frame_ascii.begin(), frame_ascii.end());
                produced = true;
            }

            if (!frame_ascii.empty()) {
                this->codec_state_.have_detection = true;
                this->codec_state_.msb_first = diag.msb_first;
                this->codec_state_.xor_key = diag.xor_key;

                if (diag.align != this->last_logged_align_ || diag.msb_first != this->last_logged_msb_first_ ||
                    diag.xor_key != this->last_logged_xor_) {
                    ESP_LOGI(TAG,
                             "Auto-Decoder: align=%d, msb_first=%s, xor=0x%02X, Länge=%zu, druckbar=%.1f%%, Score=%d",
                             diag.align, diag.msb_first ? "true" : "false", diag.xor_key, diag.decoded_length,
                             diag.printable_ratio * 100.0f, diag.score);
                    this->last_logged_align_ = diag.align;
                    this->last_logged_msb_first_ = diag.msb_first;
                    this->last_logged_xor_ = diag.xor_key;
                }
            }
            continue;
        }

        if (outcome == FrameOutcome::NeedMoreData) {
            break;
        }

        if (!this->encoded_rx_buffer_.empty()) {
            uint8_t dropped = this->encoded_rx_buffer_.front();
            this->encoded_rx_buffer_.erase(this->encoded_rx_buffer_.begin());
            ESP_LOGW(TAG, "Verwerfe einzelnes Codewort 0x%02X zur Neu-Synchronisation.", dropped);
            continue;
        }

        break;
    }

    return produced;
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
