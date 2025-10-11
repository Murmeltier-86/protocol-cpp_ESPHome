#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <limits>

#include "serial_connection.hpp"
#include "codec_db_2b4b.hpp"

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------
class JuttaConnection {
 public:
    enum class WaitResult { Pending, Success, Timeout, Error };

 private:
    serial::SerialConnection serial;

 public:
    /**
     * Initializes a new Jutta (UART) connection.
     * ESPHome provides the configured UART component.
     **/
    explicit JuttaConnection(esphome::uart::UARTComponent* parent);

    /**
     * Tries to initializes the Jutta serial (UART) connection.
     * Throws a exception in case something goes wrong.
     * [Thread Safe]
     **/
    void init();

    /**
     * Tries to read a single decoded byte.
     * This requires reading 4 JUTTA bytes and converting them to a single actual data byte.
     * The result will be stored in the given "byte" pointer.
     * Returns true on success.
     * [Thread Safe]
     **/
    bool read_decoded(uint8_t* byte);
    /**
     * Reads as many data bytes, as there are availabel.
     * Each data byte consists of 4 JUTTA bytes which will be decoded into a single data byte.
     * [Thread Safe]
     **/
    bool read_decoded(std::vector<uint8_t>& data);
    /**
     * Waits until the coffee maker responded with a "ok:\r\n".
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns the current wait status.
     **/
    WaitResult wait_for_ok(const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});
    /**
     * Writes the given data to the coffee maker and then waits for the given response with an optional timeout.
     * The response has to include the "\r\n" at the end of a message.
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns true on success.
     * Returns false when a timeout occurred or writing failed.
     * [Thread Safe]
     **/
    WaitResult write_decoded_wait_for(const std::vector<uint8_t>& data, const std::string& response,
                                      const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});
    /**
     * Writes the given data to the coffee maker and then waits for the given response with an optional timeout.
     * The response has to include the "\r\n" at the end of a message.
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns true on success.
     * Returns false when a timeout occurred or writing failed.
     * [Thread Safe]
     **/
    WaitResult write_decoded_wait_for(const std::string& data, const std::string& response,
                                      const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});

    /**
     * Writes the given data to the coffee maker and then waits for any response with an optional timeout.
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns true on success.
     * Returns false when a timeout occurred or writing failed.
     * [Thread Safe]
     **/
    std::shared_ptr<std::string> write_decoded_with_response(const std::vector<uint8_t>& data,
                                                             const std::chrono::milliseconds& timeout =
                                                                 std::chrono::milliseconds{5000});
    /**
     * Writes the given data to the coffee maker and then waits for any response with an optional timeout.
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns true on success.
     * Returns false when a timeout occurred or writing failed.
     * [Thread Safe]
     **/
    std::shared_ptr<std::string> write_decoded_with_response(const std::string& data,
                                                             const std::chrono::milliseconds& timeout =
                                                                 std::chrono::milliseconds{5000});

    /**
     * Schreibt eine ASCII-Zeile im 2b4b-Format und wartet auf die Antwort.
     */
    std::shared_ptr<std::string> write_xml_with_response(const std::string& data,
                                                         const std::chrono::milliseconds& timeout =
                                                             std::chrono::milliseconds{1500});

    /**
     * Überträgt Rohdaten (z. B. XML) über den 2b4b-Kanal.
     */
    bool write_xml_payload(const std::vector<uint8_t>& data);
    bool write_xml_payload(const std::string& data);

    /**
     * Polls for the next CRLF-terminated response line.
     * Returns true if a complete line became available and stores it in "line" without the trailing CRLF.
     * Returns false when no complete line has been received yet.
     */
    bool poll_response_line(std::string& line);

    /**
     * Clears buffered fragments collected while polling for response lines.
     */
    void reset_response_line_buffer();

    /**
     * Encodes the given byte into 4 JUTTA bytes and writes them to the coffee maker.
     * [Thread Safe]
     **/
    bool write_decoded(const uint8_t& byte);
    /**
     * Encodes each byte of the given bytes into 4 JUTTA bytes and writes them to the coffee maker.
     * [Thread Safe]
     **/
    bool write_decoded(const std::vector<uint8_t>& data);
    /**
     * Encodes each character into 4 JUTTA bytes and writes them to the coffee maker.
     *
     * An example call could look like: write_decoded("TY:\r\n");
     * This would request the device type from the coffee maker.
     * [Thread Safe]
     **/
    bool write_decoded(const std::string& data);

    /**
     * Helper function used for debugging.
     * Prints the given byte in binary, hex and as a char.
     * Does not append a new line at the end!
     *
     * Example output:
     * 0 1 0 1 0 1 0 0 -> 84    54      T
     **/
    static void print_byte(const uint8_t& byte);
    /**
     * Prints each byte in the given vector in binary, hex and as a char
     *
     * Example output:
     * 0 1 0 1 0 1 0 0 -> 84    54      T
     * 0 1 0 1 1 0 0 1 -> 89    59      Y
     * 0 0 1 1 1 0 1 0 -> 58    3a      :
     * 0 0 0 0 1 1 0 1 -> 13    0d
     * 0 0 0 0 1 0 1 0 -> 10    0a
     **/
    static void print_bytes(const std::vector<uint8_t>& data);


    /**
     * Converts the given binary vector to a string and returns it.
     **/
    static std::string vec_to_string(const std::vector<uint8_t>& data);

 private:
    /**
     * Encodes the given byte into four bytes that the coffee maker understands.
     * Based on: http://protocoljura.wiki-site.com/index.php/Protocol_to_coffeemaker
     *
     * A full documentation of the process can be found here:
     * https://github.com/Jutta-Proto/protocol-cpp#deobfuscating
     **/
    static std::array<uint8_t, 4> encode(const uint8_t& decData);
    /**
     * Decodes the given four bytes read from the coffee maker into on byte.
     * Based on: http://protocoljura.wiki-site.com/index.php/Protocol_to_coffeemaker
     *
     * A full documentation of the process can be found here:
     * https://github.com/Jutta-Proto/protocol-cpp#deobfuscating
     **/
    /**
     * Writes four bytes of encoded data to the coffee maker and then waits 8ms.
     **/
    [[nodiscard]] bool write_encoded_unsafe(const std::array<uint8_t, 4>& encData) const;
    [[nodiscard]] bool write_encoded_unsafe(const std::vector<uint8_t>& encData) const;
    /**
     * Tries to read a single decoded byte.
     * This requires reading 4 JUTTA bytes and converting them to a single actual data byte.
     * The result will be stored in the given "byte" pointer.
     * Returns true on success.
     * Not thread safe!
     **/
    [[nodiscard]] bool read_decoded_unsafe(uint8_t* byte) const;
    /**
     * Reads as many data bytes, as there are availabel.
     * Each data byte consists of 4 JUTTA bytes which will be decoded into a single data byte.
     * Not thread safe!
     **/
    [[nodiscard]] bool read_decoded_unsafe(std::vector<uint8_t>& data) const;

    void flush_serial_input() const;

    /**
     * Encodes the given byte into 4 JUTTA bytes and writes them to the coffee maker.
     * Not thread safe!
     **/
    [[nodiscard]] bool write_decoded_unsafe(const uint8_t& byte) const;
    /**
     * Encodes each byte of the given bytes into 4 JUTTA bytes and writes them to the coffee maker.
     * Not thread safe!
     **/
    [[nodiscard]] bool write_decoded_unsafe(const std::vector<uint8_t>& data) const;
    /**
     * Encodes each character into 4 JUTTA bytes and writes them to the coffee maker.
     *
     * An example call could look like: write_decoded("TY:\r\n");
     * This would request the device type from the coffee maker.
     * Not thread safe!
     **/
    [[nodiscard]] bool write_decoded_unsafe(const std::string& data) const;

    /**
     * Waits until the coffee maker responded with the given response.
     * The response has to include the "\r\n" at the end of a message.
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns true on success.
     * Returns false when a timeout occurred.
     * Not thread safe!
     **/
    [[nodiscard]] WaitResult wait_for_response_unsafe(const std::string& response,
                                                      const std::chrono::milliseconds& timeout =
                                                          std::chrono::milliseconds{5000});

    /**
     * Waits for any response with an optional timeout.
     * The default timeout for this operation is 5 seconds.
     * To disable the timeout, set the timeout to 0 seconds.
     * Returns the string on success.
     * Not thread safe!
     **/
    [[nodiscard]] std::shared_ptr<std::string> wait_for_str_unsafe(
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});

    struct WaitContext {
        bool active{false};
        std::string expected{};
        std::string recent{};
        std::chrono::milliseconds timeout{std::chrono::milliseconds{5000}};
        uint32_t start_time{0};
    };

    WaitContext wait_context_{};

    struct StringWaitContext {
        bool active{false};
        std::chrono::milliseconds timeout{std::chrono::milliseconds{5000}};
        uint32_t start_time{0};
        std::string buffer{};
    };

    StringWaitContext wait_string_context_{};


    struct CodecRuntimeState {
        bool have_detection{false};
        bool msb_first{true};
        uint8_t xor_key{0x00};
    };

    struct FrameDiagnostics {
        int align{0};
        bool msb_first{true};
        uint8_t xor_key{0x00};
        float printable_ratio{0.0f};
        size_t decoded_length{0};
        int score{std::numeric_limits<int>::min()};
    };

    enum class FrameOutcome { Success, NeedMoreData, Failure };

    // Buffer of partially received encoded bytes that haven't formed a full decoded data byte yet.
    mutable std::vector<uint8_t> encoded_rx_buffer_{};

    // Buffer for decoded bytes that were read ahead of the consumer.
    mutable std::deque<uint8_t> decoded_rx_buffer_{};

    // Buffer for decoded bytes collected while looking for complete CRLF-terminated lines.
    mutable std::string response_line_buffer_{};

    mutable CodecRuntimeState codec_state_{};
    mutable int last_logged_align_{-1};
    mutable bool last_logged_msb_first_{true};
    mutable uint8_t last_logged_xor_{0x00};

    void reinject_decoded_front(const std::string& data) const;

    void reset_codec_state() const;
    bool decode_buffer(std::vector<uint8_t>& data) const;
    FrameOutcome decode_best_ascii_frame(std::vector<uint8_t>& ascii, size_t& consumed_symbols,
                                         FrameDiagnostics& diagnostics) const;
    std::vector<uint8_t> encode_stream(const std::vector<uint8_t>& data) const;
    std::vector<uint8_t> encode_stream(const std::string& data) const;

    std::shared_ptr<std::string> wait_for_xml_line(const std::chrono::milliseconds& timeout);
    bool write_xml_encoded(const std::vector<uint8_t>& data) const;
    void reset_xml_codec_state() const;

    mutable DbCodec2B4B xml_codec_{};
    mutable std::vector<uint8_t> xml_rx_buffer_{};

};
//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
