#pragma once

#include <array>
#include <string>
#include <vector>

#include "esphome/components/uart/uart.h"

//---------------------------------------------------------------------------
namespace serial {
//---------------------------------------------------------------------------
class SerialConnection : public esphome::uart::UARTDevice {
 public:
    explicit SerialConnection(esphome::uart::UARTComponent* parent);

    /**
     * Initializes the serial (UART) connection.
     * ESPHome handles the low level initialisation.
     **/
    void init();

    /**
     * Reads at maximum four bytes.
     * Returns how many bytes have been actually read.
     **/
    [[nodiscard]] size_t read_serial(std::array<uint8_t, 4>& buffer) const;

    /**
     * Reads up to "length" bytes into the provided buffer without blocking.
     * Returns how many bytes have been read.
     */
    [[nodiscard]] size_t read_serial_buffer(uint8_t* data, size_t length) const;

    [[nodiscard]] size_t read_serial_buffer(std::vector<uint8_t>& data) const {
        return read_serial_buffer(data.data(), data.size());
    }

    /**
     * Reads a single byte from the UART if available.
     * Returns true when a byte has been read.
     */
    [[nodiscard]] bool read_serial_byte(uint8_t* byte) const;
    /**
     * Writes the given data buffer to the serial connection.
     * Returns true on success.
     **/
    [[nodiscard]] bool write_serial(const std::array<uint8_t, 4>& data) const;
    /**
     * Writes the given raw buffer to the serial connection.
     * Returns true on success.
     **/
    [[nodiscard]] bool write_serial_buffer(const uint8_t* data, size_t length) const;
    [[nodiscard]] bool write_serial_buffer(const std::vector<uint8_t>& data) const {
        return write_serial_buffer(data.data(), data.size());
    }
    /**
     * Writes a single byte to the serial connection.
     * Returns true on success.
     **/
    [[nodiscard]] bool write_serial_byte(uint8_t byte) const;
    void flush() const;

    esphome::uart::UARTComponent* get_parent() const;

    /**
     * Returns all available serial port paths for this device.
     **/
    static std::vector<std::string> get_available_ports();
};
//---------------------------------------------------------------------------
}  // namespace serial
//---------------------------------------------------------------------------
