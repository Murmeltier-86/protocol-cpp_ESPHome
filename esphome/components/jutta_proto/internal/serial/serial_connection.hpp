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
     * Writes the given data buffer to the serial connection.
     * Returns true on success.
     **/
    [[nodiscard]] bool write_serial(const std::array<uint8_t, 4>& data) const;
    void flush() const;

    /**
     * Returns all available serial port paths for this device.
     **/
    static std::vector<std::string> get_available_ports();
};
//---------------------------------------------------------------------------
}  // namespace serial
//---------------------------------------------------------------------------
