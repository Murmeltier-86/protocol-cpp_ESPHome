#include "serial_connection.hpp"

#include "esphome/core/log.h"
#include <array>

//---------------------------------------------------------------------------
namespace serial {
//---------------------------------------------------------------------------

static const char* TAG = "serial_connection";

SerialConnection::SerialConnection(esphome::uart::UARTComponent* parent) : esphome::uart::UARTDevice(parent) {}

void SerialConnection::init() {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "UART component not configured for serial connection.");
        return;
    }
    ESP_LOGI(TAG, "Serial connection handled by ESPHome UART component.");
}

size_t SerialConnection::read_serial(std::array<uint8_t, 4>& buffer) const {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "UART component not configured for serial connection.");
        return 0;
    }

    auto* self = const_cast<SerialConnection*>(this);
    size_t read = 0;
    while (read < buffer.size()) {
        if (self->available() == 0) {
            break;
        }

        if (!self->read_byte(&buffer[read])) {
            ESP_LOGW(TAG, "Failed to read UART byte while filling buffer (index=%zu).", read);
            break;
        }
        ++read;
    }

    return read;
}

bool SerialConnection::write_serial(const std::array<uint8_t, 4>& data) const {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "UART component not configured for serial connection.");
        return false;
    }
    auto* self = const_cast<SerialConnection*>(this);
    self->write_array(data.data(), data.size());
    return true;
}

size_t SerialConnection::read_serial_buffer(uint8_t* data, size_t length) const {
    if (this->parent_ == nullptr || data == nullptr || length == 0) {
        if (this->parent_ == nullptr) {
            ESP_LOGE(TAG, "UART component not configured for serial connection.");
        }
        return 0;
    }

    auto* self = const_cast<SerialConnection*>(this);
    size_t read = 0;
    while (read < length) {
        if (self->available() == 0) {
            break;
        }
        if (!self->read_byte(&data[read])) {
            ESP_LOGW(TAG, "Failed to read UART byte while filling buffer (index=%zu).", read);
            break;
        }
        ++read;
    }
    return read;
}

bool SerialConnection::read_serial_byte(uint8_t* byte) const {
    if (byte == nullptr) {
        return false;
    }
    return read_serial_buffer(byte, 1) == 1;
}

bool SerialConnection::write_serial_buffer(const uint8_t* data, size_t length) const {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "UART component not configured for serial connection.");
        return false;
    }
    if (data == nullptr || length == 0) {
        return true;
    }
    auto* self = const_cast<SerialConnection*>(this);
    self->write_array(data, length);
    return true;
}

bool SerialConnection::write_serial_byte(uint8_t byte) const {
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "UART component not configured for serial connection.");
        return false;
    }
    auto* self = const_cast<SerialConnection*>(this);
    self->write_byte(byte);
    return true;
}

void SerialConnection::flush() const {
    if (this->parent_ != nullptr) {
        ESP_LOGVV(TAG, "Flushing underlying UART component TX buffer.");
        this->parent_->flush();
    }
}

std::vector<std::string> SerialConnection::get_available_ports() {
    return {};
}
//---------------------------------------------------------------------------
}  // namespace serial
//---------------------------------------------------------------------------
