#include "serial/SerialConnection.hpp"

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
    return self->read_array(buffer.data(), buffer.size());
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

void SerialConnection::flush() const {
    if (this->parent_ != nullptr) {
        this->parent_->flush();
    }
}

std::vector<std::string> SerialConnection::get_available_ports() {
    return {};
}
//---------------------------------------------------------------------------
}  // namespace serial
//---------------------------------------------------------------------------
