// external_components/jura/jura.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace esphome {
namespace jura {

class Jura : public PollingComponent, public uart::UARTDevice {
 public:
  explicit Jura(uart::UARTComponent *parent = nullptr);

  // Lifecycle
  void setup() override;
  void dump_config() override;
  void update() override;
  uint32_t get_update_interval() const override { return 30000; }  // 30s heartbeat

  // Public API (use lambdas in YAML)
  void power_on();
  void power_off();
  bool brew_raw(const std::string &hex_code);          // send "PR:XX" style (hex_code like "02")
  bool brew_product_by_code(uint8_t code);             // send product by numeric code
  bool brew_by_name(const std::string &name);          // name -> code mapping

  // sensors
  void set_text_sensor(text_sensor::TextSensor *s) { this->last_message_sensor_ = s; }
  void set_total_counter_sensor(sensor::Sensor *s) { this->total_counter_sensor_ = s; }

  // configure gap between 4-byte blocks (ms)
  void set_jutta_gap_ms(int ms) { this->jutta_gap_ms_ = ms; }

 protected:
  // core helpers
  bool send_command(const std::string &cmd);
  bool send_raw(const std::vector<uint8_t> &raw);

  // obfuscation (cmd2jura style: bits 2 & 5 used)
  static std::array<uint8_t,4> obfuscate_byte(uint8_t b);
  static uint8_t deobfuscate_4bytes(const std::array<uint8_t,4> &enc);

  // parsing inbound
  bool parse_incoming();
  void handle_full_message_(const std::string &msg);

  // mapping name -> product code (from your XML)
  const std::map<std::string, uint8_t> product_name_to_code_ = {
    {"Espresso", 0x02},
    {"Coffee", 0x03},
    {"Cappuccino", 0x04},
    {"Milk Foam", 0x08},
    {"Hotwater", 0x0D},
    {"2 Espressi", 0x12},
    {"2 Coffee", 0x13},
    {"Espresso (02)", 0x02},
    {"Coffee (03)", 0x03},
    {"Cappuccino (04)", 0x04},
    {"Milk Foam (08)", 0x08},
    {"Hotwater (0D)", 0x0D},
    {"2 Espressi (12)", 0x12},
    {"2 Coffee (13)", 0x13},
    // You can add more from the XML as needed
  };

  // state
  int jutta_gap_ms_{8};
  std::vector<uint8_t> rxbuf_;
  uint32_t last_recv_{0};
  unsigned long last_heartbeat_ts_{0};

  // optional sensors where we publish parsed messages
  text_sensor::TextSensor *last_message_sensor_{nullptr};
  sensor::Sensor *total_counter_sensor_{nullptr};
};

}  // namespace jura
}  // namespace esphome
