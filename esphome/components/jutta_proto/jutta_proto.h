#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#if defined(__has_include)
#if __has_include("esphome/components/sensor/sensor.h")
#include "esphome/components/sensor/sensor.h"
#else
#define ESPHOME_JUTTA_SENSOR_STUB
#endif
#else
#define ESPHOME_JUTTA_SENSOR_STUB
#endif

#ifdef ESPHOME_JUTTA_SENSOR_STUB
namespace esphome {
namespace sensor {

class Sensor {
 public:
  virtual ~Sensor() = default;
  virtual void publish_state(float) {}
  virtual void set_accuracy_decimals(int) {}
  virtual void set_name(const std::string &) {}
  virtual void set_internal(bool) {}
};

}  // namespace sensor
}  // namespace esphome
#endif

#undef ESPHOME_JUTTA_SENSOR_STUB

#if defined(__has_include)
#if __has_include("esphome/components/text_sensor/text_sensor.h")
#include "esphome/components/text_sensor/text_sensor.h"
#else
#define ESPHOME_JUTTA_TEXT_SENSOR_STUB
#endif
#else
#define ESPHOME_JUTTA_TEXT_SENSOR_STUB
#endif

#ifdef ESPHOME_JUTTA_TEXT_SENSOR_STUB
namespace esphome {
namespace text_sensor {

class TextSensor {
 public:
  virtual ~TextSensor() = default;
  virtual void publish_state(const std::string &) {}
};

}  // namespace text_sensor
}  // namespace esphome
#endif
#undef ESPHOME_JUTTA_TEXT_SENSOR_STUB
#include "esphome/components/uart/uart.h"

#include "coffee_maker.hpp"
#include "jutta_config.h"
#include "jutta_connection.hpp"
#include "jutta_commands.hpp"

namespace esphome {
namespace jutta_component {

class JuraComponent : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  ~JuraComponent();
  void setup() override;
  void loop() override;
  void dump_config() override;

  void start_brew(::jutta_proto::CoffeeMaker::coffee_t coffee);
  void start_custom_brew(uint32_t grind_duration_ms, uint32_t water_duration_ms);
  void cancel_custom_brew();
  void switch_page(uint32_t page);
  void run_sequence(const std::vector<::jutta_proto::CoffeeMaker::SequenceStep> &steps);

  bool is_ready() const { return this->handshake_stage_ == HandshakeStage::DONE && this->coffee_maker_ != nullptr; }
  bool is_busy() const;
  const std::string &device_type() const { return this->device_type_; }

  void set_machine_data_sensor(text_sensor::TextSensor *sensor) { this->machine_data_sensor_ = sensor; }
  void set_xml_enabled(bool enabled) {
#if JUTTA_XML_ENABLE
    this->xml_enabled_ = enabled;
#else
    (void) enabled;
    this->xml_enabled_ = false;
#endif
  }
  void set_xml_poll_interval(uint32_t interval_ms) { this->xml_poll_interval_ms_ = interval_ms; }
  void set_xml_rx_timeout(uint32_t timeout_ms) { this->xml_rx_timeout_ms_ = timeout_ms; }

 protected:
  enum class HandshakeStage { IDLE, HELLO, SEND_T1, WAIT_T2, SEND_T2, WAIT_T3, SEND_T3, DONE, FAILED };

  static const char *handshake_stage_name(HandshakeStage stage);

  void process_handshake();
  void restart_handshake(const char *reason);
  bool read_handshake_bytes();
  static bool time_reached(uint32_t now, uint32_t target);
  void process_machine_data_query();
  void publish_machine_data_(const std::string &response);
  void process_xml_polling();
  void load_xml_mapping_();
  void ensure_xml_sensors_created_();
  void reset_xml_cycle_state_();
  bool perform_xml_cycle_();
  bool read_db_data_frame_(std::vector<uint8_t> &decoded, uint32_t timeout_ms,
                           std::string_view last_cmd_ascii);
  bool send_db_command_(const std::string &command);
  void publish_xml_values_();
  void update_sensor_names_();

  std::unique_ptr<::jutta_proto::JuttaConnection> connection_;
  std::unique_ptr<::jutta_proto::CoffeeMaker> coffee_maker_;
  HandshakeStage handshake_stage_{HandshakeStage::IDLE};
  HandshakeStage last_logged_stage_{HandshakeStage::FAILED};
  std::string handshake_buffer_;
  std::string device_type_;
  std::string handshake_t2_response_;
  std::string handshake_t3_response_;
  uint32_t handshake_deadline_{0};
  bool handshake_hello_request_sent_{false};
  bool custom_cancel_flag_{false};
  text_sensor::TextSensor *machine_data_sensor_{nullptr};
  uint32_t machine_data_query_next_{0};
  bool machine_data_request_pending_{false};
  uint32_t machine_data_request_start_{0};

  struct XmlMapping {
    bool valid{false};
    std::string tr32_command{"@TR:32"};
    std::string tg43_command{"@TG:43"};
    std::string tgc0_command{"@TG:C0"};
    std::array<std::string, 10> tr32_labels{};
    std::array<std::string, 6> tg43_labels{};
    std::array<std::string, 3> tgc0_labels{};
  };

  XmlMapping xml_mapping_{};
  bool xml_enabled_{JUTTA_XML_ENABLE != 0};
  uint32_t xml_poll_interval_ms_{JUTTA_XML_POLL_MS};
  uint32_t xml_rx_timeout_ms_{JUTTA_XML_RX_TIMEOUT_MS};
  uint32_t xml_next_poll_{0};
  bool xml_initialized_{false};
  bool xml_mapping_logged_{false};
  std::array<sensor::Sensor *, 10> xml_tr32_sensors_{};
  std::array<sensor::Sensor *, 6> xml_tg43_sensors_{};
  std::array<sensor::Sensor *, 3> xml_tgc0_sensors_{};
  std::array<uint16_t, 10> xml_tr32_values_{};
  std::array<uint16_t, 6> xml_tg43_values_{};
  std::array<uint32_t, 3> xml_tgc0_values_{};
  bool xml_has_values_{false};
  bool xml_busy_{false};
};

class StartBrewAction : public esphome::Action<> {
 public:
  explicit StartBrewAction(JuraComponent *parent) : parent_(parent) {}
  void set_coffee(::jutta_proto::CoffeeMaker::coffee_t coffee) { coffee_ = coffee; }
  void play() override { this->parent_->start_brew(coffee_); }

 protected:
  JuraComponent *parent_;
  ::jutta_proto::CoffeeMaker::coffee_t coffee_{::jutta_proto::CoffeeMaker::coffee_t::ESPRESSO};
};

class CustomBrewAction : public esphome::Action<> {
 public:
  explicit CustomBrewAction(JuraComponent *parent) : parent_(parent) {}
  void set_grind_duration(uint32_t grind) { grind_duration_ms_ = grind; }
  void set_water_duration(uint32_t water) { water_duration_ms_ = water; }
  void play() override { this->parent_->start_custom_brew(grind_duration_ms_, water_duration_ms_); }

 protected:
  JuraComponent *parent_;
  uint32_t grind_duration_ms_{3600};
  uint32_t water_duration_ms_{40000};
};

class CancelCustomBrewAction : public esphome::Action<> {
 public:
  explicit CancelCustomBrewAction(JuraComponent *parent) : parent_(parent) {}
  void play() override { this->parent_->cancel_custom_brew(); }

 protected:
  JuraComponent *parent_;
};

class SwitchPageAction : public esphome::Action<> {
 public:
  explicit SwitchPageAction(JuraComponent *parent) : parent_(parent) {}
  void set_page(uint32_t page) { page_ = page; }
  void play() override { this->parent_->switch_page(page_); }

 protected:
  JuraComponent *parent_;
  uint32_t page_{0};
};

class RunSequenceAction : public esphome::Action<> {
 public:
  explicit RunSequenceAction(JuraComponent *parent) : parent_(parent) {}
  void add_command_step(const std::string &command, uint32_t delay_ms, uint32_t timeout_ms,
                        const std::string &description) {
    ::jutta_proto::CoffeeMaker::SequenceStep step;
    step.type = ::jutta_proto::CoffeeMaker::SequenceStep::Type::Command;
    step.command = command;
    step.delay_ms = delay_ms;
    step.timeout = std::chrono::milliseconds{timeout_ms};
    step.description = description;
    steps_.push_back(step);
  }
  void add_delay_step(uint32_t delay_ms, const std::string &description) {
    ::jutta_proto::CoffeeMaker::SequenceStep step;
    step.type = ::jutta_proto::CoffeeMaker::SequenceStep::Type::Delay;
    step.delay_ms = delay_ms;
    step.description = description;
    steps_.push_back(step);
  }
  void play() override { this->parent_->run_sequence(steps_); }

 protected:
  JuraComponent *parent_;
  std::vector<::jutta_proto::CoffeeMaker::SequenceStep> steps_{};
};

}  // namespace jutta_component
}  // namespace esphome

