#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include "coffee_maker.hpp"
#include "jutta_connection.hpp"
#include "jutta_commands.hpp"

namespace esphome {
namespace jutta_component {

class JuraComponent : public esphome::Component, public esphome::uart::UARTDevice {
 public:
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
  void set_machine_settings_sensor(text_sensor::TextSensor *sensor) { this->machine_settings_sensor_ = sensor; }
  void set_xml_handshake_sensor(text_sensor::TextSensor *sensor) { this->xml_handshake_sensor_ = sensor; }
  void set_xml_poll_enabled(bool enabled) { this->xml_poll_enabled_ = enabled; }
  void set_xml_poll_interval(uint32_t interval_ms) { this->xml_poll_interval_ms_ = interval_ms; }
  void register_xml_sensor(const std::string &field, const std::string &command, const std::string &key,
                           float multiplier, float offset, sensor::Sensor *sensor);
  void request_machine_settings();
  void write_machine_settings(const std::string &xml);

 protected:
  enum class HandshakeStage { IDLE, HELLO, SEND_T1, WAIT_T2, SEND_T2, WAIT_T3, SEND_T3, DONE, FAILED };

  enum class LegacyCodecMode { Unknown, Plain, Auto, Escaped };

  static const char *handshake_stage_name(HandshakeStage stage);

  void process_handshake();
  void restart_handshake(const char *reason);
  bool read_handshake_bytes();
  static bool time_reached(uint32_t now, uint32_t target);
  void process_machine_data_query();
  void publish_machine_data_(const std::string &response);
  void process_xml_poll();
  void run_legacy_probe_handshake_();
  void perform_xml_handshake_if_needed_();
  std::shared_ptr<std::string> wait_for_response_(::jutta_proto::JuttaConnection *connection,
                                                  const std::string &command, uint32_t timeout_ms);
  bool ensure_transaction_ready_(const char *operation);
  void publish_machine_settings_(const std::string &payload);

  struct XmlSensorEntry {
    std::string field;
    std::string command;
    std::string key;
    float multiplier{1.0f};
    float offset{0.0f};
    sensor::Sensor *sensor{nullptr};
  };

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
  text_sensor::TextSensor *machine_settings_sensor_{nullptr};
  uint32_t machine_data_query_next_{0};
  bool machine_data_request_pending_{false};
  uint32_t machine_data_request_start_{0};
  bool xml_poll_enabled_{false};
  uint32_t xml_poll_interval_ms_{60000};
  uint32_t xml_next_poll_{0};
  std::vector<XmlSensorEntry> xml_sensors_{};
  LegacyCodecMode legacy_codec_mode_{LegacyCodecMode::Unknown};
  std::string legacy_probe_command_{};
  std::string legacy_probe_response_{};
  std::string last_machine_settings_xml_{};
  bool last_machine_settings_write_ok_{false};
  bool machine_settings_write_attempted_{false};
  text_sensor::TextSensor *xml_handshake_sensor_{nullptr};
  bool xml_handshake_attempted_{false};
  bool xml_handshake_ok_{false};
  std::string xml_handshake_request_{};
  std::string xml_handshake_response_{};
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

class RequestMachineSettingsAction : public esphome::Action<> {
 public:
  explicit RequestMachineSettingsAction(JuraComponent *parent) : parent_(parent) {}
  void play() override { this->parent_->request_machine_settings(); }

 protected:
  JuraComponent *parent_;
};

class WriteMachineSettingsAction : public esphome::Action<> {
 public:
  explicit WriteMachineSettingsAction(JuraComponent *parent) : parent_(parent) {}
  void set_xml(const std::string &xml) { this->xml_ = xml; }
  void play() override { this->parent_->write_machine_settings(this->xml_); }

 protected:
  JuraComponent *parent_;
  std::string xml_{};
};

}  // namespace jutta_component
}  // namespace esphome

