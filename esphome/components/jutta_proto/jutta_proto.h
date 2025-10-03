#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
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
#include "jutta_proto_xml.h"

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
  void set_enable_xml_poll(bool enabled) { this->enable_xml_poll_ = enabled; }
  void set_xml_mapping_path(const std::string &path) { this->xml_mapping_path_ = path; }
  void set_xml_mapping_blob(const std::string &blob) {
    this->xml_mapping_blob_ = blob;
    this->xml_mapping_has_blob_ = true;
  }
  void set_xml_poll_interval(uint32_t interval_ms) { this->xml_poll_interval_ms_ = interval_ms; }

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
  bool ensure_xml_mapping_loaded_();
  void log_xml_mapping_status_();
  void ensure_xml_sensors_created_();
  void reset_xml_cycle_state_();
  void start_xml_cycle_(uint32_t now);
  void handle_xml_cycle_(uint32_t now);
  bool try_receive_xml_frame_(uint32_t now);
  void finish_xml_cycle_(uint32_t now, bool success);
  void publish_xml_stats_();
  void publish_single_stat_(const std::string &name, double value, const std::string &label);
  sensor::Sensor *get_or_create_sensor_(const std::string &name, const std::string &label);
  static const char *xml_command_for_index_(size_t index);
  static const char *xml_log_label_for_index_(size_t index);

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

  bool enable_xml_poll_{false};
  uint32_t xml_poll_interval_ms_{30000};
  std::string xml_mapping_path_{"/data/jura_machine.xml"};
  std::string xml_mapping_blob_{};
  bool xml_mapping_has_blob_{false};
  uint32_t xml_next_poll_{0};
  bool xml_mapping_logged_{false};
  bool xml_mapping_loaded_{false};
  XmlMapping xml_mapping_{};
  MachineStats xml_stats_{};
  std::unordered_map<std::string, sensor::Sensor *> xml_sensors_{};
  std::unordered_map<std::string, std::string> xml_sensor_labels_{};

  struct XmlCycleState {
    enum class Phase { Idle, WaitingForFrame, DelayBeforeNext };

    void reset();

    Phase phase{Phase::Idle};
    size_t command_index{0};
    uint32_t deadline_ms{0};
    uint32_t next_action_ms{0};
    std::array<std::vector<uint8_t>, 3> responses{};
  } xml_cycle_{};
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

