#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
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

enum class StateClass {
  STATE_CLASS_NONE,
  STATE_CLASS_MEASUREMENT,
  STATE_CLASS_TOTAL_INCREASING,
};

class Sensor {
 public:
  virtual ~Sensor() = default;
  virtual void publish_state(float) {}
  virtual void set_accuracy_decimals(int) {}
  virtual void set_name(const std::string &) {}
  virtual void set_name(const char *) {}
  virtual void set_internal(bool) {}
  virtual void set_unique_id(const std::string &) {}
  virtual void set_unique_id(const char *) {}
  virtual void set_state_class(StateClass) {}
  virtual void set_unit_of_measurement(const std::string &) {}
  virtual void set_unit_of_measurement(const char *) {}
  virtual void set_icon(const std::string &) {}
  virtual void set_icon(const char *) {}
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

enum class XmlSensorKind { Measurement, Counter };

struct XmlSensorMeta {
  XmlSensorKind kind{XmlSensorKind::Measurement};
  double min_value{0.0};
  double max_value{0.0};
  int accuracy_decimals{0};
  bool configured{false};
  bool has_last_value{false};
  float last_value{0.0f};
  uint32_t last_publish_ms{0};
  bool has_unit{false};
  std::string unit_of_measurement;
  bool has_icon{false};
  std::string icon;
  bool is_tgc0{false};
};

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
  void set_xml_mapping_source(const char *data, size_t length) {
    this->xml_mapping_data_ = data;
    this->xml_mapping_length_ = length;
  }
  void set_xml_poll_interval(uint32_t interval_ms) { this->xml_poll_interval_ms_ = interval_ms; }
  void add_configured_xml_sensor(const std::string &field, sensor::Sensor *sensor);

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
  void log_xml_mapping_status_(bool force = false);
  void ensure_xml_sensors_created_();
  void reset_xml_cycle_state_();
  void publish_xml_stats_();
  void publish_single_stat_(const std::string &name, double value, const std::string &label);
  void register_xml_sensor_(const XmlField &field, XmlSensorKind kind);
  sensor::Sensor *get_or_create_sensor_(const std::string &name, const std::string &label);
  sensor::Sensor *create_internal_sensor_(const std::string &name, const std::string &label);
  enum class XmlPollState {
    IDLE,
    SEND_TR32,
    WAIT_TR32,
    PARSE_TR32,
    SEND_TG43,
    WAIT_TG43,
    PARSE_TG43,
    SEND_TGC0,
    WAIT_TGC0,
    PARSE_TGC0,
    SLEEP
  };

  bool send_xml_command_(const char *command, const char *label, uint32_t now);
  void handle_wait_state_(const char *command, const char *label, XmlPollState parse_state,
                          XmlPollState next_send_state, const XmlCommandMapping &mapping, uint32_t now);
  void handle_parse_state_(const char *command, const char *label, const XmlCommandMapping &mapping,
                           XmlPollState next_state, uint32_t now);
  void complete_xml_cycle_(uint32_t now);
  bool parse_and_stage_frame_(const char *command, const char *label, const XmlCommandMapping &mapping,
                              const std::vector<uint8_t> &frame);
  bool check_counter_coherence_(const std::unordered_map<std::string, StatValue> &values,
                                std::unordered_set<std::string> &skip) const;

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
  std::string xml_mapping_path_{"embedded"};
  const char *xml_mapping_data_{nullptr};
  size_t xml_mapping_length_{0};
  uint32_t xml_next_poll_{0};
  bool xml_mapping_logged_{false};
  bool xml_mapping_loaded_{false};
  XmlMapping xml_mapping_{};
  Stats xml_stats_{};
  std::unordered_map<std::string, sensor::Sensor *> xml_sensors_{};
  std::vector<std::unique_ptr<sensor::Sensor>> xml_owned_sensors_{};
  std::unordered_map<std::string, XmlSensorMeta> xml_sensor_meta_{};
  std::unordered_map<std::string, bool> xml_missing_sensor_logged_{};
  std::unordered_map<std::string, bool> xml_unconfigured_sensor_logged_{};
  XmlPollState xml_state_{XmlPollState::IDLE};
  bool xml_inflight_{false};
  uint32_t xml_deadline_ms_{0};
  uint32_t xml_next_action_ms_{0};
  std::string xml_last_command_;
  std::vector<uint8_t> xml_pending_frame_{};
  bool xml_cycle_had_value_{false};
  float last_tgc0_percent_{std::numeric_limits<float>::quiet_NaN()};
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

