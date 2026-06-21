#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <deque>
#include <unordered_map>
#include <vector>
#include <limits>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#if defined(__has_include)
#if __has_include("esphome/core/entity_base.h")
#include "esphome/core/entity_base.h"
#elif __has_include("esphome/core/entity_category.h")
#include "esphome/core/entity_category.h"
#else
#define ESPHOME_JUTTA_ENTITY_CATEGORY_STUB
#endif
#else
#define ESPHOME_JUTTA_ENTITY_CATEGORY_STUB
#endif

#ifdef ESPHOME_JUTTA_ENTITY_CATEGORY_STUB
namespace esphome {

enum EntityCategory : uint8_t {
  ENTITY_CATEGORY_NONE,
  ENTITY_CATEGORY_DIAGNOSTIC,
  ENTITY_CATEGORY_CONFIG,
  ENTITY_CATEGORY_SYSTEM,
};

namespace entity {
using EntityCategory = esphome::EntityCategory;
}  // namespace entity

}  // namespace esphome
#endif

#undef ESPHOME_JUTTA_ENTITY_CATEGORY_STUB
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
  virtual void set_force_update(bool) {}
  virtual void set_entity_category(esphome::EntityCategory) {}
  virtual void set_unit_of_measurement(const std::string &) {}
  virtual void set_unit_of_measurement(const char *) {}
  virtual void set_icon(const std::string &) {}
  virtual void set_icon(const char *) {}
  virtual void set_device_class(const std::string &) {}
  virtual void set_device_class(const char *) {}
  virtual void set_disabled_by_default(bool) {}
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
  virtual void set_name(const std::string &) {}
  virtual void set_name(const char *) {}
  virtual void set_internal(bool) {}
  virtual void set_unique_id(const std::string &) {}
  virtual void set_unique_id(const char *) {}
  virtual void set_entity_category(esphome::EntityCategory) {}
  virtual void set_icon(const std::string &) {}
  virtual void set_icon(const char *) {}
  virtual void set_disabled_by_default(bool) {}
};

}  // namespace text_sensor
}  // namespace esphome
#endif
#undef ESPHOME_JUTTA_TEXT_SENSOR_STUB

#if defined(__has_include)
#if __has_include("esphome/components/binary_sensor/binary_sensor.h")
#include "esphome/components/binary_sensor/binary_sensor.h"
#else
#define ESPHOME_JUTTA_BINARY_SENSOR_STUB
#endif
#else
#define ESPHOME_JUTTA_BINARY_SENSOR_STUB
#endif

#ifdef ESPHOME_JUTTA_BINARY_SENSOR_STUB
namespace esphome {
namespace binary_sensor {

class BinarySensor {
 public:
  virtual ~BinarySensor() = default;
  virtual void publish_state(bool) {}
  virtual void set_name(const std::string &) {}
  virtual void set_name(const char *) {}
  virtual void set_internal(bool) {}
  virtual void set_unique_id(const std::string &) {}
  virtual void set_unique_id(const char *) {}
  virtual void set_entity_category(esphome::EntityCategory) {}
  virtual void set_device_class(const std::string &) {}
  virtual void set_device_class(const char *) {}
  virtual void set_icon(const std::string &) {}
  virtual void set_icon(const char *) {}
  virtual void set_disabled_by_default(bool) {}
};

}  // namespace binary_sensor
}  // namespace esphome
#endif
#undef ESPHOME_JUTTA_BINARY_SENSOR_STUB
#include "esphome/components/uart/uart.h"

#include "coffee_maker.hpp"
#include "jutta_config.h"
#include "jutta_connection.hpp"
#include "jutta_commands.hpp"
#include "jutta_proto_xml.h"
#include "xml_settings.hpp"
#include "xml_errors.hpp"

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
  bool has_unit{false};
  std::string unit_of_measurement;
  bool has_icon{false};
  std::string icon;
  bool is_tgc0{false};
  bool is_percent{false};
  uint32_t last_update_ms{0};
  std::string command_label;
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
  void set_raw_rx_sensor(text_sensor::TextSensor *sensor) { this->raw_rx_sensor_ = sensor; }
  void set_last_command_result_sensor(text_sensor::TextSensor *sensor) { this->last_command_result_sensor_ = sensor; }
  void set_machine_type_sensor(text_sensor::TextSensor *sensor) { this->machine_type_sensor_ = sensor; }
  void set_machine_status_sensor(text_sensor::TextSensor *sensor) { this->machine_status_sensor_ = sensor; }
  void set_machine_display_status_sensor(text_sensor::TextSensor *sensor) { this->machine_display_status_sensor_ = sensor; }
  void set_machine_warning_sensor(text_sensor::TextSensor *sensor) { this->machine_warning_sensor_ = sensor; }
  void set_active_alerts_sensor(text_sensor::TextSensor *sensor) { this->active_alerts_sensor_ = sensor; }
  void set_live_status_source_sensor(text_sensor::TextSensor *sensor) { this->live_status_source_sensor_ = sensor; }
  void set_live_db_status_raw_hex_sensor(text_sensor::TextSensor *sensor) {
    this->live_db_status_raw_hex_sensor_ = sensor;
  }
  void set_live_db_status_decoded_sensor(text_sensor::TextSensor *sensor) {
    this->live_db_status_decoded_sensor_ = sensor;
  }
  void set_live_db_status_source_sensor(text_sensor::TextSensor *sensor) {
    this->live_db_status_source_sensor_ = sensor;
  }
  void set_live_db_status_last_update_sensor(text_sensor::TextSensor *sensor) {
    this->live_db_status_last_update_sensor_ = sensor;
  }
  void set_status_probe_last_response_sensor(text_sensor::TextSensor *sensor) {
    this->status_probe_last_response_sensor_ = sensor;
  }
  void set_debug_command_last_response_sensor(text_sensor::TextSensor *sensor) {
    this->debug_command_last_response_sensor_ = sensor;
  }
  void set_last_t2_status_raw_sensor(text_sensor::TextSensor *sensor) { this->last_t2_status_raw_sensor_ = sensor; }
  void set_last_t2_status_decoded_sensor(text_sensor::TextSensor *sensor) {
    this->last_t2_status_decoded_sensor_ = sensor;
  }
  void set_machine_online_sensor(binary_sensor::BinarySensor *sensor) { this->machine_online_sensor_ = sensor; }
  void set_machine_ready_sensor(binary_sensor::BinarySensor *sensor) { this->machine_ready_sensor_ = sensor; }
  void set_fill_water_required_sensor(binary_sensor::BinarySensor *sensor) { this->fill_water_required_sensor_ = sensor; }
  void set_tf_welcome_sensor(binary_sensor::BinarySensor *sensor) { this->tf_welcome_sensor_ = sensor; }
  void set_tf_coffee_ready_sensor(binary_sensor::BinarySensor *sensor) { this->tf_coffee_ready_sensor_ = sensor; }
  void set_tf_energy_safe_sensor(binary_sensor::BinarySensor *sensor) { this->tf_energy_safe_sensor_ = sensor; }
  void set_tf_active_rf_filter_sensor(binary_sensor::BinarySensor *sensor) {
    this->tf_active_rf_filter_sensor_ = sensor;
  }
  void set_tf_status_bits_sensor(text_sensor::TextSensor *sensor) { this->tf_status_bits_sensor_ = sensor; }
  void set_log_decoded_tx(bool enabled) { this->log_decoded_tx_ = enabled; }
  void set_log_encoded_uart(bool enabled) { this->log_encoded_uart_ = enabled; }
  void set_debug_uart_frames(bool enabled) { this->debug_uart_frames_ = enabled; }
  void set_enable_machine_xml_poll(bool enabled) { this->enable_machine_xml_poll_ = enabled; }
  void set_enable_xml_poll(bool enabled) { this->enable_xml_poll_ = enabled; }
  void set_xml_stats_enabled(bool enabled) {
    this->xml_stats_enabled_ = enabled;
    if (!enabled) {
      return;
    }
    this->enable_xml_poll_ = true;
    this->enable_machine_xml_poll_ = false;
    this->xml_dongle_startup_ = true;
    this->xml_dongle_startup_mode_ = "full";
    this->xml_stats_use_ts_lock_ = true;
    this->xml_stats_reprime_tr37_before_cycle_ = false;
    this->xml_stats_handshake_before_cycle_ = true;
    this->xml_decode_inner_transport_ = true;
    this->xml_publish_unstable_ = false;
    this->xml_command_probe_ = false;
    this->xml_session_probe_ = false;
    this->xml_transport_selftest_ = false;
    this->xml_binary_probe_ = false;
    this->xml_key_probe_ = false;
    this->xml_deep_debug_ = false;
    this->xml_inner_decode_trace_ = false;
    this->xml_run_tablet_start_sequence_ = false;
  }
  void set_xml_stats_debug(bool enabled) {
    this->xml_stats_debug_ = enabled;
    if (enabled) {
      this->xml_dongle_startup_debug_ = true;
      this->xml_dongle_inner_tx_debug_ = true;
      this->xml_inner_decode_trace_ = true;
    }
  }
  void set_xml_publish_unstable(bool enabled) { this->xml_publish_unstable_ = enabled; }
  void set_xml_counter_max(uint32_t max_value) { this->xml_counter_max_ = max_value; }
  void set_xml_wait_for_ts_ack(bool enabled) { this->xml_wait_for_ts_ack_ = enabled; }
  void set_xml_stats_use_ts_lock(bool enabled) { this->xml_stats_use_ts_lock_ = enabled; }
  void set_xml_stats_reprime_tr37_before_cycle(bool enabled) {
    this->xml_stats_reprime_tr37_before_cycle_ = enabled;
  }
  void set_xml_stats_handshake_before_cycle(bool enabled) { this->xml_stats_handshake_before_cycle_ = enabled; }
  void set_xml_debug_compact(bool enabled) { this->xml_debug_compact_ = enabled; }
  void set_xml_decode_inner_transport(bool enabled) { this->xml_decode_inner_transport_ = enabled; }
  void set_xml_inner_decode_trace(bool enabled) { this->xml_inner_decode_trace_ = enabled; }
  void set_xml_binary_probe(bool enabled) { this->xml_binary_probe_ = enabled; }
  void set_xml_key_probe(bool enabled) { this->xml_key_probe_ = enabled; }
  void set_xml_deep_debug(bool enabled) { this->xml_deep_debug_ = enabled; }
  void set_xml_transport_selftest(bool enabled) { this->xml_transport_selftest_ = enabled; }
  void set_xml_command_probe(bool enabled) { this->xml_command_probe_ = enabled; }
  void set_xml_command_probe_with_ts_lock(bool enabled) { this->xml_command_probe_with_ts_lock_ = enabled; }
  void set_xml_session_probe(bool enabled) { this->xml_session_probe_ = enabled; }
  void set_xml_session_probe_variant(const std::string &variant) { this->xml_session_probe_variant_ = variant; }
  void set_xml_dongle_startup(bool enabled) { this->xml_dongle_startup_ = enabled; }
  void set_xml_dongle_startup_debug(bool enabled) { this->xml_dongle_startup_debug_ = enabled; }
  void set_xml_dongle_startup_mode(const std::string &mode) { this->xml_dongle_startup_mode_ = mode; }
  void set_xml_dongle_wait_t0_after_t3(bool enabled) { this->xml_dongle_wait_t0_after_t3_ = enabled; }
  void set_xml_dongle_inner_tx_debug(bool enabled) { this->xml_dongle_inner_tx_debug_ = enabled; }
  void set_xml_run_tablet_start_sequence(bool enabled) { this->xml_run_tablet_start_sequence_ = enabled; }
  void set_xml_tablet_sequence_mode(const std::string &mode) { this->xml_tablet_sequence_mode_ = mode; }
  void set_status_debug(bool enabled) { this->status_debug_ = enabled; }
  void set_status_forensics(bool enabled) {
    (void) enabled;
    this->status_forensics_ = false;
  }
  void set_status_forensics_log_interval(uint32_t interval_ms) { (void) interval_ms; }
  void set_status_forensics_verbose_candidates(bool enabled) { (void) enabled; }
  void set_status_probe_enabled(bool enabled) {
    (void) enabled;
    this->status_probe_enabled_ = false;
  }
  void set_status_probe_interval(uint32_t interval_ms) { this->status_probe_interval_ms_ = interval_ms; }
  void set_pmode_key(uint8_t key) {
    this->pmode_key_ = key;
    this->pmode_key_available_ = true;
  }
  void set_live_db_status_enabled(bool enabled) { this->live_db_status_enabled_ = enabled; }
  void set_live_db_status_debug(bool enabled) { this->live_db_status_debug_ = enabled; }
  void set_live_db_status_publish_raw(bool enabled) { this->live_db_status_publish_raw_ = enabled; }
  void set_live_db_status_poll_enabled(bool enabled) { this->live_db_status_poll_enabled_ = enabled; }
  void set_live_db_status_poll_interval(uint32_t interval_ms) { this->live_db_status_poll_interval_ms_ = interval_ms; }
  void set_live_db_status_response_timeout(uint32_t timeout_ms) {
    this->live_db_status_response_timeout_ms_ = timeout_ms;
  }
  void set_allow_unsafe_debug_commands(bool allow) {
    (void) allow;
    this->allow_unsafe_debug_commands_ = false;
  }
  void run_status_probe_command(const std::string &command);
  void run_manual_handshake_probe(uint32_t observe_ms, const std::string &mode);
  void manual_live_trigger_probe_stayinble(uint32_t observe_ms = 30000, uint32_t interval_ms = 6000);
  void manual_live_event_observe(uint32_t observe_ms = 120000, bool stayinble = true, uint32_t interval_ms = 6000);
  void manual_original_startup_observe(uint32_t observe_ms = 180000, bool respond_identity = true,
                                       bool active_probe = false, bool boot_attached_mode = true);
  void manual_original_startup_active_safe(uint32_t observe_ms = 180000, bool send_core_startup = true,
                                           bool respond_identity = true, bool active_probe = true);
  void manual_original_startup_active_stateful(uint32_t observe_ms = 180000, bool send_core_startup = true,
                                               bool respond_identity = true, bool active_probe = true);
  void run_ble2_transport_probe(const std::string &probe);
  void run_debug_command(const std::string &command, const std::string &transport);
  void set_xml_mapping_path(const std::string &path) { this->xml_mapping_path_ = path; }
  void set_xml_mapping_source(const char *data, size_t length) {
    this->xml_mapping_data_ = data;
    this->xml_mapping_length_ = length;
  }
  void set_xml_poll_interval(uint32_t interval_ms) { this->set_xml_poll_interval_ms(interval_ms); }
  void set_xml_poll_interval_ms(uint32_t interval_ms) { this->xml_poll_interval_ms_ = interval_ms; }
  void set_xml_startup_delay(uint32_t delay_ms) { this->xml_startup_delay_ms_ = delay_ms; }
  void add_configured_xml_sensor(const std::string &field, sensor::Sensor *sensor);
  void register_setting_sensor(const std::string &id, sensor::Sensor *sensor);
  void register_setting_text_sensor(const std::string &id, text_sensor::TextSensor *sensor);
  void set_error_code_sensor(sensor::Sensor *sensor) { this->error_code_sensor_ = sensor; }
  void set_error_text_sensor(text_sensor::TextSensor *sensor) { this->error_text_sensor_ = sensor; }
  void set_error_severity_sensor(text_sensor::TextSensor *sensor) { this->error_severity_sensor_ = sensor; }
  void set_error_active_sensor(binary_sensor::BinarySensor *sensor) { this->error_active_sensor_ = sensor; }

 protected:
  enum class HandshakeStage { IDLE, HELLO, SEND_T1, WAIT_T2, SEND_T2, WAIT_T3, SEND_T3, DONE, FAILED };
  enum class TabletSeqState {
    IDLE,
    SEND_D1,
    WAIT_D1,
    SEND_TY,
    WAIT_TY,
    SEND_T1,
    WAIT_T1,
    SEND_T2,
    WAIT_T2,
    SEND_T3,
    WAIT_T3,
    SEND_TR37,
    WAIT_TR37,
    DONE,
    FAILED
  };
  enum class TransportSelftestState {
    IDLE,
    STATS_SEND_TY,
    STATS_WAIT_TY,
    NORMAL_SEND_TY,
    NORMAL_WAIT_TY,
    DONE
  };
  enum class XmlCommandProbeState { IDLE, SEND, WAIT, DONE };
  enum class XmlSessionProbeState { IDLE, SEND, WAIT, DONE, FAILED };
  enum class StatusProbeState { IDLE, SEND, WAIT, DONE };
  enum class ManualHandshakeProbeState { IDLE, RUN_HANDSHAKE, OBSERVE };
  enum class ManualHandshakeProbeMode {
    NORMAL,
    TEST_C_APP_INITIAL_READS,
    LIVE_TRIGGER_STAYINBLE,
    LIVE_EVENT_OBSERVE,
    ORIGINAL_STARTUP_OBSERVE,
    ORIGINAL_STARTUP_ACTIVE_SAFE,
    ORIGINAL_STARTUP_ACTIVE_STATEFUL
  };
  enum class OriginalStartupActiveStage {
    IDLE,
    SEND_T0,
    WAIT_AFTER_T0,
    SEND_H1,
    WAIT_AFTER_H1,
    SEND_TY,
    WAIT_TY,
    SEND_T1,
    WAIT_T1,
    SEND_TR37,
    WAIT_TR37,
    OBSERVE
  };
  enum class Ble2ProbeState { IDLE, WAIT };
  enum class DebugCommandState { IDLE, WAIT };
  enum class DongleStartupState {
    IDLE,
    START_CLEAR,
    PROBE_D1,
    PROBE_TY,
    SEND_T1,
    WAIT_T2,
    SEND_T2,
    WAIT_T3,
    SEND_T3,
    WAIT_T0_AFTER_T3,
    WAIT_AFTER_T3,
    PREP_TR37,
    SEND_TR37,
    WAIT_TR37,
    READY,
    FAILED
  };

  static const char *handshake_stage_name(HandshakeStage stage);

  void process_handshake();
  void restart_handshake(const char *reason);
  bool read_handshake_bytes();
  static bool time_reached(uint32_t now, uint32_t target);
  void handle_decoded_response_(const std::string &response, const char *parser_branch);
  void publish_raw_rx_(const std::string &response, const char *parser_branch);
  void publish_last_command_result_(const std::string &result);
  void publish_machine_type_();
  void publish_machine_status_(const std::string &status);
  void publish_machine_online_(bool online);
  void publish_machine_ready_(bool ready);
  bool publish_tf_status_(const std::string &response);
  bool handle_tv_progress_(const std::string &response);
  bool handle_t2_status_debug_(const std::string &response);
  void update_machine_status_from_state_(const char *source);
  void publish_status_probe_last_response_(const std::string &text);
  void process_status_probe_(uint32_t now);
  void start_status_probe_(uint32_t now);
  void start_manual_status_probe_(const std::string &command, uint32_t now);
  const char *status_probe_candidate_(size_t index) const;
  size_t status_probe_candidate_count_() const;
  bool status_probe_command_allowed_(const std::string &command) const;
  bool send_status_probe_candidate_(const std::string &command, uint32_t now);
  bool handle_status_probe_line_(const std::string &line, bool complete, uint32_t now);
  void finish_status_probe_candidate_(uint32_t now, const char *reason);
  void finish_status_probe_cycle_(uint32_t now, const char *result);
  bool start_manual_handshake_probe_(uint32_t observe_ms, const std::string &mode, uint32_t now);
  bool handle_manual_handshake_probe_line_(const std::string &line, const char *table_name, uint32_t now);
  bool read_virtual_app_initial_cache_(uint16_t characteristic, std::string &cache_hex) const;
  void run_manual_handshake_app_initial_reads_(uint32_t now);
  const char *manual_handshake_mode_name_() const;
  bool guard_manual_observe_tx_(const char *source, const std::string &frame);
  bool manual_live_trigger_stayinble_tx_allowed_(const std::string &frame) const;
  bool send_manual_live_trigger_stayinble_(uint32_t now);
  bool manual_original_startup_observe_active_() const;
  bool manual_original_startup_mode_active_() const;
  bool manual_original_startup_tx_allowed_(const std::string &frame) const;
  bool manual_original_startup_active_tx_allowed_(const std::string &frame) const;
  bool start_manual_original_startup_observe_(uint32_t observe_ms, bool respond_identity, bool active_probe,
                                              bool boot_attached_mode, uint32_t now);
  bool start_manual_original_startup_active_safe_(uint32_t observe_ms, bool send_core_startup, bool respond_identity,
                                                  bool active_probe, uint32_t now, bool stateful = false);
  bool handle_manual_original_startup_observe_line_(const std::string &line, const char *table_name, uint32_t now);
  bool send_manual_original_startup_identity_reply_(const std::string &rx_line, const std::string &tx_line,
                                                    const char *confidence, const char *reason, uint32_t now);
  bool send_manual_original_startup_active_command_(const std::string &line, bool inner_uart0, const char *reason,
                                                    uint32_t now);
  void process_manual_original_startup_active_safe_(uint32_t now);
  void trace_machine_tx_startup_(const char *source, const std::string &line, bool encoded, const char *reason);
  void reset_startup_tx_trace_();
  void log_startup_tx_diff_();
  void log_normal_startup_sequence_();
  void log_startup_sequence_diff_original_vs_esp_();
  std::string startup_pending_followup_tx_() const;
  void update_original_like_flags88_from_line_(const std::string &line);
  void log_original_like_tx_tr37_(const char *source, const char *reason);
  void log_original_like_session_summary_(const char *context);
  void log_original_like_core_session_diff_();
  void log_startup_state_after_rx_(const std::string &line);
  void log_original_startup_state_diff_();
  void log_startup_sequence_result_(const char *mode);
  const char *startup_tx_reason_(const std::string &line) const;
  void process_manual_handshake_probe_(uint32_t now);
  void finish_manual_handshake_probe_(uint32_t now, const char *result);
  bool start_ble2_transport_probe_(const std::string &probe, uint32_t now);
  bool send_ble2_transport_probe_(const std::string &probe, const std::string &command, uint32_t now);
  bool handle_ble2_probe_line_(const std::string &line, const char *table_name, uint32_t now);
  void process_ble2_transport_probe_(uint32_t now);
  void finish_ble2_transport_probe_(uint32_t now, const char *result);
  void publish_debug_command_last_response_(const std::string &text);
  std::string normalize_debug_command_(const std::string &command) const;
  bool is_unsafe_debug_command_(const std::string &command) const;
  bool start_debug_command_(const std::string &command, const std::string &transport, uint32_t now);
  bool handle_debug_command_line_(const std::string &line, const char *table_name, uint32_t now);
  void process_debug_command_(uint32_t now);
  void finish_debug_command_(uint32_t now, const char *result);
  void log_status_forensics_frame_(const std::string &raw, const char *source);
  void log_status_forensics_decoded_(const std::string &line, const char *source, const char *table_name);
  bool handle_live_db_transport_frame_(const std::string &response);
  void publish_live_db_status_raw_(const std::string &response, const char *parser_branch);
  void publish_live_db_status_decoded_(const std::string &summary, const std::string &table_trace);
  void publish_text_if_changed_(text_sensor::TextSensor *sensor, std::string &last_value, const std::string &value);
  bool decode_and_publish_status_(const std::string &response, const char *parser_branch);
  std::string format_decoded_status_(const std::vector<JuraDecodedField> &fields) const;
  bool is_printable_status_text_(const std::string &text) const;
  void process_machine_data_query();
  void process_live_db_status_poll_(uint32_t now);
  void schedule_live_db_status_retry_(uint32_t now, const char *reason);
  void publish_machine_data_(const std::string &response);
  void process_xml_polling();
  void process_xml_command_probe_scheduler_(uint32_t now);
  void log_xml_command_probe_wait_(const char *reason, const char *owner = nullptr);
  bool process_transport_selftest_(uint32_t now);
  void start_transport_selftest_(uint32_t now);
  void send_transport_selftest_command_(const char *path, const std::string &command, uint32_t now);
  void finish_transport_selftest_step_(const char *path, const std::string &command, const char *expected,
                                       bool timeout, uint32_t now);
  const char *transport_selftest_state_name_(TransportSelftestState state) const;
  bool process_xml_command_probe_(uint32_t now);
  void start_xml_command_probe_(uint32_t now);
  const char *xml_command_probe_command_(size_t index) const;
  size_t xml_command_probe_command_count_() const;
  void send_xml_command_probe_command_(const std::string &command, uint32_t now);
  void finish_xml_command_probe_step_(const std::string &command, uint32_t now);
  const char *classify_xml_probe_response_(const std::string &response, bool line_complete = false) const;
  void process_xml_session_probe_scheduler_(uint32_t now);
  void log_xml_session_probe_wait_(const char *reason, const char *owner = nullptr);
  bool process_xml_session_probe_(uint32_t now);
  void start_xml_session_probe_(uint32_t now);
  const char *xml_session_probe_command_(size_t index) const;
  size_t xml_session_probe_command_count_() const;
  std::string xml_session_probe_format_command_(const char *command) const;
  void send_xml_session_probe_command_(const std::string &command, uint32_t now);
  void finish_xml_session_probe_step_(const std::string &command, uint32_t now);
  const char *classify_xml_session_decoded_response_(const std::string &response) const;
  bool xml_session_probe_expected_match_(const std::string &command, const std::string &response) const;
  void finish_xml_session_probe_cycle_(uint32_t now, const char *result, const char *reason = nullptr);
  const char *dongle_startup_state_name_(DongleStartupState state) const;
  void transition_dongle_startup_(DongleStartupState state, uint32_t now);
  bool process_dongle_startup_(uint32_t now);
  bool send_dongle_startup_command_(const std::string &command, uint32_t now, bool inner_uart0 = false);
  bool write_inner_uart0_command_(const std::string &command, uint32_t now, bool no_rx_flush = false);
  void fail_dongle_startup_(uint32_t now, const char *reason);
  void update_dongle_events_from_line_(const std::string &line);
  void process_dongle_startup_rx_(uint32_t now);
  bool ensure_xml_mapping_loaded_();
  void log_xml_mapping_status_(bool force = false);
  void ensure_xml_sensors_created_();
  void reset_xml_poll_state_();
  void publish_xml_stats_();
  void publish_single_stat_(const std::string &name, double value, const std::string &label);
  void register_xml_sensor_(const XmlField &field, XmlSensorKind kind, const char *command_label);
  sensor::Sensor *find_configured_sensor_(const std::string &name) const;
  void apply_sensor_metadata_(const std::string &name, sensor::Sensor *sensor);
  bool stage_tgc0_value_(const std::string &name, const std::string &label, float filtered_value,
                         uint16_t header_value, uint16_t encoded_value, uint16_t raw_value);
  bool decode_field_value_(const std::vector<uint8_t> &decoded, const XmlField &field, bool little_endian,
                           std::uint64_t &out) const;
  void handle_xml_state_machine_(uint32_t now);
  void start_new_xml_cycle_(uint32_t now);
  enum class XmlPollState {
    IDLE,
    STATS_HANDSHAKE,
    REPRIME_TR37,
    WAIT_REPRIME_TR37,
    TS_LOCK,
    WAIT_TS_LOCK,
    TR32_PAGE,
    WAIT_TR32_PAGE,
    TG43,
    WAIT_TG43,
    TGC0,
    WAIT_TGC0,
    TS_UNLOCK,
    WAIT_TS_UNLOCK,
    DONE,
    SEND_TR32,
    WAIT_TR32,
    PARSE_TR32,
    SEND_TG43,
    PARSE_TG43,
    SEND_TGC0,
    PARSE_TGC0,
    SLEEP
  };
  enum class DbTransactionOwner {
    NONE,
    XML_POLL,
    MACHINE_XML,
    LIVE_DB_STATUS,
    STATS_HANDSHAKE,
    MANUAL_HANDSHAKE_PROBE,
    STATUS_PROBE,
    BLE2_PROBE,
    DEBUG_COMMAND
  };
  size_t xml_command_index_(XmlPollState state) const;
  const char *db_transaction_owner_name_(DbTransactionOwner owner) const;
  bool begin_xml_transaction_(const char *command, uint32_t now);
  void end_xml_transaction_(const char *reason);
  void clear_db_transaction_(DbTransactionOwner owner);
  void flush_xml_rx_(bool flush_serial);
  bool validate_xml_frame_(XmlPollState state, const std::vector<uint8_t> &decoded, bool had_crlf,
                           size_t decoded_len, std::vector<uint8_t> &payload, size_t &expected_min_len,
                           uint8_t &head0, std::string &reason) const;
  bool validate_counter_frame_(XmlPollState state, const XmlCommandMapping &mapping, const std::vector<uint8_t> &frame,
                               const char *command_label, std::string &reason) const;
  bool counter_frame_is_stable_(XmlPollState state, const std::vector<uint8_t> &frame, const char *command_label,
                                std::string &reason);
  bool stage_counter_frame_(const XmlCommandMapping &mapping, const std::vector<uint8_t> &frame,
                            const char *command_label);
  bool process_valid_tgc0_frame_(const std::vector<uint8_t> &frame, bool stage_values);
  bool should_retry_current_(XmlPollState wait_state, uint32_t now);
  void handle_xml_failure_(XmlPollState wait_state, bool is_timeout, size_t decoded_len, uint32_t now,
                           const char *reason = nullptr);
  void complete_command_success_(XmlPollState wait_state);
  bool xml_state_has_mapping_(XmlPollState state) const;
  const char *xml_state_command_(XmlPollState state) const;
  const char *xml_state_label_(XmlPollState state) const;
  void transition_to_state_(XmlPollState state, uint32_t now, uint32_t delay_ms = 0);
  bool write_stats_command_(const std::string &command, uint32_t now, bool fire_and_forget);
  bool forward_post_gate_app_command_(const std::string &command, const std::string &expected_prefix,
                                      uint32_t timeout_ms, XmlPollState wait_state, uint32_t now);
  bool send_stats_ascii_command_(const std::string &command, XmlPollState wait_state, uint32_t now);
  bool send_stats_fire_and_forget_(const std::string &command, XmlPollState next_state, uint32_t now,
                                   uint32_t settle_delay_ms);
  bool process_tablet_start_sequence_(uint32_t now);
  void start_tablet_start_sequence_(uint32_t now);
  void send_tablet_sequence_command_(const std::string &command, TabletSeqState wait_state, uint32_t now);
  void finish_tablet_sequence_command_(uint32_t now, bool timeout);
  const char *tablet_sequence_state_name_(TabletSeqState state) const;
  bool read_stats_line_(std::string &line);
  bool finish_stats_rx_capture_(std::string &line, uint32_t now);
  void log_stats_binary_probe_(const std::string &frame);
  bool probe_stats_inner_key_variants_(const std::string &frame, std::string &decoded_line);
  bool handle_stats_line_(const std::string &line, uint32_t now);
  bool decode_stats_inner_transport_line_(const std::string &raw_line, std::string &decoded_line,
                                          bool frame_complete = true);
  bool handle_stats_binary_response_(uint32_t now);
  void advance_after_stats_timeout_(uint32_t now);
  void advance_after_stats_reject_(uint32_t now);
  bool parse_tr32_page_line_(const std::string &line, uint8_t expected_page);
  bool parse_tg43_line_(const std::string &line);
  bool parse_tgc0_line_(const std::string &line);
  bool extract_stats_hex_payload_(const std::string &line, const std::string &prefix, std::string &payload) const;
  bool parse_hex_bytes_(const std::string &hex, std::vector<uint8_t> &bytes, std::string &reason) const;
  bool stage_xml_stat_value_(const std::string &name, const std::string &label, double value,
                             XmlSensorKind kind, const char *command_label);
  std::string product_counter_field_name_(uint8_t product_index, std::string &label) const;
  std::string raw_product_counter_field_name_(uint8_t page, uint8_t slot) const;
  void finish_stats_cycle_(uint32_t now, const char *reason);
  void handle_xml_timeout_(XmlPollState next_state, const char *label, uint32_t now);
  void poll_settings_once_();
  void poll_settings_refresh_();
  void poll_error_cycle_();
  void ensure_setting_entities_created_();
  void publish_setting_value_(const SettingDesc &desc, float value, const std::string &raw_text);
  void publish_error_state_(uint32_t code);
  bool query_setting_command_(const std::string &command, std::vector<uint8_t> &decoded);
  bool query_error_command_(const std::string &command, std::vector<uint8_t> &decoded);
  bool request_machine_xml_(std::string &xml);
  void handle_machine_xml_(const std::string &xml);
  bool ensure_machine_xml_(uint32_t max_age_ms, std::string &xml_out);
  std::string format_machine_status_summary_(const std::string &xml) const;
  void update_settings_from_xml_(const std::string &xml);
  void update_errors_from_xml_(const std::string &xml);
  bool xml_get_value_(const std::string &xml, const std::string &path, std::string &out) const;
  std::string determine_setting_path_(const SettingDesc &desc) const;

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
  text_sensor::TextSensor *raw_rx_sensor_{nullptr};
  text_sensor::TextSensor *last_command_result_sensor_{nullptr};
  text_sensor::TextSensor *machine_type_sensor_{nullptr};
  text_sensor::TextSensor *machine_status_sensor_{nullptr};
  text_sensor::TextSensor *machine_display_status_sensor_{nullptr};
  text_sensor::TextSensor *machine_warning_sensor_{nullptr};
  text_sensor::TextSensor *active_alerts_sensor_{nullptr};
  text_sensor::TextSensor *live_status_source_sensor_{nullptr};
  text_sensor::TextSensor *live_db_status_raw_hex_sensor_{nullptr};
  text_sensor::TextSensor *live_db_status_decoded_sensor_{nullptr};
  text_sensor::TextSensor *live_db_status_source_sensor_{nullptr};
  text_sensor::TextSensor *live_db_status_last_update_sensor_{nullptr};
  text_sensor::TextSensor *status_probe_last_response_sensor_{nullptr};
  text_sensor::TextSensor *debug_command_last_response_sensor_{nullptr};
  text_sensor::TextSensor *last_t2_status_raw_sensor_{nullptr};
  text_sensor::TextSensor *last_t2_status_decoded_sensor_{nullptr};
  binary_sensor::BinarySensor *machine_online_sensor_{nullptr};
  binary_sensor::BinarySensor *machine_ready_sensor_{nullptr};
  binary_sensor::BinarySensor *fill_water_required_sensor_{nullptr};
  binary_sensor::BinarySensor *tf_welcome_sensor_{nullptr};
  binary_sensor::BinarySensor *tf_coffee_ready_sensor_{nullptr};
  binary_sensor::BinarySensor *tf_energy_safe_sensor_{nullptr};
  binary_sensor::BinarySensor *tf_active_rf_filter_sensor_{nullptr};
  text_sensor::TextSensor *tf_status_bits_sensor_{nullptr};
  bool log_decoded_tx_{true};
  bool log_encoded_uart_{false};
  bool debug_uart_frames_{false};
  uint32_t machine_data_query_next_{0};
  uint32_t machine_xml_busy_backoff_until_{0};
  uint32_t machine_xml_next_request_ms_{0};
  bool machine_xml_use_fallback_next_{false};
  std::string machine_xml_cache_{};
  uint32_t machine_xml_timestamp_{0};
  std::string current_display_status_{};
  std::string current_machine_warning_{};
  std::string current_active_alerts_{};
  std::string current_live_status_source_{"nicht verfügbar"};
  std::string last_tf_status_frame_{};
  std::string last_tv_progress_frame_{};
  std::string current_live_db_status_raw_hex_{};
  std::string current_live_db_status_decoded_{};
  std::string current_live_db_status_source_{};
  std::string current_live_db_status_last_update_{};
  bool machine_online_state_{false};
  bool machine_ready_state_{false};
  bool has_valid_tf_status_{false};
  bool has_valid_tv_status_{false};
  bool fill_water_required_{false};
  bool tf_coffee_ready_active_{false};
  bool status_debug_{false};
  bool status_forensics_{false};
  bool status_forensics_verbose_candidates_{false};
  uint32_t status_forensics_log_interval_ms_{2000};
  uint32_t status_forensics_next_log_ms_{0};
  uint32_t status_forensics_next_suppressed_log_ms_{0};
  std::string status_forensics_last_raw_hex_{};
  bool status_forensics_decode_log_allowed_{false};
  bool status_probe_enabled_{false};
  bool live_db_status_enabled_{true};
  bool live_db_status_debug_{false};
  bool live_db_status_publish_raw_{true};
  bool live_db_status_poll_enabled_{false};
  bool live_db_status_use_fallback_next_{false};
  uint32_t live_db_status_poll_interval_ms_{10000};
  uint32_t live_db_status_response_timeout_ms_{1200};
  uint32_t live_db_status_next_poll_ms_{0};
  uint32_t live_db_status_after_stats_hold_until_ms_{0};
  uint32_t status_probe_interval_ms_{300000};
  bool allow_unsafe_debug_commands_{false};
  uint32_t status_probe_next_ms_{0};
  uint8_t pmode_key_{0x02};
  bool pmode_key_available_{true};
  StatusProbeState status_probe_state_{StatusProbeState::IDLE};
  size_t status_probe_index_{0};
  std::string status_probe_current_cmd_{};
  std::string status_probe_rx_buffer_{};
  uint32_t status_probe_deadline_ms_{0};
  uint16_t status_probe_frames_{0};
  ManualHandshakeProbeState manual_handshake_probe_state_{ManualHandshakeProbeState::IDLE};
  ManualHandshakeProbeMode manual_handshake_probe_mode_{ManualHandshakeProbeMode::NORMAL};
  uint32_t manual_handshake_observe_ms_{5000};
  uint32_t manual_handshake_deadline_ms_{0};
  uint16_t manual_handshake_frames_{0};
  uint16_t manual_handshake_control_count_{0};
  uint16_t manual_handshake_tf_count_{0};
  uint16_t manual_handshake_tv_count_{0};
  uint16_t manual_handshake_event_other_count_{0};
  uint16_t manual_handshake_unknown_count_{0};
  bool manual_handshake_app_initial_reads_done_{false};
  bool manual_handshake_original_gate_done_{false};
  bool manual_handshake_cache_1531_present_{false};
  bool manual_handshake_cache_1524_present_{false};
  bool manual_handshake_cache_1527_present_{false};
  bool manual_observe_no_tx_guard_{false};
  bool manual_handshake_tx_violation_{false};
  bool manual_handshake_tf_seen_{false};
  bool manual_handshake_tv_seen_{false};
  uint32_t manual_live_trigger_interval_ms_{6000};
  uint32_t manual_live_trigger_next_tx_ms_{0};
  uint32_t manual_live_trigger_last_tx_ms_{0};
  uint16_t manual_live_trigger_stayinble_tx_count_{0};
  bool manual_live_trigger_allow_stayinble_tx_{false};
  bool manual_live_event_observe_stayinble_{true};
  bool manual_original_startup_respond_identity_{true};
  bool manual_original_startup_active_probe_{false};
  bool manual_original_startup_boot_attached_mode_{true};
  uint16_t manual_original_startup_host_identity_request_count_{0};
  uint16_t manual_original_startup_safe_identity_response_count_{0};
  uint16_t manual_original_startup_unhandled_identity_request_count_{0};
  uint16_t manual_original_startup_noop_identity_request_count_{0};
  uint16_t manual_original_startup_gate_count_{0};
  uint16_t manual_original_startup_machine_identity_count_{0};
  bool manual_original_startup_seen_hb_{false};
  bool manual_original_startup_seen_gb_{false};
  bool manual_original_startup_seen_hy_{false};
  bool manual_original_startup_seen_hl_{false};
  bool manual_original_startup_seen_hc_{false};
  bool manual_original_startup_seen_hi_{false};
  bool manual_original_startup_seen_hr_{false};
  bool manual_original_startup_seen_hf_{false};
  bool manual_original_startup_seen_hp_{false};
  bool manual_original_startup_seen_ht_{false};
  bool manual_original_startup_seen_hw_{false};
  bool manual_original_startup_identity_tx_allowed_{false};
  bool manual_original_startup_active_tx_guard_open_{false};
  bool manual_original_startup_send_core_startup_{true};
  bool manual_original_startup_active_probe_requested_{true};
  bool manual_original_startup_stateful_{false};
  OriginalStartupActiveStage manual_original_startup_active_stage_{OriginalStartupActiveStage::IDLE};
  uint32_t manual_original_startup_active_next_ms_{0};
  uint32_t manual_original_startup_active_deadline_ms_{0};
  bool manual_original_startup_sent_t0_{false};
  bool manual_original_startup_sent_h1_{false};
  bool manual_original_startup_sent_ty_{false};
  bool manual_original_startup_sent_t1_{false};
  bool manual_original_startup_sent_tr37_{false};
  bool manual_original_startup_got_ty_{false};
  bool manual_original_startup_got_t1_{false};
  bool manual_original_startup_got_tr37_{false};
  std::vector<std::string> manual_original_startup_sent_sequence_;
  std::vector<std::string> manual_original_startup_rx_sequence_;
  bool startup_trace_sends_t0_{false};
  bool startup_trace_sends_t1_{false};
  bool startup_trace_sends_h1_{false};
  bool startup_trace_sends_ty_{false};
  bool startup_trace_sends_tr37_{false};
  bool startup_trace_sends_t3_{false};
  bool startup_trace_sends_t2_{false};
  bool startup_trace_sends_tp_{false};
  bool startup_trace_sends_d1_{false};
  std::vector<std::string> startup_trace_tx_sequence_;
  std::vector<std::string> startup_trace_rx_sequence_;
  // Diagnostic-only mirror of selected original BlueFrog flags_88 bits.
  // This must not drive control flow or TX decisions.
  uint32_t original_like_flags88_{0};
  std::string original_like_last_rx_{};
  std::string original_like_last_tx_{};
  uint8_t original_like_t2_first_byte_{0};
  bool original_like_t2_first_byte_known_{false};
  std::string original_like_t3_code_{};
  bool original_like_tr37_seen_{false};
  bool original_like_tf_seen_{false};
  bool original_like_tv_seen_{false};
  bool manual_handshake_prev_xml_dongle_startup_{false};
  bool manual_handshake_prev_xml_dongle_startup_debug_{false};
  std::string manual_handshake_prev_xml_dongle_startup_mode_{};
  bool manual_handshake_prev_stats_session_ready_{false};
  bool manual_handshake_prev_stats_inner_tx_required_{false};
  bool manual_handshake_prev_post_gate_tx_ready_event_{true};
  Ble2ProbeState ble2_probe_state_{Ble2ProbeState::IDLE};
  std::string ble2_probe_name_{};
  std::string ble2_probe_command_{};
  uint32_t ble2_probe_deadline_ms_{0};
  uint16_t ble2_probe_frames_{0};
  DebugCommandState debug_command_state_{DebugCommandState::IDLE};
  std::string debug_command_cmd_{};
  std::string debug_command_transport_{};
  uint32_t debug_command_deadline_ms_{0};
  uint16_t debug_command_frames_{0};

  bool enable_machine_xml_poll_{true};
  bool enable_xml_poll_{false};
  bool xml_stats_enabled_{false};
  bool xml_stats_debug_{false};
  bool xml_publish_unstable_{false};
  bool xml_wait_for_ts_ack_{false};
  bool xml_stats_use_ts_lock_{false};
  bool xml_stats_reprime_tr37_before_cycle_{false};
  bool xml_stats_handshake_before_cycle_{true};
  bool xml_debug_compact_{true};
  bool xml_decode_inner_transport_{true};
  bool xml_inner_decode_trace_{false};
  bool xml_binary_probe_{false};
  bool xml_key_probe_{false};
  bool xml_deep_debug_{false};
  bool xml_transport_selftest_{false};
  bool xml_command_probe_{false};
  bool xml_command_probe_with_ts_lock_{true};
  bool xml_session_probe_{false};
  std::string xml_session_probe_variant_{"minimal"};
  bool xml_dongle_startup_{false};
  bool xml_dongle_startup_debug_{false};
  std::string xml_dongle_startup_mode_{"full"};
  bool xml_dongle_wait_t0_after_t3_{false};
  bool xml_dongle_inner_tx_debug_{false};
  uint32_t dongle_events_{0};
  bool stats_session_ready_{false};
  bool stats_inner_tx_required_{false};
  bool post_gate_tx_ready_event_{true};
  bool post_gate_reprime_required_for_next_stats_{true};
  bool stats_handshake_before_cycle_active_{false};
  uint16_t startup_t2_word_{0};
  std::string dongle_machine_identity_{};
  std::string dongle_tr_payload_{};
  DongleStartupState dongle_startup_state_{DongleStartupState::IDLE};
  std::string dongle_startup_rx_buffer_{};
  uint32_t dongle_startup_deadline_ms_{0};
  uint32_t dongle_startup_next_action_ms_{0};
  uint32_t dongle_startup_next_retry_ms_{0};
  uint32_t dongle_startup_quiet_start_ms_{0};
  uint8_t dongle_startup_probe_attempt_{0};
  uint8_t dongle_startup_t1_attempt_{0};
  uint8_t dongle_startup_tr37_attempt_{0};
  bool dongle_startup_t3_seen_during_quiet_{false};
  bool dongle_startup_t3_seen_while_waiting_tr37_{false};
  bool dongle_startup_quiet_then_prep_tr37_{true};
  std::string dongle_startup_last_error_{};
  uint8_t dongle_inner_tx_key_counter_{0x42};
  bool xml_run_tablet_start_sequence_{false};
  std::string xml_tablet_sequence_mode_{"minimal"};
  bool xml_tablet_start_sequence_done_{false};
  TabletSeqState tablet_seq_state_{TabletSeqState::IDLE};
  std::string tablet_seq_rx_buffer_{};
  std::string tablet_seq_current_cmd_{};
  uint32_t tablet_seq_deadline_ms_{0};
  bool tablet_seq_tx_failed_{false};
  TransportSelftestState transport_selftest_state_{TransportSelftestState::IDLE};
  std::string transport_selftest_rx_buffer_{};
  std::string transport_selftest_current_cmd_{};
  uint32_t transport_selftest_deadline_ms_{0};
  XmlCommandProbeState xml_command_probe_state_{XmlCommandProbeState::IDLE};
  size_t xml_command_probe_index_{0};
  std::string xml_command_probe_rx_buffer_{};
  std::string xml_command_probe_current_cmd_{};
  uint32_t xml_command_probe_deadline_ms_{0};
  uint32_t xml_command_probe_next_ms_{0};
  std::string xml_command_probe_last_wait_reason_{};
  XmlSessionProbeState xml_session_probe_state_{XmlSessionProbeState::IDLE};
  size_t xml_session_probe_index_{0};
  std::string xml_session_probe_rx_buffer_{};
  std::string xml_session_probe_current_cmd_{};
  uint32_t xml_session_probe_deadline_ms_{0};
  uint32_t xml_session_probe_next_ms_{0};
  std::string xml_session_probe_last_wait_reason_{};
  uint8_t xml_session_probe_timeouts_{0};
  uint32_t xml_counter_max_{20000};
  uint32_t xml_poll_interval_ms_{300000};
  uint32_t xml_startup_delay_ms_{10000};
  std::string xml_mapping_path_{"embedded"};
  const char *xml_mapping_data_{nullptr};
  size_t xml_mapping_length_{0};
  uint32_t xml_next_poll_{0};
  uint32_t xml_stats_cycle_id_{0};
  bool xml_next_poll_is_retry_{false};
  uint16_t xml_stats_changed_count_{0};
  std::string xml_stats_changed_fields_{};
  XmlPollState xml_state_{XmlPollState::IDLE};
  uint32_t xml_deadline_ms_{0};
  uint32_t xml_next_action_ms_{0};
  bool xml_inflight_{false};
  DbTransactionOwner db_transaction_owner_{DbTransactionOwner::NONE};
  std::string xml_transaction_cmd_{};
  std::string xml_last_command_{};
  std::string xml_expected_prefix_{};
  std::string xml_rx_line_{};
  uint32_t xml_stats_capture_start_ms_{0};
  std::string xml_stats_reject_reason_{};
  std::string xml_stats_reject_decoded_{};
  bool xml_stats_rx_logged_{false};
  bool xml_stats_binary_response_{false};
  uint32_t xml_cycle_started_ms_{0};
  uint32_t xml_command_started_ms_{0};
  uint16_t xml_command_frames_{0};
  uint16_t xml_command_noise_frames_{0};
  uint8_t xml_tr32_page_{0};
  std::string xml_binary_probe_prev_tr32_payload_{};
  uint8_t xml_binary_probe_prev_tr32_page_{0};
  bool xml_binary_probe_has_prev_tr32_{false};
  bool xml_stats_locked_{false};
  bool xml_cycle_failed_{false};
  uint8_t xml_stats_consecutive_failures_{0};
  uint8_t xml_tr32_pages_ok_{0};
  bool xml_tg43_ok_{false};
  bool xml_tgc0_ok_{false};
  std::vector<uint8_t> xml_rx_buffer_{};
  bool xml_mapping_logged_{false};
  bool xml_mapping_loaded_{false};
  XmlMapping xml_mapping_{};
  Stats xml_stats_{};
  std::unordered_map<std::string, sensor::Sensor *> xml_sensors_{};
  struct Tgc0FilterState {
    std::deque<float> window;
    uint8_t consecutive_valid{0};
    bool logged_once{false};
  };
  std::unordered_map<std::string, Tgc0FilterState> tgc0_filters_{};
  std::unordered_map<std::string, XmlSensorMeta> xml_sensor_meta_{};
  std::unordered_map<std::string, bool> xml_missing_sensor_logged_{};
  std::unordered_map<std::string, bool> xml_unconfigured_sensor_logged_{};
  static constexpr size_t XML_COMMAND_COUNT = 3;
  std::array<uint8_t, XML_COMMAND_COUNT> xml_retry_count_{{0, 0, 0}};
  std::array<bool, XML_COMMAND_COUNT> xml_invalid_len_seen_{{false, false, false}};
  std::array<size_t, XML_COMMAND_COUNT> xml_last_invalid_len_{{0, 0, 0}};
  std::array<std::vector<uint8_t>, XML_COMMAND_COUNT> xml_counter_candidate_frame_{};
  std::array<uint8_t, XML_COMMAND_COUNT> xml_counter_candidate_count_{{0, 0, 0}};
  uint8_t xml_tgc0_timeout_streak_{0};
  bool xml_skip_tgc0_{false};
  void prepare_tgc0_request_();

  bool settings_boot_polled_{false};
  uint32_t settings_next_refresh_{0};
  uint32_t errors_next_poll_{0};
  sensor::Sensor *error_code_sensor_{nullptr};
  text_sensor::TextSensor *error_text_sensor_{nullptr};
  text_sensor::TextSensor *error_severity_sensor_{nullptr};
  binary_sensor::BinarySensor *error_active_sensor_{nullptr};
  std::unordered_map<std::string, sensor::Sensor *> setting_sensors_{};
  std::unordered_map<std::string, text_sensor::TextSensor *> setting_text_sensors_{};
  std::unordered_map<std::string, SettingDesc> setting_descs_{};
  bool settings_entities_created_{false};
  bool errors_entities_created_{false};
  uint32_t last_error_code_{0};
  std::vector<JuraDecodedField> last_decoded_fields_{};
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

class ManualStatusProbeAction : public esphome::Action<> {
 public:
  explicit ManualStatusProbeAction(JuraComponent *parent) : parent_(parent) {}
  void set_command(const std::string &command) { command_ = command; }
  void play() override { this->parent_->run_status_probe_command(command_); }

 protected:
  JuraComponent *parent_;
  std::string command_{};
};

class ManualHandshakeProbeAction : public esphome::Action<> {
 public:
  explicit ManualHandshakeProbeAction(JuraComponent *parent) : parent_(parent) {}
  void set_observe_ms(uint32_t observe_ms) { observe_ms_ = observe_ms; }
  void set_mode(const std::string &mode) { mode_ = mode; }
  void play() override { this->parent_->run_manual_handshake_probe(observe_ms_, mode_); }

 protected:
  JuraComponent *parent_;
  uint32_t observe_ms_{5000};
  std::string mode_{"normal"};
};

class ManualLiveTriggerProbeStayInBleAction : public esphome::Action<> {
 public:
  explicit ManualLiveTriggerProbeStayInBleAction(JuraComponent *parent) : parent_(parent) {}
  void set_observe_ms(uint32_t observe_ms) { observe_ms_ = observe_ms; }
  void set_interval_ms(uint32_t interval_ms) { interval_ms_ = interval_ms; }
  void play() override { this->parent_->manual_live_trigger_probe_stayinble(observe_ms_, interval_ms_); }

 protected:
  JuraComponent *parent_;
  uint32_t observe_ms_{30000};
  uint32_t interval_ms_{6000};
};

class ManualLiveEventObserveAction : public esphome::Action<> {
 public:
  explicit ManualLiveEventObserveAction(JuraComponent *parent) : parent_(parent) {}
  void set_observe_ms(uint32_t observe_ms) { observe_ms_ = observe_ms; }
  void set_stayinble(bool stayinble) { stayinble_ = stayinble; }
  void set_interval_ms(uint32_t interval_ms) { interval_ms_ = interval_ms; }
  void play() override { this->parent_->manual_live_event_observe(observe_ms_, stayinble_, interval_ms_); }

 protected:
  JuraComponent *parent_;
  uint32_t observe_ms_{120000};
  bool stayinble_{true};
  uint32_t interval_ms_{6000};
};

class ManualOriginalStartupObserveAction : public esphome::Action<> {
 public:
  explicit ManualOriginalStartupObserveAction(JuraComponent *parent) : parent_(parent) {}
  void set_observe_ms(uint32_t observe_ms) { observe_ms_ = observe_ms; }
  void set_respond_identity(bool respond_identity) { respond_identity_ = respond_identity; }
  void set_active_probe(bool active_probe) { active_probe_ = active_probe; }
  void set_boot_attached_mode(bool boot_attached_mode) { boot_attached_mode_ = boot_attached_mode; }
  void play() override {
    this->parent_->manual_original_startup_observe(observe_ms_, respond_identity_, active_probe_, boot_attached_mode_);
  }

 protected:
  JuraComponent *parent_;
  uint32_t observe_ms_{180000};
  bool respond_identity_{true};
  bool active_probe_{false};
  bool boot_attached_mode_{true};
};

class ManualOriginalStartupActiveSafeAction : public esphome::Action<> {
 public:
  explicit ManualOriginalStartupActiveSafeAction(JuraComponent *parent) : parent_(parent) {}
  void set_observe_ms(uint32_t observe_ms) { observe_ms_ = observe_ms; }
  void set_send_core_startup(bool send_core_startup) { send_core_startup_ = send_core_startup; }
  void set_respond_identity(bool respond_identity) { respond_identity_ = respond_identity; }
  void set_active_probe(bool active_probe) { active_probe_ = active_probe; }
  void play() override {
    this->parent_->manual_original_startup_active_safe(observe_ms_, send_core_startup_, respond_identity_,
                                                       active_probe_);
  }

 protected:
  JuraComponent *parent_;
  uint32_t observe_ms_{180000};
  bool send_core_startup_{true};
  bool respond_identity_{true};
  bool active_probe_{true};
};

class ManualOriginalStartupActiveStatefulAction : public esphome::Action<> {
 public:
  explicit ManualOriginalStartupActiveStatefulAction(JuraComponent *parent) : parent_(parent) {}
  void set_observe_ms(uint32_t observe_ms) { observe_ms_ = observe_ms; }
  void set_send_core_startup(bool send_core_startup) { send_core_startup_ = send_core_startup; }
  void set_respond_identity(bool respond_identity) { respond_identity_ = respond_identity; }
  void set_active_probe(bool active_probe) { active_probe_ = active_probe; }
  void play() override {
    this->parent_->manual_original_startup_active_stateful(observe_ms_, send_core_startup_, respond_identity_,
                                                           active_probe_);
  }

 protected:
  JuraComponent *parent_;
  uint32_t observe_ms_{180000};
  bool send_core_startup_{true};
  bool respond_identity_{true};
  bool active_probe_{true};
};

class Ble2TransportProbeAction : public esphome::Action<> {
 public:
  explicit Ble2TransportProbeAction(JuraComponent *parent) : parent_(parent) {}
  void set_probe(const std::string &probe) { probe_ = probe; }
  void play() override { this->parent_->run_ble2_transport_probe(probe_); }

 protected:
  JuraComponent *parent_;
  std::string probe_{};
};

template<typename... Ts> class SendDebugCommandAction : public esphome::Action<Ts...> {
 public:
  explicit SendDebugCommandAction(JuraComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, command)
  TEMPLATABLE_VALUE(std::string, transport)
  void play(Ts... x) override { this->parent_->run_debug_command(this->command_.value(x...), this->transport_.value(x...)); }

 protected:
  JuraComponent *parent_;
};

}  // namespace jutta_component
}  // namespace esphome
