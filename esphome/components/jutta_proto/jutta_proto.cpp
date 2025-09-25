#include "esphome/components/jutta_proto/jutta_proto.h"

#include <cctype>
#include <iomanip>
#include <sstream>
#include <utility>

#include "esphome/core/time.h"

namespace esphome {
namespace jutta_component {

namespace {

static const char *const TAG = "jutta_proto";

constexpr size_t HANDSHAKE_LOG_PREVIEW_LIMIT = 64;

const char *handshake_stage_name(JuraComponent::HandshakeStage stage) {
  switch (stage) {
    case JuraComponent::HandshakeStage::IDLE:
      return "idle";
    case JuraComponent::HandshakeStage::HELLO:
      return "hello";
    case JuraComponent::HandshakeStage::SEND_T1:
      return "send_t1";
    case JuraComponent::HandshakeStage::WAIT_T2:
      return "wait_t2";
    case JuraComponent::HandshakeStage::SEND_T2:
      return "send_t2";
    case JuraComponent::HandshakeStage::WAIT_T3:
      return "wait_t3";
    case JuraComponent::HandshakeStage::SEND_T3:
      return "send_t3";
    case JuraComponent::HandshakeStage::DONE:
      return "done";
    case JuraComponent::HandshakeStage::FAILED:
      return "failed";
  }
  return "unknown";
}

std::string format_printable_char(uint8_t byte) {
  switch (byte) {
    case '\\r':
      return "\\r";
    case '\\n':
      return "\\n";
    case '\\t':
      return "\\t";
    default:
      break;
  }
  if (std::isprint(static_cast<int>(byte)) != 0) {
    return std::string(1, static_cast<char>(byte));
  }
  std::ostringstream stream;
  stream << "\\x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
         << static_cast<int>(byte);
  return stream.str();
}

std::string format_printable_string(const std::string &value) {
  std::ostringstream stream;
  for (unsigned char c : value) {
    stream << format_printable_char(c);
  }
  return stream.str();
}

std::string format_hex_string(const std::string &value) {
  if (value.empty()) {
    return "[]";
  }
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < value.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
           << static_cast<int>(static_cast<unsigned char>(value[i]));
  }
  stream << "]";
  return stream.str();
}

std::string format_buffer_preview(const std::string &value) {
  if (value.size() <= HANDSHAKE_LOG_PREVIEW_LIMIT) {
    return format_printable_string(value);
  }
  std::string suffix = value.substr(value.size() - HANDSHAKE_LOG_PREVIEW_LIMIT);
  return std::string("...") + format_printable_string(suffix);
}

std::string format_buffer_hex_preview(const std::string &value) {
  if (value.size() <= HANDSHAKE_LOG_PREVIEW_LIMIT) {
    return format_hex_string(value);
  }
  std::string suffix = value.substr(value.size() - HANDSHAKE_LOG_PREVIEW_LIMIT);
  std::string formatted_suffix = format_hex_string(suffix);
  if (formatted_suffix.size() > 1) {
    return std::string("...") + formatted_suffix;
  }
  return formatted_suffix;
}

}  // namespace

void JuraComponent::setup() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not configured for JUTTA Proto component.");
    this->mark_failed();
    return;
  }

  this->connection_ = std::make_unique<::jutta_proto::JuttaConnection>(this->parent_);
  this->connection_->init();

  this->handshake_stage_ = HandshakeStage::HELLO;
  ESP_LOGI(TAG, "Starting handshake with coffee maker...");
}

void JuraComponent::loop() {
  if (this->handshake_stage_ != this->last_logged_stage_) {
    ESP_LOGI(TAG, "Handshake stage changed: %s -> %s (buffer size=%zu, preview='%s', hex %s)",
             handshake_stage_name(this->last_logged_stage_),
             handshake_stage_name(this->handshake_stage_), this->handshake_buffer_.size(),
             format_buffer_preview(this->handshake_buffer_).c_str(),
             format_buffer_hex_preview(this->handshake_buffer_).c_str());
    this->last_logged_stage_ = this->handshake_stage_;
  }

  if (this->connection_ != nullptr && this->handshake_stage_ != HandshakeStage::DONE &&
      this->handshake_stage_ != HandshakeStage::FAILED) {
    this->process_handshake();
  }

  if (this->coffee_maker_ != nullptr) {
    this->coffee_maker_->loop();
    if (!this->coffee_maker_->is_locked()) {
      this->custom_cancel_flag_ = false;
    }
  }
}

void JuraComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "JUTTA Proto");
  if (!this->device_type_.empty()) {
    ESP_LOGCONFIG(TAG, "  Detected device: %s", this->device_type_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Detected device: (pending)");
  }

  const char *state = "unknown";
  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      state = "idle";
      break;
    case HandshakeStage::HELLO:
      state = "awaiting type";
      break;
    case HandshakeStage::SEND_T1:
      state = "waiting for @t1";
      break;
    case HandshakeStage::WAIT_T2:
      state = "waiting for @T2";
      break;
    case HandshakeStage::SEND_T2:
      state = "sending @t2";
      break;
    case HandshakeStage::WAIT_T3:
      state = "waiting for @T3";
      break;
    case HandshakeStage::SEND_T3:
      state = "sending @t3";
      break;
    case HandshakeStage::DONE:
      state = "ready";
      break;
    case HandshakeStage::FAILED:
      state = "failed";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Handshake state: %s", state);

  if (!this->handshake_t2_response_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last key exchange T2: %s", this->handshake_t2_response_.c_str());
  }
  if (!this->handshake_t3_response_.empty()) {
    ESP_LOGCONFIG(TAG, "  Last key exchange T3: %s", this->handshake_t3_response_.c_str());
  }

  if (this->coffee_maker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(true));
  } else {
    ESP_LOGCONFIG(TAG, "  Coffee maker ready: %s", YESNO(false));
  }
}

void JuraComponent::process_handshake() {
  using ::jutta_proto::JuttaConnection;
  using ::jutta_proto::JUTTA_GET_TYPE;

  switch (this->handshake_stage_) {
    case HandshakeStage::IDLE:
      break;
    case HandshakeStage::HELLO: {
      ESP_LOGD(TAG, "HELLO: requesting device type with payload '%s' (hex %s).",
               format_printable_string(JUTTA_GET_TYPE).c_str(),
               format_hex_string(JUTTA_GET_TYPE).c_str());
      auto response = this->connection_->write_decoded_with_response(JUTTA_GET_TYPE, std::chrono::milliseconds{1000});
      if (response != nullptr) {
        this->device_type_ = *response;
        ESP_LOGI(TAG, "Detected coffee maker response: %s", this->device_type_.c_str());
        this->handshake_buffer_.clear();
        this->handshake_stage_ = HandshakeStage::SEND_T1;
      }
      break;
    }
    case HandshakeStage::SEND_T1: {
      ESP_LOGD(TAG, "SEND_T1: writing '@T1\\r\\n' and waiting for '@t1\\r\\n' (timeout=1000 ms).");
      auto wait_result = this->connection_->write_decoded_wait_for("@T1\r\n", "@t1\r\n", std::chrono::milliseconds{1000});
      if (wait_result == JuttaConnection::WaitResult::Success) {
        ESP_LOGD(TAG, "Received @t1 acknowledgment.");
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
        this->handshake_stage_ = HandshakeStage::WAIT_T2;
      } else if (wait_result == JuttaConnection::WaitResult::Timeout) {
        this->restart_handshake("timeout waiting for @t1");
      } else if (wait_result == JuttaConnection::WaitResult::Error) {
        this->restart_handshake("failed to send @T1");
      }
      break;
    }
    case HandshakeStage::WAIT_T2: {
      if (this->handshake_deadline_ == 0) {
        this->handshake_deadline_ = esphome::millis() + 5000;
        ESP_LOGD(TAG, "WAIT_T2: started response timer (deadline in 5000 ms).");
      }
      bool any = this->read_handshake_bytes();
      if (any) {
        auto pos = this->handshake_buffer_.find("@T2");
        if (pos != std::string::npos) {
          auto end = this->handshake_buffer_.find("\r\n", pos);
          if (end != std::string::npos) {
            this->handshake_t2_response_ = this->handshake_buffer_.substr(pos, end - pos);
          } else {
            this->handshake_t2_response_ = this->handshake_buffer_.substr(pos);
          }
          ESP_LOGD(TAG, "Received %s", this->handshake_t2_response_.c_str());
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = 0;
          this->handshake_stage_ = HandshakeStage::SEND_T2;
        }
      }
      if (this->handshake_deadline_ != 0 && time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for @T2");
      }
      break;
    }
    case HandshakeStage::SEND_T2: {
      ESP_LOGD(TAG, "SEND_T2: sending '@t2:8120000000\\r\\n'.");
      if (this->connection_->write_decoded("@t2:8120000000\r\n")) {
        ESP_LOGD(TAG, "Sent @t2 response.");
        this->handshake_stage_ = HandshakeStage::WAIT_T3;
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
      } else {
        this->restart_handshake("failed to send @t2");
      }
      break;
    }
    case HandshakeStage::WAIT_T3: {
      if (this->handshake_deadline_ == 0) {
        this->handshake_deadline_ = esphome::millis() + 5000;
        ESP_LOGD(TAG, "WAIT_T3: started response timer (deadline in 5000 ms).");
      }
      bool any = this->read_handshake_bytes();
      if (any) {
        auto pos = this->handshake_buffer_.find("@T3");
        if (pos != std::string::npos) {
          auto end = this->handshake_buffer_.find("\r\n", pos);
          if (end != std::string::npos) {
            this->handshake_t3_response_ = this->handshake_buffer_.substr(pos, end - pos);
          } else {
            this->handshake_t3_response_ = this->handshake_buffer_.substr(pos);
          }
          ESP_LOGD(TAG, "Received %s", this->handshake_t3_response_.c_str());
          this->handshake_buffer_.clear();
          this->handshake_deadline_ = 0;
          this->handshake_stage_ = HandshakeStage::SEND_T3;
        }
      }
      if (this->handshake_deadline_ != 0 && time_reached(esphome::millis(), this->handshake_deadline_)) {
        this->restart_handshake("timeout waiting for @T3");
      }
      break;
    }
    case HandshakeStage::SEND_T3: {
      ESP_LOGD(TAG, "SEND_T3: sending '@t3\\r\\n' to finish handshake.");
      if (this->connection_->write_decoded("@t3\r\n")) {
        ESP_LOGI(TAG, "Handshake finished successfully.");
        this->handshake_stage_ = HandshakeStage::DONE;
        this->handshake_buffer_.clear();
        this->handshake_deadline_ = 0;
      } else {
        this->restart_handshake("failed to send @t3");
      }
      break;
    }
    case HandshakeStage::DONE:
    case HandshakeStage::FAILED:
      break;
  }

  if (this->handshake_stage_ == HandshakeStage::DONE && this->connection_ != nullptr &&
      this->coffee_maker_ == nullptr) {
    auto connection = std::move(this->connection_);
    this->coffee_maker_ = std::make_unique<::jutta_proto::CoffeeMaker>(std::move(connection));
    ESP_LOGI(TAG, "Coffee maker controller initialized.");
  }
}

void JuraComponent::restart_handshake(const char *reason) {
  if (reason != nullptr) {
    ESP_LOGW(TAG, "Restarting handshake: %s", reason);
  }
  this->handshake_buffer_.clear();
  this->handshake_deadline_ = 0;
  this->handshake_stage_ = HandshakeStage::HELLO;
  this->last_logged_stage_ = HandshakeStage::FAILED;
}

bool JuraComponent::read_handshake_bytes() {
  if (this->connection_ == nullptr) {
    return false;
  }
  bool read_any = false;
  uint8_t byte = 0;
  while (this->connection_->read_decoded(&byte)) {
    read_any = true;
    this->handshake_buffer_.push_back(static_cast<char>(byte));
    if (this->handshake_buffer_.size() > 128) {
      this->handshake_buffer_.erase(0, this->handshake_buffer_.size() - 128);
    }
    ESP_LOGV(TAG,
             "Handshake buffered byte: '%s' (0x%02X); buffer size=%zu; buffer now '%s' (hex %s)",
             format_printable_char(byte).c_str(), static_cast<unsigned int>(byte),
             this->handshake_buffer_.size(),
             format_buffer_preview(this->handshake_buffer_).c_str(),
             format_buffer_hex_preview(this->handshake_buffer_).c_str());
  }
  return read_any;
}

bool JuraComponent::time_reached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

void JuraComponent::start_brew(::jutta_proto::CoffeeMaker::coffee_t coffee) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot start brew - component not ready.");
    return;
  }
  this->coffee_maker_->brew_coffee(coffee);
}

void JuraComponent::start_custom_brew(uint32_t grind_duration_ms, uint32_t water_duration_ms) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot brew custom coffee - component not ready.");
    return;
  }
  this->custom_cancel_flag_ = false;
  this->coffee_maker_->brew_custom_coffee(&this->custom_cancel_flag_, std::chrono::milliseconds{grind_duration_ms},
                                          std::chrono::milliseconds{water_duration_ms});
}

void JuraComponent::cancel_custom_brew() {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot cancel custom brew - component not ready.");
    return;
  }
  if (!this->custom_cancel_flag_) {
    ESP_LOGI(TAG, "Cancelling custom brew.");
  }
  this->custom_cancel_flag_ = true;
}

void JuraComponent::switch_page(uint32_t page) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Cannot switch page - component not ready.");
    return;
  }
  this->coffee_maker_->switch_page(page);
}

bool JuraComponent::is_busy() const {
  if (this->coffee_maker_ == nullptr) {
    return false;
  }
  return this->coffee_maker_->is_locked();
}

}  // namespace jutta_component
}  // namespace esphome

