// external_components/jura/jura.cpp
#include "jura.h"
#include "esphome/core/log.h"
#include <cctype>
#include <sstream>
#include <iomanip>

namespace esphome {
namespace jura {

static const char *const TAG = "jura";

// constructor
Jura::Jura(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

// lifecycle
void Jura::setup() {
  // ensure UART parameters — use fully qualified enum
  this->check_uart_settings(9600, 1, esphome::uart::UART_CONFIG_PARITY_NONE, 8);
  rxbuf_.clear();
  ESP_LOGD(TAG, "Jura setup; jutta_gap_ms=%d", this->jutta_gap_ms_);
}

void Jura::dump_config() {
  ESP_LOGCONFIG(TAG, "Jura component:");
  ESP_LOGCONFIG(TAG, "  Baud: 9600, 8N1");
  ESP_LOGCONFIG(TAG, "  jutta_gap_ms: %d ms", this->jutta_gap_ms_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %.0f s", this->get_update_interval() / 1000.0f);
}

void Jura::update() {
  unsigned long now = millis();
  if (now - this->last_heartbeat_ts_ > this->get_update_interval()) {
    this->last_heartbeat_ts_ = now;
    ESP_LOGD(TAG, "Heartbeat: sending TY:");
    this->send_command("TY:");  // heartbeat / status probe as per Jutta doc
  }
  // also try to parse incoming if any
  while (parse_incoming()) { /* drain */ }
}

// --- obfuscation (cmd2jura.ino style) ---
std::array<uint8_t,4> Jura::obfuscate_byte(uint8_t b) {
  std::array<uint8_t,4> out;
  out.fill(0xFF);
  // For each group of two bits (LSB-first as in .ino): s=0,2,4,6 -> enc[0..3]
  for (int g = 0; g < 4; ++g) {
    int bit0 = (b >> (g * 2 + 0)) & 0x1;
    int bit1 = (b >> (g * 2 + 1)) & 0x1;
    if (!bit0) out[g] &= ~(1 << 2);  // clear bit 2 if payload bit==0
    if (!bit1) out[g] &= ~(1 << 5);  // clear bit 5 if payload bit==0
  }
  return out;
}

uint8_t Jura::deobfuscate_4bytes(const std::array<uint8_t,4> &enc) {
  uint8_t out = 0;
  // enc[0] -> bits 0,1 ; enc[1] -> bits 2,3 ; enc[2] -> 4,5 ; enc[3] -> 6,7
  for (int i = 0; i < 4; ++i) {
    uint8_t b0 = (enc[i] >> 2) & 0x1;
    uint8_t b1 = (enc[i] >> 5) & 0x1;
    out |= (b0 << (i * 2 + 0));
    out |= (b1 << (i * 2 + 1));
  }
  return out;
}

// --- sending ---
bool Jura::send_raw(const std::vector<uint8_t> &raw) {
  for (auto v : raw) this->write(v);
  this->flush();
  ESP_LOGD(TAG, "Sent %d raw bytes", (int)raw.size());
  return true;
}

bool Jura::send_command(const std::string &cmd) {
  ESP_LOGD(TAG, "send_command: '%s'", cmd.c_str());

  // clear RX buffer before sending
  while (this->available()) {
    uint8_t tmp;
    if (!this->read_byte(&tmp)) break;
  }

  std::string with_term = cmd + "\r\n";
  std::vector<uint8_t> out;
  out.reserve(with_term.size() * 4);

  // obfuscate and send
  for (unsigned char ch : with_term) {
    auto enc = obfuscate_byte((uint8_t)ch);
    out.push_back(enc[0]);
    out.push_back(enc[1]);
    out.push_back(enc[2]);
    out.push_back(enc[3]);

    // flush after each char to mimic original timing
    for (int i = 0; i < 4; ++i) this->write(enc[i]);
    this->flush();
    if (this->jutta_gap_ms_ > 0) delay(this->jutta_gap_ms_);
  }

  ESP_LOGD(TAG, "Sent obfuscated command (%zu bytes)", out.size());

  // read response (assemble from incoming raw bytes -> decoded chars)
  std::string decoded;
  int timeout = 0;
  uint8_t collect = 0;
  int bits_filled = 0;

  // We'll push raw bytes into rxbuf_ until a CRLF decoded is found OR timeout
  unsigned long start = millis();
  while ((millis() - start) < 2000) {  // 2s total wait (adjustable)
    // drain any immediate bytes into rxbuf_ (they may have been pushed by ISR)
    while (this->available()) {
      uint8_t b;
      if (!this->read_byte(&b)) break;
      rxbuf_.push_back(b);
    }

    // try to decode as much as possible
    while (rxbuf_.size() >= 4) {
      std::array<uint8_t,4> enc{ rxbuf_[0], rxbuf_[1], rxbuf_[2], rxbuf_[3] };
      uint8_t ch = deobfuscate_4bytes(enc);
      decoded.push_back((char)ch);
      // consume 4 bytes
      rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + 4);

      // if CRLF detected -> done
      size_t n = decoded.size();
      if (n >= 2 && decoded[n-2] == '\r' && decoded[n-1] == '\n') {
        // remove CRLF and finish
        std::string msg = decoded.substr(0, decoded.size() - 2);
        ESP_LOGI(TAG, "Received message: %s", msg.c_str());
        handle_full_message_(msg);
        last_recv_ = millis();
        return true;
      }

      // safety guard on decoded length
      if (decoded.size() > 512) {
        ESP_LOGW(TAG, "Decoded message too long, flushing");
        decoded.clear();
        rxbuf_.clear();
        return false;
      }
    }
    delay(10);
  }

  ESP_LOGW(TAG, "No complete response within timeout after sending '%s'", cmd.c_str());
  // optional: if there's a partial decoded string, publish as last message
  if (!decoded.empty()) {
    std::string msg = decoded;
    ESP_LOGD(TAG, "Partial decoded: %s", msg.c_str());
    handle_full_message_(msg);
  }
  return false;
}

// --- parsing inbound continuous (called from loop/update) ---
bool Jura::parse_incoming() {
  // This is used when bytes may arrive independently (we already push bytes to rxbuf_ in send_command read phase)
  if (rxbuf_.size() < 4) return false;

  std::vector<uint8_t> decoded;
  size_t i = 0;
  while (i + 3 < rxbuf_.size()) {
    std::array<uint8_t,4> enc{ rxbuf_[i], rxbuf_[i+1], rxbuf_[i+2], rxbuf_[i+3] };
    uint8_t dec = deobfuscate_4bytes(enc);
    decoded.push_back(dec);
    i += 4;
    size_t n = decoded.size();
    if (n >= 2 && decoded[n-2] == '\r' && decoded[n-1] == '\n') {
      std::string msg(reinterpret_cast<char*>(decoded.data()), decoded.size());
      if (msg.size() >= 2) msg = msg.substr(0, msg.size()-2);
      ESP_LOGI(TAG, "Received message (background): %s", msg.c_str());
      rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + i);
      handle_full_message_(msg);
      last_recv_ = millis();
      return true;
    }
  }
  // cap buffer
  if (rxbuf_.size() > 2048) {
    size_t keep = 128;
    if (keep % 4) keep = (keep/4)*4;
    rxbuf_.erase(rxbuf_.begin(), rxbuf_.end() - keep);
    ESP_LOGW(TAG, "RX buffer trimmed to last %u bytes", (unsigned)keep);
  }
  return false;
}

// --- handle a complete, decoded message: publish sensors, do heuristics ---
void Jura::handle_full_message_(const std::string &msg) {
  if (this->last_message_sensor_) this->last_message_sensor_->publish_state(msg);

  // Heuristic: total counter tokens in XML were @TR:32 etc; detect numeric tokens like "@TR:" "TR:" "TC:"
  const std::vector<std::string> prefixes = {"@TR:", "TR:", "TC:", "@TS:", "TS:"};
  for (auto &p : prefixes) {
    auto pos = msg.find(p);
    if (pos != std::string::npos && pos + p.size() < msg.size()) {
      // extract following contiguous hex or decimal digits
      size_t i = pos + p.size();
      // try decimal first
      int val = 0;
      bool dec_found = false;
      while (i < msg.size() && isdigit((unsigned char)msg[i])) {
        dec_found = true;
        val = val * 10 + (msg[i] - '0');
        ++i;
      }
      if (dec_found && this->total_counter_sensor_) {
        this->total_counter_sensor_->publish_state((float)val);
        return;
      }
      // try hex
      i = pos + p.size();
      int hexval = 0;
      bool hex_found = false;
      while (i < msg.size() && isxdigit((unsigned char)msg[i])) {
        hex_found = true;
        char ch = msg[i];
        int v = (ch >= '0' && ch <= '9') ? ch - '0' : ( (ch>='a' && ch<='f') ? 10 + ch - 'a' : 10 + ch - 'A');
        hexval = hexval * 16 + v;
        ++i;
      }
      if (hex_found && this->total_counter_sensor_) {
        this->total_counter_sensor_->publish_state((float)hexval);
        return;
      }
    }
  }
  // no counter found → nothing else
}

// --- public brew helpers ---
bool Jura::brew_raw(const std::string &hex_code) {
  // expects e.g. "02" -> constructs PR:02
  std::string cmd = "PR:" + hex_code;
  return send_command(cmd);
}

bool Jura::brew_product_by_code(uint8_t code) {
  char buf[8];
  snprintf(buf, sizeof(buf), "PR:%02X", code);
  return send_command(std::string(buf));
}

bool Jura::brew_by_name(const std::string &name) {
  auto it = this->product_name_to_code_.find(name);
  if (it == this->product_name_to_code_.end()) {
    ESP_LOGW(TAG, "Unknown product name: %s", name.c_str());
    return false;
  }
  return brew_product_by_code(it->second);
}

void Jura::power_on()  { send_command("AN:01"); }
void Jura::power_off() { send_command("AN:02"); }

}  // namespace jura
}  // namespace esphome
