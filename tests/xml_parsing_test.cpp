#include "esphome/components/jutta_proto/xml_settings.hpp"
#include "esphome/components/jutta_proto/xml_errors.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace esphome::jutta_component;

int main() {
  const std::string sample_xml =
      "<settings>"
      "<setting id=\"water_hardness\" name=\"Water Hardness\" unit=\"°dH\" source_cmd=\"@TM:02\" offset=\"0\" width=\"1\" scale=\"1\" type=\"u8\" />"
      "<setting id=\"brew_temp\" name=\"Brew Temp\" unit=\"°C\" source_cmd=\"@TM:09\" offset=\"1\" width=\"1\" scale=\"0.5\" type=\"u8\" />"
      "</settings>"
      "<errors source_cmd=\"@ER:00\">"
      "<error code=\"0\" text=\"OK\" severity=\"none\" />"
      "<error code=\"1\" text=\"Generic\" severity=\"warning\" />"
      "</errors>";


  bool settings_loaded = load_settings_from_xml(sample_xml);
  if (!settings_loaded) {
    std::cerr << "Settings block not parsed" << std::endl;
  }
  const auto &settings = get_settings();
  std::cout << "Settings parsed: " << settings.size() << std::endl;
  assert(settings.size() == 2);
  assert(settings[0].id == "water_hardness");
  assert(settings[1].id == "brew_temp");

  bool errors_loaded = load_errors_from_xml(sample_xml);
  if (!errors_loaded) {
    std::cerr << "Errors block not parsed" << std::endl;
  }
  const auto &errors = all_errors();
  std::cout << "Errors parsed: " << errors.size() << std::endl;
  assert(errors.size() == 2);
  assert(error_source_command() == "@ER:00");
  const ErrorDesc *err = find_error(1);
  assert(err != nullptr);
  assert(err->text == "Generic");
  assert(err->severity == "warning");

  // Simple decode emulation for the settings.
  const SettingDesc &hardness = settings[0];
  std::vector<uint8_t> payload = {0x05, 0x02};  // hardness=5, temp raw=0x02 -> 1°C after scaling 0.5
  std::uint64_t raw_hardness = payload[hardness.offset];
  float scaled_hardness = static_cast<float>(raw_hardness) * hardness.scale;
  assert(scaled_hardness == 5.0f);

  const SettingDesc &temp = settings[1];
  std::uint64_t raw_temp = payload[temp.offset];
  float scaled_temp = static_cast<float>(raw_temp) * temp.scale;
  assert(scaled_temp == 1.0f);

  std::cout << "XML settings/errors tests passed." << std::endl;
  return 0;
}

