# Jutta Proto Component

The Jutta Proto component integrates the custom JURA protocol implementation with ESPHome. It establishes the UART handshake
with a JURA coffee maker and exposes convenient automation actions for brewing drinks via YAML.

## Configuration

```yaml
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap:
    ssid: "JURA Coffee Maker Fallback"
    password: !secret wifi_ap_password

api:

ota:

status_led:
  pin: GPIO2

uart:
  id: jura_uart
  tx_pin: 17
  rx_pin: 16
  baud_rate: 9600
  parity: NONE
  stop_bits: 1

jutta_proto:
  id: jura
  uart_id: jura_uart
```

The component takes care of the handshake during startup. Once the handshake finishes, all brewing actions become available.

!!! note
    The example above includes the standard ESPHome `wifi`, `api`, and `ota` blocks so Home Assistant can connect to the device and you can perform wireless updates. Replace the Wi-Fi credentials with values that match your network (ideally using the `secrets.yaml` file) before flashing the firmware. Define `wifi_ap_password` in `secrets.yaml` if you keep the fallback access point enabled.

!!! tip
    Provision the device close to your wireless access point and keep the optional fallback access point (`wifi.ap`) enabled until the node shows up in Home Assistant. The fallback AP lets you connect directly to the ESP32 and update the Wi-Fi credentials if it cannot reach your network yet.

## Automation Actions

Use the registered actions inside automations or button handlers. When only one `jutta_proto` component is configured, the
`id` argument can be omitted.

### Start a predefined recipe

```yaml
button:
  - platform: template
    name: "Brew Espresso"
    on_press:
      - jutta_proto.start_brew:
          coffee: espresso
```

Available options for `coffee` are `espresso`, `coffee`, `cappuccino`, `milk_foam`, `hot_water`, `caffe_barista`, `lungo_barista`,
`espresso_doppio`, `macchiato`, `two_espresso` (alias `two_espressi`), and `two_coffee` (alias `two_coffees`).

### Brew with custom timing

```yaml
script:
  - id: brew_lungo
    mode: restart
    then:
      - jutta_proto.custom_brew:
          id: jura
          grind_duration: 4s
          water_duration: 45s
```

### Cancel an ongoing custom brew

```yaml
switch:
  - platform: template
    name: "Cancel Brew"
    turn_on_action:
      - jutta_proto.cancel_custom_brew: jura
```

### Switch between front panel pages

```yaml
script:
  - id: jura_next_page
    then:
      - jutta_proto.switch_page:
          id: jura
          page: 1
```

### Run a manual command sequence

```yaml
script:
  - id: brew_manual_recipe
    mode: restart
    then:
      - jutta_proto.run_sequence:
          id: jura
          sequence:
            - command: grinder_on
              description: "Grind on"
            - delay: 3s
              description: "Let the grinder run"
            - command: grinder_off
            - command: brew_group_to_brewing_position
            - command: coffee_press_on
            - delay: 500ms
              description: "Compress the coffee"
            - command: coffee_press_off
            - command: water_heater_on
            - command: water_pump_on
            - delay: 2s
              description: "Pre-brew"
            - command: water_pump_off
            - command: water_heater_off
            - delay: 2s
            - command: water_heater_on
            - command: water_pump_on
            - delay: 40s
              description: "Dispense water"
            - command: water_pump_off
            - command: water_heater_off
            - command: brew_group_reset
```

Use `raw` instead of `command` when you need to send a custom UART command string. Raw commands automatically append `\r\n` if it is missing.

## Machine Data Sensors

The Jura firmware exposes several machine data blocks (statistics, errors, status, and running processes). The component can
publish them as individual sensors so the values can easily be displayed in Home Assistant dashboards or used in automations.
Text sensors are still supported for the raw XML payload and the formatted section summaries, while all numeric fields can be
exposed as regular `sensor` entities.

!!! tip
    Each entry inside the `machine_data` block must point to a text sensor that already exists. When referencing a sensor by ID
    (for example `raw: jura_machine_raw`) make sure to declare that sensor elsewhere in the YAML file. Otherwise ESPHome will
    raise an error such as `Couldn't find ID 'jura_machine_raw'` during compilation. You can either create the sensor ahead of
    time under the `text_sensor:` key or define it inline inside `machine_data` as shown below.

Add a `machine_data` section to the component configuration and register one or multiple sensors. When a single `text_sensor`
entry is provided the raw XML payload is published. Alternatively, the individual sections can be configured to receive
pre-formatted text output parsed from the XML response. All sensors are optional and can be combined as needed. Each entry can
either declare a new sensor inline or reference an existing `text_sensor` via its `id`.

```yaml
text_sensor:
  - platform: template
    name: "JURA Machine Statistics"
    id: jura_stats
  - platform: template
    name: "JURA Machine Errors"
    id: jura_errors
  - platform: template
    name: "JURA Machine Status"
    id: jura_status
  - platform: template
    name: "JURA Machine Processes"
    id: jura_processes
  - platform: template
    name: "JURA Machine Data Raw"
    id: jura_machine_raw

jutta_proto:
  id: jura
  uart_id: jura_uart
  machine_data:
    statistics: jura_stats
    errors: jura_errors
    status: jura_status
    processes: jura_processes
    raw: jura_machine_raw

## Troubleshooting API Connectivity

If Home Assistant still shows `Can't connect to ESPHome API` after flashing the firmware, try the following diagnostics:

1. **Verify the node is on Wi-Fi.** Connect the ESP32 over USB and run `esphome logs <config.yaml>` to check for `WiFi connected` and `API server listening` messages. If the device repeatedly retries Wi-Fi, double-check the SSID and password or temporarily disable `fast_connect` if you are using it with a visible network.
2. **Check the IP address.** Look for `Got IP` in the serial logs and confirm that the reported address matches what Home Assistant is trying to reach. You can also ping the IP from the Home Assistant host to confirm connectivity.
3. **Match API credentials.** If you enable API encryption or set an API password, make sure the same key/password is configured in Home Assistant (`Configuration → Integrations → ESPHome → Configure`). A mismatch will result in connection failures even though the TCP port is reachable.
4. **Inspect firewalls.** Ensure port 6053 is open between Home Assistant and the ESP32. Some routers or VLAN setups block device-to-device communication by default.

The device should stay online as long as it maintains Wi-Fi connectivity. Use the optional `status_led` block from the sample configuration to see the connection state without opening the logs (slow blink = Wi-Fi only, fast blink = connecting, solid = API connected).

To publish the built-in set of machine data field sensors, enable `auto_fields`. All known statistics, maintenance counters,
maintenance percentages, and helper fields are pre-registered up front and reuse the same entity names on every boot. Numeric
values (for example drink counters and percentage indicators) are exposed as regular sensors with the proper unit of
measurement, while helper fields such as the header, decoded text, and raw hexadecimal payload remain text sensors. Each
sensor name starts with `field_prefix` (if provided) followed by a human-readable description of the section and field. Omit
`field_prefix` to keep the built-in names without an additional prefix:

```yaml
text_sensor:
  - platform: template
    name: "JURA Machine Data Raw"
    id: jura_machine_raw

jutta_proto:
  id: jura
  uart_id: jura_uart
  machine_data:
    raw: jura_machine_raw
    auto_fields: true
    field_prefix: "JURA Machine"

```

To reuse existing template sensors instead of the auto-generated entities, map specific machine data fields to sensor IDs with
the `fields` option. The keys are dot-separated paths that use the slugified section, element, and field names (for example
`statistic.maintenancecounter.cappuclean`). Each segment is case-insensitive and ignores whitespace, so you can enter either the
original uppercase identifiers from the JURA XML or the lowercase slug equivalents. Only the mapped fields are redirected to the
provided sensors—any remaining fields continue to use the automatically created entities. Use the nested `sensor:` key to map a
numeric field to a regular sensor and `text_sensor:` for text values.

```yaml
sensor:
  - platform: template
    name: "${devicename} Cleaning Cycles"
    id: stat_counter_cleaning
    state_class: total_increasing
  - platform: template
    name: "${devicename} Cleaning %"
    id: stat_percent_cleaning
    unit_of_measurement: "%"
    accuracy_decimals: 1
    state_class: measurement
  - platform: template
    name: "${devicename} Cleaning Raw"
    id: stat_percent_cleaning_raw
    state_class: measurement

text_sensor:
  - platform: template
    name: "${devicename} Maintenance Header"
    id: stat_counter_header
  - platform: template
    name: "${devicename} Maintenance Text"
    id: stat_counter_text

jutta_proto:
  id: jura
  uart_id: jura_uart
  machine_data:
    auto_fields: true
    fields:
      statistic.maintenancecounter.cleaning:
        sensor: stat_counter_cleaning
      statistic.maintenancecounter.header:
        text_sensor: stat_counter_header
      statistic.maintenancecounter.text:
        text_sensor: stat_counter_text
      statistic.maintenancepercent.cleaning_percent:
        sensor: stat_percent_cleaning
      statistic.maintenancepercent.cleaning_raw:
        sensor: stat_percent_cleaning_raw
```

Inline definition example:

```yaml
jutta_proto:
  id: jura
  uart_id: jura_uart
  machine_data:
    raw:
      id: jura_machine_raw
      name: "JURA Machine Data Raw"
    auto_fields: true
    field_prefix: "JURA Machine"
```

With `auto_fields` enabled, the component publishes sensors for every pre-defined field in the
machine data response (for example, individual product counters or maintenance percentages). The
sensors update automatically whenever the coffee maker reports new values. Fields that are not
returned by the machine stay present as entities but keep an empty state instead of being created or
removed dynamically.

!!! note
    The repository includes a `joe_codes_example.xml` file under `jura_joe_xml_bundle_final/`. Copy it to your ESPHome config
    folder and import it with `!include` if you want quick access to the original J.O.E. machine data layout while testing the
    parsed sensor output.

Each formatted sensor contains a human-readable rendering of its XML section. The parser supports the different response
variants that have been observed in the official firmware so mixed content and nested tags are handled automatically. If only
the `raw` sensor is configured the untouched XML payload is published instead.

## Diagnostics

The component logs handshake progress during startup. The `dump_config()` output lists the detected machine type as well as the
latest key exchange messages, which can help troubleshoot UART or wiring issues.
