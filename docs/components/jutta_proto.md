# Jutta Proto Component

The Jutta Proto component integrates the custom JURA protocol implementation with ESPHome. It establishes the UART handshake
with a JURA coffee maker and exposes convenient automation actions for brewing drinks via YAML.

## Configuration

```yaml
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
publish them as individual `text_sensor` entities so the values can easily be displayed in Home Assistant dashboards or used in
automations.

!!! tip
    Each entry inside the `machine_data` block must point to a text sensor that already exists. When referencing a sensor by ID
    (for example `raw: jura_machine_raw`) make sure to declare that sensor elsewhere in the YAML file. Otherwise ESPHome will
    raise an error such as `Couldn't find ID 'jura_machine_raw'` during compilation. You can either create the sensor ahead of
    time under the `text_sensor:` key or define it inline inside `machine_data` as shown below.

Add a `machine_data` section to the component configuration and register one or multiple text sensors. When a single
`text_sensor` entry is provided the raw XML payload is published. Alternatively, the individual sections can be configured to
receive pre-formatted text output parsed from the XML response. All sensors are optional and can be combined as needed. Each
entry can either declare a new sensor inline or reference an existing `text_sensor` via its `id`.

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

To publish the built-in set of machine data field sensors, enable `auto_fields`.
All known statistics, maintenance counters, maintenance percentages, and helper fields are
pre-registered up front and reuse the same entity names on every boot. Each sensor name starts with
`field_prefix` (if provided) followed by a human-readable description of the section and field. Omit
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
