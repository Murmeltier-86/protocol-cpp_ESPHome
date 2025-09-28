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


text_sensor:
  - platform: jutta_proto
    jutta_proto_id: jura

```

The component takes care of the handshake during startup. Once the handshake finishes, all brewing actions become available.

### Machine data text sensor


Add a `text_sensor` entry using the `jutta_proto` platform to expose the raw status response published by the coffee maker. The
component automatically polls the `&STAT?` command in the background and publishes the trimmed response whenever it changes.

```yaml
text_sensor:
  - platform: jutta_proto
    name: "Machine state"
```

When multiple `jutta_proto` components are present, set `jutta_proto_id` to specify which controller should publish data through
the text sensor.


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

## Diagnostics

The component logs handshake progress during startup. The `dump_config()` output lists the detected machine type as well as the
latest key exchange messages, which can help troubleshoot UART or wiring issues.
