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
  jutta_xml_enable: true           # optional, default false
  jutta_xml_poll_ms: 30000         # optional, poll interval in ms
  jutta_xml_rx_timeout_ms: 900     # optional, frame timeout in ms
  xml_path: "/data/jura_machine.xml"   # optional, override XML mapping path
```

The component takes care of the handshake during startup. Once the handshake finishes, all brewing actions become available.

When `jutta_xml_enable` is set to `true`, the component periodically polls the JURA statistics interface using DB framing.
During initialization it scans the mapping file (default `/data/jura_machine.xml`, configurable via `xml_path`, typically a
J.O.E. export) to discover the exact command strings and optional labels for the exposed statistics blocks. If the file is
missing or malformed the defaults `@TR:32`, `@TG:43`, and `@TG:C0` are used and the sensors keep their generic names.

### XML statistics sensors

Enabling XML polling registers fixed pools of numeric sensors that Home Assistant can consume directly. Every value is
published only after a complete, valid DB frame was received for the respective block; failed cycles publish nothing.

| Block  | Sensor count | Value width | Default names     |
| ------ | ------------- | ----------- | ----------------- |
| TR32   | 10            | 16-bit      | `TR32 1` … `TR32 10` |
| TG43   | 6             | 16-bit      | `TG43 1` … `TG43 6`  |
| TG:C0  | 3             | 32-bit      | `TGC0 1` … `TGC0 3`  |

If the XML mapping provides labels for a block, those replace the defaults in the same order. All sensors report integers
with `accuracy_decimals: 0`, so no templating is required on the ESPHome side.

### DB frame handling

The XML transport uses escaped DB frames. Some legacy Wi-Fi bridges reply with an ASCII echo of the `@TR:32`, `@TG:43`, or
`@TG:C0` command before the actual data frame arrives. The component drains the UART briefly before the first command in a
poll cycle, filters those echoed bytes strictly, and validates the decoded payload length (21/13/13 bytes) per block. Frames
that are too short or too long are treated as unrelated telemetry and dropped without touching the timeout budget. The
per-command timeout starts once the first non-echo byte was received (otherwise after the TX) and defaults to 1.5 seconds.
Keep the poll interval at or above 25 seconds so the telemetry stream does not collide with the XML polling.

**Problem.** Encodierte TX-Bytes wurden wieder in den RX-Strom eingespeist. Der Framer erhielt dadurch Mischdaten,
die nicht zur erwarteten Nutzlastlänge passten und regelmäßig in `frame decode failed` bzw. Timeouts mündeten.

**Lösung.** Ein Echo-Suppressor verwirft jetzt die encodierten TX-Bytes zeitlich begrenzt, bevor sie den Framer erreichen.
Der Framer schneidet Frames am Terminator, de-stufft nur den einzelnen Block und prüft anschließend die Soll-Länge. Die
Timeout-Logik startet erst bei den ersten echten RX-Bytes, wodurch robuste Antworten ohne zusätzliche Verzögerung
ausgewertet werden können.

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
