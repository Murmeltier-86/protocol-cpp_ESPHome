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
  enable_xml_poll: true            # optional, default false
  xml_mapping_path: embedded       # optional, "embedded" nutzt die mitgelieferte XML
  xml_poll_interval_ms: 30000      # optional, Pollintervall in ms (min. 25000)
```

The component takes care of the handshake during startup. Once the handshake finishes, all brewing actions become available.

When `enable_xml_poll` is set to `true`, the component periodically polls the JURA statistics interface using DB framing.
During initialization it loads the mapping file referenced by `xml_mapping_path` (default `embedded`, which resolves to
`esphome/components/jutta_proto/jura_mapping_embed.xml`). The file is read **at compile time**, embedded into the firmware
image and parsed during boot to discover the exact command strings and optional labels for the exposed statistics blocks.
If the XML is missing or malformed the defaults `@TR:32`, `@TG:43`, and `@TG:C0` are used and the sensors keep their
generic names.

### CODEX: J.O.E.-XML ohne `includes`

1. Exportiere die gewünschte XML über die J.O.E.-App.
2. Lege die unveränderte Datei im ESPHome-Projektordner ab (z. B. neben der YAML).
3. Setze in der YAML den Pfad via `xml_mapping_path: mein_export.xml`. Relative Pfade werden gegen den YAML-Ordner
   aufgelöst, absolute Pfade funktionieren ebenfalls.
4. Keine zusätzlichen `esphome: includes:` oder Dateisystem-Zugriffe nötig – die Datei wird beim Kompilieren in einen
   PROGMEM-String umgewandelt.
5. Beim Booten wertet der Parser nur die Statistik-Banks (`@TR:32`, `@TG:43`, `@TG:C0`) aus. Produkt-Befehle aus der XML
   bleiben unangetastet, damit Legacy-Flows unverändert bleiben.

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

#### TG43 Maintenance counter decoding

**Zusammenfassung.** Die TG43-Wartungszähler werden jetzt exakt anhand der Offsets, Breiten und Endianness-Angaben der
bereitgestellten J.O.E.-XML ausgewertet. Jeder Zähler landet als numerischer Sensor in Home Assistant; zusätzliche
Diagnose-Logs zeigen zu jedem Frame die dekodierten Bytes, das genutzte Mapping und den veröffentlichten Wert.

**Konfiguration.** Nutze weiterhin das XML aus `jura_joe_xml_bundle_final` oder einen eigenen Export. Optionale Attribute
wie `offset`, `size` und `endian="le"` können pro Feld gesetzt werden und überschreiben die Standardwerte (Offset=1,
Breite=2, Big-Endian). Änderungen an der XML-Datei erfordern keinen Eingriff am Legacy-Handshake.

**Troubleshooting.** Bei unplausiblen Zählerständen (<0 oder >100000) meldet das Log eine Warnung inklusive Offsets und
Rohdaten. Stimmen erwartete und empfangene Länge nicht überein, zeigt die Warnung die ersten 32 dekodierten Bytes
hexadezimal an – so lassen sich Offsets und Endianness im XML schnell nachjustieren, ohne den Ablauf der Polling-Logik zu
ändern.

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
ausgewertet werden können. Die Unterdrückung der Echo-Frames bleibt bis zu 200 ms aktiv und verlängert sich mit jedem
erkannten Byte, sodass auch träge Legacy-Bridges keine halben Kommandos mehr in den Puffer drücken können.

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

## Fehlerbehebung bei der Integration

Wenn während des Kompiliervorgangs Build-Fehler in `jutta_proto.cpp` auftreten, prüfen Sie die verwendete ESPHome-Version. Die
Komponentenfunktionen `map_tr32`, `map_tg43` und `map_tgc0` sind Teil des mitgelieferten XML-Mappers und werden als Funktionszeiger
übergeben. In älteren Snapshots kann es notwendig sein, einen vollständigen Funktionsprototyp zu übergeben, damit der C++-Compiler
keine Überladungs-Konflikte meldet. Ab der korrigierten Variante werden die drei Mapper explizit als Funktionszeiger mit den
Argumenten `(const std::vector<uint8_t> &, const XmlMapping &, MachineStats &)` übergeben, womit sie sowohl auf aktuellen als auch
auf älteren Toolchains fehlerfrei kompilieren.

Zusätzlich wurde die Hilfsfunktion zur Formatierung druckbarer Zeichen so angepasst, dass sie ohne nicht genutzten Template-Parameter
auskommt. Damit bleibt die Ausgabe in den ESPHome-Logs (z. B. beim UART-Handshaking) unverändert, gleichzeitig verhindert die Änderung
jedoch, dass GCC oder Clang bei der Vorlagendeduktion scheitern.

### XML-Mapping der J.O.E.-Exporte

**Symptom.** Im Log erscheint dauerhaft `XML mapping valid: NO`, obwohl die J.O.E.-XML-Datei korrekt geladen wurde. Alle Sensoren für
`@TR:32`, `@TG:43` und `@TG:C0` bleiben inaktiv.

**Ursache.** Der XML-Parser hat das erste Wort eines Tags irrtümlich als Attribut behandelt. Bei Exporten, die Großbuchstaben für
`<BANK Command="…">` verwenden, wurde daher das `Command`-Attribut nicht erkannt und der zugehörige Block übersprungen.

**Lösung.** Die Attribut-Erkennung ignoriert nun das Tag-Präfix, sobald hinter dem Tag-Namen kein `=` folgt. Dadurch werden auch J.O.E.-
Exporte mit vollständig großgeschriebenen `BANK`-Tags korrekt ausgewertet, die Sensorfelder werden wieder angelegt und der Status
wechselt auf `XML mapping valid: YES`.

### XML-Statuswerte als Sensoren

* Die Befehle `@TR:32`, `@TG:43` und `@TG:C0` werden mit flexiblen Rahmenlängen verarbeitet. Antworten, die den erwarteten Mindestumfang
  erreichen und mit dem Startbyte `0x26` beginnen, werden auch dann ausgewertet, wenn zusätzliche Bytes folgen. Erst wenn der Frame zu kurz ist
  oder der Marker fehlt, erscheint eine Warnung.
* Vor jedem XML-Kommando wird der UART-Empfangspuffer geleert, zwischen den Abfragen liegt eine nicht-blockierende Pause von 25 ms und das
  Timeout pro Kommando wurde auf 1 s gesetzt. So bleiben die Antworten sauber getrennt und kommen auch bei langsameren Maschinen stabil an.
* Das XML-Mapping markiert veröffentlichte Felder explizit. Für diese Felder legt die Komponente numerische Sensoren mit Namen im Schema
  `Jura E6 <Label>` und eindeutiger `unique_id` an. Zähler erhalten den Status `total_increasing`, Messwerte den Status `measurement`.
* Werte werden nur veröffentlicht, wenn sie sich tatsächlich geändert haben und innerhalb der hinterlegten Plausibilitätsgrenzen liegen.
  Ausreißer (z. B. fehlerhafte Counter-Werte) werden im Debug-Log dokumentiert, aber nicht an Home Assistant weitergegeben.
* Die YAML-Integration bleibt unverändert: Die Mapping-Datei wird per `!include` ins Projekt übernommen und zur Laufzeit unter
  `/config/esphome/e6.xml` geladen. Der Legacy-Handshake und alle klassischen Befehle funktionieren weiterhin wie bisher.
