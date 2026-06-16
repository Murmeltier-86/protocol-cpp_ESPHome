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
  xml_sensors:                     # optional – verknüpft YAML-Sensoren mit XML-Feldern
    - field: tr32_total_products
      name: "Gesamtbezüge"
      unit_of_measurement: "Bezüge"
      accuracy_decimals: 0
    - field: tgc0_bean_container
      name: "Füllstand Bohnen"
      icon: "mdi:coffee"
      accuracy_decimals: 1
```

The component takes care of the handshake during startup. Once the handshake finishes, all brewing actions become available.

When `enable_xml_poll` is set to `true`, the component periodically polls the JURA statistics interface using DB framing.
During initialization it loads the mapping file referenced by `xml_mapping_path` (default `embedded`, which resolves to
`esphome/components/jutta_proto/jura_mapping_embed.xml`). The file is read **at compile time**, embedded into the firmware
image and parsed during boot to discover the exact command strings and optional labels for the exposed statistics blocks.
If the XML is missing or malformed the defaults `@TR:32`, `@TG:43`, and `@TG:C0` are used and the sensors keep their
generic names.

### CODEX-Anpassung: Stabilisiertes XML-Polling

- **Polling-Ablauf.** Alle 30 s werden die DB-Kommandos strikt sequenziell gesendet: `@TR:32`, anschließend `@TG:43` und
  zum Schluss `@TG:C0`. Zwischen zwei Kommandos gibt es keine parallelen Übertragungen. Nach jedem TX wartet der Treiber
  120 ms ruhig, bevor er mit dem Empfang fortfährt. Jede Anfrage hat 1000 ms Zeit für eine Antwort und genau einen Retry,
  danach geht es mit dem nächsten Kommando weiter.
- **Validierung.** Ein Frame gilt als gültig, sobald der DB-Decoder Nutzdaten liefert, die mindestens so lang wie das
  Mapping vorgeben (TR32 ≥ 21 Byte, TG43/TGC0 ≥ 13 Byte) und mit `0x26` beginnen. Ein CRLF ist nicht mehr erforderlich;
  das Frame-Ende wird zusätzlich per 15 ms Byte-Gap erkannt. Überlange Frames werden akzeptiert, es werden nur die
  gemappten Felder ausgewertet.
- **Werteinterpretation.** `@TR:32` und `@TG:43` bleiben unveränderte Ganzzahlzähler (`state_class: total_increasing`).
  `@TG:C0` liefert Prozentwerte gemäß Mapping-Skalierung. Werte kleiner 0 % oder größer 130 % verwerfen den gesamten Frame
  und lösen den einmaligen Retry aus. Gültige Prozentwerte werden direkt veröffentlicht – ohne Clamping auf 100 %.
- **Bestandteile, die bleiben.** Legacy-Handshakes und UART-Grundkonfigurationen bleiben unverändert. Es werden keine
  zusätzlichen Flushes innerhalb des XML-Zyklus durchgeführt; Ruhezeit und Byte-Gap sorgen für stabile, sauber getrennte
  Frames.

### CODEX: J.O.E.-XML ohne `includes`

1. Exportiere die gewünschte XML über die J.O.E.-App.
2. Lege die unveränderte Datei im ESPHome-Projektordner ab (z. B. neben der YAML).
3. Setze in der YAML den Pfad via `xml_mapping_path: mein_export.xml`. Relative Pfade werden gegen den YAML-Ordner
   aufgelöst, absolute Pfade funktionieren ebenfalls.
4. Keine zusätzlichen `esphome: includes:` oder Dateisystem-Zugriffe nötig – die Datei wird beim Kompilieren in einen
   PROGMEM-String umgewandelt.
5. Beim Booten wertet der Parser nur die Statistik-Banks (`@TR:32`, `@TG:43`, `@TG:C0`) aus. Produkt-Befehle aus der XML
   bleiben unangetastet, damit Legacy-Flows unverändert bleiben.

### XML-Statistiksensoren

Die zyklische XML-Abfrage liest zusätzlich zu den Legacy-Kommandos die Status- und Zählerwerte aus den Blöcken
`@TR:32`, `@TG:43` und `@TG:C0` aus und stellt sie als numerische Sensoren bereit. Damit Home Assistant die Werte stabil
verarbeiten kann, gelten die folgenden Eckpunkte:

- **Ziel.** Die J.O.E.-XML liefert Wartungszähler und Füllstände; diese werden zyklisch dekodiert und als Sensoren mit
  eindeutiger `unique_id` im Schema `Jura E6 <Label>` veröffentlicht. Legacy-Handshake und klassische UART-Befehle bleiben
  unverändert.
- **XML-Quelle.** Die Mapping-Datei wird wie bisher per YAML in das Firmware-Image eingebunden und zur Laufzeit aus
  `/config/esphome/e6.xml` gelesen. Zusätzliche Dateisystem- oder `include`-Schritte sind nicht nötig.
- **Sensoraufbau.** Für jedes aktivierte Feld legt die Firmware beim Start einen numerischen Sensor an. Zähler verwenden
  `state_class: total_increasing`, Prozent- und Messwerte `state_class: measurement`. Die Genauigkeit richtet sich nach dem
  Skalierungsfaktor der XML, Prozentwerte erscheinen beispielsweise mit zwei Nachkommastellen.
- **Plausibilität & Änderungserkennung.** Jeder Messwert wird nur veröffentlicht, wenn er sich seit der letzten Meldung
  geändert hat und innerhalb plausibler Grenzen liegt. Zähler werden auf den Bereich `0…1.000.000` begrenzt, Prozent- und
  Statuswerte auf `0,0…250,0`. Ausreißer (z. B. `2121187790`) werden im DEBUG-Log dokumentiert und verworfen.
- **Logging.** Die Initialisierung protokolliert das geladene Mapping und die Anzahl der Sensoren. Für jeden veröffentlichen
  Wert bleibt die bestehende DEBUG-Zeile mit Feldname und Wert erhalten; Längenabweichungen der Rohframes erscheinen nur
  noch auf DEBUG-Level.

#### Sensoraufbau im Detail

| Block  | Sensoranzahl | Datenbreite | Standardnamen         | State-Class                |
| ------ | ------------- | ----------- | --------------------- | -------------------------- |
| TR32   | 10            | 16 Bit      | `TR32 1` … `TR32 10`  | `total_increasing`         |
| TG43   | 6             | 16 Bit      | `TG43 1` … `TG43 6`   | `total_increasing`         |
| TG:C0  | 3             | 32 Bit      | `TGC0 1` … `TGC0 3`   | `measurement`              |

Lieferte die XML sprechende Labels, ersetzen diese automatisch die Standardnamen. Die Sensoren behalten ihre eindeutigen
IDs auch dann, wenn die Bezeichnung in der XML nachträglich angepasst wird.

#### YAML-Verknüpfung der XML-Werte

Damit die Messwerte auch als Entitäten in Home Assistant auftauchen, muss jeder gewünschte Zähler explizit in der YAML
angelegt werden. Das geschieht über die neue Liste `xml_sensors` innerhalb des `jutta_proto`-Blocks. Jeder Eintrag
benötigt den Schlüssel `field`, der exakt dem Feldnamen aus dem XML-Mapping entsprechen muss (z. B.
`tr32_total_products`, `tg43_filter`, `tgc0_bean_container`). Die Feldnamen leiten sich aus den XML-Labels ab und werden
im ESPHome-Log beim Laden des Mappings ausgegeben. Zusätzlich lassen sich wie gewohnt `name`, `icon`,
`unit_of_measurement` oder `accuracy_decimals` setzen.

Beispiel:

```yaml
jutta_proto:
  id: jura
  uart_id: jura_uart
  enable_xml_poll: true
  xml_sensors:
    - field: tr32_total_products
      name: "Bezüge gesamt"
    - field: tg43_filter
      name: "Filterwechsel"
      unit_of_measurement: "Zyklen"
    - field: tgc0_bean_container
      name: "Bohnenfüllstand"
      unit_of_measurement: "%"
      accuracy_decimals: 1
```

Wenn ein konfigurierter Feldname im Mapping fehlt, meldet der Logger dies einmalig beim Start. Ist ein Feld vorhanden,
aber kein Sensor in `xml_sensors` hinterlegt, bleibt der Wert intern und wird protokolliert, jedoch nicht an Home
Assistant übergeben – Legacy-Funktionen sind davon unberührt.

### Robuste XML-Frames (CODEX)

- **Tolerante Länge.** Frames werden weiterhin bis zum CRLF-Terminator eingelesen. Abweichungen zwischen erwarteter und
  tatsächlicher Länge führen nur noch zu einem DEBUG-Hinweis. Warnungen erscheinen ausschließlich dann, wenn der
  Startmarker `0x26` fehlt oder der Frame kürzer als das benötigte Minimum ist.
- **Offset-gesteuertes Parsing.** Die dekodierten Bytes werden strikt anhand der im Mapping hinterlegten Offsets, Breiten
  und Endianness interpretiert; überstehende Bytes am Frameende werden ignoriert.
- **Stabiles Timing.** Vor jedem XML-Kommando wird der UART-Puffer komplett geleert. Zwischen den drei Abfragen liegt eine
  nicht-blockierende Pause von 25 ms, das Einzelkommando-Timeout beträgt rund 1 s. Der globale Poll-Takt aus der YAML bleibt
  unverändert.
- **RX-Puffer.** Der bestehende Empfangspuffer (≥256 Byte) bleibt erhalten und verhindert, dass zusammenhängende Frames
  verloren gehen.

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
