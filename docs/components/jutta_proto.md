# Jutta-Proto-Komponente

Die Jutta-Proto-Komponente integriert die kundenspezifische JURA-Protokollimplementierung in ESPHome. Sie baut den UART-Handshake
mit einem JURA-Vollautomaten auf und stellt komfortable Automationsaktionen zum Brühen von Getränken per YAML zur Verfügung.

## Konfiguration

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
  machine_data:
    name: "JURA Status"
  machine_settings:
    name: "JURA XML-Konfiguration"
  xml_handshake:
    name: "JURA XML-Handshake"
```

Die Komponente führt den Handshake automatisch beim Start durch. Sobald dieser abgeschlossen ist, stehen alle Brühfunktionen zur
Verfügung. Optionale Textsensoren veröffentlichen sowohl die Rohantwort des Legacy-Befehls `&STAT?` (`machine_data`) als auch die
zuletzt gelesene oder geschriebene Einstellungs-XML (`machine_settings`). Der Sensor `xml_handshake` zeigt das erste Antwortfragment
des XML-Startbefehls an oder meldet `fehlgeschlagen`, wodurch sich die Initialisierung der XML-Schnittstelle überprüfen lässt.

### Handshake-Überblick

1. **Legacy-Probe** – Zu Beginn wird im Modus `DB_AUTO` `&WHO` gesendet. Bleibt die Antwort aus, folgen `@TR:37`, `@TR:32`, `@t2:8188`
   und `@TS:00`. Die Leitung nutzt einen 2b4b-Codec mit den Symbolen `{0xFF, 0xDF, 0xFB, 0xDB}`. Der Treiber dekodiert eingehende
   Antworten erst, nachdem ein Auto-Detektor Symbolmapping, Bitreihenfolge (MSB- oder LSB-zuerst) und eine mögliche Startverschiebung
   (`align`) ermittelt hat. Solange weniger als 90 % druckbare Zeichen erkannt werden, protokolliert das Debug-Log die jeweils beste
   Schätzung (`Auto-Detektor Kandidat: …`). Sobald ein gültiges Mapping gefunden ist, folgt eine `Auto-Detektor aktiviert`-Meldung
   mit Beispielzeile. Eingehende Blöcke mit weniger als 70 % druckbaren ASCII-Zeichen werden verworfen, damit Störsignale den Ablauf
   nicht blockieren.
2. **T1/T2/T3-Handshake** – Nach erfolgreicher Probe erfolgt der etablierte Sequenztausch `@T1` → `@t1` → `@T2` → `@t2` → `@T3` →
   `@t3`, womit der Bediencontroller freigeschaltet wird.
3. **XML-Handshake** – Sobald die Maschine betriebsbereit ist, wird automatisch `@hr:00` (Fallback `@hr:05`) angefragt. Die Antwort
   bzw. ein Fehlerstatus landet im Textsensor `xml_handshake`. Erst danach startet – sofern konfiguriert – das zyklische XML-Polling.

Sende- und Empfangspfad arbeiten mit vollständigen ASCII-Zeilen: Befehle wie `@TR:32\r\n` werden zuerst komplett aufgebaut und
anschließend in den erkanntem 2b4b-Modus kodiert, sodass die Vierergruppen der Symbolfolge garantiert zusammen auf der Leitung
landen.

## Automationsaktionen

Nutze die registrierten Aktionen in Automationen oder Button-Handlern. Wenn nur eine `jutta_proto`-Instanz konfiguriert ist, kann
das `id`-Argument entfallen.

### Vorgefertigtes Rezept starten

```yaml
button:
  - platform: template
    name: "Espresso brühen"
    on_press:
      - jutta_proto.start_brew:
          coffee: espresso
```

Verfügbare Werte für `coffee` sind `espresso`, `coffee`, `cappuccino`, `milk_foam`, `hot_water`, `caffe_barista`, `lungo_barista`,
`espresso_doppio`, `macchiato`, `two_espresso` (Alias `two_espressi`) sowie `two_coffee` (Alias `two_coffees`).

### Brühen mit individuellen Zeiten

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

### Laufenden individuellen Brühvorgang abbrechen

```yaml
switch:
  - platform: template
    name: "Brühvorgang abbrechen"
    turn_on_action:
      - jutta_proto.cancel_custom_brew: jura
```

### Frontplatten-Seiten wechseln

```yaml
script:
  - id: jura_next_page
    then:
      - jutta_proto.switch_page:
          id: jura
          page: 1
```

### Eigene Befehlssequenz ausführen

```yaml
script:
  - id: brew_manual_recipe
    mode: restart
    then:
      - jutta_proto.run_sequence:
          id: jura
          sequence:
            - command: grinder_on
              description: "Mahlwerk einschalten"
            - delay: 3s
              description: "Mahlwerk laufen lassen"
            - command: grinder_off
            - command: brew_group_to_brewing_position
            - command: coffee_press_on
            - delay: 500ms
              description: "Kaffee verdichten"
            - command: coffee_press_off
            - command: water_heater_on
            - command: water_pump_on
            - delay: 2s
              description: "Vorbrühen"
            - command: water_pump_off
            - command: water_heater_off
            - delay: 2s
            - command: water_heater_on
            - command: water_pump_on
            - delay: 40s
              description: "Wasser ausgeben"
            - command: water_pump_off
            - command: water_heater_off
            - command: brew_group_reset
```

Verwende `raw` anstelle von `command`, wenn ein individueller UART-Befehl gesendet werden soll. Rohbefehle ergänzen automatisch ein
fehlendes `\r\n`.

### Maschinen-Einstellungen lesen und schreiben

Die dedizierten Aktionen lesen oder schreiben die vollständige XML-Konfiguration des Vollautomaten. Die Antwort landet im optionalen
Textsensor `machine_settings`, wodurch sich die Nutzdaten bequem archivieren lassen.

```yaml
button:
  - platform: template
    name: "Einstellungen sichern"
    on_press:
      - jutta_proto.request_machine_settings:
          id: jura

script:
  - id: restore_settings
    mode: queued
    then:
      - jutta_proto.write_machine_settings:
          id: jura
          xml: !secret jura_settings_xml
```

Gespeicherte XML-Zeichenketten können direkt übergeben werden (z. B. via `!secret`). Für komplexere Abläufe zunächst
`request_machine_settings` ausführen, das Ergebnis sichern (etwa via HTTP oder auf einer SD-Karte) und anschließend mit
`write_machine_settings` zurückspielen.

## XML-Daten abrufen

Die originale JURA-Dongle-Firmware stellt strukturierte Status- und Wartungsinformationen über XML-Bänke wie `@TR:32`, `@TG:43`
oder `@TG:C0` bereit. Mit der erweiterten `jutta_proto`-Komponente lassen sich diese Bänke in festen Intervallen abfragen und den
in der `1.0.xml` hinterlegten Feldern zuordnen.

Aktiviere die Abfrage über `enable_xml_poll` und gib den Pfad zur Mapping-Datei (z. B. die extrahierte `1.0.xml`) an. Die
`xml_sensors` definieren, welche Felder ausgewertet werden sollen. Für jedes Feld wird einmal der zugehörige Bank-Befehl
abgefragt, die Antwort entschlüsselt und der Wert auf einen ESPHome-Sensor veröffentlicht.

```yaml
jutta_proto:
  id: jura
  uart_id: jura_uart
  enable_xml_poll: true
  xml_mapping_path: "/config/esphome/e6.xml"
  xml_poll_interval_ms: 60000
  xml_sensors:
    - field: "tgc0_37"
      name: "Reinigung"
      unit_of_measurement: "%"
      accuracy_decimals: 1
      entity_category: diagnostic
    - field: "tgc0_39"
      name: "Filterwechsel"
      unit_of_measurement: "%"
      accuracy_decimals: 1
      entity_category: diagnostic
    - field: "tg43_33"
      name: "Reinigungzähler"
    - field: "tr32_total_products"
      name: "Gesamte Brühvorgänge"
```

Die Feldnamen orientieren sich an der Mapping-Datei:

* `tr32_*` bezieht sich auf den Produktzähler (`@TR:32`). Die Namen werden aus den Produktdefinitionen der XML-Datei erzeugt.
* `tg43_*` und `tgc0_*` adressieren Wartungszähler (`@TG:43`) bzw. Restlaufzeiten in Prozent (`@TG:C0`). Die numerischen Suffixe
  (`33`, `37`, …) entsprechen den in der XML-Datei hinterlegten Text-IDs. Zusätzlich existieren sprechende Aliase (z. B.
  `tg43_cleaning`).

### Verfügbare XML-Felder

| Feld | Bank | Schlüssel | Beschreibung |
|------|------|-----------|---------------|
| `tg43_33` | `@TG:43` | `33` | Cleaning (Reinigung) |
| `tg43_34` | `@TG:43` | `34` | FilterChange (Filterwechsel) |
| `tg43_35` | `@TG:43` | `35` | Decalc (Entkalken) |
| `tg43_36` | `@TG:43` | `36` | CoffeeRinse (Kaffeespülung) |
| `tg43_40` | `@TG:43` | `40` | CappuRinse (Cappuccino-Spülung) |
| `tg43_41` | `@TG:43` | `41` | CappuClean (Cappuccino-Reinigung) |
| `tg43_cappuclean` | `@TG:43` | `41` | CappuClean (Cappuccino-Reinigung) |
| `tg43_cappurinse` | `@TG:43` | `40` | CappuRinse (Cappuccino-Spülung) |
| `tg43_cleaning` | `@TG:43` | `33` | Cleaning (Reinigung) |
| `tg43_coffeerinse` | `@TG:43` | `36` | CoffeeRinse (Kaffeespülung) |
| `tg43_decalc` | `@TG:43` | `35` | Decalc (Entkalken) |
| `tg43_filterchange` | `@TG:43` | `34` | FilterChange (Filterwechsel) |
| `tgc0_37` | `@TG:C0` | `37` | Cleaning (Restlaufzeit Reinigung) |
| `tgc0_38` | `@TG:C0` | `38` | Decalc (Restlaufzeit Entkalken) |
| `tgc0_39` | `@TG:C0` | `39` | FilterChange (Restlaufzeit Filterwechsel) |
| `tgc0_cleaning` | `@TG:C0` | `37` | Cleaning (Restlaufzeit Reinigung) |
| `tgc0_decalc` | `@TG:C0` | `38` | Decalc (Restlaufzeit Entkalken) |
| `tgc0_filterchange` | `@TG:C0` | `39` | FilterChange (Restlaufzeit Filterwechsel) |
| `tr32_2_coffee` | `@TR:32` | `13` | 2 Coffee (Zwei Kaffee) |
| `tr32_2_espressi` | `@TR:32` | `12` | 2 Espressi (Zwei Espresso) |
| `tr32_barista_lungo` | `@TR:32` | `29` | Barista Lungo |
| `tr32_cafe_barista` | `@TR:32` | `28` | Cafe Barista |
| `tr32_cappuccino` | `@TR:32` | `04` | Cappuccino |
| `tr32_coffee` | `@TR:32` | `03` | Coffee (Kaffee) |
| `tr32_espresso` | `@TR:32` | `02` | Espresso |
| `tr32_espresso_doppio` | `@TR:32` | `30` | Espresso Doppio |
| `tr32_espresso_macchiato` | `@TR:32` | `06` | Espresso Macchiato |
| `tr32_hotwater_portion_normal` | `@TR:32` | `0D` | Hotwater Portion (normal) |
| `tr32_milk_foam` | `@TR:32` | `08` | Milk Foam (Milchschaum) |
| `tr32_total_products` | `@TR:32` | `00` | Total Products (Gesamtbezüge) |
| `tr32_two_coffee` | `@TR:32` | `13` | 2 Coffee (Zwei Kaffee) |
| `tr32_two_espressi` | `@TR:32` | `12` | 2 Espressi (Zwei Espresso) |
| `tr32_two_espresso` | `@TR:32` | `12` | 2 Espressi (Zwei Espresso) |

Über die optionalen Parameter `multiplier` und `offset` lassen sich Rohwerte skalieren. Standardmäßig werden die Werte unverändert
veröffentlicht.

## Diagnose

Die Komponente protokolliert den Fortschritt des Handshakes während des Starts. Die Ausgabe von `dump_config()` nennt den
erkannten Maschinentyp sowie die letzten Nachrichten des Schlüsseltauschs, was bei der Fehlersuche rund um UART oder Verdrahtung
hilft. Zusätzlich weist die Ausgabe den ermittelten Legacy-Modus aus und zeigt die zuletzt abgefragte Probe samt Antwort, um
Codec-Unterschiede schnell zu erkennen.
