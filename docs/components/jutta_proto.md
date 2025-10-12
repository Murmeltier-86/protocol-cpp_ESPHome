# Jutta-Proto-Komponente

Die Jutta-Proto-Komponente verbindet ESPHome mit dem proprietären UART-Protokoll der JURA-Maschinen. Sie führt ausschließlich den klassischen Handshake aus, nutzt die bewährten Legacy-Kommandos und stellt sämtliche Automationsaktionen aus `coffee_maker.*` bereit. Zusätzlich kann sie Statuszeilen über den 2b4b-Codec aus `codec_db.c` abrufen, ohne dass ein separater XML-Handshake erforderlich ist.

## Grundkonfiguration

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

## Statusabruf über den DB-Codec

Der optionale Textsensor `machine_data` veranlasst die Komponente, zyklisch eine Diagnosezeile anzufordern. Dabei wird die ASCII-Sequenz `@TR:32\r\n` mit dem in `codec_db.c` beschriebenen 2b4b-Verfahren übertragen. Die Antwort wird mit demselben Codec dekodiert; Bitreihenfolge und eventuelle Escape-Sequenzen erkennt der Decoder automatisch. Sobald ein vollständiges Zeilenende (`\r\n`) vorliegt, veröffentlicht der Sensor den Inhalt ohne Steuerzeichen.

```yaml
jutta_proto:
  id: jura
  uart_id: jura_uart
  machine_data:
    name: "Jura Rohstatus"
```

* Abfrageintervall: 30 s (kann durch Automationen angepasst werden).
* Timeout je Versuch: 2 s; bei Überschreitung wird im Log gewarnt und der nächste Versuch verschoben.
* Kodierung: 2b4b-Symbole `{0xFF, 0xDF, 0xFB, 0xDB}`, Zuordnung gemäß `codec_db.c`.

Die Legacy-Kommandos (`FN:xx`, `@Tn`, `PR:xy` …) laufen weiterhin unverändert über den klassischen Encoder. Der DB-Kanal wird nur für den oben beschriebenen Statusabruf genutzt.

## Aktionen

- `start_brew`: Löst ein Getränk aus der jeweiligen Getränke-Seite aus. Unterstützte Werte sind `espresso`, `coffee`, `cappuccino`, `milk_foam`, `hot_water`, `caffe_barista`, `lungo_barista`, `espresso_doppio`, `macchiato`, `two_espresso` und `two_coffee` (inklusive der Alias-Namen aus der YAML-Validierung).
- `custom_brew`: Startet einen individuellen Bezug unter Angabe der Mahl- und Wasserzeit.
- `cancel_custom_brew`: Bricht einen laufenden individuellen Bezug ab.
- `switch_page`: Wechselt auf eine andere Getränke-Seite.
- `run_sequence`: Führt eine frei definierte Abfolge von Legacy-Kommandos und Verzögerungen aus. Jede Stufe kann entweder einen benannten Befehl aus der Tabelle (`grinder_on`, `water_pump_off`, …) oder eine Rohzeile enthalten; optional lassen sich Pausen (`delay`/`sleep`) und eigene Beschreibungen hinterlegen. Zeitlimits pro Schritt verhindern das endlose Warten auf Bestätigungen.

Weitere Details zu den Parametern finden sich direkt im Code der Komponente.
