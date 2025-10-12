# Jutta-Proto-Komponente

Die Jutta-Proto-Komponente verbindet ESPHome mit dem proprietären UART-Protokoll der JURA-Maschinen. Sie führt den klassischen Handshake aus, stellt die vorhandenen Automationsaktionen bereit und arbeitet mit den Funktionen aus `coffee_maker.*` zusammen. Zusätzlich steht ein separater 2b4b-Kanal für XML-basierte Status- und Einstellungsabfragen zur Verfügung, der unabhängig vom Legacy-Protokoll betrieben wird.

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

## XML-Abfragen über den 2b4b-Kanal

Optional kann die Komponente den neuen 2b4b-Kanal verwenden, um strukturierte Diagnosedaten abzurufen. Die Kodierung orientiert sich an den in `codec_db.c` beschriebenen Symbolpaaren; der Decoder ermittelt Bitreihenfolge, Ausrichtung und ggf. XOR-Schlüssel automatisch pro Frame. Nach drei erfolglosen Versuchen pausiert die Abfrage zunächst und protokolliert den Grund im Log.

```yaml
jutta_proto:
  id: jura
  uart_id: jura_uart
  enable_xml_poll: true
  xml_poll_interval_ms: 60000
  xml_mapping_path: "/config/esphome/1.0.xml"
  xml_status:
    name: "Jura XML-Status"
  xml_sensors:
    - field: "tgc0_37"
      name: "Reinigung"
      unit_of_measurement: "%"
      accuracy_decimals: 1
      entity_category: diagnostic
    - field: "tr32_total_products"
      name: "Gesamte Bezüge"
```

- `enable_xml_poll`: Aktiviert den XML-Kanal. Legacy-Kommandos laufen weiterhin über den bisherigen Encoder.
- `xml_poll_interval_ms`: Abstand zwischen zwei Lesezyklen in Millisekunden.
- `xml_mapping_path`: Pfad zu einer Referenz-XML (z. B. die im Repo enthaltene `1.0.xml`), die für eigene Auswertungen genutzt werden kann. Die Komponente liest die Datei nicht automatisch ein, der Pfad wird lediglich im Log angegeben.
- `xml_status`: Optionaler Text-Sensor, der den aktuellen Zustand des XML-Kanals meldet (z. B. "bereit", "pausiert nach Fehlern").
- `xml_sensors`: Liste numerischer Sensoren. Für jedes Feld wird der erste Schlüssel-Wert-Eintrag der empfangenen XML-/Statuszeilen ausgewertet. Die Werte werden als Fließkommazahlen interpretiert; nicht numerische Werte werden protokolliert.

Ein Poll besteht aus dem Versand einer kompletten 2b4b-Zeile (`@TR:32\r\n`) und der Auswertung sämtlicher Antworten, die mit `@` oder `&` beginnen. Die Antworten werden nach `:` bzw. `=` getrennt und den konfigurierten Feldern zugeordnet. Bleiben drei Abfragen hintereinander erfolglos, setzt der Kanal eine Minute aus und versucht anschließend erneut einen XML-Handshake.

## Aktionen

- `start_brew`: Löst ein Getränk aus der jeweiligen Getränke-Seite aus.
- `custom_brew`: Startet einen individuellen Bezug unter Angabe der Mahl- und Wasserzeit.
- `cancel_custom_brew`: Bricht einen laufenden individuellen Bezug ab.
- `switch_page`: Wechselt auf eine andere Getränke-Seite.

Weitere Details zu den Parametern finden sich direkt im Code der Komponente.
