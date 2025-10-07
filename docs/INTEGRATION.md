# Einbindung der neuen Entities

## Benötigte Plattformen

* `sensor.jutta_proto` – numerische Einstellungen sowie Fehlercode.
* `text_sensor.jutta_proto` – String-Einstellungen und Fehlertexte.
* `binary_sensor.jutta_proto` – Fehlerstatus (`jura_has_error`).

Alle Plattformen akzeptieren optional `jutta_id`, falls mehrere Komponenten
konfiguriert sind.

## Minimalbeispiel

```yaml
jutta_proto:
  id: my_jura
  uart_id: uart_bus
  enable_xml_poll: true

sensor:
  - platform: jutta_proto
    jutta_id: my_jura
    id: jura_error_code
    name: "${devicename} Fehlercode"

  - platform: jutta_proto
    jutta_id: my_jura
    id: jura_setting_water_hardness
    name: "${devicename} Wasserhärte"
    unit_of_measurement: "°dH"

text_sensor:
  - platform: jutta_proto
    jutta_id: my_jura
    id: jura_error_text
    name: "${devicename} Fehlertext"

binary_sensor:
  - platform: jutta_proto
    jutta_id: my_jura
    id: jura_has_error
    name: "${devicename} Fehler aktiv"
```

## Hinweise

* Die IDs `jura_error_code`, `jura_error_text`, `jura_error_severity` und
  `jura_has_error` sind fest verdrahtet.
* Einstellungs-Entities müssen mit dem Präfix `jura_setting_` beginnen. Der
  Rest der ID muss mit dem XML-Attribut `id` übereinstimmen.
* Werte werden automatisch aktualisiert (Einstellungen alle 10 Minuten,
  Fehler alle 5 Sekunden).

