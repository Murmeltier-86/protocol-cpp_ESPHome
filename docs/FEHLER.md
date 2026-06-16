# JUTTA Proto – Fehlercodes

Die Fehlerzustände der Maschine werden über den in der XML hinterlegten
`<Errors>`-Block ermittelt. Jede Fehlermeldung ist ein Mapping von numerischem
Code zu Text und Schweregrad.

## Polling

* Abfragekommando: Attribut `source_cmd` des `<Errors>`-Elements (z. B. `@ER:00`).
* Intervall: alle 5 s (`kErrorPollIntervalMs`).
* Dekodierung: identisch zu den Einstellungen – die Antwort wird als hexadezimale
  Bytefolge ausgewertet und zu einem 32-Bit-Code zusammengesetzt.

## Entities

| Entity ID             | Typ            | Beschreibung                               |
|-----------------------|----------------|---------------------------------------------|
| `sensor.jura_error_code`      | `sensor`       | Numerischer Fehlercode.                     |
| `text_sensor.jura_error_text` | `text_sensor`  | Fehlerbeschreibung laut XML.                |
| `text_sensor.jura_error_severity` | `text_sensor` | Severity (`info`, `warning`, `block`, …).   |
| `binary_sensor.jura_has_error` | `binary_sensor` | `true`, wenn `code != 0`.                   |

Werden neue Fehlercodes in der XML ergänzt, steht der Text automatisch zur
Verfügung (`find_error`). Unbekannte Codes werden mit `unbekannt`/`unknown`
veröffentlicht.

## Beispielkonfiguration

```yaml
text_sensor:
  - platform: jutta_proto
    id: jura_error_text
    name: "${devicename} Fehlertext"
  - platform: jutta_proto
    id: jura_error_severity
    name: "${devicename} Fehler-Schwere"

sensor:
  - platform: jutta_proto
    id: jura_error_code
    name: "${devicename} Fehlercode"
    unit_of_measurement: ""

binary_sensor:
  - platform: jutta_proto
    id: jura_has_error
    name: "${devicename} Fehler aktiv"
```

## Beispiel-Ansicht in Home Assistant

```
Fehlercode: 0
Fehlertext: kein Fehler
Fehler-Schwere: none
Fehler aktiv: aus
```

