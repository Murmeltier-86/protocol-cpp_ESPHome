# JUTTA Proto – Geräteeinstellungen

Die Komponente liest alle in der J.O.E.-XML hinterlegten `<Setting>`-Einträge und
stellt sie als zusätzliche Home-Assistant-Entities zur Verfügung. Die Zuordnung
erfolgt ausschließlich über die XML-Definition.

## Felder und Metadaten

| Feld                | Beschreibung                                    |
|---------------------|--------------------------------------------------|
| `id`                | Stabiles Kennzeichen, wird als Entity-Suffix genutzt. |
| `name`              | Menschlich lesbarer Name (Debug-Log).            |
| `unit`              | Einheit (falls vorhanden, z. B. `°dH`).          |
| `source_cmd`        | UART-Befehl, der den Wert liefert (z. B. `@TM:02`). |
| `offset` / `width`  | Byte-Offset und -breite im Antwortframe.         |
| `scale`             | Multiplikator zur Skalierung des Rohwerts.       |
| `type`              | `u8`, `u16`, `u32`, `bool`, `enum` oder `string`. |

Numerische Einstellungen werden als `sensor` veröffentlicht. String-Werte werden
als `text_sensor` bereitgestellt.

## Quelle (source_cmd)

Jeder Eintrag in der XML enthält das Attribut `source_cmd`. Der Befehl wird beim
Komponentenstart einmalig abgesetzt und danach zyklisch alle 10 Minuten
(`kSettingsRefreshMs`) erneut abgefragt. Der gleiche Dekodierpfad wie bei den
bestehenden TR/TG/TGC0-Polls wird verwendet (`write_decoded_with_response`).

## Einheit und Skalierung

Die Unit (`unit`) sowie `scale` werden direkt aus der XML übernommen. `scale`
wird auf den Rohwert angewendet, bevor der Sensorwert veröffentlicht wird. Damit
lassen sich z. B. Halbgrad-Schritte (`scale = 0.5`) korrekt darstellen.

## Update-Strategie

* Initiales Bootstrap (`settings_boot_polled_`) unmittelbar nach erfolgreichem
  Handshake.
* Wiederholungsabfrage alle 600 s (`kSettingsRefreshMs`).
* Eine Abfrage kann optional per `jutta_id` auf eine von mehreren Instanzen
  geroutet werden.

## YAML-Beispiel

```yaml
sensor:
  - platform: jutta_proto
    jutta_id: my_jura
    id: jura_setting_water_hardness
    name: "${devicename} Einstellung Wasserhärte"
    unit_of_measurement: "°dH"
```

String-Werte verwenden die Text-Sensor-Plattform:

```yaml
text_sensor:
  - platform: jutta_proto
    id: jura_setting_welcome_text
    name: "${devicename} Begrüßung"
```

