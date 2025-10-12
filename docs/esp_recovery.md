# Wiederherstellung eines ESP-Geräts nach fehlgeschlagenem Flash-Vorgang

Wenn ein ESP8266/ESP32 weder über OTA noch über die serielle Schnittstelle flashbar ist, helfen die folgenden Schritte, das Gerät wieder in einen definierten Zustand zu bringen.

## 1. Stromversorgung trennen
* Trenne die Versorgung vollständig (USB-Stecker bzw. Netzteil ziehen).
* Warte mindestens 10 Sekunden, damit alle Kondensatoren entladen sind.

## 2. Serielle Verdrahtung prüfen
* RX des Programmieradapters muss an TX des ESP, TX an RX.
* Verbinde GND des Adapters **immer** mit GND des ESP.
* Bei ESP32/ESP8266 unbedingt auf 3,3 V-Pegel achten – 5 V zerstören den Chip.

## 3. Boot-Modus erzwingen
* Halte beim Einschalten GPIO0 (meist `BOOT` oder `FLASH` beschriftet) auf Masse.
* Drücke ggf. gleichzeitig die `EN`/`RST`-Taste, während GPIO0 auf GND liegt.
* Lasse GPIO0 los, sobald der Programmieradapter eine Verbindung aufgebaut hat.

## 4. Flash löschen
```bash
esptool.py --chip auto --port /dev/ttyUSB0 erase_flash
```
* Ersetze `/dev/ttyUSB0` durch den tatsächlichen Port (unter Windows z. B. `COM4`).
* Ein vollständiges Löschen entfernt beschädigte Firmware und Boot-Schleifen.

## 5. Minimale Test-Firmware schreiben
* Kompiliere in ESPHome eine einfache YAML (z. B. nur WLAN und Status-LED).
* Flashe diese Firmware per `esphome run <config.yaml> --device /dev/ttyUSB0`.
* Prüfe, ob der ESP nun wieder per OTA erreichbar ist.

## 6. Ursprüngliche Firmware erneut kompilieren
* Wenn der Test erfolgreich ist, baue die gewünschte Konfiguration frisch.
* Flashe zuerst seriell, dann – nach einem funktionierenden Start – per OTA.

## 7. Logging kontrollieren
* Aktiviere in ESPHome vorübergehend `logger:` mit moderater Baudrate (115200).
* Beobachte über `esphome logs <config.yaml> --device /dev/ttyUSB0`, ob beim Start Fehlermeldungen auftreten.

## 8. Häufige Stolperfallen
* **Dauer-Reset:** Prüfe, ob sich das Gerät in einer Schleife befindet (z. B. wegen `restart`-Automationen). Entferne solche Automationen testweise.
* **Unpassende Baudrate:** Stelle sicher, dass die UART-Baudrate zum Adapter passt.
* **Stromversorgung:** Große Lasten (Pumpen, Relais) können beim Flashen die Spannung einbrechen lassen – ggf. extern versorgen.

## 9. Sicherheitshinweise
* Arbeiten an Geräten mit Netzspannung nur spannungsfrei durchführen.
* Bei Geräten im Gehäuse auf ausreichende Entkopplung und Isolation achten.

Wenn die oben genannten Schritte nicht helfen, kann eine serielle Konsole mit Werks-Firmware oder ein externer Programmierer (z. B. JTAG für ESP32) erforderlich sein. Dokumentiere alle Schritte und Log-Ausgaben, um weitere Analysen zu erleichtern.
