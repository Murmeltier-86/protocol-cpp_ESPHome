# JURA Protocol
`C++` JURA protocol implementation for controlling a Jura coffee maker over a serial (UART) connection.

This work is based on the excellent work done by the people over at [Protocol JURA](http://protocoljura.wiki-site.com/index.php/Hauptseite).
They were able to figure out the basic protocol used for communication with older JURA coffee makers.

Since newer models **do not use** this old V1-Protocol any more I started this project to understand the new one and create a reference implementation for it.

## Table of Contents
0. [Example](#example)
1. [Protocol](#protocol)
2. [JURA Commands](#jura-commands)
3. [Requirements](#requirements)
4. [Building](#building)

## ESPHome-Integration (Kurzüberblick)

### Aktivierung des XML-Pollings

- `enable_xml_poll`: Schaltet den zusätzlichen XML-Pfad frei (Standard `false`).
- `xml_poll_interval_ms`: Abstand zwischen zwei Abfragen in Millisekunden (Standard `30000`).

Beispiel-Konfiguration in ESPHome:

```yaml
uart:
  id: uart_bus
  baud_rate: 9600

jutta_proto:
  id: jura_e6
  uart_id: uart_bus
  enable_xml_poll: true
  xml_mapping_path: joe_export.xml  # Pfad zur exportierten J.O.E.-XML
  xml_poll_interval_ms: 30000
```

### Mapping-Datei

- Die Komponente liest die angegebene J.O.E.-Exportdatei bereits beim Generieren des C++-Codes ein und bettet den Inhalt als PROGMEM-String in die Firmware ein.
- Standardmäßig wird `esphome/components/jutta_proto/jura_mapping_embed.xml` verwendet (eine unveränderte Original-Exportdatei, die dem Repo beiliegt).
- Über den YAML-Parameter `xml_mapping_path` kann ein alternativer Pfad angegeben werden; Pfade dürfen absolut oder relativ zum YAML-Verzeichnis sein.
- Die Datei muss nicht angepasst werden – einfach die unveränderte App-Exportdatei referenzieren.
- Der tatsächlich genutzte Pfad wird in der Konfigurationsausgabe als `XML mapping Quelle` protokolliert.

**Kurz-Dokumentation**

1. XML in der J.O.E.-App exportieren.
2. Datei ins ESPHome-Projekt kopieren (oder absoluten Pfad merken).
3. Keine zusätzlichen `includes:` erforderlich – der Pfad unter `xml_mapping_path` genügt.
4. Beim Kompilieren wird die Datei automatisch eingebunden, vom Parser ausgewertet und anschließend als numerische Sensorwerte veröffentlicht.

### Automatisch erzeugte Sensoren

- Für jedes Feld der Frames `@TR:32`, `@TG:43` und `@TG:C0` wird ein numerischer `sensor::Sensor` erstellt.
- Typische Werte (abhängig vom Mapping): `coffee_total`, `espresso_total`, `cappuccino_total`, `cleaning_counter`, `decalc_counter`, `filter_change_counter`, `cleaning_percent`, `decalc_percent`, `filter_change_percent`.
- Die sichtbaren Home-Assistant-Entitäten tragen den im Mapping hinterlegten Label-Text.

### Ablauf & Logging

- Nach erfolgreichem Handshake sendet die Firmware alle `xml_poll_interval_ms` nacheinander `@TR:32`, `@TG:43` und `@TG:C0`.
- Jeder Befehl wird als `TX_DB` geloggt, Antworten erscheinen als `RX_DB <Frame> decoded_len=<N> reason=<gap|CRLF>`.
- Zeitüberschreitungen führen lediglich zu einer Warnung; beim nächsten Intervall wird automatisch weiter versucht.
- Erfolgreich geparste Felder werden mit `XML publish: <Name>=<Wert>` protokolliert.

## Example
The following example shows the interaction with a JURA coffee maker over [XMPP](https://xmpp.org/).
The complete implementation for this demo can be found [here](https://github.com/COM8/esp32-jura).
[![Watch the video](https://user-images.githubusercontent.com/11741404/89994342-489af800-dc88-11ea-9a4e-c4407ce79a8d.png)](https://twitter.com/UWPX_APP/status/1293461429677436931)

## Protocol

### General
There are several steps of obfuscation being done by the JURA coffee maker to prevent others from reading the bare protocol.

#### Connecting to an JURA coffee maker
To connect to an JURA coffee maker we are using a 5V UART signal with the following configuration:
* **Baud Rate:** 9600
* **Data Bits:** 8
* **Parity:** Disabled
* **Stop Bits:** 1
* **Flow Control:** Hardware flow control disabled
* **RX Flow Control Threshold:** 0

#### Deobfuscating
Once a connection has been established we can start sending and receiving data.  
**But** all data send and received is obfuscated.
The following description shows how to **deobfuscate** data received from the coffee maker.  
To obfuscate data just follow the steps in reverse.

**Step 0**
The coffee maker always sends 4 "raw" byte per one byte of data with a break of 8ms in between each "raw" byte.
This looks something like this:
```
01011011 <8ms break> 01011111 <8ms break> 01011111 <8ms break> 01011111 <8ms break>
01011111 <8ms break> 01111011 <8ms break> 01011111 <8ms break> 01011111 <8ms break>
01111011 <8ms break> 01111011 <8ms break> 01111111 <8ms break> 01011011 <8ms break>
01011111 <8ms break> 01111111 <8ms break> 01011011 <8ms break> 01011011 <8ms break>
01111011 <8ms break> 01111011 <8ms break> 01011011 <8ms break> 01011011 <8ms break>
```
Each line corresponds to one actual data byte.

**Step 1**  
Each of our 4 "raw" bytes (each line) contains only 2 bits of our resulting data bit.  
Bit 2 and 5.  
All other bits (except 0) are set to 1.

```
0b01011011
    ^  ^
  DB1  DB2
```
`DB1` and `DB2` are our actual data bits here.
We have to combine alle 8 (of our 4 "raw" bytes) into a single data byte.

Examples:
```C++
const std::array<uint8_t, 4> encData{0b01011011, 0b01011111, 0b01011111, 0b01011111};

// Bit mask for the 2. bit from the left:
constexpr uint8_t B2_MASK = (0b10000000 >> 2);

// Bit mask for the 5. bit from the left:
constexpr uint8_t B5_MASK = (0b10000000 >> 5);

uint8_t decData = 0;
decData |= (encData[0] & B2_MASK) << 2;
decData |= (encData[0] & B5_MASK) << 4;
decData |= (encData[1] & B2_MASK);
decData |= (encData[1] & B5_MASK) << 2;
decData |= (encData[2] & B2_MASK) >> 2;
decData |= (encData[2] & B5_MASK);
decData |= (encData[3] & B2_MASK) >> 4;
decData |= (encData[3] & B5_MASK) >> 2; // 0b00010101
```
```
Input            -> Output
0 0 0 1  0 1 0 1 -> 0 1 0 1  0 0 0 1
0 1 1 0  0 1 0 1 -> 0 1 0 1  0 1 1 0
1 0 1 0  1 1 0 0 -> 1 1 0 0  1 0 1 0
0 1 1 1  0 0 0 0 -> 0 0 0 0  0 1 1 1
1 0 1 0  0 0 0 0 -> 0 0 0 0  1 0 1 0
```

**Step 2**  
In this step we switch both nibbles (4 bit) of each byte.
Examples:
`1100 1100 -> 0011 0011`
```C++
uint8_t in = 0b00010101;
uint8_t out = ((in & 0xF0) >> 4) | ((in & 0x0F) << 4); // 0b01010001
```
```
Input                                                                               -> Output
01011011 <8ms break> 01011111 <8ms break> 01011111 <8ms break> 01011111 <8ms break> -> 0 1 0 1  0 0 0 1
01011111 <8ms break> 01111011 <8ms break> 01011111 <8ms break> 01011111 <8ms break> -> 0 1 0 1  0 1 1 0
01111011 <8ms break> 01111011 <8ms break> 01111111 <8ms break> 01011011 <8ms break> -> 1 1 0 0  1 0 1 0
01011111 <8ms break> 01111111 <8ms break> 01011011 <8ms break> 01011011 <8ms break> -> 0 0 0 0  0 1 1 1
01111011 <8ms break> 01111011 <8ms break> 01011011 <8ms break> 01011011 <8ms break> -> 0 0 0 0  1 0 1 0
```

**Step 3**  
A last time we have to shift all bits in our byte around.
Here we have to split up our byte into two nibbles (4 bit) and switch two bits each.  
Examples:
`1100 1100 -> 0011 0011`
```C++
uint8_t in = 0b01010001;
uint8_t out = ((in & 0xC0) >> 2) | ((in & 0x30) << 2) | ((in & 0x0C) >> 2) | ((in & 0x03) << 2); // 0b01010100
```
```
Input            -> Output           -> Output_Hex Output_Dec Output_Char
0 1 0 1  0 0 0 1 -> 0 1 0 1  0 1 0 0 -> 54	84	T 
0 1 0 1  0 1 1 0 -> 0 1 0 1  1 0 0 1 -> 59	89	Y 
1 1 0 0  1 0 1 0 -> 0 0 1 1  1 0 1 0 -> 3A	58	:
0 0 0 0  0 1 1 1 -> 0 0 0 0  1 1 0 1 -> 0d	13	'\r'
0 0 0 0  1 0 1 0 -> 0 0 0 0  1 0 1 0 -> 0a	10	'\n'
```
Which results in the message `TY:\r\n`.

## JURA Commands
Every message/command send from or to the coffee maker has to end with `\r\n` to be valid.
For simplicity reasons we omit the `\r\n` from all of the following messages and examples.

### Command Structure
In general for every **valid** command a response will be send from the coffee maker.
The actual command is always uppercase (e.g. `TY:`) and the response send back is lowercase (`ty:EF532M V02.03`).

### Available Commands
The following list of commands has been tested on an `Jura E6 2019 platin (15326)`.

| Name | Command | Response | Description |
|----|----|----|----|
| UNKNOWN | `AN:01` | `ok:` | - |
| Turn off | `AN:02` | `ok:` | Turns off the coffee maker. |
| Erase EPROM | `AN:0A` | UNKNOWN | **Untested!** Erases the EPROM. Do not use. |
| Test UCHI | `AN:0C` | `ok:` | Test the UCHI steam plate. |
| Test Mode on | `AN:20` | `ok:` | Turns on the test mode. |
| Test Mode off | `AN:21` | `ok:` | Turns off the test mode. |
| UNKNOWN | `AN:40` | `an:40` | - |
| UNKNOWN | `AN:AA` | `ok:` | - |
| Get Type of Machine | `TY:` | `ty:` (e.g. `ty:EF532M V02.03`) | Returns the type of the machine. |
| UNKNOWN | `FA:01` | `ok:` | - |
| (Button 1) | `FA:04` | `ok:` | Simulates the button 1 press (left top). |
| (Button 2) | `FA:05` | `ok:` | Simulates the button 2 press (left center). |
| (Button 3) | `FA:06` | `ok:` | Simulates the button 3 press (left bottom). |
| (Button 4) | `FA:07` | `ok:` | Simulates the button 4 press (right top). |
| (Button 5) | `FA:08` | `ok:` | Simulates the button 5 press (right center). |
| (Button 6) | `FA:09` | `ok:` | Simulates the button 6 press (right bottom). |
| Coffee Pump on | `FN:01` | `ok:` | Turns on the coffee pump. |
| Coffee Pump off | `FN:02` | `ok:` | Turns off the coffee pump. |
| Coffee Heater on | `FN:03` | `ok:` | Turns on the coffee heater. |
| Coffee Heater off | `FN:04` | `ok:` | Turns off the coffee heater. |
| Grinder on | `FN:07` | `ok:` | Turns on the coffee grinder. |
| Grinder off | `FN:08` | `ok:` | Turns off the coffee grinder. |
| Brew Group **Something** on | `FN:09` | `ok:` | Turns **something** in relation to the brew group on. |
| Brew Group **Something** off | `FN:0A` | `ok:` | Turns **something** in relation to the brew group off. |
| Coffee press on | `FN:0B` | `ok:` | Turns on the coffee press. |
| Coffee press off | `FN:0C` | `ok:` | Turns off the coffee press. |
| Init Brew Group | `FN:0D` | `ok:` | Initializes the brew group. |
| Brew Group to **open** Postion | `FN:0E` | `ok:` | Moves the brew group into the "open" position. |
| Brew Group to **grinding** Postion | `FN:0F` | `ok:` | Moves the brew group into the grinding position. |
| Brew Group to **unknown** Postion XYZ | `FN:13` | `ok:` | Moves the brew group into an currently unknown position. |
| Brew Group to **unknown** Postion XYZ | `FN:1B` | `ok:` | Moves the brew group into an currently unknown position. |
| Brew Group to **throw out position?!** Postion XYZ | `FN:1C` | `ok:` | Moves the brew group into the throw out position. |
| Brew Group to **brewing** Position | `FN:22` | `ok:` | Moves the brew group into the brewing position. |
| UNKNOWN | `FN:24` | `ok:` | - |
| UNKNOWN | `FN:25` | `ok:` | - |
| UNKNOWN | `FN:26` | `ok:` | - |
| UNKNOWN | `FN:27` | `ok:` | - |
| UNKNOWN | `FN:44` | `ok:` | - |
| UNKNOWN | `FN:45` | `ok:` | - |
| UNKNOWN | `FN:50` | `ok:` | - |
| Turn off | `FN:51` | `ok:` | Turns off the coffee maker. |
| UNKNOWN | `FN:54` | `ok:` | - |
| UNKNOWN | `FN:55` | `ok:` | - |
| UNKNOWN | `FN:60` | `ok:` | - |
| UNKNOWN | `FN:61` | `ok:` | - |
| UNKNOWN | `FN:62` | `ok:` | - |
| UNKNOWN | `FN:63` | `ok:` | - |
| UNKNOWN | `FN:64` | `ok:` | - |
| UNKNOWN | `FN:65` | `ok:` | - |
| UNKNOWN | `FN:66` | `ok:` | - |
| UNKNOWN | `FN:67` | `ok:` | - |
| UNKNOWN | `FN:70` | `ok:` | - |
| UNKNOWN | `FN:71` | `ok:` | - |
| UNKNOWN | `FN:72` | `ok:` | - |
| UNKNOWN | `FN:73` | `ok:` | - |
| UNKNOWN | `FN:80` | `ok:` | - |
| UNKNOWN | `FN:81` | `ok:` | - |
| UNKNOWN | `FN:88` | `ok:` | - |
| UNKNOWN | `FN:89` | `ok:` | - |
| Debug mode on | `FN:89` | `ku:`, `Ku:` pause `ku:`, `Ku:`, ... | Enables the debug mode. Sends continuously `ku:`, `Ku:`, ... Once an action like opening the hot water valve accrues, outputs information like percentage done. To disable it again disconnect the coffee maker from power.  |
| UNKNOWN | `FN:90` | `ok:` | - |
| UNKNOWN | `FN:99` | `ok:` | - |

### Coffee Brewing Sequence
* `FN:07` # Grind on
* Sleep 3 seconds # Determines how strong the coffee will be
* `FN:08` # Grind off
* `FN:22` # Brew group to brewing position
* `FN:0B` # Coffee press on
* Sleep 0.5 seconds # Compress the coffee
* `FN:0C` # Coffee press off
* `FN:03` # Turn on the coffee water heater
* `FN:01` # Coffee water pump on
* Sleep 2 seconds # Initial amount of water
* `FN:02` # Coffee water pump off
* `FN:04` # Turn off the coffee water heater
* Sleep 2 seconds # Allow the water to run everywhere
* `FN:03` # Turn on the coffee water heater
* `FN:01` # Coffee water pump on
* Sleep 40 seconds # 40 seconds of water result in 200 ml of coffee
* `FN:02` # Coffee water pump off
* `FN:04` # Turn off the coffee water heater
* `FN:0D` # Reset the brew group and throw out the old coffee grain

## Requirements
This repository now focuses on the ESPHome integration of the protocol implementation. To compile the firmware and flash it to
your ESP32 you need:
* Python 3.9 or newer with `pip`
* The [ESPHome CLI](https://esphome.io/guides/getting_started_command_line.html) (installable via `pip install esphome`)
* An ESP32 development board with a free hardware UART for communicating with the coffee maker

On Fedora, Raspberry Pi OS, or other Linux distributions install Python and the `esphome` package using your preferred package
manager and `pip`. Refer to the [ESPHome installation guide](https://esphome.io/guides/installing_esphome.html) for
platform-specific details.

## Building
Use ESPHome to build and upload the firmware. The custom component lives in `esphome/components/jutta_proto` and can either be
copied into an existing ESPHome project or referenced via [`external_components`](https://esphome.io/components/external_components.html).
After adding the component to your YAML configuration (see [docs/components/jutta_proto.md](docs/components/jutta_proto.md) for
an example) run:
```bash
esphome run your_config.yaml
```
ESPHome handles dependency management, compilation, and flashing of the firmware for you.

### XML-Zähler automatisch bereitstellen

Die Firmware kann die JURA-internen XML-Zähler zyklisch abfragen und als numerische Sensoren in Home Assistant zur Verfügung stellen. Dafür ist keine zusätzliche YAML-Konfiguration nötig – alle Sensorinstanzen werden während `setup()` angelegt, registriert und bleiben dauerhaft sichtbar.

**Konfiguration**

* Die J.O.E.-XML (nur `@TR:32`, `@TG:43`, `@TG:C0`) wird automatisch als Binärressource eingebettet.
* In der ESPHome-YAML muss lediglich `xml_mapping_path` auf die exportierte Datei zeigen.
* Keine Dateisysteme erforderlich.

**Ablauf**

* Nach Legacy-Handshake wird das Mapping aus dem eingebetteten String geladen.
* Alle 30 s werden die drei DB-Kommandos gepollt; Frames werden ohne feste Länge gelesen (CRLF oder Gap).
* Die numerischen Felder werden gemäß Mapping (offset/size/endian/scale) extrahiert und als Sensoren veröffentlicht.

**Troubleshooting**

* Log sollte **keine** „expected length“-Warnungen mehr enthalten.
* Bei „RX_DB timeout @TG:43/@TG:C0“: `xml_poll_interval_ms` erhöhen und inter-Command-Delay (150–200 ms) prüfen.
* Wenn Sensoren in HA fehlen: prüfen, ob `publish_state` nach Parse wirklich aufgerufen wird (Log hinzufügen).

#### Datenrahmen-Handling

**Beschreibung:** Der Jura-Dongle sendet auf DB-Kommandos wie `@TR:32` oft zuerst ein Echo-Frame mit dem ASCII des Befehls
und danach das Daten-Frame. Der Reader erkennt den encoded Trailer `DF FF DB DB FB FB DB DB`, entfernt ihn, unescaped
`0xDB xx → xx^0x20`, verwirft Echo-Frames und liest weiter, bis ein Daten-Frame mit der erwarteten Decoded-Länge vorliegt:
TR32=21, TG43=13, TGC0=13. Eine Echo-Unterdrückung bleibt bis zu 200 ms aktiv und verlängert sich bei jedem passenden Byte,
damit auch späte Antworten der Legacy-Bridge nicht fälschlich ausgewertet werden. Sensoren werden beim Start registriert,
Werte nur bei vollständigem Erfolg publiziert. Keine Textsensoren, keine HA-Templates erforderlich.

**Troubleshooting:**

* `decoded_len ≠ erwartet` → Reader wartet weiter innerhalb des Gesamt-Timeouts.
* `RX_DB timeout` → `JUTTA_XML_RX_TIMEOUT_MS` erhöhen und sicherstellen, dass während des Polls kein anderer Code
  `uart.read()` konsumiert.
* Sensoren fehlen → Pools in `setup()` registrieren, `set_internal(false)`.

`[1]`: https://uk.jura.com/en/homeproducts/accessories/SmartConnect-Main-72167
