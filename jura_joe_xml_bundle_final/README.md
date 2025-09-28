
# Jura J.O.E.-XML → UART (C & C#)

Dieses Repo-Ready-Bundle enthält alles, um **J.O.E.-ähnliche XML-Codes** per UART an die Jura-Maschine zu senden
und Antworten zu lesen. Der Codec entspricht der in der Original-Firmware beobachteten **DB-Kodierung** inkl.
8-Byte-Terminatorsignatur.

## Inhalt
- `C/`
  - `jura_proto.h` / `jura_proto.c`: DB-Encoder/Decoder, `jura_send_from_code(...)`, **`jura_query(...)`**
  - UART-**Stubs**: `jura_uart_write_bytes(...)`, `jura_uart_read_bytes(...)` musst du für deine Plattform implementieren.
- `CSharp/`
  - `Program.cs`: Konsolen-Tool, das `joe_codes_example.xml` lädt und Codes (plain oder db) sendet
- `joe_codes_example.xml`: Beispiel-XML
- `examples/query_example.c`: Minimalbeispiel (Desktop): zeigt `jura_query()`
- `CMakeLists.txt`: baut das Desktop-Beispiel (Linux/macOS/Windows mit MinGW)

## Senden: Plain vs. DB
- **Plain** (wie FW): wenn `(flags & 0x0A) == 0x08`
- **DB-kodiert** + Terminator: alle anderen Fälle

## Schnellstart (C, Desktop)
```bash
mkdir build && cd build
cmake ..
cmake --build . -j
./query_example
```
> Die UART-Stubs sind für Desktop stubbed: write = dump auf stdout, read = simuliert Timeout.
> Für ESP32/Hardware: implementiere die beiden Stubs mit `uart_write_bytes` / `uart_read_bytes` (ESP-IDF).

## Schnellstart (C#, SerialPort)
- In `CSharp/Program.cs` `PORT_NAME` und `BAUD` anpassen.
- `dotnet run` (oder `csc Program.cs` & ausführen).
- Eintrag aus `joe_codes_example.xml` wählen.

## Lizenz
Für Interoperabilitäts-/Reverse-Engineering-Zwecke an deinem eigenen Gerät gedacht.
