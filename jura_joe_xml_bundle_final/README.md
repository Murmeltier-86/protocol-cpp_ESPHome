Jura J.O.E.-XML → UART bundle
=============================

Dieses Paket erlaubt dir, **Codes aus einer (J.O.E.-ähnlichen) XML** direkt über UART an die Maschine zu schicken – wahlweise
in C (z. B. auf ESP32) oder in C# (Windows/Linux, via System.IO.Ports). Der Code enthält die **DB-Kodierung**
inklusive 8-Byte-Terminatorsignatur, und kann optional auch Plain-ASCII senden.

Inhalt
------
- C/jura_proto.h / C/jura_proto.c
- CSharp/Program.cs
- joe_codes_example.xml
