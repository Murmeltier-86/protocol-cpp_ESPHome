# Jutta-Proto-Komponente

Die Jutta-Proto-Komponente verbindet ESPHome mit dem proprietären UART-Protokoll der JURA-Maschinen. Sie führt den klassischen Handshake aus, stellt die vorhandenen Automationsaktionen bereit und arbeitet mit den Funktionen aus `coffee_maker.*` zusammen.

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

## Aktionen

- `start_brew`: Löst ein Getränk aus der jeweiligen Getränke-Seite aus.
- `custom_brew`: Startet einen individuellen Bezug unter Angabe der Mahl- und Wasserzeit.
- `cancel_custom_brew`: Bricht einen laufenden individuellen Bezug ab.
- `switch_page`: Wechselt auf eine andere Getränke-Seite.

Weitere Details zu den Parametern finden sich direkt im Code der Komponente.
