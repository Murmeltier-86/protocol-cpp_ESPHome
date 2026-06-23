# BlueFrog BLE to UART Map

Scope: classic BlueFrog / BLE 0x152x and 0x153x path. This document separates confirmed cache reads from
confirmed or inferred machine-UART side effects. It intentionally does not define new runtime probes.

## Current conclusions

- The captured UART 0x26 CSV contains only Machine-to-Dongle frames. It does not contain Dongle-to-Machine 0x26
  requests, so it cannot by itself prove a BLE-request to UART-request mapping.
- The Android app initial BlueFrog flow reads `0x1531`, `0x1524`, and `0x1527`.
- Reverse-map evidence still describes `@TF` and `@TV` as machine-originated cachewriter frames in the classic
  cache path. No active `@TF` or `@TV` query is confirmed.
- Exact BLE payload `00 7F 80` on `0x1529` is not proven to send `@TP:007F`; the visible follow-up path dispatches
  BLE/cache events and has no confirmed UART TX.
- The replayed 0x26 frames currently produce only core/session responses such as `@tr:37`, `@T3`, and `@t0`.

## Mapping table

| BLE command/handle/id | Direction | Payload | Meaning | Corresponding UART / 0x26 frame | Decoded UART line | Source | Confidence |
|---|---|---|---|---|---|---|---|
| `0x1531` | App read | none | About Machine cache | Cache/source populated from machine identity handler | `@T3:<id><machine/version>` | JADX `BluetoothGattCoffeeMachineCallbackImpl.t()` queues read; Reverse-map `0x1531` source is `@T3` | confirmed |
| `0x1524` | App read / notify | none | Machine Status Cache | No read-triggered UART TX confirmed; cache updated by machine RX cachewriter | `@TF...` if machine emits it | JADX queues read and uses `StatusParser`; Reverse-map `machine_rx_tf_status_cache_handler` updates `0x1524` | confirmed |
| `0x1527` | App read / notify | none | Product Progress / Display Cache | No read-triggered UART TX confirmed; cache updated by machine RX cachewriter | `@TV...` if machine emits it | JADX queues read and uses `ProgressParser`; Reverse-map `machine_rx_tv_progress_cache_handler` updates `0x1527` | confirmed |
| `0x1529` | App write | `00 7F 80` | stayInBLE / app-presence follow-up | UART `@TP` not proven for this exact payload; follow-up byte `0x80` handled locally | none confirmed | JADX `CoffeeMachineBleCommandParser.h()` builds `007F80`; Reverse-map `pmode_write_followup_or_cache_notify(0x80)` | confirmed no direct UART TX in visible path |
| `0x1529` | App write | `00 7F A5` | Bootloader / DFU | Dangerous; not a status/progress path | none for live-status work | JADX bootloader handling in `BluetoothGattCoffeeMachineCallbackImpl.g()`; Reverse-map safety table | confirmed dangerous |
| `0x1529` | App write | `00 4D...` | PMode/settings write family | State-changing setting path; not a live-status probe | dynamic PMode/settings path, exact UART not used here | JADX `CoffeeMachineBleCommandParser.a/g()` and adapter PMode methods | confirmed state-changing |
| `0x1529` | App write | `00 47...` | Process navigation / OK / cancel family | Process/state-changing path | not used for live-status work | Reverse-map PMode/control payload table | likely |
| `0x1533` | App write/read chain | `00 00 01...`, `00 00 04...`, `00 00 08...` style from parser | Statistics command/status | Statistics command characteristic; firmware relation to `@TS/@TR/@TG` is inferred from known stats behavior | `@TS:01`, `@TR:32,<page>`, `@TG:43`, `@TG:C0`, `@TS:00` in ESP ASCII stats path | JADX `CoffeeMachineBleCommandParser.d/e/f`, Reverse-map stats section, ESP implementation | likely |
| `0x1534` | App read | none | Statistics data | Data cache/read result after `0x1533` command flow | statistics data, not live cache | JADX statistics parser chain; Reverse-map characteristic table | likely |
| `0x1528` | App/firmware callback | unknown / single byte | Update Product Progress adjacent callback | Corrected callback has no confirmed machine UART TX | none confirmed | Reverse-map corrected descriptor `0x1528 -> 0x18830` | confirmed no UART TX found |
| `0x1530` | App/firmware write/control | high-nibble `0x9x` path | Progress/control transfer path | Can arm `@TV:81/@TV:82`; state-changing, not safe live-status enable | `@TV:81...`, `@TV:82...` | Reverse-map corrected descriptor around `0x1530` / `0x18844` | confirmed state-changing |
| UART 0x26 observed CSV | Machine to Dongle | 20 clusters, lengths 8/9/26/27/28/29 | Unsolicited or periodic machine frames | No Dongle-to-Machine request present in CSV | not decoded in CSV | `jura_jutta_decoded_resync_26_frames.csv` and sequences doc | confirmed observation, unknown meaning |
| 0x26 replay frame 1 | ESP/Dongle to Machine diagnostic | `26 85 74 BD 75 E5 54 0D 0A` | Captured replay diagnostic | Machine answers with core/session 0x26 frames | observed decode includes `@tr:37` in ESP logs | ESP runtime log / current code path | confirmed diagnostic only |
| 0x26 replay frame 2 | ESP/Dongle to Machine diagnostic | `26 1C 0B 6A 29 B0 AA 7C 11 DE 0D 0A` | Captured replay diagnostic | Machine answers with core/session 0x26 frames | observed decode includes `@T3`, `@t0` in ESP logs | ESP runtime log / current code path | confirmed diagnostic only |

## Source notes

### JADX classic app flow

`BluetoothGattCoffeeMachineCallbackImpl.t()` queues these startup cache reads:

- `0x1531` About Machine, parsed by `AboutCMParser`.
- `0x1524` Machine Status Cache, parsed by `StatusParser`.
- `0x1527` Product Progress Cache, parsed by `ProgressParser`.

`StatusParser` turns the cache hex bytes into a `Status` object using the machine status mapping. `ProgressParser`
turns progress/display hex into a `Progress` object and, for BLE, removes the byte pair at positions 14/15 when the
BLE-specific marker byte is present.

### Reverse-map cachewriter relationship

The nRF reverse-map documents:

- `machine_rx_tf_status_cache_handler` parses machine RX `@TF...`, updates internal status cache, copies to BLE value
  `0x1524`, and dispatches a BLE characteristic event.
- `machine_rx_tv_progress_cache_handler` parses machine RX `@TV...`, updates internal progress/display cache, copies to
  BLE value `0x1527`, and dispatches a BLE characteristic event.
- Read handlers for `0x1524` and `0x1527` are cache-only in the mapped path.

### UART 0x26 log relationship

The current generated CSV has:

- `frames_total=20`
- all frames are `machine_to_dongle`
- all roles are `periodic_or_unsolicited_machine_frame`
- no captured Dongle-to-Machine 0x26 request rows

Therefore the CSV confirms that binary 0x26 machine frames exist, but does not identify which BLE write or app action
caused them.

## Open points

- No confirmed BLE request currently maps to a safe live-status UART request.
- No confirmed active `@TF` or `@TV` query exists.
- The binary/non-printable 0x26 clusters may contain modern cache/status payloads, but their field layout is still
  unknown.
- ESP32 `factory.bin`, `ota_0.bin`, `ota_1.bin`, `ota_*_strings.txt`, and `drom_3f400020_command_strings.csv` were not
  found in the checked workspace paths during this pass.
