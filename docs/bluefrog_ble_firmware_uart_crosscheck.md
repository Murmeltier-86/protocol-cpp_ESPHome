# BlueFrog BLE / Firmware / UART Crosscheck

Scope: classic BlueFrog/JURA BLE dongle path, using:

- Original UART/Jutta log: `/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/jura_jutta_decoded_resync.log_BLE DONGLE`
- Decompiled app: `/Users/flo/Documents/GitHub/J.O.E. JADX`
- Open ReVa/Ghidra firmware program: `/jura_nrf51822_flash_nrfsec.bin`
- Supporting generated notes:
  - `/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/reverse/analysis/original_ble_dongle_uart_timeline.md`
  - `/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/reverse/analysis/jadx_ble_uuid_map.md`
  - `/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/reverse/analysis/firmware_ble_uart_findings.md`

The Original-Dongle log has UART/Jutta events but no BLE/GATT/ATT records. The crosscheck therefore uses App/JADX for BLE operation identity, Firmware/ReVa for BLE-characteristic and UART/cache behavior, and the Original-Dongle log for the actual UART timing.

## Crosscheck Table

| Thema | App/JADX-Fundstelle | Firmware/Ghidra-Fundstelle | Original-Dongle-Log-Fundstelle | Interpretation | Sicherheit |
|---|---|---|---|---|---|
| `0x1524` Machine Status Cache | `/Users/flo/Documents/GitHub/J.O.E. JADX/app/src/main/java/joe_android_connector/src/connection/bluetooth/own/BluetoothGattCoffeeMachineCallbackImpl.java:49,80-96`; `StatusParser.java:43-52` | `machine_rx_tf_status_cache_handler` `0x0001d8e4`; `copy_machine_cache_to_ble_value_buffer` `0x000191e0`; `ble_characteristic_event_dispatch` `0x0001908c` | No BLE events in log; no visible `@TF` line found in extracted ASCII; binary `0x26` runtime begins at `2026-06-23 16:42:59.312` MACHINE_TO_DONGLE | App reads status cache; firmware updates it from `@TF` RX/cachewriter path; original log does not show visible `@TF`. | confirmed app/firmware, unknown log payload |
| `0x1527` Product Progress Cache | `BluetoothGattCoffeeMachineCallbackImpl.java:53,87-96`; `ProgressParser.java:56-65` | `machine_rx_tv_progress_cache_handler` `0x0001da94`; `copy_machine_cache_to_ble_value_buffer` `0x000191e0`; `ble_characteristic_event_dispatch` `0x0001908c` | No BLE events in log; no visible `@TV` line found; binary `0x26` frames present | App reads progress cache; firmware updates it from `@TV` RX/cachewriter path; original log does not show visible `@TV`. | confirmed app/firmware, unknown log payload |
| `0x1529` Control/PMode Write | `CoffeeMachineBleCommandParser.java:223-232` writes `00 7F 80` to UUID `5a401529...`; static payloads at `CoffeeMachineBleCommandParser.java:49,53` | `ble_char_1529_pmode_write_callback` `0x00018900`; `pmode_write_followup_or_cache_notify` `0x00017ef4` | No BLE write events in log; UART `0x26` late request candidates at `17:47:35.769` and `17:47:36.877` are not tied to BLE by this log | App write exists; firmware write callback exists; direct live-status UART refresh from `00 7F 80` is not proven. | confirmed app/firmware, unknown BLE↔UART timing |
| `0x1531` About Machine | `BluetoothGattCoffeeMachineCallbackImpl.java:50,175-181` with `AboutCMParser` | `register_bluefrog_gatt_services_and_characteristics` `0x00018368` dispatches `0x1531`; `machine_rx_t3_identity_handler` `0x0001da24` stores identity source | Visible `@T3:3C11EF532M V02.03` appears in log; see timeline source lines around startup and `reverse/analysis/original_ble_dongle_uart_timeline.md` | About Machine read corresponds to cached identity from `@T3`. | confirmed |
| `0x1533` Statistics Command | `CoffeeMachineBleCommandParser.java:132-150` creates command to UUID `5A401533...` | `pmode_write_followup_or_cache_notify` dispatches `0x1533`; init/default path dispatches `0x1533`; statistics machine-UART path not the focus of this log | No BLE event in log; no `@TS/@TR:32/@TG` visible in primary Original-Dongle log excerpt extraction | App/firmware characteristic exists; this log does not prove a statistics run. | likely |
| `0x1534` Statistics Data | `CoffeeMachineBleCommandParser.java:149-150` prepares read from UUID `5A401534...` after stats command | `pmode_write_followup_or_cache_notify` dispatches `0x1534`; init/default path dispatches `0x1534` | No BLE event in log | Statistics data characteristic exists; no direct live-status relation proven. | likely |
| `@TF` | App only sees parsed `0x1524` cache through `StatusParser`; no app UART `@TF` query | `machine_rx_tf_status_cache_handler` `0x0001d8e4`; called through ASCII dispatcher table; updates `0x1524` | Not visible as ASCII in primary log; not decoded from `0x26` frames in this pass | Real firmware RX/cachewriter prefix; not a dongle TX query. | confirmed firmware, absent visible log |
| `@TV` | App only sees parsed `0x1527` cache through `ProgressParser`; no app UART `@TV` query | `machine_rx_tv_progress_cache_handler` `0x0001da94`; updates `0x1527`; separate `@TV:81/82` transfer exists but is not passive live query | Not visible as ASCII in primary log | Real firmware RX/cachewriter prefix; not a passive query. | confirmed firmware, absent visible log |
| `@tr:37` | No direct app parser; session/gate is dongle-machine | `machine_rx_tr37_gate_handler` `0x0001d788`; `bluefrog_machine_state_pump` uses prefix-hex sender for gate-related TX | No visible ASCII `@tr:37` line was found in the primary Original-Dongle log. The log does show binary `0x26` runtime frames after `@t3`. | Gate/session handler exists in firmware; the primary log does not expose it as visible ASCII. | confirmed firmware, absent visible log |
| `@T3` | `0x1531` About Machine read uses cached identity via app `AboutCMParser` | `machine_rx_t3_identity_handler` `0x0001da24` stores code/string and sets flags | Visible ASCII `@T3:3C11EF532M V02.03` appears in log summary and timeline | Machine identity source for About Machine. | confirmed |
| `@t0` | No app-level BLE characteristic maps directly to `@t0` | `machine_rx_t0_state_handler` `0x0001d9b4` clears state and calls `FUN_0001bc6c` | No visible ASCII `@t0` line was found in the primary Original-Dongle log. | Startup/control handler exists in firmware, but this log does not show it as visible ASCII. | confirmed firmware, absent visible log |
| first runtime `0x26` after `@t3` | No BLE event present in log to correlate | `machine_uart_encode_0x26_frame` `0x0001a184`; `machine_uart_decode_0x26_frame` `0x0001a3cc`; `machine_uart_send_line_encoded` `0x00016c18` | `2026-06-23 16:42:57.040` DONGLE_TO_MACHINE `@t3`; `2026-06-23 16:42:59.312` MACHINE_TO_DONGLE `26 0C 64 ... 0D`; `2026-06-23 16:42:59.392` DONGLE_TO_MACHINE `26 DF F4 ... 0D` | Original dongle enters bidirectional 0x26 runtime after `@t3`. | confirmed UART |
| Settings-write candidates | App settings writes not identified in this log because BLE records are absent; `0x1529` is app control/write family | `0x1529` callback and statepump `@TP/@TD/@TS/@T...` branches exist; no specific setting linked to late frames | Late DONGLE_TO_MACHINE `0x26` frames at `17:47:35.769` and `17:47:36.877`, followed by machine response burst through `17:47:41.982` | Best timing candidate for settings changes, but no decoded setting kind and no BLE payload. | likely timing / unknown function |

## Required Questions

### 1. Welche App-Funktion liest `0x1524`?

`BluetoothGattCoffeeMachineCallbackImpl` constructs the `0x1524` command in its constructor. Evidence: `/Users/flo/Documents/GitHub/J.O.E. JADX/app/src/main/java/joe_android_connector/src/connection/bluetooth/own/BluetoothGattCoffeeMachineCallbackImpl.java:49,80-96`; parser `StatusParser` is attached at line 95.

### 2. Welche Firmware-Funktion bedient `0x1524`?

`machine_rx_tf_status_cache_handler` at `0x0001d8e4` updates the status cache and dispatches `0x1524`; `copy_machine_cache_to_ble_value_buffer(0)` at `0x000191e0` copies the parsed cache to BLE value storage; `ble_characteristic_event_dispatch` at `0x0001908c` dispatches the event.

### 3. Wird bei `0x1524` ein UART-/0x26-Refresh ausgelöst oder nur ein Cache gelesen?

Current evidence says cache read/update path, not read-triggered UART refresh. App evidence: `BluetoothGattCoffeeMachineCallbackImpl.java:80-96` creates a read/cache command with `StatusParser`. Firmware evidence: `machine_rx_tf_status_cache_handler` `0x0001d8e4` is a machine RX cachewriter, and `copy_machine_cache_to_ble_value_buffer` `0x000191e0` copies cache to BLE. No firmware read callback side-effect to UART was identified in the ReVa findings.

### 4. Welche App-Funktion liest `0x1527`?

`BluetoothGattCoffeeMachineCallbackImpl` constructs the `0x1527` command in its constructor. Evidence: `/Users/flo/Documents/GitHub/J.O.E. JADX/app/src/main/java/joe_android_connector/src/connection/bluetooth/own/BluetoothGattCoffeeMachineCallbackImpl.java:53,87-96`; BLE-mode `ProgressParser` is attached at line 96.

### 5. Welche Firmware-Funktion bedient `0x1527`?

`machine_rx_tv_progress_cache_handler` at `0x0001da94` updates progress/display cache and dispatches `0x1527`; `copy_machine_cache_to_ble_value_buffer(1)` at `0x000191e0` copies progress cache to BLE value storage; `ble_characteristic_event_dispatch` at `0x0001908c` dispatches the event.

### 6. Wird bei `0x1527` ein UART-/0x26-Refresh ausgelöst oder nur ein Cache gelesen?

Current evidence says cache read/update path, not read-triggered UART refresh. App evidence: `BluetoothGattCoffeeMachineCallbackImpl.java:87-96` creates the command with `ProgressParser`. Firmware evidence: `machine_rx_tv_progress_cache_handler` `0x0001da94` is a machine RX cachewriter; no read side-effect to UART is documented in the ReVa findings.

### 7. Welche App-Funktion schreibt `0x1529`?

`CoffeeMachineBleCommandParser.h()` writes payload `00 7F 80` to UUID `5a401529...`. Evidence: `/Users/flo/Documents/GitHub/J.O.E. JADX/app/src/main/java/joe_android_connector/src/connection/bluetooth/parser/CoffeeMachineBleCommandParser.java:223-232`.

### 8. Welche Firmware-Funktion verarbeitet `0x1529`?

`ble_char_1529_pmode_write_callback` at `0x00018900` processes BLE writes to `0x1529`. Its follow-up helper is `pmode_write_followup_or_cache_notify` at `0x00017ef4`.

### 9. Löst `0x1529` nachweislich einen UART-/0x26-Live-Status-Refresh aus?

No. Firmware evidence: `pmode_write_followup_or_cache_notify` `0x00017ef4` dispatches BLE characteristic events for `0x1524`, `0x1525`, `0x1527`, `0x1530`, `0x1538`, `0x1533`, and `0x1534`, but the function itself has no Machine-UART TX call. Original-log evidence: no BLE write records exist, so no BLE `0x1529` write can be correlated to a UART `0x26` frame from this log.

### 10. Welche Firmware-Funktion schreibt den Machine-Status-Cache?

`machine_rx_tf_status_cache_handler` at `0x0001d8e4` writes the Machine Status Cache from machine RX `@TF...`; it then calls `ble_characteristic_event_dispatch(0x1524)` and `copy_machine_cache_to_ble_value_buffer(0)`.

### 11. Welche Firmware-Funktion schreibt den Product-Progress-Cache?

`machine_rx_tv_progress_cache_handler` at `0x0001da94` writes the Product Progress / Display cache from machine RX `@TV...`; it then calls `copy_machine_cache_to_ble_value_buffer(1)` and dispatches `0x1527`.

### 12. Sind `@TF` und `@TV` sichtbare UART-Zeilen im Original-Dongle-Log?

No visible ASCII `@TF` or `@TV` lines were extracted from `/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/jura_jutta_decoded_resync.log_BLE DONGLE`. The log does contain 178 binary `0x26` frames; these were not decoded into `@TF/@TV` in this pass.

### 13. Sind `@TF` und `@TV` in der Firmware echte RX-Handler, Cachewriter-Namen oder beides?

Both as firmware concepts: they are real machine-RX handler prefixes and cachewriter paths. Evidence: `machine_rx_tf_status_cache_handler` at `0x0001d8e4` and `machine_rx_tv_progress_cache_handler` at `0x0001da94`. They are not confirmed dongle-to-machine query commands.

### 14. Welche `0x26`-Dongle→Machine-Frames im Original-Log sind nicht durch ESP-Replay abgedeckt?

This pass did not use ESP replay artifacts as a primary source, per instruction. From the Original-Dongle log alone, there are 48 DONGLE_TO_MACHINE `0x26` frames, all listed in `/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/reverse/analysis/original_ble_dongle_uart_timeline.md` under “All DONGLE_TO_MACHINE `0x26` Frames”. A formal “not covered by ESP replay” list requires a permitted replay-frame reference set. Qualitatively, the late-session frames at `2026-06-23 17:47:35.769` and `2026-06-23 17:47:36.877` are separate timing candidates from the early post-`@t3` startup/runtime exchange.

### 15. Welche dieser fehlenden Frames sind sichere Kandidaten für Settings-Write?

None are safe/confirmed settings-write candidates. The best timing candidate is the late-session burst:

- `2026-06-23 17:47:35.769` DONGLE_TO_MACHINE `26 36 C9 A3 97 55 1E 0D`
- `2026-06-23 17:47:36.877` DONGLE_TO_MACHINE `26 A8 69 F8 E3 AC 1B 9B 8E C2 BE 0D`
- followed by MACHINE_TO_DONGLE frames at `17:47:37.240`, `17:47:37.889`, `17:47:39.894`, `17:47:40.244`, `17:47:41.982`

Because no BLE write payload is present and no `0x26` payload fields are decoded, confidence is only `likely timing candidate`.

### 16. Welche dieser fehlenden Frames sind sichere Kandidaten für Live-Status/Display/Alerts?

None are safe/confirmed live-status/display/alert candidates. The machine response burst after the late dongle frames may contain status/cache/update data, but this is not decoded or linked to `0x1524/0x1527` in the primary log. Firmware confirms `@TF/@TV` cachewriter paths, but the Original-Dongle log does not show visible `@TF/@TV`.

### 17. Welche konkrete ESP-Codeänderung ist durch App + Firmware + Original-Log belegt?

Only diagnostic/non-semantic changes are strongly supported by all three sources:

- Keep a global UART/Jutta dispatcher that does not discard `0x26` frames.
- Keep logging of `0x26` frames with direction, timestamp, length, and hex.
- Keep statistics and replay/debug paths isolated.

No new live-status command is proven by App + Firmware + Original-Log together.

### 18. Welche konkrete ESP-Codeänderung ist ausdrücklich nicht belegt?

The following are not proven and should not be implemented as protocol behavior from this evidence:

- Sending `@TF` or `@TV` as queries.
- Adding new hardcoded `0x26` replay frames.
- Treating `0x1524` or `0x1527` reads as UART refresh triggers.
- Sending `@TP` / `00 7F 80` as a live-status trigger.
- Inferring live sensors from unknown binary `0x26` payloads.
- Sending PMode/DFU/`@TV:81`/`@TV:82`/`@TS:9x`/product/settings commands.

## Remaining Static Gaps

| Gap | Next required source/export |
|---|---|
| BLE event timing absent from Original-Dongle log | A real BLE HCI/GATT capture with timestamps, handles, payloads, reads/writes/notifies, synchronized to UART. |
| Binary `0x26` payload field meaning unknown | A stateful firmware decoder trace or ReVa export showing decoded plaintext/binary payload interpretation for the exact Original-Dongle frames. |
| Settings-write candidate not decoded | BLE write payload around `17:47:35` plus decoded `0x26` request/response semantics. |
| Live/status/display/alert frame identity not proven | Firmware path that maps binary `0x26` payloads into `0x1524/0x1527` buffers, or a synchronized BLE notification/cache update after a known UART frame. |
