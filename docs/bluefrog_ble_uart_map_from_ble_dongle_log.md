# BlueFrog BLE/UART Map From Original Dongle Log

Primary source for this pass:

`/Users/flo/Documents/GitHub/protocol-cpp_ESPHome/jura_jutta_decoded_resync.log_BLE DONGLE`

This is the requested Original-Dongle/Jutta log. It exists and was used as the primary timeline source. It contains decoded UART/Jutta `DEC_LINE` records for `DONGLE_TO_MACHINE` and `MACHINE_TO_DONGLE`, including `0x26` frames.

Important limitation: this file contains no textual BLE/GATT/ATT/handle/characteristic/read/write/notify records. A scan for BLE/GATT/ATT/handle/characteristic/UUID/read/write/notify and `0x1524/0x1527/0x1529/0x1531/0x1533/0x1534` found zero BLE-like non-UART lines. Therefore the table below can confirm UART timing from the Original-Dongle log, but cannot by itself confirm BLE handle-to-UART causality.

## Source Inventory

| Artifact | Status | Notes |
|---|---:|---|
| `jura_jutta_decoded_resync.log_BLE DONGLE` | found/used | Primary Original-Dongle UART/Jutta log for this document. |
| `docs/bluefrog_reverse_map.md` | found/used | Reverse-map evidence for BLE characteristic roles and cachewriter paths. |
| `/Users/flo/Documents/GitHub/J.O.E. JADX` | found/used | Decompiled Android app classes used for BLE UUIDs and parsers. |
| `factory.bin`, `ota_0.bin`, `ota_1.bin` | not found in searched roots | No firmware-dump binary evidence added in this pass. |
| `ota_0_strings.txt`, `ota_1_strings.txt` | not found in searched roots | No string-dump evidence added in this pass. |
| `drom_3f400020_command_strings.csv` | not found in searched roots | No ESP32 string table evidence added in this pass. |
| `jura_jutta_decoded_resync_26_frames.csv` / `jura_jutta_decoded_resync_26_sequences.md` | intentionally not used as primary | User explicitly excluded prior ESP-replay derived artifacts. |

## Correct Log Summary

| Metric | Value |
|---|---:|
| `DEC_LINE` records | 1010 |
| `DONGLE_TO_MACHINE` lines | 424 |
| `MACHINE_TO_DONGLE` lines | 586 |
| `0x26` frames total | 178 |
| `0x26` DONGLE_TO_MACHINE | 48 |
| `0x26` MACHINE_TO_DONGLE | 130 |

Non-`0x26` startup/control lines in this log include repeated `TY:`, `ty:EF532M V02.03`, `@T1`, `@t1`, `@T2:010001B228`, `@t2:8100000000`, `@t2:8120000000`, `@t2:818811B2280000`, `@T3:3C11EF532M V02.03`, and `@t3`.

The first `0x26` runtime frame appears after the dongle sends `@t3`:

| Timestamp | Direction | Frame |
|---|---|---|
| `2026-06-23 16:42:57.040` | DONGLE_TO_MACHINE | `@t3` |
| `2026-06-23 16:42:59.312` | MACHINE_TO_DONGLE | `26 0C 64 38 A8 86 D1 A5 40 AD 1A F9 74 A6 71 0E 0E 6C E8 3C 0D` |
| `2026-06-23 16:42:59.392` | DONGLE_TO_MACHINE | `26 DF F4 71 69 10 A2 CD 5F AD 0D` |

## BLE Characteristic Evidence From JADX / Reverse Map

| ID | Name / Role | App Evidence | Reverse-Map Evidence | UART Trigger Status |
|---|---|---|---|---|
| `0x1524` | Machine Status Cache | `BluetoothGattCoffeeMachineCallbackImpl` defines UUID `5a401524...`; constructor attaches `StatusParser`. | `docs/bluefrog_reverse_map.md`: value buffer `0x20002c11`, length `0x14`; updated by `machine_rx_tf_status_cache_handler`. | Read is cache-only from current reverse-map evidence; no UART refresh on read confirmed. |
| `0x1527` | Product Progress Cache | UUID `5a401527...`; constructor attaches BLE-mode `ProgressParser`. | value buffer `0x20002c25`, length `0x14`; updated by `machine_rx_tv_progress_cache_handler`. | Read is cache-only from current reverse-map evidence; no UART refresh on read confirmed. |
| `0x1529` | PMode / Control Write | `CoffeeMachineBleCommandParser.h()` writes payload `00 7F 80` to UUID `5a401529...`. | callback `ble_char_1529_pmode_write_callback`; `00 7F 80` follow-up path exists, but direct live/status UART trigger is not proven. | Write/control path, not safe to generalize into live trigger. |
| `0x1531` | About Machine | Initial app read uses `AboutCMParser`. | Source is cached/about data derived from `@T3` identity. | Cache/info read; no UART trigger confirmed. |
| `0x1533` | Statistics Command | App writes command payloads for statistic modes. | Statistics command/control characteristic. | Triggers stats path, separate from live/status cache. |
| `0x1534` | Statistics Data | App reads data after `0x1533` command/status. | Statistics data characteristic. | Data/cache read after stats command. |

## BLE/UART Correlation Table

Because the primary log has no BLE event records, all BLE direction/handle/payload cells below are marked `not present in log`. The UART side is confirmed from the requested Original-Dongle log.

| Timestamp | BLE direction | BLE handle / characteristic / command id | BLE payload hex | Decoded BLE meaning | Nearest UART direction | UART frame hex | Decoded UART line | Vermutete Funktion | Quelle | Sicherheit |
|---|---|---|---|---|---|---|---|---|---|---|
| `2026-06-23 16:42:57.040` | not present in log | not present in log | not present in log | not present in log | DONGLE_TO_MACHINE | `40 74 33 0D` | `@t3` | End of visible identity response before `0x26` runtime begins. | BLE-Dongle log line 4247 | confirmed |
| `2026-06-23 16:42:59.312` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 0C 64 38 A8 86 D1 A5 40 AD 1A F9 74 A6 71 0E 0E 6C E8 3C 0D` | binary `0x26`, no ASCII decode in log | First observed Original-Dongle runtime `0x26` frame after `@t3`. | BLE-Dongle log line 4259 | confirmed UART, unknown payload |
| `2026-06-23 16:42:59.392` | not present in log | not present in log | not present in log | not present in log | DONGLE_TO_MACHINE | `26 DF F4 71 69 10 A2 CD 5F AD 0D` | binary `0x26`, no ASCII decode in log | Dongle response/request after first machine `0x26` frame. | BLE-Dongle log line 4267 | confirmed UART, unknown payload |
| `2026-06-23 16:42:59.752` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 19 82 D8 25 46 C4 55 AA DD CD 08 7A 6F 61 24 9C 7C 69 AF 96 20 B8 C0 12 8F 41 0D` | binary `0x26`, no ASCII decode in log | Machine follow-up/runtime frame. | BLE-Dongle log line 4283 | confirmed UART, unknown payload |
| `2026-06-23 16:42:59.802` | not present in log | not present in log | not present in log | not present in log | DONGLE_TO_MACHINE | `26 69 71 92 44 31 62 0D` | binary `0x26`, no ASCII decode in log | Dongle short follow-up/request. | BLE-Dongle log line 4295 | confirmed UART, unknown payload |
| `2026-06-23 16:43:00.020` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 4B DE 9E B0 71 AB 58 78 87 8A 82 DD A5 41 A2 04 5C 9B DF 0D` | binary `0x26`, no ASCII decode in log | Machine runtime frame. | BLE-Dongle log line 4304 | confirmed UART, unknown payload |
| `2026-06-23 16:43:00.144` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 E7 35 A2 EA E1 2B 0D` | binary `0x26`, no ASCII decode in log | Machine short runtime/ack frame. | BLE-Dongle log line 4311 | confirmed UART, unknown payload |
| `2026-06-23 17:47:35.769` | not present in log | not present in log | not present in log | not present in log | DONGLE_TO_MACHINE | `26 36 C9 A3 97 55 1E 0D` | binary `0x26`, no ASCII decode in log | Late-session dongle request before a burst of machine `0x26` frames; possible user/settings interaction candidate, not proven. | BLE-Dongle log line 6126 | likely timing candidate, unknown function |
| `2026-06-23 17:47:36.877` | not present in log | not present in log | not present in log | not present in log | DONGLE_TO_MACHINE | `26 A8 69 F8 E3 AC 1B 9B 8E C2 BE 0D` | binary `0x26`, no ASCII decode in log | Second late-session dongle request before machine response burst. | BLE-Dongle log line 6132 | likely timing candidate, unknown function |
| `2026-06-23 17:47:37.240` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 D1 27 39 2F 45 7A FB 89 D9 88 5E 8F C1 87 1E 79 1C 96 94 27 1F EF 7C A7 64 E1 0D` | binary `0x26`, no ASCII decode in log | Machine response/follow-up after late dongle `0x26` requests. | BLE-Dongle log line 6147 | likely response, unknown payload |
| `2026-06-23 17:47:37.889` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 6D 16 9A 05 01 6E B5 FD 78 D7 C2 96 DE 45 58 44 D2 25 59 EF B7 F2 AF 56 0D` | binary `0x26`, no ASCII decode in log | Machine follow-up/cache/status candidate after late interaction. | BLE-Dongle log line 6161 | likely response/update, unknown payload |
| `2026-06-23 17:47:39.894` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 BA BE A7 7C C5 09 24 67 63 58 BD 61 F8 AF 9E 33 71 79 D0 4A DB B8 AF 20 0D` | binary `0x26`, no ASCII decode in log | Machine follow-up/cache/status candidate. | BLE-Dongle log line 6177 | likely response/update, unknown payload |
| `2026-06-23 17:47:40.244` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 EC FD 20 5B 80 8B FA A4 40 E6 BE FC 79 A7 07 9F 1B 9B 47 90 42 15 F0 75 F3 0D` | binary `0x26`, no ASCII decode in log | Machine follow-up/cache/status candidate. | BLE-Dongle log line 6189 | likely response/update, unknown payload |
| `2026-06-23 17:47:41.982` | not present in log | not present in log | not present in log | not present in log | MACHINE_TO_DONGLE | `26 4D 54 5B 71 3C 31 0D` | binary `0x26`, no ASCII decode in log | Short machine tail frame after response burst. | BLE-Dongle log line 6200 | likely response/update, unknown payload |

## Settings Write Candidates

The user noted that settings were changed during the capture. The primary log has no BLE write records, so these are UART timing candidates only.

| Candidate | Timestamp(s) | BLE Write Payload | UART Before / During | UART After | Cacheframe Changed? | Vermutete Setting-Art | Sicherheit |
|---|---|---|---|---|---|---|---|
| Early runtime exchange | `16:42:59.312` to `16:43:00.144` | not present in log | `@t3` at `16:42:57.040`, then first machine `0x26` len 21 | dongle `0x26` len 11 and len 8, then machine `0x26` len 28/21/8 | unknown | More likely runtime/session negotiation than user setting, because it starts immediately after `@t3`. | likely not setting, payload unknown |
| Late-session request burst | `17:47:35.769`, `17:47:36.877` | not present in log | two DONGLE_TO_MACHINE `0x26` frames: len 8 and len 12 | machine replies with len 28, 26, 26, 27, 8 over the next ~5 s | unknown | Best settings-change timing candidate in this log because dongle becomes active late and machine answers with a burst. Exact setting cannot be derived without BLE events or `0x26` payload decoding. | likely timing candidate |

## Evaluation

### 1. Which BLE request/write demonstrably generates UART `0x26` traffic?

None from this log alone. The requested Original-Dongle log contains confirmed bidirectional UART `0x26` traffic, but no BLE event records with handles, characteristics, payloads, reads, writes, or notifications. A BLE-to-UART trigger cannot be proven from this file alone.

### 2. Which BLE request reads only cache and generates no UART traffic?

From `docs/bluefrog_reverse_map.md` and JADX:

- `0x1524` is the Machine Status Cache read path. It is parsed by `StatusParser` and is documented as cache-only; no UART refresh on read is confirmed.
- `0x1527` is the Product Progress Cache read path. It is parsed by BLE-mode `ProgressParser` and is documented as cache-only; no UART refresh on read is confirmed.
- `0x1531` is About Machine/cache info derived from identity data; no UART trigger on read is confirmed.

This is not proven by BLE timing in the primary log, because BLE events are absent there; it is based on app/reverse-map evidence.

### 3. Is there a proven path for live-status/display/alerts?

Partially only:

- The Original-Dongle UART log proves that the real dongle enters a substantial bidirectional `0x26` runtime transport after `@t3`.
- The log does not decode those `0x26` payloads into status/display/alert fields.
- The reverse-map proves classic cache buffers for Machine Status (`0x1524`) and Product Progress (`0x1527`) and cachewriter handlers for decoded `@TF`/`@TV`.
- A BLE request/write that triggers live-status/display/alert UART traffic is not proven from this log.

### 4. Are `@TF`/`@TV` real UART lines or internal cachewriter names?

The reverse-map contains real machine-RX handler prefixes `@TF` and `@TV` in the classic firmware path, and documents them as cachewriter RX paths rather than dongle-to-machine queries. In the requested Original-Dongle log, no visible ASCII `@TF` or `@TV` lines appear. The observed runtime traffic is binary `0x26`, so live/status/cache information may be transported in binary `0x26` frames in this capture, or may decode to classic prefixes only with the correct stateful `0x26` decoder state. There is still no evidence to send `@TF` or `@TV` as queries.

### 5. Sensible ESP code changes after this evidence

- Keep the global `0x26` RX/TX logging and do not discard unknown binary frames.
- Keep normal statistics isolated from debug replay paths.
- Add offline tooling or diagnostics that clusters Original-Dongle `0x26` frames by length, direction, timing, and decoded/plaintext success.
- Only implement targeted live/status requests after a BLE/GATT log with handles/payloads can be correlated to the UART `0x26` timeline, or after the `0x26` payload format is statically decoded from firmware.

### 6. Code changes that should not be made yet

- Do not add new `@TP`, `@TF`, or `@TV` sends.
- Do not add additional hardcoded `0x26` replay frames to normal operation.
- Do not infer live sensors from unknown binary `0x26` payloads.
- Do not send PMode, DFU, `@TV:81`, `@TV:82`, `@TS:9x`, product, or settings writes.
- Do not treat `0x1524`/`0x1527` reads as machine UART refresh triggers without a new primary-source proof.

