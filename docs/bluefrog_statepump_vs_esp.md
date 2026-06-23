# BlueFrog Statepump vs ESP Startup

Purpose: compare confirmed `bluefrog_machine_state_pump` branches from the classic BLE dongle firmware with the current ESPHome `jutta_proto` startup/statistics path. This document is intentionally diagnostic. It does not justify blind `@TF`, `@TV`, `@TP`, raw `0x26`, PMode, DFU, `@TV:81`, `@TV:82`, or `@TS:9x` probes.

Primary firmware references:
- `0x0001bd0c` `bluefrog_machine_state_pump`
- `0x00016c18` `machine_uart_send_line_encoded`
- `0x00016da4` `machine_uart_send_prefix_hex_line`
- `0x00016d6c` `machine_uart_sendf_line_encoded`
- `0x0001d8e4` `machine_rx_tf_status_cache_handler`
- `0x0001da94` `machine_rx_tv_progress_cache_handler`
- `0x0001908c` `ble_characteristic_event_dispatch`
- `0x000191e0` `copy_machine_cache_to_ble_value_buffer`
- Existing reverse-map notes in `docs/bluefrog_reverse_map.md`

Current ESP references:
- `esphome/components/jutta_proto/jutta_proto.cpp`
- Normal dongle startup state machine: `process_dongle_startup_`
- Startup TX trace: `trace_machine_tx_startup_`
- Original-like diagnostics: `original_like_flags88_`, `log_original_like_core_session_diff_`
- Replay isolation constants: `kAllowBluefrog26ReplayInStartupPath=false`, `enable_bluefrog_26_replay=false` default

## Statepump Comparison Table

| Firmware-Adresse / Branch | Firmware-Funktionsname | Bedingung / Flag | gesendete UART-Zeile oder 0x26-Plaintext | plain oder encoded | erwartete Antwort | aktueller ESP-Codepfad | fehlt im ESP ja/nein | Risiko | Empfehlung |
|---|---|---|---|---|---|---|---|---|---|
| `0x0001bd0c` entry / TX idle preconditions | `bluefrog_machine_state_pump` | TX marker/state idle; transport not busy; state subfield clear | none; branch gating only | n/a | n/a | ESP has independent startup states and DB transaction owner checks | Partial | Low | Keep as diagnostics only. Do not invent TX from this precondition. |
| `0x0001bd0c` state70 bit5/bit6 branch | `bluefrog_machine_state_pump` | `state+0x70` bit5 set. bit6 clear -> `@H1`; bit6 set -> `@T1`. | `@H1` or `@T1` | firmware line sender / encoded transport path | `@h...` or `@t1` depending branch | Normal startup sends `@T1`; Active-Safe test can send `@H1`; normal path does not require `@H1` | `@H1` yes in normal path; `@T1` no | Medium: `@H1` is identity/control-adjacent but not proven core-required | Do not add `@H1` to normal startup. Runtime already showed no passive host dialog from `@H1`. |
| `0x0001bd0c` flags_88 `0x400` branch | `bluefrog_machine_state_pump` | `flags_88 & 0x400`, produced by `@T2` handler | `@t2:<payload>` via prefix-hex/status branch | encoded by firmware helper | next identity/session traffic, including `@T3` path | ESP sends first handshake `@t2:8120000000`; normal startup sends `@t2:818811<word>0000` | No | Low | Keep current ESP behavior; log `t2_seen_0x400` in `firmware_flag_match`. |
| `0x0001bd0c` flags_88 `0x800` branch | `bluefrog_machine_state_pump` | `flags_88 & 0x800`, produced by `@T3` handler | cached/static line, identified in reverse map as `@t3` startup response | firmware line sender / encoded transport path | runtime `0x26`/startup continuation; sometimes `@t0`/`@T3` repeats | ESP sends `@t3` after `@T3` | No | Low | Keep current `@t3`; no raw replay after this in normal path. |
| `0x0001bd0c` flags_88 `0x40` branch | `bluefrog_machine_state_pump` | `flags_88 & 0x40`; first producer not fully resolved; cachewriter helper can set it later when `0x100` and `0x04` set and `0x200`/`0x40` clear | `@TR:37` prefix with zero payload | prefix-hex helper / encoded transport path | `@tr:37,<hex>` | ESP sends `@TR:37` after `@t3` quiet window | No | Low/Medium: producer is not fully mapped, but command is existing core gate path | Keep current gate path. Use diagnostics: `firmware_flag_match` and `cachewriter_gate`. |
| `0x0001bd0c` flags_88 `0x4000` branch | `bluefrog_machine_state_pump` | transfer flag set by `set_tv81_tv82_transfer_flags` / 0x1527 9x path | `@TV:81,<payload><checksum>` | formatted encoded line | transfer/control response, not live-cachewriter | ESP does not send | No | High: state-changing/progress-transfer path | Do not implement. |
| `0x0001bd0c` flags_88 `0x8000` branch | `bluefrog_machine_state_pump` | transfer flag set by `set_tv81_tv82_transfer_flags` / 0x1527 9x path | `@TV:82,<payload><checksum>` | formatted encoded line | transfer/control response, not live-cachewriter | ESP does not send | No | High: state-changing/progress-transfer path | Do not implement. |
| `0x0001bd0c` flags_88 `0x10000` branch | `bluefrog_machine_state_pump` | 0x1529 staged payload arm field; visible branch requires staged payload first byte nonzero | `@TD:<staged hex payload>` | prefix-hex encoded line | PMode/control follow-up | ESP does not use this for live | No | High: PMode/control path | Do not use as live trigger. |
| `0x0001bd0c` `machine_cache_base+0x3a` branch | `bluefrog_machine_state_pump` | single-byte field nonzero and related cache field clear | `@TP:<byte>` | prefix-hex encoded line | not proven as live/status response | ESP normal startup does not send `@TP:` | Yes | Medium: app heartbeat/control-adjacent; not a proven live trigger | Do not add `@TP:` until the 0x1529 follow-up path is fully proven safe and relevant. |
| `0x0001bd0c` dynamic `@TS` branch | `bluefrog_machine_state_pump` | stats/control staging fields and substate | `@TS:%02X` or `@TS:%02X,<payload>` | formatted encoded line | stats/session replies | ESP XML stats sends `@TS:01`, `@TR:32,..`, `@TG:43`, `@TG:C0`, `@TS:00` | No | Low for existing stats path | Keep stats path stable; replay must not interleave with stats. |
| `0x0001bd0c` BLE event branch | `bluefrog_machine_state_pump` | countdown/cache condition; no machine UART TX | BLE characteristic event, including `0x1527` in known branch | n/a | BLE notify/update | ESP has no real BLE GATT server equivalent here | Yes | Low for UART; unknown for BLE parity | Keep as documentation only; do not convert to UART probe. |
| `0x0001d8e4` RX handler | `machine_rx_tf_status_cache_handler` | Machine RX line begins `@TF`; handler updates status cache and dispatches BLE `0x1524` | none; it is RX/cachewriter, not query | n/a | BLE `0x1524` update/cache copy | ESP only logs/parses `@TF` if received | No query missing | Low for parser, High for querying | Do not send `@TF`. Log `cachewriter_gate` to show whether original-like acceptance conditions are present. |
| `0x0001da94` RX handler | `machine_rx_tv_progress_cache_handler` | Machine RX line begins `@TV`; handler updates progress cache and dispatches BLE `0x1527` | none; passive cachewriter path is RX, not query | n/a | BLE `0x1527` update/cache copy | ESP only logs/parses `@TV` if received | No query missing | Low for parser, High for querying | Do not send `@TV`. Keep waiting/logging for received cachewriter frames. |

## Flag Roles Used By Diagnostics

| Flag | Firmware role from reverse map | ESP diagnostic equivalent |
|---|---|---|
| `flags_88 0x04` | `ty:` / type/session context needed by cachewriter rearm helper | Set by RX `ty:` as `ORIGINAL_LIKE_FLAGS88_TY_CONTEXT` |
| `flags_88 0x40` | `@TR:37` arm/rearm branch in statepump | Not automatically set; logged as missing/or present |
| `flags_88 0x100` | core/session latch candidate from `@T2`/`@T3` conditions | Set by original-like `@T2`/`@T3` heuristic |
| `flags_88 0x200` | gate/session active after `@tr:37` | Set by RX `@tr:37` as `ORIGINAL_LIKE_FLAGS88_GATE_ACTIVE` |
| `flags_88 0x400` | `@T2` received/state available, enables `@t2:` branch | Set by RX `@T2` |
| `flags_88 0x800` | `@T3` received/identity available, enables `@t3` branch | Set by RX `@T3` |
| `flags_88 0x10000` | 0x1529/PMode staged payload arm for `@TD:` | Not mirrored for control; intentionally not used |

## Current Code Outcome

- Fixed raw `0x26` replay is not used in normal operation:
  - `CONF_ENABLE_BLUEFROG_26_REPLAY` defaults to `false`.
  - `kAllowBluefrog26ReplayInStartupPath` is `false`.
  - Even if `enable_bluefrog_26_replay` is set, the startup path logs `bluefrog_26_replay_not_started reason=isolated_from_startup_stats_path` and continues to `WAIT_T3`.
- The 180 s post-startup idle blocker is disabled:
  - `kPostStartupLiveIdleObserveMs = 0`
  - `kDelayBootStatsForLiveObserve = false`
- Normal startup remains:
  - `TY:`
  - `@T1`
  - `@t2:818811<word>0000`
  - `@t3`
  - `@TR:37`
- XML stats remain the only normal statistics path:
  - `@TS:01`
  - `@TR:32,00..0F`
  - `@TG:43`
  - `@TG:C0`
  - `@TS:00`

## Recommendation

For the next flash, use the existing startup sequence with the added diagnostics only. Success should be judged by:

- `xml_stats cycle=1 start` and normal stats frames appear without replay interleaving.
- `bluefrog_26_tx_replay` does not appear in normal boot/statistics logs.
- `firmware_flag_match ...` appears at startup ready/failure summaries.
- `cachewriter_gate ...` shows whether the original-like cachewriter rearm helper conditions would be satisfied.
- No `@TF`, `@TV`, `@TP`, `@TV:81`, `@TV:82`, `@TS:9x`, PMode, DFU, or fixed raw `0x26` replay command is sent by normal operation.
