# BlueFrog BLE Dongle Reverse Map

Firmware: `jura_nrf51822_flash_nrfsec.bin`

Scope: classic Jura/BlueFrog BLE dongle, nRF51822. This document mirrors the
Ghidra/ReVa markup applied to the open project and records which names are
confirmed, inferred, or still uncertain.

## 1. Overview

- The dongle is a BLE-to-machine-UART protocol adapter.
- Machine UART uses plain ASCII lines and a stateful `0x26` inner transport for selected `@T...` frames.
- `@TF...` and `@TV...` are machine-originated RX cachewriter frames in the classic cache path.
- BLE characteristic `0x1524` is the Machine Status Cache value.
- BLE characteristic `0x1527` is the Product Progress Cache value and also has a separate write/control callback.
- BLE characteristic `0x1529` is PMode/control write; known app heartbeat `00 7F 80` maps to machine UART `@TP:<key>7F`.
- App/mode frames such as `@mn`, `@mo`, `@me`, `@me1` are generated from an internal app-mode state block, not by the visible classic app startup/status flow seen so far.
- The exact trigger that makes the machine emit `@TF/@TV` remains unknown.

## 2. Ghidra Markup Status

Applied through ReVa:

- Function names were set with `set_function_prototype`.
- Important functions were tagged with `bluefrog_reverse_map`.
- Focused decompiler comments were added for cachewriters, codec, PMode, 0x1527 control, TV81/TV82 transfer, and passthrough FIFO.
- Several local variables were renamed in small, well-understood functions.

Limitations:

- The visible ReVa toolset did not expose a direct data-symbol rename command.
- Struct creation was not exposed; `apply_data_type` failed on descriptor/string regions because conflicting Ghidra data/instruction markup already exists.
- RAM/state labels below are therefore documented as intended names and field maps, but may still need manual Ghidra label/struct application.
- `0x1898c` could only be defined as a 1-byte function stub. Keep the `maybe_` name and treat this address as not fully resolved.

## 3. Function Map

| Address | New name | Role | Confidence | Notes |
|---|---|---|---|---|
| `0x000163cc` | `main_loop_or_scheduler` | Main loop/scheduler pump. Calls machine/BLE pumps. | inferred | Role from central caller/callee shape. |
| `0x0001bd0c` | `bluefrog_machine_state_pump` | Central machine state pump. Emits many dongle-to-machine frames based on flags/state. | confirmed_code | Consumes `bluefrog_state+0x88` flags and TX helpers. |
| `0x0001dd74` | `machine_ascii_dispatcher` | Dispatches decoded machine ASCII frames to handlers. | confirmed_code | Handler table/branches lead to `@TF`, `@TV`, `@T2`, `@T3`, `@tr`. |
| `0x00016c18` | `machine_uart_send_line_encoded` | Central dongle-to-machine line sender. | confirmed_code | Plain copy or `0x26` encode path depending on session and line. |
| `0x00016d6c` | `machine_uart_sendf_line_encoded` | Formats a line, appends CRLF, sends via line sender. | confirmed_code | Used by `@TV:81/@TV:82` branch. |
| `0x00016da4` | `machine_uart_send_prefix_hex_line` | Sends prefix plus hex payload plus CRLF. | confirmed_code | Machine TX helper. |
| `0x0001a3cc` | `machine_uart_decode_0x26_frame` | Decodes machine `0x26` inner UART frames. | confirmed_code | Table/index based decoder, in-place style. |
| `0x0001a184` | `machine_uart_encode_0x26_frame` | Encodes cleartext into `0x26` inner UART frame. | confirmed_code | Uses per-frame seed/table nibbles and escaping. |
| `0x0001a99c` | `ble_service_queue_pump` | BLE service/event queue pump. | inferred | Event-near queue routine. |
| `0x0001c428` | `ble_internal_packet_dispatcher` | Internal BLE packet dispatcher. | confirmed_code | Calls app-mode parser and FIFO/passthrough-related paths. |
| `0x0001c5d8` | `machine_passthrough_fifo_drain` | Drains queued passthrough bytes and sends complete line to machine. | confirmed_code | Stops on LF or `0x5f` bytes; does not invent commands. |
| `0x0001c404` | `machine_passthrough_fifo_init` | Initializes passthrough FIFO. | confirmed_code | FIFO setup. |
| `0x0001a770` | `fifo_write_bytes` | Writes bytes into FIFO. | confirmed_code | Used by passthrough/internal packet paths. |
| `0x0001c890` | `ble_chunk_response_pump` | Chunked BLE response pump. | inferred | BLE response buffering/chunking. |
| `0x0001c644` | `ble_chunk_response_build` | Builds chunked BLE response data. | inferred | Paired with response pump. |
| `0x0001e444` | `app_mode_state_machine_tx_drain` | Drains `app_mode_state` into `@mn/@mo/@me/@me1` or dynamic machine TX. | confirmed_code | Uses `0x20002db0`. |
| `0x0001e0b4` | `app_mode_payload_parser` | Parses internal app-mode payloads such as `?@m...` / `?@t...`. | confirmed_code | Checks payload byte 1 `@`, byte 2 `m` or `t`. |
| `0x0001e578` | `ble_mode_state_update_enqueue` | Enqueues mode-state updates. | inferred | Feeds app-mode state path. |
| `0x00018900` | `ble_char_1529_pmode_write_callback` | PMode/control write callback. | confirmed_code | Known `00 7F 80` stayInBLE path. |
| `0x00017ef4` | `pmode_write_followup_or_cache_notify` | Internal followup after PMode write final byte. | inferred | Called with final payload byte such as `0x80`. |
| `0x00018844` | `ble_char_1527_progress_control_write_callback` | Product Progress write/control callback. | confirmed_code | Copies up to `0x13` bytes to cache staging; `0x9x` sets TV81/TV82 flags. |
| `0x0001898c` | `maybe_ble_char_1528_update_product_progress_callback` | Candidate callback for 0x1528 Update Product Progress. | unknown | Defined as 1-byte stub; address/shape not fully understood. |
| `0x0001a6c0` | `set_tv81_tv82_transfer_flags` | Sets flags for `@TV:81` and `@TV:82` transfer TX. | confirmed_code | `mask&1 -> 0x4000`; `mask&2 -> 0x8000`. |
| `0x0001d8e4` | `machine_rx_tf_status_cache_handler` | Handles machine RX `@TF...` status/alert cachewriter. | confirmed_code | Updates cache and characteristic `0x1524`. |
| `0x0001da94` | `machine_rx_tv_progress_cache_handler` | Handles machine RX `@TV...` progress/display cachewriter. | confirmed_code | Updates cache and characteristic `0x1527`; also parses TV81 transfer response form. |
| `0x0001ced8` | `machine_rx_t2_state_handler` | Handles machine RX `@T2...` state/cache bytes. | confirmed_code | State/cache path. |
| `0x0001da24` | `machine_rx_t3_identity_handler` | Handles machine RX `@T3...` identity/handshake. | confirmed_code | Identity string such as EF532M. |
| `0x0001d788` | `machine_rx_tr37_gate_handler` | Handles machine RX `@tr:37,...` gate response. | confirmed_code | Sets post-gate/session flags. |
| `0x0001908c` | `ble_characteristic_event_dispatch` | Dispatches BLE characteristic/event updates by id. | confirmed_code | Jump table over 0x152x/0x153x ids. |
| `0x000191e0` | `copy_machine_cache_to_ble_value_buffer` | Copies parsed machine cache into BLE value buffers. | confirmed_code | Selector 0 -> 0x1524, selector 1 -> 0x1527. |
| `0x000192d0` | `maybe_prepare_tv_transfer_buffer` | Prepares TV transfer buffer for slot 1/2. | inferred | Writes TV81/TV82-related buffers. |
| `0x00018bc4` | `maybe_prepare_tv81_buffer` | Prepares TV81 buffer. | inferred | Writes `machine_cache_base+0x9c`. |
| `0x00018c38` | `maybe_prepare_tv82_buffer` | Prepares TV82 buffer. | inferred | Writes `machine_cache_base+0xb0`. |
| `0x00019628` | `maybe_prepare_tv_transfer_buffers` | Updates/propagates TV transfer buffers. | inferred | Related to TV81/TV82 buffer maintenance. |
| `0x0003ef60` | `bootloader_memset_worker` | Bootloader-local byte-fill helper. | confirmed_code | Tiny memset worker used by memzero/memset wrappers. |
| `0x0003ef6e` | `bootloader_memzero` | Bootloader-local zero-fill helper. | confirmed_code | Equivalent to `memset(dst, 0, length)`; clears DFU/control structs. |
| `0x0003ef72` | `bootloader_memset_return_dst` | Bootloader-local memset wrapper returning `dst`. | confirmed_code | No direct caller found, but exact wrapper behavior is clear. |
| `0x0003ef84` | `bootloader_load_u32_le_helper` | Helper for unaligned little-endian 32-bit loads. | confirmed_code | Called only by `bootloader_load_u32_le`. |
| `0x0003e76c` | `bootloader_load_u32_le` | Bootloader parser helper loading a little-endian `uint32`. | confirmed_code | Used by `FUN_0003e324` while parsing bootloader/DFU data structures. |
| `0x0003efde` | `bootloader_copy_words_region` | Bootloader C-runtime word-copy worker. | confirmed_code | Called through init table entry from `bootloader_runtime_init_copy_table_then_main`. |
| `0x0003efd6` | `thunk_bootloader_copy_words_region` | Thunk to `bootloader_copy_words_region`. | confirmed_code | Referenced as handler pointer in the bootloader init table. |
| `0x0003ef98` | `bootloader_runtime_init_copy_table_then_main` | Iterates bootloader init table, then enters bootloader main. | confirmed_code | Table entry copies `0x0003f1f4 -> 0x20002000`, length `0x124`. |
| `0x0003b1cc` | `bootloader_entry_after_reset` | Bootloader entry selected by reset/startup dispatcher. | confirmed_code | Called from reset handler path; writes hardware register `0x40000524`. |
| `0x0003b320` | `bootloader_main` | Nordic-style bootloader/DFU main loop. | confirmed_code | App reaches this mode via classic BLE PMode write `007FA5`. |

## 4. RAM and Struct Map

These are intended names for RAM/state areas. Apply manually as labels/structs if the ReVa data-symbol tool is unavailable.

| Address | Suggested name | Meaning | Confidence |
|---|---|---|---|
| `0x2000291c` | `bluefrog_state` | Main BlueFrog state block. | inferred |
| `0x2000291c+0x88` | `bluefrog_state_flags_88` | Gate/session/TV-transfer flags. Includes `0x4000`, `0x8000`. | confirmed_code |
| `0x20002a80` | `machine_cache_base` | Main machine cache/state area. | confirmed_code |
| `0x20002a80+0x50` | `progress_control_write_staging` | 0x1527 write staging buffer, max `0x13`. | confirmed_code |
| `0x20002a80+0x63` | `progress_control_write_length` | Length for 0x1527 write staging. | confirmed_code |
| `0x20002a80+0x9c` | `tv81_payload_string_buffer` | Source buffer for `@TV:81,...`. | confirmed_code |
| `0x20002a80+0xb0` | `tv82_payload_string_buffer` | Source buffer for `@TV:82,...`. | confirmed_code |
| `0x20002a80+0x190` | `machine_cache_flags_190` | Cache/progress flags; 0x1527 callback sets bits `0x0c`. | confirmed_code |
| `0x20002a93` | `progress_cache_source` | Source area copied toward progress value buffer. | inferred |
| `0x20002c11` | `ble_value_1524_status` | BLE value buffer for Machine Status Cache. | confirmed_code |
| `0x20002c25` | `ble_value_1527_progress` | BLE value buffer for Product Progress Cache. | confirmed_code |
| `0x200026ac` | `machine_passthrough_fifo` | FIFO state. | confirmed_code |
| `0x200026d0` | `machine_passthrough_fifo_buffer` | FIFO data buffer. | inferred |
| `0x20002db0` | `app_mode_state` | App/mode state consumed by `app_mode_state_machine_tx_drain`. | confirmed_code |
| `0x20002db0+0x04` | `app_mode_state_flags` | Branch flags for mode TX. | confirmed_code |
| `0x20002db0+0x05` | `app_mode_dynamic_string` | Dynamic machine TX string buffer. | confirmed_code |

### Suggested Structs

```c
struct BluefrogState {
    uint8_t unknown_00[0x88];
    uint32_t flags_88;   // Gate/session/TV-transfer flags; includes 0x4000 and 0x8000.
};
```

```c
struct MachineCache {
    uint8_t status_or_progress_base[0x50];
    uint8_t progress_control_write_staging[0x13];
    uint8_t progress_control_write_length;
    uint8_t unknown_64[0x38];
    char tv81_payload_string_buffer[0x14];
    char tv82_payload_string_buffer[0x14];
    uint8_t unknown_c4[0xcc];
    uint8_t flags_190;
};
```

```c
struct AppModeState {
    uint8_t mode;
    uint8_t unknown_01;
    uint8_t unknown_02;
    uint8_t unknown_03;
    uint8_t flags;
    char dynamic_string[];
};
```

Descriptor table reconstruction is still incomplete because the table region is already typed/marked by Ghidra in a conflicting way. Working model:

```c
struct BleCharacteristicDescriptor {
    void *uuid_ptr;
    void *service_ptr;
    uint16_t properties_or_flags;
    void *value_buffer;
    uint16_t value_length;
    void *callback;
    uint16_t characteristic_id;
};
```

Validate field sizes row-by-row before applying globally.

## 5. BLE Characteristic Table

| ID | Name | Buffer / callback | Direction | Notes |
|---|---|---|---|---|
| `0x1524` | Machine Status Cache | value `0x20002c11`, length `0x14` | Read/cache, notify/update | Read is cache-only. Updated by `machine_rx_tf_status_cache_handler`; update via `ble_characteristic_event_dispatch(0x1524)` and `copy_machine_cache_to_ble_value_buffer(0)`. |
| `0x1527` | Product Progress Cache | value `0x20002c25`, length `0x14`; write callback `0x18844` | Read/cache plus write/control | Read is cache-only. Updated by `machine_rx_tv_progress_cache_handler`; separate write callback is state-changing/progress-transfer. |
| `0x1528` | Update Product Progress | callback candidate `0x1898c` | Unknown | Not fully understood; visible function definition is only a 1-byte stub. |
| `0x1529` | PMode / Control Write | callback `ble_char_1529_pmode_write_callback` | Write | Known payloads include `00 7F 80` stayInBLE, `00 47 01` process next/OK, `00 47 FF` cancel/back, `00 4D...` PMode/settings, `00 7F 82...` PIN/auth, `00 7F A5` DFU/bootloader. |
| `0x1531` | About Machine | source: `@T3` identity | Read/cache/info | Initial app read. |
| `0x1533` | Statistics Command | command/control | Write/read status | Modes observed in app model: product counter, maintenance counter, maintenance status. |
| `0x1534` | Statistics Data | data | Read/cache | Statistics data pages. |
| `0x1538` | PMode / Settings Read | data/cache | Read | PMode/settings readback. |

## 6. 0x26 Encoder/Decoder

Summary:

```text
uart_0x26_encoding_stateful=YES
same_plaintext_can_have_different_raw=YES
state_source=table_index/random_seed_nibbles
decoder_function=machine_uart_decode_0x26_frame
encoder_function=machine_uart_encode_0x26_frame
```

Key observations:

- Encoded frames start with byte `0x26` (`'&'`).
- The byte after the start marker carries seed/table-index nibbles.
- If the seed byte is reserved (`0x00`, `0x0A`, `0x0D`, `0x1B`, `0x26`), it is escaped as `0x1B`, `byte ^ 0x80`.
- Payload bytes are transformed nibble-by-nibble through lookup tables using the seed nibbles and byte position.
- Decode stops at CR (`0x0D`).
- Escaped payload bytes use the same `0x1B`, `byte ^ 0x80` mechanism.
- No fixed raw frame should be used as a test oracle for a cleartext command.
- Reimplementations should send cleartext through the encoder.

Readable pseudocode, simplified:

```c
bool decode_0x26_frame(uint8_t *out, uint8_t *raw) {
    uint8_t seed = raw[1];
    if (seed == 0x1b) {
        seed = raw[2] ^ 0x80;
        raw += 1;
    }

    uint8_t seed_hi = seed >> 4;
    uint8_t seed_lo = seed & 0x0f;
    uint8_t pos = 0;

    for (uint8_t *p = raw + 2; *p != 0x0d; p++) {
        uint8_t b = *p;
        if (b == 0x1b) {
            b = *++p ^ 0x80;
        }

        uint8_t hi = decode_nibble((b >> 4), seed_hi, seed_lo, pos + 0);
        uint8_t lo = decode_nibble((b & 0x0f), seed_hi, seed_lo, pos + 1);
        *out++ = (hi << 4) | lo;
        pos += 2;
    }

    return true;
}
```

```c
size_t encode_0x26_frame(const char *plain, uint8_t *out, size_t out_len) {
    out[0] = 0x26;
    uint8_t seed_hi = random_nibble();
    uint8_t seed_lo = random_nibble();
    uint8_t seed = (seed_hi << 4) | seed_lo;
    write_escaped(&out, seed);

    for (size_t pos = 0; plain[pos] != 0; pos++) {
        uint8_t b = plain[pos];
        uint8_t enc = encode_byte_by_table(b, seed_hi, seed_lo, pos);
        write_escaped(&out, enc);
    }

    *out++ = 0x0d;
    return out_len_used;
}
```

The pseudocode describes control flow and data dependency, not exact table math.

## 7. App Standard Path

Classic Android app startup reads are cache reads:

- `0x1531`: About Machine.
- `0x1524`: Machine Status Cache.
- `0x1527`: Product Progress Cache.

Observed in JADX:

- `BluetoothGattCoffeeMachineCallbackImpl.t()` enqueues reads for 0x1531, 0x1524, 0x1527.
- No visible classic app write to 0x1527 was found.
- 0x1528 exists as a named characteristic/update path, but no visible classic app use was found in the examined callgraph.

## 8. Machine-UART Handshake

Startup/gate strings and dynamic frames observed in firmware include:

- `TY:`
- `@t2:...`
- `@t3`
- `@T0`
- `@T1`
- `@H1`
- `@TR:37`

`machine_rx_tr37_gate_handler` handles `@tr:37,...` and sets the post-gate/session state. The gate state enables later inner transport behavior through `machine_uart_send_line_encoded`.

## 9. @TF/@TV Cachewriter Path

### `machine_rx_tf_status_cache_handler`

Role:

```text
Handles machine RX frame @TF...
This is not a query and not sent by the dongle.
It updates the status/alert cache and then updates BLE characteristic 0x1524.
```

Evidence:

- Called from machine ASCII dispatcher for `@TF`.
- Parses payload into status cache.
- Calls `ble_characteristic_event_dispatch(0x1524)` through a constant.
- Calls `copy_machine_cache_to_ble_value_buffer(0)`.

Conclusion:

```text
tf_is_rx_only=YES
tf_active_query_found=NO
```

### `machine_rx_tv_progress_cache_handler`

Role:

```text
Handles machine RX frame @TV...
This is the passive progress/display cachewriter path.
It updates the progress/display cache and then updates BLE characteristic 0x1527.
```

Evidence:

- Called from machine ASCII dispatcher for `@TV`.
- Copies normal progress/display payload into cache bytes.
- Calls `copy_machine_cache_to_ble_value_buffer(1)`.
- Calls `ble_characteristic_event_dispatch(0x1527)` through a constant.

Important distinction:

- Passive `@TV...` RX cachewriter is not a query.
- Separate dongle-to-machine `@TV:81/@TV:82` transfer/control frames exist and are generated by `bluefrog_machine_state_pump` when flags `0x4000/0x8000` are set.

Conclusion:

```text
tv_is_rx_only=YES for passive cachewriter path
tv_active_query_found=NO
```

## 10. 0x1527 Write / TV81 / TV82 Path

`ble_char_1527_progress_control_write_callback`:

- Caps write length at `0x13`.
- Copies payload to `machine_cache_base+0x50`.
- Stores length at `machine_cache_base+0x63`.
- Sets cache flags `machine_cache_base+0x190 |= 0x0c`.
- If `(payload[0] >> 4) == 9`, calls `set_tv81_tv82_transfer_flags(3)`.

`set_tv81_tv82_transfer_flags(3)`:

- Sets `bluefrog_state_flags_88 |= 0x4000`.
- Sets `bluefrog_state_flags_88 |= 0x8000`.

`bluefrog_machine_state_pump`:

- `0x4000` branch sends:

```text
@TV:81,<tv81_payload><checksum>\r\n
```

- Source buffer: `machine_cache_base+0x9c`.
- `0x8000` branch sends:

```text
@TV:82,<tv82_payload><checksum>\r\n
```

- Source buffer: `machine_cache_base+0xb0`.
- Format string model:

```text
"%s%01X,%s%02X"
prefix="@TV:8"
slot=1 or 2
payload=<tv81/tv82 payload string>
checksum=sum_ascii(payload)
```

Safety notes:

- This is not a confirmed live-status enable path.
- `payload[0]` is later used as a command/control byte in nearby `@TS:%02X` paths.
- No visible classic Android app write to 0x1527 was found.
- No safe runtime payload can be defined from current evidence.

Status:

```text
char_1527_9x_path_confirmed=YES
app_write_to_1527_found=NO
can_define_safe_test_payload=NO
risk=state_changing/progress_transfer
```

## 11. App-Mode Path

Functions:

- `app_mode_payload_parser`
- `app_mode_state_machine_tx_drain`

Summary:

```text
Legacy/internal app-mode path.
Accepts payloads where payload[1] == '@' and payload[2] is 'm' or 't'.
Prepares app_mode_state at 0x20002db0.
Later app_mode_state_machine_tx_drain sends @mn, @mo, @me, @me1 or dynamic strings.
```

Known machine TX frames from this family:

- `@mn`
- `@mo`
- `@me`
- `@me1`
- dynamic strings from `app_mode_dynamic_string`

No direct visible classic Android startup/status write path was found for these payloads.

## 12. FIFO / Passthrough Path

Functions:

- `machine_passthrough_fifo_init`
- `fifo_write_bytes`
- `machine_passthrough_fifo_drain`
- `ble_internal_packet_dispatcher`

Summary:

```text
Raw machine passthrough FIFO drain.
Does not invent commands.
Only sends complete lines that another function has queued.
```

Details:

- Drains FIFO `0x200026ac`.
- Stops on LF (`0x0a`) or max `0x5f` bytes.
- NUL-terminates the collected line.
- Sends the line through `machine_uart_send_line_encoded`.
- Source is internal/legacy `0x7e` packet/passthrough path, not the normal visible classic app startup/status path.

## 13. What Is Safely Excluded

- `@TF` as dongle-to-machine query: no confirmed TX path found.
- `@TV` as passive status query: no confirmed TX path found.
- `0x1524` read as refresh trigger: cache-only from current evidence.
- `0x1527` read as refresh trigger: cache-only from current evidence.
- `0x1527` `0x9x` write as safe status enable: not safe; state-changing/progress-transfer.
- Fixed raw `0x26` frames as test material: not valid, because encoding is state/table-index based.

## 14. Open Points

- Exact trigger for machine-originated `@TF/@TV` remains unknown.
- `0x1528` callback is not fully understood; `0x1898c` currently looks like an unresolved stub/thunk/alignment issue.
- No active `@TF/@TV` query was found.
- No safe runtime test payload for `0x1527/0x9x` is defined.
- `event_driven_likely=YES`, but the concrete machine event or mode state that emits `@TF/@TV` is not yet proven.
- Descriptor table field sizes should be verified row-by-row before applying a global `BleCharacteristicDescriptor` struct in Ghidra.

## 15. Highest-Priority Next Targets

1. Resolve the descriptor/table row around `0x1528` and `0x1898c` without forcing a 1-byte stub.
2. Continue from `machine_ascii_dispatcher` and all state guards that gate `@TF/@TV` handler side effects.
3. Trace all writers of `bluefrog_state_flags_88` that may enable/disable status/progress acceptance.
4. Trace all callers and writes feeding `machine_cache_base+0x9c` and `+0xb0` to understand TV81/TV82 transfer payload provenance.
5. Compare with the WLAN firmware only after this BLE map is saved and stable.

## 16. Bootloader / DFU Runtime Init

The bootloader code lives in the `0x0003b000` range. The function previously
named `FUN_0003efde` is not a Jura machine protocol handler. It is a tiny
C-runtime style copy worker used by the bootloader startup path.

Neighboring helper functions in the `0x0003ef60..0x0003ef97` range are also
generic bootloader runtime/parser helpers:

```text
0x0003ef60 bootloader_memset_worker
  Writes one byte value to dst for length bytes.

0x0003ef6e bootloader_memzero
  Wrapper around bootloader_memset_worker(dst, length, 0).
  Used by DFU/bootloader routines to clear local control structs before SVC calls.

0x0003ef72 bootloader_memset_return_dst
  Wrapper around bootloader_memset_worker(dst, length, value), then returns dst.
  No direct caller found in the current program graph.

0x0003ef84 bootloader_load_u32_le_helper
  Reads src[3], src[2], src[1], src[0] into an accumulator.
  Called by bootloader_load_u32_le.

0x0003e76c bootloader_load_u32_le
  Returns a little-endian uint32 from a byte pointer.
  Used by FUN_0003e324 while parsing bootloader/DFU data structures.
```

Call chain:

```text
Reset handler 0x000006d0
  -> bootloader_entry_after_reset 0x0003b1cc
     -> bootloader_runtime_init_copy_table_then_main 0x0003ef98
        -> thunk_bootloader_copy_words_region 0x0003efd6
           -> bootloader_copy_words_region 0x0003efde
        -> bootloader_main 0x0003b320
```

Init table:

```text
table_start = 0x0003f1d4
table_end   = 0x0003f1f4
entry_size  = 0x10

entry[0]:
  src        = 0x0003f1f4
  dst        = 0x20002000
  byte_count = 0x124
  handler    = 0x0003efd6 / thunk_bootloader_copy_words_region
```

`bootloader_copy_words_region` pseudocode:

```c
void bootloader_copy_words_region(uint32_t *src, uint32_t *dst, int byte_count) {
    while (byte_count != 0) {
        *dst++ = *src++;
        byte_count -= 4;
    }
}
```

Android app tie-in:

- `CoffeeMachineBleCommandParser` defines `f40473a = ByteOperations.g("007FA5")`.
- `CoffeeMachineAdapterBle.Q(...)`, decompiled as `sendFrogToBootloader`, writes that payload to the classic BlueFrog PMode characteristic `0x1529`.
- `BluetoothGattCoffeeMachineCallbackImpl.g(...)` special-cases writes to characteristic `E` whose decoded hex ends in `7FA5`, logs bootloader command status, drops the normal connection, and starts scanning for the bootloader.
- `DFUBleCommandParser` then builds the Nordic DFU command sequence over DFU control/packet characteristics.

Conclusion:

```text
FUN_0003efde = bootloader_copy_words_region
role = bootloader runtime RAM/data initialization helper
not_machine_uart_protocol = YES
not_live_status_related = YES
app_trigger_to_reach_bootloader = BLE 0x1529 payload 00 7F A5
risk = dangerous/DFU_bootloader_mode
```

## 17. Original Dongle Startup / Identity Emulation

This section focuses only on the original classic BlueFrog dongle startup,
identity, and session initialization against the Jura machine. It deliberately
does not treat `@TF` or `@TV` as queries.

### Confirmed Startup/Identity Code Points

```text
0x000163cc main_loop_or_scheduler
  Initializes UART/transport, app-mode defaults, passthrough FIFO, BLE service
  defaults, timing defaults, and then repeatedly calls bluefrog_machine_state_pump.
  Sets bluefrog_state.flags_88 bit 0x40000 during startup.
  Confidence: confirmed_code for the init flow; inferred for exact bit meaning.

0x0001bd0c bluefrog_machine_state_pump
  Main machine-side state pump. Reads machine RX, dispatches ASCII frames, drains
  passthrough/app-mode TX, and sends startup/session/identity/control frames based
  on bluefrog_state and machine_cache flags.
  Confidence: confirmed_code.

0x00016c18 machine_uart_send_line_encoded
  Central dongle->machine line sender. Copies short plain lines or encodes selected
  @T... lines as 0x26 inner frames depending on transport/session guard state.
  Confidence: confirmed_code.

0x00016d6c machine_uart_sendf_line_encoded
  snprintf-style formatter, appends CRLF, then calls machine_uart_send_line_encoded.
  Confidence: confirmed_code.

0x00016da4 machine_uart_send_prefix_hex_line
  Builds prefix + payload-as-uppercase-hex + CRLF, then calls machine_uart_send_line_encoded.
  Confidence: confirmed_code.

0x0001dd74 machine_ascii_dispatcher
  Matches the first three bytes of machine RX lines against a dispatcher table and
  calls the corresponding handler. The dispatcher table starts at 0x00024c2c.
  Confidence: confirmed_code.
```

### Confirmed Startup TX Literals

These literals were verified from the flash literal block around `0x00024950`.

| Address | Frame / format | Role | Sender / consumer | Confidence |
|---|---|---|---|---|
| `0x00024970` | `@hpd\r\n` | Host/adapter response frame | referenced by state/host paths | confirmed_bytes |
| `0x00024978` | `@hf\r\n` | Host/adapter response frame | used by multiple host subpaths | confirmed_bytes |
| `0x00024980` | `@hp6\r\n` | Host/adapter response frame | state pump retry/static line path | confirmed_bytes |
| `0x00024988` | `@hp0\r\n` | Host/adapter response frame | state pump retry/static line path | confirmed_bytes |
| `0x00024990` | `TY:\r\n` | Machine type query | state pump prefix/static startup path | confirmed_bytes |
| `0x00024998` | `@t2:` | Prefix for `@t2:<hex>` response/transfer | prefix-hex sender | confirmed_bytes |
| `0x000249a0` | `@t3\r\n` | Startup/session response line | state pump static line path | confirmed_bytes |
| `0x000249a8` | `@H1\r\n` | Host/session identity request/line | state pump static startup path | confirmed_bytes |
| `0x000249b0` | `@T0\r\n` | Startup/control line | state pump static startup path | confirmed_bytes |
| `0x000249b8` | `@T1\r\n` | Startup/control line | state pump static startup path | confirmed_bytes |
| `0x000249c0` | `@TR:37` | Gate/session command format without CRLF in literal | state pump cached line path | confirmed_bytes |
| `0x000249c8` | `@tv` | Lowercase tv path / app-mode related | state/app-mode paths | confirmed_bytes |
| `0x000249cc` | `@T%c` | Dynamic `@T?` format | state pump dynamic T path | confirmed_bytes |
| `0x000249d4` | `@T%c:%02X` | Dynamic `@T?:xx` format | state pump dynamic T path | confirmed_bytes |
| `0x000249e0` | `@T%c:%02X,%s` | Dynamic `@T?:xx,<payload>` format | state pump dynamic T path | confirmed_bytes |
| `0x000249f0` | `%s%01X,%s%02X` | TV transfer payload/checksum format fragment | TV81/TV82 transfer path | confirmed_bytes |
| `0x00024a00` | `@TV:8` | TV81/TV82 transfer prefix | state pump transfer path | confirmed_bytes |
| `0x00024a08` | `@TS:%02X` | Dynamic TS command | progress/control path | confirmed_bytes |
| `0x00024a14` | `@TS:%02X,%s` | Dynamic TS command with payload | progress/control path | confirmed_bytes |
| `0x00024a20` | `@TP:` | PMode/stayInBLE prefix | 0x1529/state pump path | confirmed_bytes |
| `0x00024a28` | `@TD:` | TD payload prefix | state pump transfer path | confirmed_bytes |
| `0x00024a30` | `@hp4\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024a38` | `@hp5\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024a40` | `@hp7\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024a48` | `@hy:TT214H V05.08F\r\n` | Dongle identity/version | host identity path | confirmed_bytes |
| `0x00024a60` | `@ho\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024d54` | `@ok:\r\n` | ACK response, e.g. `@GB -> @ok:` | `0x0001d064` small ACK handler | confirmed_bytes |
| `0x00024d5c` | `@hl:BL_nRF51822 V00.00 TT214H,00\r\n` | Bootloader/host info response | host info path | confirmed_bytes |
| `0x00024d80` | `@hp1\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024d88` | `@hpv\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024d90` | `@ht\r\n` | Host/adapter response frame | host paths | confirmed_bytes |
| `0x00024d98` | `@hr:xxxx\r\n` | Host response template / placeholder | host paths | confirmed_bytes |
| `0x00024da4` | `@hw:ok\r\n` | Host write ACK | host paths | confirmed_bytes |

### Dispatcher Table Highlights

The machine RX dispatcher table at `0x00024c2c` confirms these startup/identity
RX prefixes and handlers:

| RX prefix | Handler pointer | Meaning |
|---|---:|---|
| `@HY` | `0x0001d080` | Host identity subdispatcher |
| `@t1` | `0x0001ce78` | Startup response |
| `@t0` | `0x0001d9b4` | Startup/session response |
| `@T2` | `0x0001ced8` | Machine startup/session data handler |
| `@T3` | `0x0001da24` | Machine identity/version handler |
| `@ts` | `0x0001ceb8` | Statistics/session response |
| `@tr` | `0x0001d788` | Gate/session response dispatcher |
| `@tg` | `0x0001d6cc` | Dynamic/app-mode stage path |
| `ty:` | `0x0001d9d0` | Machine type response; clears/sets startup flags |
| `@HI/@HB/@HM/@HS/@HP/@HT/@HF/@HL/@HC/@HZ/@HH/@HU/@HR/@HW` | `0x0001d080` | Host subdispatcher keyed by third byte |
| `@GB` | `0x0001d064` | Sends `@ok:\r\n`, sets a transport marker, waits for TX idle |

The `@H?` host subdispatcher at `0x0001d080` uses `line[2] - 'B'` as index,
checks `index < 0x18`, and dispatches through the 24-entry table at
`0x00024a68`. Entries are 32-bit even branch-label addresses, not Thumb-bit
function pointers. Confidence: confirmed_code and confirmed_bytes.

| RX prefix | Index | Raw table bytes | Target | Confirmed response / behavior | Safe to emulate | Confidence |
|---|---:|---|---:|---|---|---|
| `@HB` | 0 | `06 d1 01 00` | `0x0001d106` | Sends `@ok:\r\n`, sets `bluefrog_state+0x19 = 1`, waits for TX idle | YES, exact RX only | confirmed_code |
| `@HC` | 1 | `18 d1 01 00` | `0x0001d118` | Parses two 6-digit values from request and replies dynamic `@hc:<4hex>\r\n` | NO, dynamic/request-dependent | confirmed_code |
| `@HD` | 2 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HE` | 3 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HF` | 4 | `a4 d0 01 00` | `0x0001d0a4` | Stateful handler; parses hex at `line+4`, may send `@hf\r\n`, mutates startup/session flags | CAUTION, observed original flow only | confirmed_code |
| `@HG` | 5 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HH` | 6 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HI` | 7 | `96 d1 01 00` | `0x0001d196` | Builds dynamic `@hi:<8hex><8hex>\r\n` from device identity/cache values | YES if dynamic values are known | confirmed_code |
| `@HJ` | 8 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HK` | 9 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HL` | 10 | `4a d2 01 00` | `0x0001d24a` | Copies template `@hl:BL_nRF51822 V00.00 TT214H,00\r\n` and patches version/identity digits | YES, identity-only if exact RX | confirmed_code |
| `@HM` | 11 | `b2 d2 01 00` | `0x0001d2b2` | State-changing app-mode/control handler; no direct static reply in default branch | NO | confirmed_code |
| `@HN` | 12 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HO` | 13 | `ee d0 01 00` | `0x0001d0ee` | no-op in this machine host table. The literal `@ho\r\n` belongs to an internal BLE/chunk path, not this handler. | NO response | confirmed_code |
| `@HP` | 14 | `d4 d2 01 00` | `0x0001d2d4` | Subdispatcher using `line[3]-'0'`; handles `HP0/HP1/HP2/HP?/HPD/HPV` and stateful paths | NO blanket auto-reply | confirmed_code |
| `@HQ` | 15 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HR` | 16 | `e8 d2 01 00` | `0x0001d2e8` | Copies template `@hr:xxxx\r\n`, patches four hex digits from internal byte when request matches `00...` | YES only if dynamic value known | confirmed_code |
| `@HS` | 17 | `3e d3 01 00` | `0x0001d33e` | Clears `app_mode_state.flags` bit 0; no TX | NO | confirmed_code |
| `@HT` | 18 | `4a d3 01 00` | `0x0001d34a` | Subdispatcher using `line[4]-'0'`; `HT0/HT1` send `@ht\r\n`, other entries mutate flags or call reset/config paths | NO blanket auto-reply | confirmed_code |
| `@HU` | 19 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HV` | 20 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HW` | 21 | `5e d3 01 00` | `0x0001d35e` | Parses hex-ish payload, updates internal global state, sends `@hw:ok\r\n` | CAUTION, write ACK/control | confirmed_code |
| `@HX` | 22 | `ee d0 01 00` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HY` | 23 | `fe d0 01 00` | `0x0001d0fe` | Sends static `@hy:TT214H V05.08F\r\n` | YES, identity-only if exact RX | confirmed_code |

`@GB` is a separate dispatcher-table entry at `0x0001d064`, not part of the
`@H?` table. It sends the same `@ok:\r\n` literal as `@HB`, sets the same
transport marker at `bluefrog_state+0x19`, and waits for TX idle.

The nested `@HP` subdispatcher starts at `0x00024ac8`, uses
`line[3] - '0'`, has 39 possible entries, and only a few non-noop targets:

| RX prefix | Target | Confirmed response / behavior | Safe to emulate | Confidence |
|---|---:|---|---|---|
| `@HP0` | `0x0001d44c` | Clears a state bit, resets a timeout/flag, sends `@hp0\r\n` | YES, exact RX only | confirmed_code |
| `@HP1` | `0x0001d3f0` | Calls a guard/setup helper and, on success, sends `@hp1\r\n` | CAUTION | confirmed_code |
| `@HP2` | `0x0001d3dc` | Sets `bluefrog_state+0x46 |= 2`; no direct TX | NO | confirmed_code |
| `@HP?` | `0x0001d3cc` | Conditional call into another service path; no confirmed static TX | NO | inferred |
| `@HPD` | `0x0001d522` | Sets large state bits in `flags_88`; no direct TX | NO | confirmed_code |
| `@HPV` | `0x0001d506` | Requires `line[4]==':' && line[5]=='M'`, calls setup helper, sends `@hpv\r\n` | CAUTION | confirmed_code |

The nested `@HT` subdispatcher starts at `0x00024b64`, uses
`line[4] - '0'`, and has eight entries:

| RX prefix | Target | Confirmed response / behavior | Safe to emulate | Confidence |
|---|---:|---|---|---|
| `@HT0` | `0x0001d4f2` | Clears a `flags_88` mode bit and sends `@ht\r\n` | CAUTION | confirmed_code |
| `@HT1` | `0x0001d566` | Sets a `flags_88` mode bit and sends `@ht\r\n` | CAUTION | confirmed_code |
| `@HT2` | `0x0001d55e` | Calls fatal/reset helper with argument `4` | NO / dangerous | confirmed_code |
| `@HT3` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HT4` | `0x0001d0ee` | no-op / return | NO response | confirmed_code |
| `@HT5` | `0x0001d54c` | Sub-subdispatcher that mutates `bluefrog_state+0x28`; no direct TX | NO | confirmed_code |
| `@HT6` | `0x0001d544` | Calls fatal/reset helper with argument `0xb1` | NO / dangerous | confirmed_code |
| `@HT7` | `0x0001d466` | Parses hex bytes and calls a config/write helper; no direct TX | NO | confirmed_code |

### Machine RX Startup Handlers

```text
machine_rx_t2_state_handler @ 0x0001ced8
  Input: @T2...
  Parses 13 bytes of hex from line+4.
  Writes parsed bytes into bluefrog_state+0x2c and machine_cache_base+0x72.
  Sets bluefrog_state.flags_88 |= 0x400.
  If first parsed byte & 0x7f == 0, also sets flags_88 |= 0x100.
  Confidence: confirmed_code.

machine_rx_t3_identity_handler @ 0x0001da24
  Input: @T3:<hex><text>
  Parses a 16-bit value at line+4.
  Stores it at machine_cache_base+0x68.
  Copies the identity/version string from line+8 into machine_cache_base+0x88.
  Sets bluefrog_state.flags_88 |= 0x800.
  If bluefrog_state+0x2c & 0x7f != 0, also sets flags_88 |= 0x100.
  Confidence: confirmed_code.

machine_rx_tr37_gate_handler @ 0x0001d788
  Input: @tr:<code>,...
  Parses byte at line+4 and dispatches codes 0x32..0x53 through
  machine_rx_tr37_subhandler_table.
  The @tr:37 subhandler is table-driven and still needs full function carving.
  Confidence: confirmed_code for dispatch, inferred for exact @tr:37 subhandler.

ty_response_handler @ 0x0001d9d0
  Input: ty:...
  Clears startup flags and sets flags_88 bit 0x04 when appropriate.
  May arm state event 0x20.
  Confidence: confirmed_code from decompilation preview.

gb_ack_handler @ 0x0001d064
  Input: @GB...
  Sends @ok:\r\n.
  Sets a transport marker at state+0x19 and waits until TX busy flag clears.
  Confidence: confirmed_code from decompilation preview and literal pointer.
```

### App Start vs Machine Start

```text
App initial reads 0x1531/0x1524/0x1527
  BLE cache reads only. No machine UART TX is generated by these reads.

App stayInBLE write 0x1529 payload 00 7F 80
  ble_char_1529_pmode_write_callback copies payload to machine_cache_base+0x26,
  shortens length by one, calls pmode_write_followup_or_cache_notify(0x80),
  and later the state pump emits @TP:<key>7F via prefix-hex path.

Machine-start frames without app
  bluefrog_machine_state_pump can emit @T0/@T1/@H1/TY:/@t2:/@t3/@TR:37-related
  lines based on internal startup/session flags.

Host identity frames
  @hy:TT214H V05.08F and @hl:BL_nRF51822 V00.00 TT214H,00 are static firmware
  identity strings. These are adapter identity responses, not product actions.
```

### ESP vs Original Startup Differences

| Item | Original BLE dongle | Current ESP observation | Impact | Confidence |
|---|---|---|---|---|
| Host identity `@hy` | Static `@hy:TT214H V05.08F\r\n` exists | Not observed in current ESP logs | Machine may not see exact adapter identity if it asks `@HY` | confirmed_bytes |
| Bootloader/host info `@hl` | Static `@hl:BL_nRF51822 V00.00 TT214H,00\r\n` exists | Not observed in current ESP logs | Missing host-info reply if machine asks `@HL` | confirmed_bytes |
| Host/adapter HP responses | `@hpd/@hf/@hp0/@hp1/@hp4/@hp5/@hp6/@hp7/@hpv/@ht/@ho/@hr/@hw` exist | Not generally emulated unless already added manually | Potential identity/session completeness gap | confirmed_bytes |
| `@GB -> @ok:` | Confirmed small ACK handler | ESP has optional auto-ack path from WLAN finding | Good candidate for safe passive response only when RX `@GB` appears | confirmed_code |
| `@HB` / host subdispatcher | Dispatcher sends many `@H?` prefixes to host subdispatcher | ESP behavior unclear/incomplete | Potential missing identity dialogue | inferred |
| `@T2/@T3/ty:/@tr` | Confirmed startup/session handlers | ESP currently handles observed core handshake | Core machine identity/gate mostly present | confirmed_code |
| Machine boot early window | Main loop has one-shot startup bits | ESP may connect after machine already running | Could affect whether machine asks host identity frames | inferred |

### Safe Original-Startup Emulation Sequence

Only emulate responses after matching machine RX triggers. Do not send product,
DFU, PMode-setting, 0x1527/9x, or TV81/TV82 transfer frames as probes.

```text
1. Passive boot-attached observe
   TX: none
   Wait for machine RX: @HY/@HL/@HP/@HB/@GB/@T2/@T3/ty:/@tr
   safe: YES
   confidence: confirmed safe because no new command is sent

2. If machine sends @GB exactly
   TX: @ok:\r\n
   safe: YES, ACK-only
   confidence: confirmed_code

3. If machine sends @HY exactly
   TX: @hy:TT214H V05.08F\r\n
   safe: likely identity-only
   confidence: confirmed_bytes, host subhandler still partly uncarved

4. If machine sends @HL exactly
   TX: @hl:BL_nRF51822 V00.00 TT214H,00\r\n
   safe: likely identity-only
   confidence: confirmed_bytes, host subhandler still partly uncarved

5. If machine sends a known @HP/@HF/@HT/@HO style identity request
   TX: only the matching static lowercase host response confirmed in firmware
   safe: likely identity-only
   confidence: confirmed_bytes for literals, incomplete for exact request mapping

6. Continue existing core session/gate
   TX: TY:, @T1, @t2:<existing generated value>, @t3, @TR:37
   safe: existing startup/session path
   confidence: already runtime-confirmed in ESP logs
```

### Runtime Test Plan: Original Startup Observe

The ESPHome/jutta_proto test action `jutta_proto.manual_original_startup_observe`
is intended for boot-attached observation of the original host/adapter identity
dialog. It is not a live-status trigger test and must not send product, PMode,
transfer, DFU, `@TF`, or `@TV` commands.

Default runtime parameters:

```text
observe_ms=180000
respond_identity=YES
active_probe=NO
boot_attached_mode=YES
```

Boot-attached procedure:

```text
1. Keep the ESP connected to the machine UART.
2. Start the Home Assistant button/action before powering or waking the machine.
3. Power on / wake the machine.
4. Observe for 180 seconds.
```

Auto-reply allowlist when `respond_identity=YES`:

| RX request | TX reply | Reason | Confidence |
|---|---|---|---|
| `@HB` | `@ok:\r\n` | Host handler sends `@ok:` and sets transport marker | confirmed_code |
| `@GB` | `@ok:\r\n` | Separate dispatcher handler sends `@ok:` and sets transport marker | confirmed_code |
| `@HY` | `@hy:TT214H V05.08F\r\n` | Host handler sends static dongle identity | confirmed_code |

Passiv-only mode:

```yaml
respond_identity: false
```

In this mode the action only logs host identity requests and sends no UART TX.

Requests logged but not automatically answered:

| RX request | Reason |
|---|---|
| `@HL` | Identity-near, but original firmware copies `@hl:BL_nRF51822 V00.00 TT214H,00\r\n` as a template and patches dynamic digits |
| `@HC` | Dynamic `@hc:<4hex>` response derived from request data |
| `@HI` | Dynamic `@hi:<8hex><8hex>` from device/cache identity |
| `@HR` | Dynamic `@hr:<4hex>` response |
| `@HF` | Stateful startup/session handler |
| `@HP` | Nested subdispatcher with stateful branches |
| `@HT` | Nested subdispatcher with reset/config branches |
| `@HW` | Write/control ACK path that parses payload and mutates globals |
| `@HM` | App-mode/control state path |
| `@HS` | Clears app-mode state flag |

No-op requests in the `@H?` table are only logged and not answered:

```text
@HD @HE @HG @HH @HJ @HK @HN @HO @HQ @HU @HV @HX
```

Important: `@HO` is no-op in the machine host subhandler table. Do not confuse
it with the internal BLE/chunk-path literal `@ho\r\n`.

Example Home Assistant button:

```yaml
button:
  - platform: template
    name: "JURA Original Startup Observe"
    on_press:
      - jutta_proto.manual_original_startup_observe:
          id: jura
          observe_ms: 180000
          respond_identity: true
```

### Open Startup Questions

```text
- Exact @tr:37 subhandler body still needs full table/function carving.
- Exact @H? subhandler mapping from every request to every lowercase response is not fully carved.
- Machine boot-attached vs late-attach behavior remains unproven.
- Highest-confidence missing startup piece is host/adapter identity response handling
  for @HY/@HL/@HP/@HF/@HT/@HO-style requests, not a status query.
```
