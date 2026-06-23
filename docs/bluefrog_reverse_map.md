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
- The earlier `0x1528 -> 0x1898c` descriptor interpretation is stale. The corrected descriptor slice maps `0x1528` to raw callback `0x00018831` / Thumb-cleared `0x00018830`. `0x1898c` belongs to an adjacent/raw descriptor interpretation and should not be used as the `0x1528` callback without a separate descriptor-format audit.

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
| `0x00018830` | `ble_char_1528_update_product_progress_callback` | Corrected callback for characteristic `0x1528` in the raw descriptor slice around `0x24670`. | confirmed_code | The descriptor stores raw Thumb pointer `0x00018831`; this path updates a small progress/cache counter and does not emit machine UART TX. |
| `0x0001898c` | `maybe_ble_char_1529_adjacent_callback` | Adjacent descriptor callback candidate from the raw descriptor slice, not the `0x1528` callback. | unknown | Earlier docs incorrectly associated this with `0x1528`; keep unresolved until the full descriptor format/name mapping is audited. |
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
| `0x1527` | Product Progress Cache | value `0x20002c25`, length `0x14` | Read/cache, notify/update | Read is cache-only. Updated by `machine_rx_tv_progress_cache_handler`. Earlier notes associated progress-control callback `0x18844` with this area, but the corrected raw descriptor slice places raw callback `0x00018845` in the adjacent `0x1530` entry; keep the characteristic ID mapping cautious until a full descriptor struct audit. |
| `0x1528` | Update Product Progress | callback raw `0x00018831` -> `0x00018830` | Write/control callback | Corrected raw descriptor entry. This is no longer mapped to `0x1898c`; the callback updates a small cache/progress field and has no confirmed machine UART TX side effect. |
| `0x1529` | PMode / Control Write | callback `ble_char_1529_pmode_write_callback` | Write | Known payloads include `00 7F 80` stayInBLE, `00 47 01` process next/OK, `00 47 FF` cancel/back, `00 4D...` PMode/settings, `00 7F 82...` PIN/auth, `00 7F A5` DFU/bootloader. |
| `0x1530` | Progress/control adjacent descriptor | callback raw `0x00018845` -> `0x00018844` | Write/control | Corrected raw descriptor slice maps this entry to the legacy-named `ble_char_1527_progress_control_write_callback`; previous naming around `0x1527`/`0x1530` should be treated cautiously until the complete descriptor field layout is manually audited. |
| `0x1531` | About Machine | source: `@T3` identity | Read/cache/info | Initial app read. |
| `0x1533` | Statistics Command | command/control | Write/read status | Modes observed in app model: product counter, maintenance counter, maintenance status. |
| `0x1534` | Statistics Data | data | Read/cache | Statistics data pages. |
| `0x1538` | PMode / Settings Read | data/cache | Read | PMode/settings readback. |

Corrected raw descriptor slice:

```text
ble_descriptor_correction:
  stale_mapping_removed:
    - 0x1528 must not be mapped to 0x1898c
  entries:
    - characteristic: 0x1528
      descriptor_addr: 0x00024670
      callback_raw: 0x00018831
      callback_thumb_clear: 0x00018830
      corrected_name: ble_char_1528_update_product_progress_callback
      confidence: confirmed_descriptor_bytes
    - characteristic: 0x1529
      descriptor_addr: adjacent descriptor slice before 0x00024670
      callback_raw: 0x0001898d
      callback_thumb_clear: 0x0001898c
      corrected_name: maybe_ble_char_1529_adjacent_callback
      confidence: confirmed_descriptor_bytes_unresolved_semantics
    - characteristic: 0x1530
      descriptor_addr: adjacent descriptor slice after 0x00024670
      callback_raw: 0x00018845
      callback_thumb_clear: 0x00018844
      corrected_name: ble_char_1527_progress_control_write_callback
      confidence: confirmed_descriptor_bytes_descriptor_layout_still_needs_global_audit
```

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
- The corrected local descriptor slice maps `0x1528` to callback `0x18830`; the wider descriptor field layout around `0x1529/0x1530` still needs a full manual audit before applying a global struct.
- No active `@TF/@TV` query was found.
- No safe runtime test payload for `0x1527/0x9x` is defined.
- `event_driven_likely=YES`, but the concrete machine event or mode state that emits `@TF/@TV` is not yet proven.
- Descriptor table field sizes should be verified row-by-row before applying a global `BleCharacteristicDescriptor` struct in Ghidra.

## 15. Highest-Priority Next Targets

1. Audit the complete BLE descriptor field layout around `0x1529/0x1530`; do not reuse the stale `0x1528 -> 0x1898c` mapping.
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

  Startup-relevant branches reconstructed from decompilation:
  - `state+0x70` bit0 with substate `3` sends `@T0`.
  - `state+0x70` bit5 sends either `@H1` or, when bit6 is also set, `@T1`.
  - `flags_88` prefix-hex branch sends `TY:`/`@t2:`-family frames through
    `machine_uart_send_prefix_hex_line`.
  - `flags_88` static-line branch sends the currently selected startup line
    such as `@t3`/cached gate-adjacent line.
  - retry branches send `@T0`, `@T1`, and `@TP:`/`@TD:`-prefix frames with
    counters before clearing the corresponding bit.
  Confidence: confirmed_code for branch structure; inferred for exact semantic
  names where the same literal helper is reused by multiple branches.

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

### Runtime Test Plan: Original Startup Active Safe

The passive `manual_original_startup_observe` test showed only startup/control
traffic such as `@T3` and `@t0` in the observed window. No `@H?` host identity
request and no `@GB` request was seen passively. This suggests the machine may
not start the full host/adapter identity dialog by itself in the tested
late-attach/observe window.

Next hypothesis: the original BLE dongle may actively send a small core startup
sequence before the machine asks for, or accepts, adapter identity details. The
new ESPHome/jutta_proto action `jutta_proto.manual_original_startup_active_safe`
is limited to confirmed startup/session frames and does not send product,
PMode-setting, transfer, DFU, `@TF`, or `@TV` commands.

Active-safe defaults:

```text
observe_ms=180000
send_core_startup=YES
respond_identity=YES
active_probe=YES
```

Active TX allowlist:

| TX frame | Encoding | Reason |
|---|---|---|
| `@T0` | inner uart0 | startup control literal present in original firmware |
| `@H1` | inner uart0 | startup identity probe literal present in original firmware |
| `TY:` | plaintext | machine type query used by existing safe startup path |
| `@T1` | inner uart0 | startup control used by existing safe startup path |
| `@TR:37` | inner uart0 | gate command used by existing safe startup path |

Sequence used by the active-safe action:

```text
TX @T0
wait 500 ms
TX @H1
wait 500 ms
TX TY:
wait for ty: or timeout
TX @T1
wait for @t1 or timeout
TX @TR:37
wait for @tr:37 or timeout
observe until observe_ms
```

Safe identity auto-replies remain identical to the passive observe test:

```text
@HB -> @ok:
@GB -> @ok:
@HY -> @hy:TT214H V05.08F
```

Explicitly not sent by this test:

```text
@t2:
@t3
@TP:
@TD:
@TV:81
@TV:82
@TS:9x
0x1527 0x9x
@mn / @mo / @me / @me1
DFU / bootloader / product actions
```

The ESPHome implementation logs every allowed TX as cleartext via
`machine_tx_startup_trace`, and emits `startup_tx_diff` at the end so the
observed ESP sequence can be compared against the original startup literals:

```text
@T0 @T1 @H1 TY: @TR:37 @t3 @t2: @TP:
```

Example Home Assistant button:

```yaml
button:
  - platform: template
    name: "JURA Original Startup Active Safe"
    on_press:
      - jutta_proto.manual_original_startup_active_safe:
          id: jura
          observe_ms: 180000
          send_core_startup: true
          respond_identity: true
          active_probe: true
```

### Startup State After @T2/@T3/@t0/ty/@t1/@tr37

This section focuses on state consequences after the machine-originated startup
responses, not on live/status cachewriter behavior.

Confirmed original BLE firmware handlers:

| RX frame | Handler | Original state/cache writes | Follow-up relevance | Confidence |
|---|---|---|---|---|
| `@T2...` | `machine_rx_t2_state_handler` (`0x0001ced8`) | parses 13 hex bytes from `line+4`; writes `bluefrog_state+0x2c`; writes `machine_cache_base+0x72`; sets `bluefrog_state_flags_88 |= 0x400`; conditionally sets `flags_88 |= 0x100` when the first parsed byte masked with `0x7f` is zero | Enables the state pump to proceed toward generated `@t2:` handling | confirmed_code |
| `@T3...` | `machine_rx_t3_identity_handler` (`0x0001da24`) | parses 16-bit value at `line+4`; stores it in `machine_cache_base+0x68`; copies identity/version text from `line+8` to `machine_cache_base+0x88`; sets `flags_88 |= 0x800`; conditionally sets `flags_88 |= 0x100` when `bluefrog_state+0x2c & 0x7f` is nonzero | Enables the state pump to proceed toward generated `@t3` / quiet-before-gate handling | confirmed_code |
| `ty:...` | `ty_response_handler` / startup branch | sets/clears startup/session-adjacent bits; confirmed effect includes `flags_88` bit `0x04` in the existing reverse map | Allows type query phase to advance toward `@T1`/`@T2` handling | confirmed_code/inferred for exact sub-bits |
| `@t0` | startup/control handler path | ESP tracks as `DONGLE_EVENT_T0`; original firmware uses it as a startup-control event around `@T3` quiet/gate timing | Can satisfy wait-after-`@t3` timing branch when that branch is active | inferred |
| `@t1` | startup/control handler path | ESP tracks as `DONGLE_EVENT_T1`; original firmware uses it as the response to `@T1` | Allows progression toward waiting for `@T2` | confirmed by ESP path, original exact handler still partly table-driven |
| `@tr:37,...` | `machine_rx_tr37_gate_handler` (`0x0001d788`) plus subhandler table | parses byte at `line+4`, dispatches through `machine_rx_tr37_subhandler_table`; exact `0x37` subhandler remains table-driven | Establishes gate/session readiness when required `@T2`/`@T3` bits are also present | confirmed_code for dispatch, subhandler partially unresolved |

Original statepump follow-up TX candidates after these flags:

| Candidate TX | Required original state | Consumed/affected state | ESP equivalent | Safe in current tests |
|---|---|---|---|---|
| `TY:` | startup type phase | waits for `ty:` | `DONGLE_EVENT_TY` | yes, already used |
| `@T1` | post-type startup phase | waits for `@t1` | `DONGLE_EVENT_T1` | yes, already used |
| `@t2:<...>` | `@T2` parsed word/state available | advances toward `@T3` | partially represented by `startup_t2_word_`; not sent by Active-Safe | not in Active-Safe allowlist |
| `@t3` | `@T3` identity/cache state available | starts quiet/gate preparation | represented only as `startup_trace_sends_t3_` when normal full startup sends it | not in Active-Safe allowlist |
| `@TR:37` | quiet/gate prep after `@T3`/`@t3` path | waits for `@tr:37` | `DONGLE_EVENT_TR` / `manual_original_startup_got_tr37_` | yes, already used |
| `@T0` | startup/identity TX block | machine may answer with `@T3`/`@t0` | `startup_trace_sends_t0_`, `DONGLE_EVENT_T0` | yes, Active-Safe |
| `@H1` | startup/identity probe literal | no host dialog observed from this alone | `startup_trace_sends_h1_` | yes, Active-Safe |
| `@TP:` | PMode/stayInBLE path from 0x1529 | separate heartbeat/control path | `startup_trace_sends_tp_` | not part of Active-Safe |

ESP state mapping:

| Original flag/state | ESP state | Status |
|---|---|---|
| `flags_88 |= 0x400` after `@T2` | `DONGLE_EVENT_T2` plus optional `startup_t2_word_` | mapped coarsely |
| `flags_88 |= 0x800` after `@T3` | `DONGLE_EVENT_T3` plus `dongle_machine_identity_` | mapped coarsely |
| `flags_88 |= 0x04` around `ty:` | `DONGLE_EVENT_TY` | mapped coarsely |
| `@t0` startup event | `DONGLE_EVENT_T0` | mapped |
| `@t1` startup event | `DONGLE_EVENT_T1` | mapped |
| `@tr:37` gate event | `DONGLE_EVENT_TR`; in Active-Safe also `manual_original_startup_got_tr37_` | mapped |
| conditional `flags_88 |= 0x100` | no exact ESP bit | missing in ESP state |
| transport/peripheral startup `flags_88 |= 0x40000` | no exact ESP bit | missing in ESP state |

New ESP diagnostics:

```text
startup_state_after_rx line="<rx_line>" ...
original_startup_state_diff ...
```

### Runtime Active-Safe Startup Tests

`manual_original_startup_active_safe` originally used a fixed-delay sequence:

```text
@T0 -> @H1 -> TY: -> @T1 -> @TR:37
```

Runtime result:

- Core gate succeeds: `ty:...`, `@t1`, and `@tr:37,...` are observed.
- The machine also emits repeated `@T3:...`, `@T2:...`, and `@t0`.
- No host identity dialog was observed: no `@HB/@HY/@HL/@GB/@HP/@HF/@HT/@HW`.
- Therefore `@H1` alone is not sufficient to make the machine start a visible
  `@H?`/`@GB` host identity request dialog in the tested window.

The ESP test state now resets startup-local events and ignores stale
stats-session gate flags while this test is active. `post_gate=YES` in
`startup_state_after_rx` is only allowed after the current test has observed
`@tr:37`.

New comparison log:

```text
startup_sequence_result
  mode=<fixed_delay|stateful>
  sent_sequence=<list>
  rx_sequence=<list>
  host_identity_request_count=<n>
  gate_ok=<YES|NO>
  missing_original_conditions=<list>
```

`manual_original_startup_active_stateful` keeps the same safe TX allowlist
(`@T0`, `@H1`, `TY:`, `@T1`, `@TR:37`) but advances based on observed startup RX
instead of fixed 500 ms gaps:

- after `@T0`, wait for `@t0` or `@T3` before `@H1`;
- after `@H1`, wait for startup RX or timeout before `TY:`;
- after `TY:`, wait for `ty:` before `@T1`;
- after `@T1`, wait for `@t1` before `@TR:37`.

Open point: the original statepump can also emit `@t2:` and `@t3`, but those are
not part of the active-safe allowlist until their exact payload/state conditions
are mirrored confidently.

### ESP Normal Startup vs Original Startup

The current ESP normal `dongle_startup` path sends:

```text
@D1 -> TY: -> @T1 -> @t2:<...> -> @t3 -> @TR:37
```

`@D1` is an ESP-side legacy startup probe:

- ESP sender: `process_dongle_startup_`, state `PROBE_D1`, through
  `send_dongle_startup_command_("@D1", now)` as plaintext.
- It also appears in old XML/session probe arrays and the tablet start sequence.
- No `@D1`, `@D%c`, `@D%d`, `@D:` or generic `@D` startup literal/path has been
  found in the decompiled classic BLE/BlueFrog firmware so far.
- It is therefore not confirmed as an original BLE dongle startup frame. Current
  role classification: `legacy`.

The trace reason for `@D1` is now `legacy_startup_probe` instead of `unknown`.
This is only a logging correction; it does not add or remove any TX.

Diagnostic logs added for normal startup:

```text
normal_startup_sequence
  tx_sequence=[@D1,TY:,@T1,@t2:...,@t3,@TR:37]
  rx_sequence=[...]
  sends_T0=<YES|NO>
  sends_H1=<YES|NO>
  sends_TY=<YES|NO>
  sends_T1=<YES|NO>
  sends_t2=<YES|NO>
  sends_t3=<YES|NO>
  sends_TR37=<YES|NO>
  sends_D1=<YES|NO>

startup_sequence_diff_original_vs_esp
  original_known_frames=[@T0,@H1,TY:,@T1,@t2:,@t3,@TR:37,@TP:]
  esp_normal_frames=[...]
  missing_in_esp_normal=[...]
  extra_in_esp_normal=[...]
  uncertain_frames=[@D1]
```

Encoding note:

- ESP normal startup calls `send_dongle_startup_command_("@T1", now)` with
  `inner_uart0=false`, so the trace `@T1 encoded=NO` is the actual ESP path.
- Active-Safe calls the same core line through `write_inner_uart0_command_`, so
  `@T1 encoded=YES` is also the actual ESP path.
- The original BLE firmware always routes startup literals through
  `machine_uart_send_line_encoded`; whether this becomes raw/plain or 0x26 encoded
  depends on the original transport/session guard inside that sender.

These logs do not send any new command. They only compare the observed ESP
startup events/follow-up TX against the original firmware state consequences.

### Static `bluefrog_machine_state_pump` Startup TX Branches

This subsection is static reverse-map evidence only. It documents the confirmed
classic BLE/BlueFrog firmware callsites around `bluefrog_machine_state_pump`
(`0x0001bd0c`) and the direct machine UART TX helpers:

- `machine_uart_send_line_encoded`
- `machine_uart_sendf_line_encoded`
- `machine_uart_send_prefix_hex_line`

Common gating seen around these branches:

```text
common_tx_guards:
  tx_in_progress_marker == 0
  state_pump_substate_char == 0
  passthrough_or_chunk_transport_busy() == false
  app_mode_state_machine_tx_drain() == 0
```

Many `bluefrog_state+0x88` branches additionally require `(flags_88 & 0x1) == 0`
before a new machine TX is emitted.

```text
startup_static_gap_resolution:
  ty_branch_resolved=YES
  tr37_callsite_resolved=YES
  rx_to_statepump_chain_resolved=NO
  original_sequence_order_resolved=NO
  unresolved_reasons:
    - isolated @t0/@t1/ty: RX handlers and their writes into state70/flags_88 were not fully separated from the generic line dispatcher in this static pass
    - @t3 has multiple confirmed statepump TX branches and their relative startup order remains unresolved
    - @T0/@T1/@H1 are confirmed state70-driven branches, but the complete RX-to-state70 producer chain is still incomplete
    - @TP: is confirmed as a dynamic/cache-driven branch, not a linear startup-order proof
```

```text
original_tx_branch:
  frame_or_format: @T0
  literal_address: 0x000249b0
  sender_function: machine_uart_send_line_encoded
  exact_callsite_address:
    - 0x0001befe
    - 0x0001c0aa
  required_flags:
    - branch_a: bluefrog_state+0x70 bit0 set and substate ((state70 & 0x1f) >> 3) == 3
    - branch_b: bluefrog_state+0x88 retry/control bit selected by `flags_88 << 0x1b < 0`
  required_state_fields:
    - bluefrog_state+0x70
    - bluefrog_state+0x88
    - retry_counter_b for retry branch
  required_previous_rx: not fully resolved from this local branch alone
  required_timer_or_counter: retry_counter_b in retry branch
  encoding_path: line helper -> machine_uart_send_line_encoded
  flags_set_before_tx: none observed in branch body
  flags_cleared_after_tx:
    - branch_a clears state70 mask 0xffffffe7
    - branch_b clears the retry bit on final retry and sets follow-up/error bits
  next_expected_rx: @t0 or @T3 seen in runtime; exact static expectation not encoded at callsite
  confidence: confirmed_code

original_tx_branch:
  frame_or_format: @H1
  literal_address: 0x000249a8
  sender_function: machine_uart_send_line_encoded
  exact_callsite_address: 0x0001c012
  required_flags:
    - bluefrog_state+0x70 bit5 set
    - bluefrog_state+0x70 bit6 clear
  required_state_fields:
    - bluefrog_state+0x70
  required_previous_rx: not fully resolved
  required_timer_or_counter: none observed at callsite
  encoding_path: line helper -> machine_uart_send_line_encoded
  flags_set_before_tx: none observed
  flags_cleared_after_tx: clears state70 bit5 before TX
  next_expected_rx: none proven; runtime `@H1` alone did not start @H? dialog
  confidence: confirmed_code

original_tx_branch:
  frame_or_format: TY:
  literal_address: 0x00024990
  sender_function: machine_uart_send_line_encoded
  exact_callsite_address: 0x0001bf3a
  required_flags:
    - bluefrog_state+0x88 bit0 / 0x00000001 set
  required_state_fields:
    - bluefrog_state+0x88
    - state_pump_retry_counter_a
  required_previous_rx: producer of flags_88 bit0 not fully isolated
  required_timer_or_counter:
    - state_pump_retry_counter_a is initialized to 4 if zero
    - send path requires retry_counter_a > 1
  encoding_path: line helper -> machine_uart_send_line_encoded
  flags_set_before_tx: none observed in branch body
  flags_cleared_after_tx:
    - clears flags_88 bit2 / 0x00000004 before TX
  next_expected_rx: ty:<machine type/version>
  confidence: confirmed_code

original_tx_branch:
  frame_or_format: @T1
  literal_address: 0x000249b8
  sender_function: machine_uart_send_line_encoded
  exact_callsite_address:
    - primary branch: 0x0001c012 adjacent branch path
    - retry branch: 0x0001c0fc
  required_flags:
    - primary branch: bluefrog_state+0x70 bit5 set and bit6 set
    - retry branch: bluefrog_state+0x88 retry/control bit selected by `flags_88 << 0x1a < 0`
  required_state_fields:
    - bluefrog_state+0x70
    - bluefrog_state+0x88
    - retry_counter_b for retry branch
  required_previous_rx: not fully resolved
  required_timer_or_counter: retry_counter_b in retry branch
  encoding_path: line helper -> machine_uart_send_line_encoded
  flags_set_before_tx:
    - retry final path sets flags_88 |= 0x100 and flags_88 |= 0x2 before follow-up handling
  flags_cleared_after_tx:
    - primary branch clears state70 bit5 and bit6 before TX
    - retry branch clears the retry bit on final retry
  next_expected_rx: @t1
  confidence: confirmed_code

original_tx_branch:
  frame_or_format: @t2:<hex payload>
  literal_address: 0x00024998
  sender_function: machine_uart_send_prefix_hex_line
  exact_callsite_address: 0x0001bfc8
  required_flags:
    - (flags_88 & 0x1) == 0
    - bluefrog_state+0x88 bit selected by `flags_88 << 0x15 < 0`
  required_state_fields:
    - bluefrog_state+0x88
    - bluefrog_state+0x8c payload length/source selector
    - bluefrog_state+0x2c payload/control byte source
    - session_state_flag19-derived bits for second payload byte
  required_previous_rx: not fully resolved
  required_timer_or_counter: none observed at callsite
  encoding_path: prefix+hex builder -> machine_uart_send_line_encoded
  flags_set_before_tx: tx_in_progress_marker set to 'P'
  flags_cleared_after_tx: clears the prefix-hex status bit before TX
  next_expected_rx: no direct callsite proof; runtime startup uses this as @t2:<...> control
  confidence: confirmed_code

original_tx_branch:
  frame_or_format: @t3
  literal_address: 0x000249a0
  sender_function: machine_uart_send_line_encoded
  exact_callsite_address:
    - 0x0001be36
    - 0x0001c10c
  required_flags:
    - branch_a: bluefrog_state+0x88 bit24 set, then bit25 gates actual send
    - branch_b: bluefrog_state+0x88 bit selected by `flags_88 << 0x14 < 0`
  required_state_fields:
    - bluefrog_state+0x88
    - bluefrog_state+0x8c set to 0x0b in branch_a
  required_previous_rx: not fully resolved
  required_timer_or_counter: none observed at callsite
  encoding_path: line helper -> machine_uart_send_line_encoded
  flags_set_before_tx:
    - branch_a writes bluefrog_state+0x8c = 0x0b
    - branch_b sets tx_in_progress_marker = 1
  flags_cleared_after_tx:
    - branch_a clears bit24 and, if sending, bit25
    - branch_b clears the line-TX bit before sending
  next_expected_rx: no direct callsite proof
  confidence: confirmed_code_multiple_candidate_branches

original_tx_branch:
  frame_or_format: @TR:37
  literal_address: 0x000249c0
  sender_function: machine_uart_send_prefix_hex_line
  exact_callsite_address: 0x0001c164
  static_or_formatted: static prefix with zero-length payload; helper emits @TR:37 plus CRLF
  required_flags:
    - bluefrog_state+0x88 bit6 / 0x00000040 set
  required_state_fields:
    - bluefrog_state+0x88
    - state_pump_retry_counter_b
  required_previous_rx: producer of flags_88 bit6 not fully isolated
  required_timer_or_counter:
    - state_pump_retry_counter_b is initialized to 4 if zero
    - send path requires retry_counter_b > 1
  encoding_path: prefix+hex builder with zero payload -> machine_uart_send_line_encoded
  flags_set_before_tx: none observed in send branch
  flags_cleared_after_tx:
    - not cleared in the send path
    - final retry/failover path at 0x0001c1ce clears flags_88 bit6
  next_expected_rx: @tr:37,<hex>
  confidence: confirmed_code

original_tx_branch:
  frame_or_format: @TP:<single-byte hex payload>
  literal_address: 0x00024a20
  sender_function: machine_uart_send_prefix_hex_line
  exact_callsite_address: 0x0001c358
  required_flags:
    - bluefrog_state+0x88 bit selected by `flags_88 << 0x16 < 0`
  required_state_fields:
    - machine_cache_base+0x3a nonzero in dynamic branch
    - machine_cache_base+0x21 == 0 in dynamic branch
    - high halfword of bluefrog_state+0x8c == 0
  required_previous_rx: not fully resolved; not a linear startup-only step
  required_timer_or_counter: none observed at callsite
  encoding_path: prefix+hex builder -> machine_uart_send_line_encoded
  flags_set_before_tx: none proven
  flags_cleared_after_tx:
    - dynamic branch clears the dynamic prefix branch bit before TX
  next_expected_rx: @tp/@TP-related response not proven at this callsite
  confidence: confirmed_code
```

RX to statepump chain:

```text
rx_to_statepump_chain:
  handler: machine_rx_t2_state_handler
  rx_line: @T2...
  parsed_fields:
    - ASCII hex byte sequence
  writes_to_bluefrog_state:
    - bluefrog_state+0x2c.. style mirrored T2 bytes
  writes_to_machine_cache:
    - machine_cache_base+0x72.. style mirrored T2 bytes
  flags_88_set:
    - 0x00000400
    - 0x00000100 conditionally
  flags_88_clear: none confirmed in handler body
  enabled_statepump_branch:
    - @t2:<hex payload> branch at 0x0001bfc8 when flags_88 bit0 is clear
  next_expected_tx_candidate: @t2:<hex payload>
  confidence: confirmed_code

rx_to_statepump_chain:
  handler: machine_rx_t3_identity_handler
  rx_line: @T3...
  parsed_fields:
    - identity code at line+4
    - identity/version string at line+8
  writes_to_bluefrog_state: none beyond flags confirmed
  writes_to_machine_cache:
    - machine_cache_base+0x68 = parsed identity code
    - machine_cache_base+0x88 = identity/version string, max 0x10 bytes
  flags_88_set:
    - 0x00000800
    - 0x00000100 conditionally
  flags_88_clear: none confirmed in handler body
  enabled_statepump_branch:
    - @t3 branch candidate at 0x0001c10c
  next_expected_tx_candidate: @t3
  confidence: confirmed_code

rx_to_statepump_chain:
  handler: ty_response_handler
  rx_line: ty:<machine type/version>
  parsed_fields: not fully isolated
  writes_to_bluefrog_state: unresolved
  writes_to_machine_cache: unresolved
  flags_88_set: unresolved
  flags_88_clear: unresolved
  enabled_statepump_branch: unresolved
  next_expected_tx_candidate: unresolved
  confidence: unknown

rx_to_statepump_chain:
  handler: machine_rx_t0_handler
  rx_line: @t0
  parsed_fields: none proven
  writes_to_bluefrog_state: unresolved; likely state70-related but not statically proven
  writes_to_machine_cache: unresolved
  flags_88_set: unresolved
  flags_88_clear: unresolved
  enabled_statepump_branch: unresolved
  next_expected_tx_candidate: unresolved
  confidence: unknown

rx_to_statepump_chain:
  handler: machine_rx_t1_handler
  rx_line: @t1
  parsed_fields: none proven
  writes_to_bluefrog_state: unresolved; likely state70/session-related but not statically proven
  writes_to_machine_cache: unresolved
  flags_88_set: unresolved
  flags_88_clear: unresolved
  enabled_statepump_branch: unresolved
  next_expected_tx_candidate: unresolved
  confidence: unknown

rx_to_statepump_chain:
  handler: machine_rx_tr37_gate_handler -> @tr:37 subhandler 0x0001d806
  rx_line: @tr:37,...
  parsed_fields:
    - four 16-bit ASCII-hex values from the response payload
  writes_to_bluefrog_state:
    - bluefrog_state+0x88 flags update
  writes_to_machine_cache:
    - machine_cache_base+0x6a = parsed u16[0]
    - machine_cache_base+0x6c = parsed u16[1]
    - machine_cache_base+0x6e = parsed u16[2]
    - machine_cache_base+0x70 = parsed u16[3]
  flags_88_set:
    - 0x00000200
  flags_88_clear:
    - 0x00000040
  enabled_statepump_branch:
    - post-gate/session latch via flags_88 bit9, not a direct next startup TX by itself
  next_expected_tx_candidate: none proven at this handler
  confidence: confirmed_code
```

Derived static startup ordering:

```text
original_bluefrog_startup_sequence:
  sequence_order: not_fully_resolved
  confirmed_tx_frames:
    - @T0
    - @H1
    - TY:
    - @T1
    - @t2:
    - @t3
    - @TR:37
    - @TP:
  missing_static_evidence:
    - exact mapping from @t0/@t1/ty: RX handlers to state70/flags_88 bits
    - complete producer chain for state70 bits that select @T0/@H1/@T1
    - relative ordering between the two confirmed @t3 statepump branches
    - separation of @TP: startup/heartbeat/control use from later PMode/control branches
```

`@D1` exclusion:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
  evidence: decompilation search for @D1/@D%c/@D%d/@D:/@D returned no classic BLE firmware hits
```

### Startup Producer Chain: RX -> state70 / flags_88 -> TX

This section narrows the still-open startup sequence gap to the concrete producers and consumers of
`bluefrog_state+0x70` and `bluefrog_state_flags_88`. The focus is the classic BlueFrog state pump
only. `@D1` remains excluded as original BlueFrog evidence.

#### `bluefrog_state+0x70` writers and bit operations

```text
state70_writer:
  address: 0x000163cc / decompile line 17 in main_loop_or_scheduler
  function: main_loop_or_scheduler
  instruction_or_decompiled_stmt: piVar1[0x1c] = piVar1[0x1c] | 1
  bit_or_mask: 0x00000001
  operation: set
  caller_context: boot/init after reading FICR/device-id word
  required_rx_or_event: device-id word0 cache equals 2
  enables_tx_candidate: @T0, but only if the low substate bits also satisfy ((state70 & 0x1f) >> 3) == 3
  disables_tx_candidate: none
  confidence: confirmed_code for bit0 set; inferred for device-id meaning

state70_writer:
  address: 0x0001e876
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: *(uint *)(bluefrog_state + 0x70) |= 0x20
  bit_or_mask: 0x00000020
  operation: set
  caller_context: periodic/session tick path
  required_rx_or_event: service_channel_any_countdown_active() returns true
  enables_tx_candidate: @T1, together with state70 bit6
  disables_tx_candidate: none
  confidence: confirmed_code

state70_writer:
  address: 0x0001e87e
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: *(uint *)(bluefrog_state + 0x70) |= 0x40
  bit_or_mask: 0x00000040
  operation: set
  caller_context: periodic/session tick path
  required_rx_or_event: service_channel_any_countdown_active() returns true
  enables_tx_candidate: @T1 selector bit
  disables_tx_candidate: @H1 selector path
  confidence: confirmed_code

state70_writer:
  address: 0x0001e858
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: *(uint *)(bluefrog_state + 0x70) |= 0x20
  bit_or_mask: 0x00000020
  operation: set
  caller_context: periodic/session tick path
  required_rx_or_event: session tick counter reaches zero while no service countdown is active
  enables_tx_candidate: @H1, because bit6 is cleared in the next statement
  disables_tx_candidate: none
  confidence: confirmed_code

state70_writer:
  address: 0x0001e860
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: *(uint *)(bluefrog_state + 0x70) &= 0xffffffbf
  bit_or_mask: 0x00000040
  operation: clear
  caller_context: periodic/session tick timeout path
  required_rx_or_event: session tick counter reaches zero while no service countdown is active
  enables_tx_candidate: @H1 selector path
  disables_tx_candidate: @T1 selector path
  confidence: confirmed_code

state70_writer:
  address: 0x0001befe
  function: bluefrog_machine_state_pump
  instruction_or_decompiled_stmt: *(uint *)(bluefrog_state + 0x70) &= 0xffffffe7
  bit_or_mask: clears 0x00000018 and 0x00000001
  operation: clear
  caller_context: @T0 TX branch
  required_rx_or_event: state70 bit0 set and ((state70 & 0x1f) >> 3) == 3
  enables_tx_candidate: none
  disables_tx_candidate: @T0 retry/re-send until rearmed
  confidence: confirmed_code

state70_writer:
  address: 0x0001bfd4 / 0x0001be26
  function: bluefrog_machine_state_pump
  instruction_or_decompiled_stmt: state70 &= 0xffffffdf; optionally state70 &= 0xffffffbf
  bit_or_mask: clears 0x00000020; clears 0x00000040 on @T1 path
  operation: clear
  caller_context: @H1/@T1 TX branch
  required_rx_or_event: state70 bit5 set
  enables_tx_candidate: none
  disables_tx_candidate: @H1/@T1 retry/re-send until rearmed
  confidence: confirmed_code
```

`machine_rx_tr37_code37_subhandler` writes `machine_cache_base+0x70`, not `bluefrog_state+0x70`.
That store belongs to the parsed `@tr:37,<hex>` response cache and is not a producer for the
startup state70 TX selector bits.

#### `bluefrog_state+0x70` state-pump consumers

```text
state70_consumer:
  address: 0x0001bdb2..0x0001befe
  function: bluefrog_machine_state_pump
  tested_bit_or_mask: state70 bit0 and substate ((state70 & 0x1f) >> 3) == 3
  required_substate: low state70 substate equals 3 after shifting bits3/4 down
  tx_frame: @T0
  tx_literal_or_format_addr: 0x000249b0
  sender_helper: machine_uart_send_line_encoded
  clears_after_tx: state70 &= 0xffffffe7
  expected_rx: @T3... and/or @t0 observed in runtime; exact static expected-rx edge not encoded at callsite
  confidence: confirmed_code for branch, inferred for expected_rx

state70_consumer:
  address: 0x0001be06..0x0001bfd4
  function: bluefrog_machine_state_pump
  tested_bit_or_mask: state70 bit5 set and state70 bit6 clear
  required_substate: none beyond selector bits
  tx_frame: @H1
  tx_literal_or_format_addr: 0x000249a8
  sender_helper: machine_uart_send_line_encoded
  clears_after_tx: state70 &= 0xffffffdf
  expected_rx: none directly proven; no @H? identity dialog was statically linked to this TX branch
  confidence: confirmed_code

state70_consumer:
  address: 0x0001be06..0x0001be26
  function: bluefrog_machine_state_pump
  tested_bit_or_mask: state70 bit5 set and state70 bit6 set
  required_substate: none beyond selector bits
  tx_frame: @T1
  tx_literal_or_format_addr: 0x000249b8
  sender_helper: machine_uart_send_line_encoded
  clears_after_tx: state70 &= 0xffffffdf; state70 &= 0xffffffbf
  expected_rx: @t1
  confidence: confirmed_code
```

#### `flags_88` writers and bit operations

```text
flags88_writer:
  address: 0x0001a650
  function: arm_scheduler_event1_clear_event10_if_not_forced
  instruction_or_decompiled_stmt: flags_88 |= 0x00000001; flags_88 &= 0xffffffef
  mask: 0x00000001 set, 0x00000010 clear
  operation: set/clear
  caller_context: main_loop_or_scheduler and periodic/session loop
  required_rx_or_event: state70 bit0 is clear
  enables_tx_candidate: TY:
  disables_tx_candidate: event10 path represented by flags_88 0x10
  confidence: confirmed_code

flags88_writer:
  address: 0x0001d9c8
  function: maybe_ty_response_state_handler
  instruction_or_decompiled_stmt: flags_88 &= 0xfffffffe; flags_88 &= 0xfffffffd; flags_88 |= 4
  mask: 0x00000001 clear, 0x00000002 clear, 0x00000004 set
  operation: clear/set
  caller_context: ty: dispatcher entry points to 0x0001d9d1 Thumb, contained in this function body
  required_rx_or_event: ty:<machine type/version>
  enables_tx_candidate: later startup/session phase; also satisfies one gate for set_cachewriter_event40_if_session_flags_allow
  disables_tx_candidate: TY: retry path
  confidence: confirmed_code

flags88_writer:
  address: 0x0001a690
  function: set_cachewriter_event40_if_session_flags_allow
  instruction_or_decompiled_stmt: flags_88 |= 0x00000040
  mask: 0x00000040
  operation: set
  caller_context: called by machine_rx_tf_status_cache_handler and machine_rx_tv_progress_cache_handler
  required_rx_or_event: machine-originated @TF/@TV cachewriter, flags_88 0x200 clear, 0x100 set, 0x04 set, 0x40 clear
  enables_tx_candidate: @TR:37 statepump branch
  disables_tx_candidate: none
  confidence: confirmed_code for cachewriter producer; not confirmed as startup producer

flags88_writer:
  address: 0x0001ced8
  function: machine_rx_t2_state_handler
  instruction_or_decompiled_stmt: flags_88 &= 0xffffffdf; flags_88 |= 0x400; conditionally flags_88 |= 0x100
  mask: 0x00000020 clear, 0x00000400 set, 0x00000100 conditional set
  operation: clear/set
  caller_context: @T2 dispatcher entry
  required_rx_or_event: @T2:<hex> frame accepted and parsed
  enables_tx_candidate: @t2:<hex payload>
  disables_tx_candidate: flags_88 0x20 phase/retry marker
  confidence: confirmed_code

flags88_writer:
  address: 0x0001da24
  function: machine_rx_t3_identity_handler
  instruction_or_decompiled_stmt: flags_88 |= 0x800; conditionally flags_88 |= 0x100
  mask: 0x00000800 set, 0x00000100 conditional set
  operation: set
  caller_context: @T3 dispatcher entry
  required_rx_or_event: @T3:<identity/version> frame accepted and parsed
  enables_tx_candidate: @t3
  disables_tx_candidate: none
  confidence: confirmed_code

flags88_writer:
  address: 0x0001d806
  function: machine_rx_tr37_code37_subhandler
  instruction_or_decompiled_stmt: flags_88 &= 0xffffffbf; flags_88 |= 0x200
  mask: 0x00000040 clear, 0x00000200 set
  operation: clear/set
  caller_context: @tr dispatcher subhandler for code 0x37
  required_rx_or_event: @tr:37,<four parsed u16 fields>
  enables_tx_candidate: post-gate/session latch; no direct startup TX proven from this handler
  disables_tx_candidate: @TR:37 retry/re-send until rearmed
  confidence: confirmed_code

flags88_writer:
  address: 0x000163cc / decompile line 79
  function: main_loop_or_scheduler
  instruction_or_decompiled_stmt: flags_88 |= 0x00040000
  mask: 0x00040000
  operation: set
  caller_context: startup/init
  required_rx_or_event: boot/startup init path
  enables_tx_candidate: startup one-shot state not yet tied to a specific TX branch
  disables_tx_candidate: none
  confidence: confirmed_code for set; unknown for consumer meaning

flags88_writer:
  address: 0x0001bf3a branch
  function: bluefrog_machine_state_pump
  instruction_or_decompiled_stmt: flags_88 &= 0xfffffffb before TY: TX; retry exhaustion clears 0x01 and sets 0x02
  mask: 0x00000004 clear, 0x00000001 clear on failure, 0x00000002 set on failure
  operation: clear/set
  caller_context: TY: TX branch
  required_rx_or_event: flags_88 0x01 set
  enables_tx_candidate: failure/fallback phase if retry exhausted
  disables_tx_candidate: TY: immediate repeat until rearmed
  confidence: confirmed_code

flags88_writer:
  address: 0x0001c164 branch
  function: bluefrog_machine_state_pump
  instruction_or_decompiled_stmt: retry exhaustion clears flags_88 0x40
  mask: 0x00000040
  operation: clear
  caller_context: @TR:37 TX branch
  required_rx_or_event: flags_88 0x40 set and retry counter exhausted
  enables_tx_candidate: none
  disables_tx_candidate: @TR:37 repeat
  confidence: confirmed_code

flags88_writer:
  address: 0x0001c1a8 / decompile line 210
  function: bluefrog_machine_state_pump
  instruction_or_decompiled_stmt: flags_88 |= 0x100; flags_88 |= 0x02 after @T1 retry/failover path
  mask: 0x00000100 set, 0x00000002 set
  operation: set
  caller_context: startup-control retry/failover path
  required_rx_or_event: retry/counter exhaustion on the surrounding startup branch
  enables_tx_candidate: session transition/fallback phase
  disables_tx_candidate: none directly
  confidence: confirmed_code for writes; inferred for branch label
```

#### `flags_88` state-pump consumers

```text
flags88_consumer:
  address: 0x0001bf3a
  tested_mask: 0x00000001
  tx_frame_or_family: TY:
  tx_literal_or_format_addr: 0x00024990
  sender_helper: machine_uart_send_line_encoded
  required_cache_or_state_fields: retry counter A
  clears_before_or_after_tx: clears 0x00000004 before TX; failure path clears 0x01 and sets 0x02
  expected_rx: ty:<machine type/version>
  confidence: confirmed_code

flags88_consumer:
  address: 0x0001bf3a / 0x0001a690
  tested_mask: 0x00000004
  tx_frame_or_family: TY:/cachewriter session gate
  tx_literal_or_format_addr: 0x00024990 for TY:
  sender_helper: machine_uart_send_line_encoded for TY:
  required_cache_or_state_fields: for event40 helper requires flags_88 0x100 set and 0x200 clear
  clears_before_or_after_tx: TY: branch clears 0x04 before sending
  expected_rx: ty:<machine type/version> for TY: branch
  confidence: confirmed_code

flags88_consumer:
  address: 0x0001c164
  tested_mask: 0x00000040
  tx_frame_or_family: @TR:37
  tx_literal_or_format_addr: 0x000249c0
  sender_helper: machine_uart_send_prefix_hex_line
  required_cache_or_state_fields: retry counter B; zero payload
  clears_before_or_after_tx: success subhandler clears 0x40; retry exhaustion clears 0x40
  expected_rx: @tr:37,<hex>
  confidence: confirmed_code

flags88_consumer:
  address: 0x0001bfc8
  tested_mask: 0x00000400
  tx_frame_or_family: @t2:<hex payload>
  tx_literal_or_format_addr: 0x00024998
  sender_helper: machine_uart_send_prefix_hex_line
  required_cache_or_state_fields: bluefrog_state+0x8c, bluefrog_state+0x2c, session_state_flag19 bits
  clears_before_or_after_tx: clears the prefix-hex/status transfer bit before TX
  expected_rx: not proven at callsite
  confidence: confirmed_code

flags88_consumer:
  address: 0x0001c10c
  tested_mask: 0x00000800
  tx_frame_or_family: @t3
  tx_literal_or_format_addr: 0x000249a0
  sender_helper: machine_uart_send_line_encoded
  required_cache_or_state_fields: statepump line-TX marker
  clears_before_or_after_tx: clears line-TX bit before TX and sets TX marker 1
  expected_rx: not proven at callsite
  confidence: confirmed_code

flags88_consumer:
  address: multiple session/cache gates
  tested_mask: 0x00000100
  tx_frame_or_family: no direct startup TX proven
  tx_literal_or_format_addr: none
  sender_helper: none
  required_cache_or_state_fields: used by set_cachewriter_event40_if_session_flags_allow and session transition paths
  clears_before_or_after_tx: none tied to direct TX
  expected_rx: none
  confidence: confirmed_code for tests; inferred for meaning

flags88_consumer:
  address: unresolved direct consumer
  tested_mask: 0x00040000
  tx_frame_or_family: startup one-shot state
  tx_literal_or_format_addr: none proven
  sender_helper: none proven
  required_cache_or_state_fields: unknown
  clears_before_or_after_tx: unknown
  expected_rx: unknown
  confidence: unknown
```

#### RX handler state effects

```text
rx_handler_state_effect:
  rx_prefix: @t0
  handler_address: 0x0001d9b4
  parsed_fields: none
  writes_state70: none confirmed
  writes_flags88:
    - via FUN_0001bc6c: flags_88 &= 0xffffffef
  writes_bluefrog_state_offsets:
    - bluefrog_state+0x8f = 0
  writes_machine_cache_offsets:
    - calls init_machine_status_progress_caches_and_ble_values()
  sets_events_or_timers:
    - clears two local byte flags near the ty/startup handler block
  enables_next_statepump_branch: none directly proven
  expected_next_tx: none directly proven
  confidence: confirmed_code

rx_handler_state_effect:
  rx_prefix: @t1
  handler_address: 0x0001ce78
  parsed_fields: none
  writes_state70: none confirmed
  writes_flags88:
    - flags_88 &= 0xfffffffd
  writes_bluefrog_state_offsets:
    - clears machine_handshake_rx_pending_flag
    - clears machine_rx_line_first_byte_ptr
    - writes line_first_byte_ptr = 0x50 ('P')
  writes_machine_cache_offsets: none confirmed
  sets_events_or_timers: none confirmed
  enables_next_statepump_branch: none directly proven
  expected_next_tx: none directly proven
  confidence: confirmed_code

rx_handler_state_effect:
  rx_prefix: ty:
  handler_address: 0x0001d9c8 / dispatcher pointer 0x0001d9d1 Thumb
  parsed_fields: machine type/version line accepted by dispatcher; detailed field copy not isolated in this handler body
  writes_state70: none confirmed
  writes_flags88:
    - flags_88 &= 0xfffffffe
    - if 0x04 was clear: flags_88 &= 0xfffffffd; flags_88 |= 0x00000004
  writes_bluefrog_state_offsets: none directly confirmed beyond flags_88
  writes_machine_cache_offsets: none directly confirmed in handler body
  sets_events_or_timers:
    - optional app_mode_set_flag08_if_state_allows()
    - arm_state_event20_clear_event10_if_allowed()
  enables_next_statepump_branch: later startup/session phase; exact next TX not uniquely encoded in this handler
  expected_next_tx: unresolved
  confidence: confirmed_code

rx_handler_state_effect:
  rx_prefix: @T2
  handler_address: 0x0001ced8
  parsed_fields:
    - ASCII-hex bytes from line+4 up to line+0x1e
  writes_state70: none confirmed
  writes_flags88:
    - flags_88 &= 0xffffffdf
    - flags_88 |= 0x00000400
    - if ((bluefrog_state+0x2c) & 0x7f) == 0: flags_88 |= 0x00000100
  writes_bluefrog_state_offsets:
    - bluefrog_state+0x2c.. receives parsed T2 bytes
    - bluefrog_state+0x8d receives length/marker
  writes_machine_cache_offsets:
    - machine_cache_base+0x72.. mirrors parsed T2 bytes
  sets_events_or_timers: none separately proven
  enables_next_statepump_branch: @t2:<hex payload> branch at 0x0001bfc8
  expected_next_tx: @t2:<hex payload>
  confidence: confirmed_code

rx_handler_state_effect:
  rx_prefix: @T3
  handler_address: 0x0001da24
  parsed_fields:
    - u16 identity code from line+4
    - identity/version string from line+8
  writes_state70: none confirmed
  writes_flags88:
    - flags_88 |= 0x00000800
    - if ((bluefrog_state+0x2c) & 0x7f) != 0: flags_88 |= 0x00000100
  writes_bluefrog_state_offsets: none directly confirmed beyond flags_88
  writes_machine_cache_offsets:
    - machine_cache_base+0x68 = parsed identity code
    - machine_cache_base+0x88 = identity/version string, max 0x10 bytes
  sets_events_or_timers: none separately proven
  enables_next_statepump_branch: @t3 branch at 0x0001c10c
  expected_next_tx: @t3
  confidence: confirmed_code

rx_handler_state_effect:
  rx_prefix: @tr:37
  handler_address: 0x0001d788 dispatcher -> 0x0001d806 code37 subhandler
  parsed_fields:
    - four 16-bit ASCII-hex fields from @tr:37 payload
  writes_state70: none; writes machine_cache_base+0x70, not bluefrog_state+0x70
  writes_flags88:
    - flags_88 &= 0xffffffbf
    - flags_88 |= 0x00000200
  writes_bluefrog_state_offsets:
    - flags_88 only
  writes_machine_cache_offsets:
    - machine_cache_base+0x6a = parsed u16[0]
    - machine_cache_base+0x6c = parsed u16[1]
    - machine_cache_base+0x6e = parsed u16[2]
    - machine_cache_base+0x70 = parsed u16[3]
  sets_events_or_timers:
    - clears DAT_20002798 marker
  enables_next_statepump_branch: post-gate/session latch via flags_88 0x200
  expected_next_tx: none directly proven
  confidence: confirmed_code
```

#### Static sequence conclusion

```text
startup_static_gap_resolution:
  state70_writers_mapped_for_requested_bits: YES
  flags88_writers_mapped_for_requested_masks: YES
  rx_handler_state_effects_mapped: YES
  original_sequence_order_resolved: NO
```

The individual producer/consumer edges are now mapped for the requested `state70` bits and
`flags_88` masks, but the exact original startup order is still not fully statically derivable.
The remaining gaps are no longer broad "unknown handler" gaps; they are specific missing producer
or relation points:

```text
unresolved_sequence_blockers:
  - missing_writer_for: state70_bits3_4_substate
    affects_tx: @T0
    why_unresolved: state70 bit0 producer is confirmed, but the producer for the substate required by ((state70 & 0x1f) >> 3) == 3 was not found in the focused XREF pass.
    next_static_target: all low-byte/low-nibble assignments to bluefrog_state+0x70, especially init and table-driven writes around main_loop_or_scheduler.

  - missing_writer_for: session_tick_counter_state seed feeding state70 bit5/bit6
    affects_tx: @H1 / @T1
    why_unresolved: the bit5/bit6 writer is confirmed in maybe_session_tick_counter_and_timeout_pump, but the RX/event that seeds the session tick counter consumed by that function is not yet mapped.
    next_static_target: writers of session_tick_counter_state+2/+4 and caller chain around 0x0001e8b4 / 0x0001e94a.

  - missing_writer_for: flags88_0x40_startup_producer
    affects_tx: @TR:37
    why_unresolved: a confirmed 0x40 producer exists after @TF/@TV cachewriter handling, but the startup producer that arms @TR:37 before the observed gate response was not found in this pass.
    next_static_target: all writes to flags_88 bit6 through non-literal assignments, retry/failover paths, and aliases of bluefrog_state_flags_88.

  - missing_consumer_for: flags88_0x40000
    affects_tx: startup one-shot state
    why_unresolved: main_loop_or_scheduler sets 0x00040000, but no direct startup TX consumer was proven.
    next_static_target: high-bit tests in bluefrog_machine_state_pump and companion startup helpers.
```

`@D1` remains excluded:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
```

### 17.x `@TP` / stayInBLE Path Analysis

Scope of this pass:

```text
only_static_analysis: YES
esp_code_changes: NO
runtime_tests: NO
new_tx_logic: NO
focus: BLE 0x1529 payload 00 7F 80 -> @TP:...
```

#### `@TP:` TX path

```text
tp_path_analysis:
  tp_literal_addr: 0x00024a20
  tx_callsite_addresses:
    - 0x0001c358
  sender_helper: machine_uart_send_prefix_hex_line
  source_ble_characteristic: 0x1529 PMode / Control Write
  source_ble_payloads:
    - 00 7F 80: stayInBLE / heartbeat path
    - 00 7F A5: DFU / bootloader path, dangerous
    - 00 4D ...: PMode/settings path, state-changing
    - 00 47 01: process next/OK style path, state-changing
    - 00 47 FF: process cancel/back style path, state-changing
    - 00 7F 82 ...: PIN/auth path
  path_for_payload_00_7F_80:
    callback: ble_char_1529_pmode_write_callback @ 0x00018900
    copied_to:
      - machine_cache_base+0x26, based on previous decompilation notes
      - exact field name remains inferred because the descriptor/state struct is not fully typed
    length_handling:
      - payload is staged as a short 0x1529 control payload
      - previous notes indicate the last byte is separated as followup byte 0x80
    followup_call:
      - pmode_write_followup_or_cache_notify @ 0x00017ef4, called with the final byte such as 0x80
    flags_or_state_set:
      - statepump dynamic/control branch later becomes eligible
      - exact bit/field producer is not fully isolated in this pass
    statepump_branch:
      - bluefrog_machine_state_pump emits the dynamic @TP: prefix branch
      - callsite 0x0001c358 uses literal @TP: at 0x00024a20
    tx_format:
      - machine_uart_send_prefix_hex_line("@TP:", payload, length)
      - observed/reconstructed family: @TP:<key>7F
    produced_machine_line_example:
      - @TP:<key>7F
      - ESP-side diagnostics currently format this as @TP:%02X7F using pmode_key_
    confidence: PARTIAL
```

Important distinction:

```text
@TP: is confirmed as a machine-TX frame family in the original BLE firmware.
The path from BLE 0x1529 payload 00 7F 80 to a one-byte/short @TP payload is
strongly indicated by the callback/followup/statepump notes, but the exact
state field that arms the statepump branch remains partially unresolved.
```

#### Concrete `00 7F 80` to machine line

```text
tp_007f80_to_machine_line:
  resolved: PARTIAL
  cleartext_line_template: @TP:<key>7F
  key_source:
    - unresolved in original firmware
    - likely a session/PMode key byte from the BLE/control state or machine/session cache
    - ESP diagnostics use pmode_key_ when manually formatting @TP:%02X7F
  payload_bytes_consumed:
    - 00: BLE/control-family selector, not emitted as visible @TP payload
    - 7F: retained in the visible machine payload as trailing 7F
    - 80: followup/stayInBLE marker consumed by pmode_write_followup_or_cache_notify
  payload_bytes_not_sent:
    - 00 is not visible in @TP:<key>7F
    - 80 is not visible in @TP:<key>7F in the current reconstruction
  encoded:
    - sent through machine_uart_send_prefix_hex_line
    - that helper ultimately uses machine_uart_send_line_encoded, so transport encoding depends on active gate/session state
  required_prior_state:
    - BLE 0x1529 write received
    - post-gate/session state likely required for encoded machine UART transport
    - exact statepump arm bit/field for @TP branch is not fully isolated
  confidence: PARTIAL
```

Current unresolved key question:

```text
tp_key_source_resolved: PARTIAL
known:
  - ESP-side implementation derives the line from pmode_key_.
  - Original firmware sends @TP: through a prefix+hex helper, so the key is a byte payload source rather than part of the literal.
unknown:
  - whether <key> is copied from BLE payload state, machine_cache_base, a session byte, or another control-state field.
```

#### Safety classification

```text
tp_payload_safety_table:
  - payload: 00 7F 80
    meaning: stayInBLE / heartbeat / keep BLE-app mode alive
    resulting_machine_tx: @TP:<key>7F
    state_changing: NO_OR_LOW, based on app heartbeat semantics
    dangerous: NO
    safe_to_consider_for_emulation: YES_FOR_FUTURE_RUNTIME_TEST_ONLY
    confidence: PARTIAL

  - payload: 00 7F A5
    meaning: DFU / bootloader
    resulting_machine_tx: not required for this pass; app path reaches bootloader mode
    state_changing: YES
    dangerous: YES
    safe_to_consider_for_emulation: NO
    confidence: confirmed_app_and_reverse_map

  - payload: 00 4D ...
    meaning: PMode/settings/product/limit style control
    resulting_machine_tx: PMode/control dependent, not a status heartbeat
    state_changing: YES
    dangerous: UNKNOWN_BY_PAYLOAD, treat as unsafe unless fully decoded
    safe_to_consider_for_emulation: NO
    confidence: confirmed_family

  - payload: 00 47 01
    meaning: process next/OK/navigation
    resulting_machine_tx: process/control dependent
    state_changing: YES
    dangerous: POSSIBLE_PROCESS_ACTION
    safe_to_consider_for_emulation: NO
    confidence: confirmed_family

  - payload: 00 47 FF
    meaning: process cancel/back/navigation
    resulting_machine_tx: process/control dependent
    state_changing: YES
    dangerous: POSSIBLE_PROCESS_ACTION
    safe_to_consider_for_emulation: NO
    confidence: confirmed_family

  - payload: 00 7F 82 ...
    meaning: PIN/auth
    resulting_machine_tx: auth/control dependent
    state_changing: YES_OR_AUTH_STATE
    dangerous: UNKNOWN
    safe_to_consider_for_emulation: NO
    confidence: confirmed_family
```

#### Relation to `@TF/@TV`

```text
tp_to_live_status_relation:
  direct_tf_tv_trigger_found: NO
  indirect_gate_or_mode_relation_found: PARTIAL
  affects_flags_88:
    - no confirmed direct write to flags_88 0x04/0x100/0x200/0x40 from @TP:<key>7F in the documented path
    - @TP belongs to the post-gate BLE app/control keepalive family, not the core TY/T2/T3/TR37 gate chain
  affects_machine_cache:
    - 0x1529 callback stages control payload near machine_cache_base+0x26 per previous decompilation notes
    - no confirmed direct write to the 0x1524/0x1527 cache value buffers from stayInBLE
  affects_ble_cache:
    - no confirmed direct call to copy_machine_cache_to_ble_value_buffer(0/1)
    - no confirmed direct ble_characteristic_event_dispatch(0x1524/0x1527)
  likely_role:
    - BLE app heartbeat / stay-in-BLE mode keepalive
    - plausible missing app-presence signal after successful core gate
    - not statically proven as the cause of machine-originated @TF/@TV emission
  confidence: PARTIAL
```

Working conclusion:

```text
@TP:<key>7F remains the most plausible missing app-presence/heartbeat frame
because it is an original-known frame family and the classic app writes
00 7F 80 periodically. However, static analysis does not show it directly
triggering @TF/@TV or updating BLE status/progress caches. If tested later,
it should be a narrow runtime test of only 00 7F 80 / @TP:<key>7F, explicitly
excluding 00 7F A5, 00 4D..., 00 47..., PIN/auth, DFU, product, process, and
settings payloads.
```

ESP comparison:

```text
esp_current_normal_startup_missing_tp: YES
startup_sequence_diff_original_vs_esp:
  missing_in_esp_normal includes @TP:
runtime_changes_added_in_this_analysis: NO
esp_code_changes_added_in_this_analysis: NO
```

#### `@TP` blocker refinement: key source and statepump arm field

This pass narrows the two remaining static blockers without changing ESP code or
adding runtime tests.

```text
tp_key_source_trace:
  resolved: PARTIAL
  source_buffer: machine_cache_base+0x26
  source_length_field: machine_cache_base+0x39
  payload_007f80_after_callback:
    - ble_char_1529_pmode_write_callback @ 0x00018900 caps len to 0x13
    - copies payload bytes to machine_cache_base+0x26
    - writes original/capped length to machine_cache_base+0x39
    - because payload[0] == 0, decrements length from 3 to 2
    - calls pmode_write_followup_or_cache_notify @ 0x00017ef4 with payload[2] = 0x80
  bytes_used_for_machine_tx:
    - statepump @TP branch uses machine_cache_base+0x26 and length machine_cache_base+0x39
    - for staged 00 7F 80 the would-be prefix+hex payload is 00 7F if the @TP branch were armed
  bytes_not_sent:
    - 0x80 is consumed by pmode_write_followup_or_cache_notify, not included in the prefix+hex length
  key_byte_source:
    - first emitted byte would be machine_cache_base+0x26[0], copied from BLE payload[0]
    - for exact payload 00 7F 80 this byte is 0x00, not a separately proven session/random key
  value_byte_source:
    - second emitted byte would be machine_cache_base+0x26[1], copied from BLE payload[1] = 0x7F
  cleartext_line_template:
    - @TP:<hex(machine_cache_base+0x26, machine_cache_base+0x39)>
    - exact staged 00 7F 80 would format as @TP:007F only if the statepump arm bit is set
  example_if_payload_00_7F_80:
    - staged_buffer: 00 7F 80
    - stored_length_after_callback: 2
    - followup_byte: 80
    - would_be_machine_line_if_armed: @TP:007F
  confidence: confirmed_code_for_buffer_and_length, partial_for_actual_007f80_tx
```

Important correction to the earlier reconstruction:

```text
The exact BLE payload 00 7F 80 is confirmed to stage 00 7F and consume 80 as
the followup byte. However, the visible 0x1529 callback path does not set the
@TP statepump arm bit when payload[0] == 0. Therefore the earlier shorthand
"00 7F 80 -> @TP:<key>7F" is not fully proven and is likely too broad.
```

```text
tp_statepump_arm_trace:
  resolved: YES
  statepump_branch_addr:
    - test: 0x0001c2a2..0x0001c2a6
    - tx: 0x0001c346..0x0001c358
  branch_condition:
    - bluefrog_state_flags_88 bit 0x00010000 set
    - instruction pattern: ldr flags_88; lsls r2, flags_88, #0xf; bmi @TP_tx
  arm_field: bluefrog_state_flags_88 mask 0x00010000
  arm_writer_from_0x1529:
    - 0x00018942 branch when staged payload[0] != 0
    - calls 0x0001a67c
    - 0x0001a67c ORs bluefrog_state_flags_88 with (0x80 << 9) = 0x00010000
  clear_after_tx:
    - 0x0001c346..0x0001c34e clears bit with mask 0xfffeffff
  retry_or_counter:
    - none visible in the @TP branch itself
  tx_callsite: 0x0001c358
  sender_helper: machine_uart_send_prefix_hex_line @ 0x00016da4
  confidence: confirmed_code
```

```text
tp_origin_classification:
  self_started_without_ble_write: NO
  requires_ble_1529_write: YES
  app_stayinble_write_required: NO_OR_UNPROVEN_FOR_MACHINE_TX
  evidence:
    - @TP branch is armed by 0x1529 callback path via 0x0001a67c when payload[0] != 0
    - no startup/self-timer writer to flags_88 0x00010000 was found in this focused pass
    - exact 00 7F 80 path takes the payload[0] == 0 branch and calls 0x00017ef4 instead of 0x0001a67c
  confidence: confirmed_code_for_ble_write_origin, partial_for_app_stayinble_semantics
```

```text
tp_single_test_safety:
  safe_to_consider: PARTIAL
  dangerous_payloads_excluded:
    - 00 7F A5 / DFU bootloader
    - 00 4D... / PMode settings
    - 00 47... / process navigation
    - 00 7F 82... / PIN/auth
  could_enter_dfu: NO for exact 00 7F 80 path; YES for 00 7F A5, which remains excluded
  could_change_settings: UNKNOWN for the broad 0x1529 family, NO evidence for exact 00 7F 80
  expected_machine_response:
    - no direct response proven for exact 00 7F 80
    - no direct @TF/@TV trigger proven
  rollback_needed: NO for analysis only; no runtime action taken
  confidence: PARTIAL
```

```text
tp_indirect_live_relation:
  direct_tf_tv_trigger_found: NO
  indirect_mode_relation_found: PARTIAL
  affected_flags_88:
    - @TP machine TX branch: flags_88 0x00010000, but exact 00 7F 80 does not visibly arm it
    - no confirmed effect on core flags_88 0x04/0x100/0x200/0x40
  affected_machine_cache_fields:
    - machine_cache_base+0x26: staged 0x1529 payload buffer
    - machine_cache_base+0x39: staged payload length, shortened by one for payload[0] == 0
  affected_ble_fields:
    - pmode_write_followup_or_cache_notify @ 0x00017ef4 stores followup byte via 0x00019e8c
    - then dispatches BLE characteristic events 0x1524, 0x1527, 0x1534, 0x1533, 0x1538, 0x1532, 0x1535
    - this is an app/cache notification side effect, not a proven machine live-status request
  likely_role:
    - 0x1529 control/PMode/app-presence family
    - exact 00 7F 80 looks more like a local BLE/app-mode followup + characteristic event path than a confirmed @TP machine TX
  confidence: PARTIAL
```

Remaining focused blockers after this refinement:

```text
remaining_static_blockers_after_this_run:
  - exact_classic_app_semantics_of_00_7F_80_after_followup_notify
  - whether_any_other_0x1529_payload_used_by_app_arms_flags88_0x00010000_as_@TP
  - no_direct_tf_tv_trigger_from_tp_found
```

#### `pmode_write_followup_or_cache_notify(0x80)`

This pass follows only the `0x80` follow-up path reached by exact payload
`00 7F 80`. It does not add runtime tests or ESP TX logic.

```text
pmode_followup_0x80_analysis:
  function_addr: 0x00017ef4
  arg_0x80_branch_found: NO
  branch_condition:
    - no argument-specific branch found inside 0x00017ef4
    - r0 is saved in r5 and later stored generically through 0x00019e8c
  state_fields_written:
    - 0x00019e8c stores arg byte to *DAT_0x200025e4
    - DAT_0x200025e4 is initialized at 0x00018772 by 0x00019e7c
    - init pointer value is 0x20002ae4
    - 0x00019e7c initially writes 0x2A to 0x20002ae4
    - 0x00017ef4(0x80) therefore writes 0x80 to 0x20002ae4
  flags88_written:
    - none found
    - no writes to flags_88 0x04/0x40/0x100/0x200/0x40000 in this function
  machine_cache_fields_written:
    - transform/copy helper 0x00019f2c called twice before storing the new followup byte:
      - dest=0x20002d07, src=0x20002d07, len=0x28
      - dest=0x20002c3e, src=0x20002b48, len=0xc8
    - followup byte store: 0x20002ae4 = 0x80
    - no direct writes to 0x20002c11 / 0x20002c25 cache value buffers observed here
  ble_events_dispatched:
    - ble_characteristic_event_dispatch(0x1524)
    - ble_characteristic_event_dispatch(0x1527)
    - ble_characteristic_event_dispatch(0x1534)
    - ble_characteristic_event_dispatch(0x1533)
    - ble_characteristic_event_dispatch(0x1538)
    - ble_characteristic_event_dispatch(0x1532)
    - ble_characteristic_event_dispatch(0x1535)
  scheduler_events_armed:
    - none directly found
  machine_uart_tx_called:
    - NO
    - no calls to machine_uart_send_line_encoded, machine_uart_sendf_line_encoded, or machine_uart_send_prefix_hex_line
  return_effect:
    - returns after cache/key transform, followup-byte store, and BLE event dispatch
  confidence: confirmed_code
```

Callers found by Thumb BL target scan:

```text
pmode_followup_callers:
  - caller_addr: 0x00018938
    source_context: ble_char_1529_pmode_write_callback @ 0x00018900, classic BlueFrog control/PMode write path
    argument_value_or_source:
      - if staged payload[0] == 0 and len > 1:
      - length is decremented by one
      - argument is original payload[new_len]
      - for payload 00 7F 80, new_len=2 and argument=payload[2]=0x80
    payload_family:
      - exact 00 7F 80 stayInBLE/followup path
      - also same shape for any payload whose first byte is 0 and len > 1
    likely_meaning:
      - separates final control byte from staged visible payload and updates BLE/cache notification state
    confidence: confirmed_code

  - caller_addr: 0x0001971a
    source_context: BLE2 descriptor/callback path around descriptor 0x1625 callback 0x19701
    argument_value_or_source:
      - if len > 1, argument is last byte of the caller payload
      - then shortened payload is passed to 0x0001c78c
    payload_family:
      - BLE2 / 0x162x path, not classic 0x1529 BlueFrog startup path
    likely_meaning:
      - same followup-byte separation helper reused by BLE2 transport/control path
    confidence: confirmed_code_for_call, inferred_for_semantics
```

```text
followup_0x80_role:
  ble_app_presence_state: PARTIAL
  stayinble_heartbeat_state: PARTIAL
  machine_tx_state: NO
  live_cache_relation: PARTIAL
  affected_ble_characteristics:
    - 0x1524
    - 0x1527
    - 0x1534
    - 0x1533
    - 0x1538
    - 0x1532
    - 0x1535
  affected_flags88:
    - none found
  affected_app_mode_state:
    - no write to app_mode_state 0x20002db0 found in 0x00017ef4
  affected_machine_cache:
    - 0x20002ae4 receives 0x80 through global pointer DAT_0x200025e4
    - 0x20002ae4 is also the known 0x1531/About Machine value buffer base from the descriptor table
    - 0x20002d07 and 0x20002c3e regions are transformed/refreshed through 0x00019f2c before the new byte is stored
  likely_role:
    - local BLE/app control followup byte
    - cache/key or mode marker used by BLE-side value transformation/validation
    - notify/update trigger for several BLE cache characteristics
    - not a machine-UART command path
  confidence: PARTIAL
```

Relationship to machine-originated live status:

```text
followup_0x80_to_live_status_relation:
  direct_tf_tv_trigger_found: NO
  indirect_mode_or_presence_relation_found: PARTIAL
  cachewriter_acceptance_changed: NO
  evidence:
    - 0x00017ef4 does not call any machine UART TX helper
    - 0x00017ef4 does not set flags_88 0x04/0x40/0x100/0x200
    - 0x00017ef4 dispatches BLE events for 0x1524 and 0x1527, so the App may observe refreshed cache characteristics
    - no call to copy_machine_cache_to_ble_value_buffer(0/1) was found inside 0x00017ef4
    - no direct @TF/@TV emission trigger or cachewriter acceptance gate change was found
  confidence: PARTIAL
```

Consequence for the earlier `@TP` hypothesis:

```text
The exact 00 7F 80 classic path is better described as:
  BLE 0x1529 write 00 7F 80
  -> stage visible bytes 00 7F with length 2
  -> consume followup byte 80
  -> transform/update local BLE cache/key regions
  -> store 80 at 0x20002ae4
  -> dispatch BLE events for 0x1524/0x1527/0x153x

It is not statically proven to send @TP:007F, because the visible @TP branch
requires flags_88 0x00010000, and the exact payload[0] == 0 path does not call
the helper that sets that bit.
```

### Final Remaining Blocker: `session_tick_counter_state+4` seed for `@H1/@T1`

Scope for this pass:

```text
target_range:
  - 0x20002788..0x20002790

target_field:
  - 0x2000278c = session_tick_counter_state+4 halfword countdown

out_of_scope:
  - 0x25058 / 0x16294 / GPIO button watch / @T0 substate
  - @TF/@TV
  - @TV:81/@TV:82
  - @TS:9x
  - PMode / DFU / product commands
  - WLAN firmware
  - ESP runtime changes
```

#### Alias / block writer search

Direct XREFs remain restricted to the consumer/maintenance routine:

```text
session_tick_seed_alias_candidate:
  address: 0x0001e824..0x0001e887
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt:
    - 0x0001e83c reads *(uint16_t *)0x2000278c
    - 0x0001e84c reads *(uint16_t *)0x2000278c
    - 0x0001e852 writes decremented halfword back to 0x2000278c
    - 0x0001e874 clears 0x2000278c to zero on service-channel active path
  access: read/write/clear
  target_range: 0x20002788..0x20002790
  covers_0x2000278c: YES
  value_or_source:
    - decrement current value by 1
    - clear to 0 when service_channel_any_countdown_active() is true
  condition:
    - state70 bit5 currently clear
    - *(uint16_t *)0x2000278c != 0
    - function is called on the scheduler/tick cadence
  caller_chain:
    - main_loop_or_scheduler
    - backend scheduler handle registered by backend_register_callback_handle_guarded()
    - backend callback 0x0001e8b5
    - maybe_session_tick_counter_and_timeout_pump()
  confidence: confirmed_code
```

Boot/runtime init aliasing:

```text
session_tick_seed_alias_candidate:
  address: 0x0003ef98 / init table at 0x0003f1d4
  function: bootloader_runtime_init_copy_table_then_main
  instruction_or_decompiled_stmt:
    - table entry 0x0003f1e4: src=0x0003f318, dst=0x20002124, byte_count=0x186c, handler=0x0003efe6
    - range covers 0x20002124..0x20003990, including 0x2000278c
  access: init/clear
  target_range: 0x20002124..0x20003990
  covers_0x2000278c: YES
  value_or_source: zero-fill style bootloader/BSS init entry
  condition: bootloader runtime init path before bootloader_main()
  caller_chain:
    - bootloader_entry_after_reset
    - bootloader_runtime_init_copy_table_then_main
  confidence: confirmed_code for range coverage; inferred for zero-fill handler because 0x0003efe6 is table-handler data adjacent to bootloader memset helpers
  note:
    - This clears the field to zero. It does not seed a nonzero countdown for @H1/@T1.
    - It belongs to bootloader runtime initialization and is not evidence of a BlueFrog runtime seed.
```

Application `.data` copy aliasing:

```text
session_tick_seed_alias_candidate:
  address: 0x00020798..0x000207c2
  function: app/runtime .data copy helper
  instruction_or_decompiled_stmt:
    - source 0x0002504c
    - destination 0x20002000
    - end 0x200020f8
  access: copy/init
  target_range: 0x20002000..0x200020f8
  covers_0x2000278c: NO
  value_or_source: flash .data image
  condition: reset/runtime init
  caller_chain: reset/startup path
  confidence: confirmed_code
```

Other memset/copy candidates checked:

```text
session_tick_seed_alias_candidate:
  address: multiple crt_memset_bytes callers
  function: searched memset/copy callers
  instruction_or_decompiled_stmt:
    - machine_transport_state_clear clears machine_transport_state_block only
    - machine_passthrough_fifo_init clears machine_passthrough_fifo_struct only
    - init_machine_status_progress_caches_and_ble_values clears machine_cache/progress areas only
    - gpio_watch_table_init clears GPIO watch table only
    - app_mode and cache init clears unrelated blocks
  access: clear/copy
  target_range: varied, none overlap 0x20002788..0x20002790
  covers_0x2000278c: NO
  value_or_source: zero/copy unrelated state
  condition: init/runtime helpers
  caller_chain: varied
  confidence: confirmed_code from crt_memset_bytes XREF list
```

Raw pointer scan summary:

```text
raw_pointer_scan_checked: YES

raw_pointer_hits_near_target:
  - 0x20002784 referenced at 0x0001e5cc, app-mode notification byte/state; adjacent but separate
  - 0x20002788 referenced at 0x0001e888 and 0x0001e8b0; both literals for maybe_session_tick_counter_and_timeout_pump
  - 0x20002790 referenced at 0x0001eb9c, 0x0001ebdc, 0x0001ec04, 0x0001ec24; backend scheduler handle/toggle state adjacent after target object

raw_pointer_hits_absent:
  - no flash pointer to 0x2000278c
  - no flash pointer to 0x2000278a
  - no direct larger-struct pointer that is consumed as a write source for 0x2000278c
```

#### Structure around `0x20002788`

```text
session_tick_counter_state_layout:
  base: 0x20002788
  suspected_struct_start: 0x20002788
  suspected_struct_end: 0x20002790
  fields:
    - offset: +0x00
      width: 4 bytes
      access_pattern:
        - read at 0x0001e828
        - increment/write at 0x0001e830 while value <= 0x32
        - clear/write at 0x0001e86c when threshold exceeded
      meaning: coarse tick accumulator; every rollover calls decrement_service_channel_countdowns()

    - offset: +0x04
      width: 2 bytes
      access_pattern:
        - read/test nonzero at 0x0001e83c
        - read/decrement at 0x0001e84c
        - write decremented value at 0x0001e852
        - clear to zero at 0x0001e874
      meaning: H1/T1 halfword countdown gate; nonzero enables production of state70 bit5

    - offset: +0x06
      width: 2 bytes
      access_pattern: no direct XREF in this pass
      meaning: padding or unused/reserved in this small object

  neighboring_ram_symbols_or_literals:
    - 0x20002784: app-mode/notification adjacent state, referenced by ble_mode_state_update_enqueue
    - 0x20002790: backend callback/toggle state used by 0x0001e8b4 and backend_start_or_enable_handle_guarded
    - 0x20002794: backend handle slot written by backend_register_callback_handle_guarded, read by backend_start_or_enable_handle_guarded

  confidence: PARTIAL
```

Layout interpretation:

```text
session_tick_counter_state:
  0x20002788..0x2000278f is a small standalone BSS object immediately before backend handle state.
  0x20002790 begins the scheduler/toggle state for the periodic callback that invokes the tick routine.
  The target halfword is not part of bluefrog_state and not part of the 0x20002000 .data copy image.
```

#### Tick caller chain

Registration path:

```text
tick_caller_chain:
  - function: main_loop_or_scheduler
    callsite: 0x0001647a
    cadence_or_trigger: startup initialization
    condition: after GPIO/backend setup and before machine transport init/start
    relationship_to_session_tick_counter_state:
      - calls backend_register_callback_handle_guarded()

  - function: backend_register_callback_handle_guarded
    callsite: 0x0001ebc8..0x0001ebca
    cadence_or_trigger: registration only
    condition: startup init
    relationship_to_session_tick_counter_state:
      - registers callback pointer 0x0001e8b5
      - writes returned handle index into 0x20002794

  - function: main_loop_or_scheduler
    callsite: 0x00016482
    cadence_or_trigger: startup enable
    condition: after backend_register_callback_handle_guarded()
    relationship_to_session_tick_counter_state:
      - calls backend_start_or_enable_handle_guarded()

  - function: backend_start_or_enable_handle_guarded
    callsite: 0x0001ebee..0x0001ebf2
    cadence_or_trigger: backend enqueue/start
    condition: reads handle from 0x20002794
    relationship_to_session_tick_counter_state:
      - calls backend_enqueue_start_handle(handle, 0x10, 0)
      - 0x10 is the scheduler interval/timeout value for callback 0x0001e8b5

  - function: backend scheduler dispatch
    callsite: indirect, callback pointer 0x0001e8b5
    cadence_or_trigger: backend interval callback
    condition: scheduler handle active
    relationship_to_session_tick_counter_state:
      - invokes 0x0001e8b4 tick body

  - function: 0x0001e8b4 tick body
    callsite: 0x0001e94a
    cadence_or_trigger:
      - callback toggles/accumulates state at 0x20002790 and 0x20002050
      - calls maybe_session_tick_counter_and_timeout_pump() only on its internal 10-tick cadence and when state70 bit0 is set
    condition:
      - state70 bit0 set
      - internal halfword puVar6[3] exceeds 9
    relationship_to_session_tick_counter_state:
      - calls maybe_session_tick_counter_and_timeout_pump()

  - function: maybe_session_tick_counter_and_timeout_pump
    callsite: entered from 0x0001e94a
    cadence_or_trigger: every tenth backend tick while state70 bit0 is set
    condition:
      - reads/increments 0x20002788
      - only uses 0x2000278c if it is already nonzero and state70 bit5 is clear
    relationship_to_session_tick_counter_state:
      - decrements/clears 0x2000278c
      - sets state70 bit5 and optionally bit6
```

#### Seed result

```text
session_tick_counter_state_seed_result:
  seed_writer_found: NO
  direct_xrefs_checked: YES
  raw_pointer_scan_checked: YES
  block_copy_aliasing_checked: YES
  larger_struct_checked: PARTIAL
  caller_chain_checked: YES
  seed_writer_address: none found
  seed_writer_function: none found
  seed_value_or_source: none found
  condition: unresolved
  can_explain_H1_T1_timing: PARTIAL
  likely_reason_unresolved:
    - 0x2000278c is explicitly consumed as a nonzero countdown gate, but the nonzero seed is not present in direct data XREFs.
    - Boot/runtime block writers found in this pass either do not cover the target or clear it to zero.
    - The seed may be written through an unresolved alias, an event/scheduler data structure not recovered by Ghidra XREFs, or a store whose base register is not associated with 0x20002788.
  next_static_target:
    - recover backend scheduler queue/event writes that can pass context into callback 0x0001e8b5
    - inspect low-level stores into RAM window 0x20002780..0x20002798 by instruction pattern rather than symbol XREF
    - inspect unresolved computed-call targets around backend scheduler dispatch
```

Consequence for `@H1/@T1`:

```text
H1_T1_consequence:
  known:
    - If 0x2000278c is nonzero and service_channel_any_countdown_active() is false until countdown reaches zero, state70 bit5 is set and bit6 is clear -> @H1 branch.
    - If 0x2000278c is nonzero and service_channel_any_countdown_active() is true, state70 bit5 and bit6 are set -> @T1 branch.
    - The periodic backend callback path and cadence are statically mapped.
  unknown:
    - Which original event seeds 0x2000278c to a nonzero value.
  confidence:
    - confirmed_code for consumer/timing mechanics
    - unknown for seed producer
```

`@D1` remains excluded:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
```

### Backend Scheduler Role of `session_tick_counter_state`

Scope for this pass:

```text
focus:
  - maybe_session_tick_counter_and_timeout_pump
  - registered backend callback 0x0001e8b5 / clear address 0x0001e8b4
  - session_tick_counter_state = 0x20002788
  - session_tick_counter_state+4 = 0x2000278c
  - backend registration/start wrappers around 0x20002790 / 0x20002794

out_of_scope:
  - 0x25058 / 0x16294 / GPIO button watch / @T0 substate
  - @TF/@TV
  - @TV:81/@TV:82
  - @TS:9x
  - PMode / DFU / product commands
  - WLAN firmware
  - ESP runtime changes
```

#### Backend registration of callback `0x0001e8b5`

```text
tick_backend_registration:
  callsite: 0x0001647a
  caller_function: main_loop_or_scheduler
  registration_function: backend_register_callback_handle_guarded
  callback_arg: indirect, wrapper-local literal backend_callback_guard_value = 0x0001e8b5
  context_arg: none at registration site
  interval_or_period_arg: none at registration site
  flags_arg: enabled byte 0x01 passed to backend_register_handle()
  handle_storage: 0x20002794 = backend_callback_handle_state+4
  enabled_immediately: NO
  condition:
    - after backend_scheduler_init()
    - after GPIO watch/button-watch init
    - before machine_transport_ring_and_peripheral_init()
  confidence: confirmed_code

tick_backend_registration:
  callsite: 0x0001ebca
  caller_function: backend_register_callback_handle_guarded
  registration_function: backend_register_handle
  callback_arg: 0x0001e8b5
  context_arg: not passed as separate context; callback_or_context parameter is the callback pointer
  interval_or_period_arg: none
  flags_arg: 0x01
  handle_storage: 0x20002794
  enabled_immediately: NO
  condition: wrapper called from main_loop_or_scheduler during startup init
  confidence: confirmed_code

tick_backend_registration:
  callsite: 0x00016482
  caller_function: main_loop_or_scheduler
  registration_function: backend_start_or_enable_handle_guarded
  callback_arg: previously registered handle read from 0x20002794
  context_arg: 0
  interval_or_period_arg: 0x10
  flags_arg: none visible; third argument is 0
  handle_storage: 0x20002794
  enabled_immediately: YES
  condition:
    - immediately after machine_transport_ring_and_peripheral_init()
    - before session record load and BLE/service init
  confidence: confirmed_code

tick_backend_registration:
  callsite: 0x0001ebf2
  caller_function: backend_start_or_enable_handle_guarded
  registration_function: backend_enqueue_start_handle
  callback_arg: handle index read from 0x20002794
  context_arg: 0
  interval_or_period_arg: 0x10
  flags_arg: none visible
  handle_storage: 0x20002794
  enabled_immediately: YES
  condition: wrapper called from startup and after flash/session persistence operations
  confidence: confirmed_code
```

Handle and neighboring object relation:

```text
backend_handle_relation:
  0x20002788..0x2000278f:
    role: session_tick_counter_state object used directly by maybe_session_tick_counter_and_timeout_pump
    contains:
      - +0x00 word coarse tick accumulator
      - +0x04 halfword H1/T1 countdown gate
      - +0x06 no recovered access, likely padding/reserved

  0x20002790:
    role: adjacent backend callback handle state base used by start/stop wrappers
    evidence:
      - backend_start_or_enable_handle_guarded passes backend_handle_start_state+4 to read the handle
      - backend_stop_or_disable_handle_guarded passes backend_handle_stop_state+4 to read the handle

  0x20002794:
    role: stored backend handle index for callback 0x0001e8b5
    evidence:
      - backend_register_callback_handle_guarded passes (0x20002790 + 4) as out_handle_index
      - backend_start_or_enable_handle_guarded reads *(uint32_t *)0x20002794
      - backend_stop_or_disable_handle_guarded reads *(uint32_t *)0x20002794

  interpretation:
    - The handle state is adjacent to, but distinct from, the countdown object.
    - No callback context pointer to 0x20002788 is passed via backend_enqueue_start_handle(); the callback uses globals.
  confidence: confirmed_code for addresses and accesses; inferred for object boundary
```

#### Callback ABI

```text
tick_callback_abi:
  function: 0x0001e8b4 tick body, registered as Thumb pointer 0x0001e8b5
  param_1_source: none recovered / unused
  param_2_source: none recovered / unused
  uses_context_pointer: NO
  uses_global_0x20002788: YES
  reads_0x2000278c: YES
  writes_0x2000278c: YES
  writes_state70: YES
  relationship_to_H1_T1:
    - callback body reaches maybe_session_tick_counter_and_timeout_pump() on its internal cadence
    - maybe_session_tick_counter_and_timeout_pump() consumes the halfword countdown at 0x2000278c
    - when countdown expires with no active service-channel countdown, it sets state70 bit5 and clears bit6 -> @H1 branch
    - when a service-channel countdown is active, it clears 0x2000278c and sets state70 bit5 and bit6 -> @T1 branch
  confidence: confirmed_code
```

The direct consumer remains:

```text
maybe_session_tick_counter_and_timeout_pump:
  function: 0x0001e824
  signature: void maybe_session_tick_counter_and_timeout_pump(void)
  coarse_tick:
    - increments *(uint32_t *)0x20002788 while below 0x32
    - resets *(uint32_t *)0x20002788 to 0 and calls decrement_service_channel_countdowns() on rollover
  countdown_gate:
    - only evaluated when state70 bit5 is clear
    - only evaluated when *(uint16_t *)0x2000278c is nonzero
  service_channel_branch:
    - service_channel_any_countdown_active() == true:
      - *(uint16_t *)0x2000278c = 0
      - state70 |= 0x20
      - state70 |= 0x40
      - enables @T1 selector
    - service_channel_any_countdown_active() == false:
      - decrement *(uint16_t *)0x2000278c
      - if countdown reaches zero:
        - state70 |= 0x20
        - state70 &= ~0x40
        - enables @H1 selector
  confidence: confirmed_code
```

#### Classification of `0x2000278c`

```text
session_tick_counter_classification:
  candidate_role: timeout_counter
  evidence_for_startup_seed:
    - Weak/inconclusive: state70 bit5/bit6 feed startup-looking TX branches @H1/@T1.
    - No seed writer from startup RX handlers or linear startup state was found.
  evidence_for_timeout_counter:
    - registered through backend scheduler as callback 0x0001e8b5
    - started with interval_or_period_arg=0x10
    - consumed only by a periodically invoked tick/timeout pump
    - value is decremented to zero rather than matched against a startup state
    - one branch explicitly depends on service_channel_any_countdown_active()
  evidence_for_periodic_poll:
    - callback registration/start is unconditional during main_loop_or_scheduler init
    - tick body is independent of machine RX lines once state70 bit0 is set
    - repeated start/stop occurs around flash/session persistence, consistent with backend timing rather than machine startup sequencing
  evidence_for_service_channel:
    - calls decrement_service_channel_countdowns()
    - calls service_channel_any_countdown_active()
    - @T1-vs-@H1 selector is chosen by service_channel_any_countdown_active()
  evidence_against_startup_sequence:
    - no direct RX handler was found writing 0x2000278c
    - no direct startup init writer seeds 0x2000278c nonzero
    - backend_enqueue_start_handle() passes context_arg=0, so 0x20002788 is not a callback context object seeded by registration
    - @H1/@T1 production requires elapsed backend ticks and the nonzero countdown gate
  confidence: PARTIAL
```

#### Relationship to original startup sequence

```text
h1_t1_sequence_role:
  is_linear_startup_step: NO
  is_timeout_generated: YES
  is_periodic_backend_generated: YES
  should_be_required_for_esp_startup_emulation: UNKNOWN
  reason:
    - The static code path proves @H1/@T1 can be produced from a periodic backend timeout chain.
    - The path does not prove that @H1/@T1 are mandatory ordered startup frames.
    - The remaining missing piece is not a statepump order relation but the nonzero seed writer for 0x2000278c.
    - Until that seed is found, @H1/@T1 should be treated as timeout/backend side effects that may occur during startup, not as confirmed required linear startup steps.
```

Updated blocker interpretation:

```text
remaining_static_blockers_after_this_run:
  - blocker: session_tick_counter_state+4_nonzero_seed_writer
    why_unresolved:
      - The backend registration, callback ABI, callback cadence, and H1/T1 consumer logic are mapped.
      - The nonzero seed writer for 0x2000278c is still not found.
      - The code now indicates that the unresolved item is more likely a timeout/service-channel seed than a missing linear startup-sequence writer.
    next_static_target:
      - backend/service-channel setup paths that can seed delayed operations
      - stores into service-channel countdown state which may be paired with 0x2000278c
      - unresolved scheduler/event callbacks that write small global timeout objects
```

`@D1` remains excluded:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
```

### Core Session Gate Chain: `TY` / `T2` / `T3` / `TR37`

Scope for this pass:

```text
focus:
  - TY:
  - @T2 / @t2:
  - @T3 / @t3
  - @TR:37 / @tr:37
  - bluefrog_state_flags_88 masks 0x100, 0x400, 0x800, 0x40

out_of_scope:
  - @T0
  - @H1 / @T1
  - @D1
  - state70 / GPIO button watch
  - backend scheduler callback 0x0001e8b5
  - @TV:81 / @TV:82
  - @TS:9x
  - PMode / DFU / product commands
  - WLAN firmware
```

#### Core session chain steps

```text
core_session_chain_step:
  step_id: rx_t2
  direction: RX
  frame_or_prefix: @T2...
  handler_or_callsite: machine_rx_t2_state_handler @ 0x0001ced8
  required_flags_before: none proven
  required_state_before: valid machine line dispatched as @T2
  writes_flags_88:
    - clears 0x20 phase/retry marker with flags_88 &= 0xffffffdf
    - sets 0x400
    - conditionally sets 0x100 when first parsed T2 byte masked with 0x7f is zero
  clears_flags_88:
    - 0x20
  writes_machine_cache:
    - machine_cache_base+0x72.. mirrored T2 bytes
  writes_bluefrog_state:
    - bluefrog_state+0x2c.. mirrored T2 bytes
    - DAT_20002791 tx-in-progress marker/state byte is cleared
  enables_next_tx:
    - @t2:<hex payload> branch at 0x0001bfc8 when flags_88 bit selected by `flags_88 << 0x15 < 0` is consumed
  expected_next_rx:
    - not encoded in handler body; @T2 is a producer for @t2: state
  confidence: confirmed_code

core_session_chain_step:
  step_id: tx_t2
  direction: TX
  frame_or_prefix: @t2:<hex payload>
  handler_or_callsite: bluefrog_machine_state_pump @ 0x0001bfc8
  required_flags_before:
    - (flags_88 & 0x1) == 0
    - flags_88 bit tested by `flags_88 << 0x15 < 0`
  required_state_before:
    - bluefrog_state+0x8c payload length/source selector
    - bluefrog_state+0x2c payload/control byte source from @T2 handler
    - session_state_flag19-derived second payload byte
  writes_flags_88: none proven before TX
  clears_flags_88:
    - clears the consumed prefix-hex status bit before TX
  writes_machine_cache: none at TX callsite
  writes_bluefrog_state:
    - tx_in_progress_marker = 'P'
  enables_next_tx: none directly; sends response/control line toward machine
  expected_next_rx: not statically encoded at callsite
  confidence: confirmed_code

core_session_chain_step:
  step_id: rx_t3
  direction: RX
  frame_or_prefix: @T3:<identity/version>
  handler_or_callsite: machine_rx_t3_identity_handler @ 0x0001da24
  required_flags_before: none proven
  required_state_before: valid machine line dispatched as @T3
  writes_flags_88:
    - sets 0x800
    - conditionally sets 0x100 when (bluefrog_state+0x2c & 0x7f) is nonzero
  clears_flags_88: none confirmed in handler body
  writes_machine_cache:
    - machine_cache_base+0x68 = parsed u16 from line+4
    - machine_cache_base+0x88 = identity/version text from line+8, max 0x10 bytes
  writes_bluefrog_state: none confirmed beyond flags_88
  enables_next_tx:
    - @t3 branch candidate at 0x0001c10c
  expected_next_rx:
    - not encoded in handler body; @T3 is a producer for identity/cache and @t3 state
  confidence: confirmed_code

core_session_chain_step:
  step_id: tx_t3
  direction: TX
  frame_or_prefix: @t3
  handler_or_callsite:
    - bluefrog_machine_state_pump @ 0x0001be36
    - bluefrog_machine_state_pump @ 0x0001c10c
  required_flags_before:
    - branch_a: flags_88 bit24 set; bit25 gates actual send
    - branch_b: flags_88 bit tested by `flags_88 << 0x14 < 0`
  required_state_before:
    - bluefrog_state+0x88
    - branch_a writes bluefrog_state+0x8c = 0x0b
  writes_flags_88: none as a session-ready latch
  clears_flags_88:
    - branch_a clears bit24 and, if sending, bit25
    - branch_b clears the consumed line-TX bit
  writes_machine_cache: none at TX callsite
  writes_bluefrog_state:
    - branch_a writes bluefrog_state+0x8c = 0x0b
    - branch_b sets tx_in_progress_marker = 1
  enables_next_tx: none directly proven
  expected_next_rx: not statically encoded at callsite
  confidence: confirmed_code_multiple_candidate_branches

core_session_chain_step:
  step_id: tx_ty
  direction: TX
  frame_or_prefix: TY:
  handler_or_callsite: bluefrog_machine_state_pump @ 0x0001bf3a
  required_flags_before:
    - flags_88 0x01 set
  required_state_before:
    - retry_counter_a initialized to 4 if zero
    - send path requires retry_counter_a > 1
  writes_flags_88: none before TX
  clears_flags_88:
    - clears 0x04 before TX
  writes_machine_cache: none at TX callsite
  writes_bluefrog_state: retry counter state
  enables_next_tx: none directly; waits for type response
  expected_next_rx: ty:<machine type/version>
  confidence: confirmed_code

core_session_chain_step:
  step_id: rx_ty
  direction: RX
  frame_or_prefix: ty:<machine type/version>
  handler_or_callsite: ty_response_handler / startup branch, not fully isolated in this pass
  required_flags_before: unresolved
  required_state_before: valid machine type line
  writes_flags_88:
    - existing reverse map confirms interaction with 0x04 around TY phase
  clears_flags_88: unresolved
  writes_machine_cache: unresolved
  writes_bluefrog_state: unresolved
  enables_next_tx: unresolved
  expected_next_rx: unresolved
  confidence: PARTIAL

core_session_chain_step:
  step_id: tx_tr37
  direction: TX
  frame_or_prefix: @TR:37
  handler_or_callsite: bluefrog_machine_state_pump @ 0x0001c164
  required_flags_before:
    - flags_88 0x40 set
  required_state_before:
    - retry_counter_b initialized to 4 if zero
    - send path requires retry_counter_b > 1
  writes_flags_88: none before TX
  clears_flags_88:
    - not cleared in send path
    - final retry/failover path clears 0x40
  writes_machine_cache: none at TX callsite
  writes_bluefrog_state: retry counter state
  enables_next_tx: none directly; waits for gate response
  expected_next_rx: @tr:37,<hex>
  confidence: confirmed_code

core_session_chain_step:
  step_id: rx_tr37
  direction: RX
  frame_or_prefix: @tr:37,...
  handler_or_callsite:
    - machine_rx_tr37_gate_handler @ 0x0001d788
    - @tr:37 subhandler @ 0x0001d806
  required_flags_before: @TR:37 response path expected, but handler dispatch itself does not prove a flag precheck
  required_state_before: valid @tr line with subcode 0x37
  writes_flags_88:
    - clears 0x40
    - sets 0x200
  clears_flags_88:
    - 0x40
  writes_machine_cache:
    - machine_cache_base+0x6a = parsed u16[0]
    - machine_cache_base+0x6c = parsed u16[1]
    - machine_cache_base+0x6e = parsed u16[2]
    - machine_cache_base+0x70 = parsed u16[3]
  writes_bluefrog_state:
    - bluefrog_state+0x88 flags update
  enables_next_tx:
    - post-gate/session latch via flags_88 0x200; no direct next startup TX proven
  expected_next_rx: none proven
  confidence: confirmed_code
```

#### Core `flags_88` semantics

```text
flags88_core_session_bit:
  mask: 0x00000100
  known_writers:
    - machine_rx_t2_state_handler conditionally sets it when first parsed @T2 byte masked with 0x7f is zero
    - machine_rx_t3_identity_handler conditionally sets it when (bluefrog_state+0x2c & 0x7f) is nonzero
    - @T1 retry/failover path can set 0x100 together with 0x02
  known_clearers: none confirmed in the focused core-session path
  known_consumers:
    - set_cachewriter_event40_if_session_flags_allow requires 0x100 set
    - statepump/cachewriter event logic uses it as a session/cachewriter eligibility bit
  produced_by_rx:
    - @T2 and @T3, conditionally
  consumed_by_tx:
    - indirectly enables arming of 0x40 through cachewriter event helper
  likely_meaning:
    - core session data sufficient / T2/T3 handshake-completeness candidate
  relationship_to_gate_ready:
    - prerequisite for the confirmed helper that can arm @TR:37 via flags_88 0x40
  relationship_to_tf_tv_cachewriter_acceptance:
    - not required for raw parsing/copying itself, but required by set_cachewriter_event40_if_session_flags_allow() to set 0x40 after cachewriter events
  confidence: PARTIAL

flags88_core_session_bit:
  mask: 0x00000400
  known_writers:
    - machine_rx_t2_state_handler @ 0x0001ced8
  known_clearers:
    - consumed/cleared by the @t2:<hex payload> statepump branch before TX
  known_consumers:
    - bluefrog_machine_state_pump @t2:<hex payload> branch
  produced_by_rx:
    - @T2
  consumed_by_tx:
    - @t2:<hex payload>
  likely_meaning:
    - @T2 state/cache bytes available; @t2 response/control payload pending
  relationship_to_gate_ready:
    - contributes to core T2/T3 session state and may participate in 0x100 production
  relationship_to_tf_tv_cachewriter_acceptance:
    - no direct TF/TV handler gate found for 0x400
  confidence: YES

flags88_core_session_bit:
  mask: 0x00000800
  known_writers:
    - machine_rx_t3_identity_handler @ 0x0001da24
  known_clearers:
    - consumed/cleared by at least one @t3 statepump branch
  known_consumers:
    - bluefrog_machine_state_pump @t3 branch candidate
  produced_by_rx:
    - @T3
  consumed_by_tx:
    - @t3
  likely_meaning:
    - @T3 identity/version available; @t3 response/control line pending
  relationship_to_gate_ready:
    - contributes to core identity/session state and may participate in 0x100 production
  relationship_to_tf_tv_cachewriter_acceptance:
    - no direct TF/TV handler gate found for 0x800
  confidence: YES

flags88_core_session_bit:
  mask: 0x00000040
  known_writers:
    - set_cachewriter_event40_if_session_flags_allow() sets 0x40 when session/cachewriter flags allow
    - startup-specific producer before @TR:37 remains unresolved
  known_clearers:
    - @tr:37 code37 subhandler clears 0x40
    - @TR:37 retry/failover path clears 0x40 on exhaustion
  known_consumers:
    - bluefrog_machine_state_pump @TR:37 TX branch @ 0x0001c164
  produced_by_rx:
    - confirmed helper is called from @TF and @TV cachewriter handlers
    - no direct @T2/@T3 producer for 0x40 found in this focused pass
  consumed_by_tx:
    - @TR:37
  likely_meaning:
    - @TR:37 gate command pending / gate-refresh request armed
  relationship_to_gate_ready:
    - pre-gate request bit; cleared by @tr:37 response and then flags_88 0x200 is set
  relationship_to_tf_tv_cachewriter_acceptance:
    - cachewriter events can arm this bit when 0x100 is set, 0x200 is clear, 0x04 is set, and 0x40 is clear
  confidence: PARTIAL
```

#### `@TF/@TV` cachewriter acceptance relationship

This section does not identify a query or trigger for `@TF/@TV`; it only documents whether the cachewriter handlers are gated by the core-session flags.

```text
tf_tv_acceptance_gate:
  handler: machine_rx_tf_status_cache_handler @ 0x0001d8e4
  entry_condition_flags:
    - reads flags_88 at entry
    - if flags_88 bit selected by `flags_88 << 0x16 < 0` is set, calls update_status_cache_flag02_and_encode_ble_1524_value(1)
    - always then calls set_cachewriter_event40_if_session_flags_allow()
  entry_condition_state:
    - valid @TF line dispatched by machine_ascii_dispatcher
  ignored_if_not_gated: NO for parsing/cache copy; UNKNOWN for optional side effects
  cache_update_requires_flags:
    - no core-session flag gate found for parsing @TF payload into machine_cache_base
    - no core-session flag gate found for ble_characteristic_event_dispatch(0x1524)
    - no core-session flag gate found for copy_machine_cache_to_ble_value_buffer(0)
  ble_notify_requires_flags:
    - handler calls ble_characteristic_event_dispatch(0x1524) unconditionally after parsing loop completes
  relationship_to_flags_88_0x100_0x400_0x800_0x40:
    - 0x100 is used by set_cachewriter_event40_if_session_flags_allow()
    - 0x40 can be set by that helper when session flags allow
    - no direct gate found for 0x400 or 0x800 in @TF cache parsing/copying
  confidence: PARTIAL

tf_tv_acceptance_gate:
  handler: machine_rx_tv_progress_cache_handler @ 0x0001da94
  entry_condition_flags:
    - reads flags_88 at entry
    - if flags_88 bit selected by `flags_88 << 0x16 < 0` is set, calls update_status_cache_flag02_and_encode_ble_1524_value(1)
    - always then calls set_cachewriter_event40_if_session_flags_allow()
  entry_condition_state:
    - valid @TV line dispatched by machine_ascii_dispatcher
  ignored_if_not_gated: NO for normal progress-cache parsing; UNKNOWN for optional side effects
  cache_update_requires_flags:
    - no core-session flag gate found for normal @TV payload parsing into machine_cache_base+0x13..
    - no core-session flag gate found for copy_machine_cache_to_ble_value_buffer(1)
    - no core-session flag gate found for ble_characteristic_event_dispatch(0x1527)
  ble_notify_requires_flags:
    - handler calls ble_characteristic_event_dispatch(0x1527) after normal progress-cache parse
  relationship_to_flags_88_0x100_0x400_0x800_0x40:
    - 0x100 is used by set_cachewriter_event40_if_session_flags_allow()
    - 0x40 can be set by that helper when session flags allow
    - no direct gate found for 0x400 or 0x800 in @TV cache parsing/copying
  confidence: PARTIAL
```

The helper called by both cachewriters:

```text
set_cachewriter_event40_if_session_flags_allow:
  address: 0x0001a690
  callers:
    - machine_rx_tf_status_cache_handler
    - machine_rx_tv_progress_cache_handler
  condition_summary:
    - requires flags_88 0x100 set
    - requires flags_88 0x200 clear
    - requires flags_88 0x04 set
    - requires flags_88 0x40 clear
  effect:
    - flags_88 |= 0x40
  meaning:
    - cachewriter/session event can arm @TR:37 gate command when core-session state is ready but post-gate 0x200 is not set
  confidence: confirmed_code for bit tests/effect; inferred for semantic label
```

#### Minimal original core session

```text
minimal_original_core_session_resolved: PARTIAL

minimal_original_core_session:
  - step: 1
    tx: TY:
    expected_rx: ty:<machine type/version>
    required_previous_state:
      - flags_88 0x01 set; producer not part of this focused core pass
    state_after:
      - TY phase advances through a handler not fully isolated here
    confidence: PARTIAL

  - step: 2
    tx: none; machine-originated @T2 observed/handled
    expected_rx: @T2...
    required_previous_state:
      - valid decoded machine line
    state_after:
      - flags_88 0x400 set
      - possible flags_88 0x100 set
      - T2 bytes mirrored to bluefrog_state+0x2c and machine_cache_base+0x72
    confidence: confirmed_code

  - step: 3
    tx: @t2:<hex payload>
    expected_rx: not proven at callsite
    required_previous_state:
      - flags_88 0x400-style statepump branch active
      - (flags_88 & 0x1) == 0
    state_after:
      - consumed @t2 pending bit cleared
    confidence: confirmed_code

  - step: 4
    tx: none; machine-originated @T3 observed/handled
    expected_rx: @T3:<identity/version>
    required_previous_state:
      - valid decoded machine line
    state_after:
      - flags_88 0x800 set
      - possible flags_88 0x100 set
      - identity/version copied to machine_cache_base
    confidence: confirmed_code

  - step: 5
    tx: @t3
    expected_rx: not proven at callsite
    required_previous_state:
      - flags_88 0x800-style statepump branch active
    state_after:
      - consumed @t3 pending bit cleared
    confidence: confirmed_code_multiple_candidate_branches

  - step: 6
    tx: @TR:37
    expected_rx: @tr:37,<hex>
    required_previous_state:
      - flags_88 0x40 set
      - producer of 0x40 in pure startup path not fully resolved
    state_after:
      - waits for @tr:37 response
    confidence: PARTIAL

  - step: 7
    tx: none
    expected_rx: @tr:37,<hex>
    required_previous_state:
      - valid @tr response with subcode 0x37
    state_after:
      - flags_88 0x40 cleared
      - flags_88 0x200 set
      - gate response words copied to machine_cache_base+0x6a..0x70
    confidence: confirmed_code

unresolved_core_session_blockers:
  - blocker: flags88_0x40_startup_producer
    missing_relation:
      - The confirmed 0x40 writer is cachewriter/session helper set_cachewriter_event40_if_session_flags_allow().
      - A direct TY/T2/T3-to-0x40 startup writer was not found in this focused pass.
    affected_frame: @TR:37
    next_static_target:
      - non-cachewriter writers of flags_88 0x40 or aliases of bluefrog_state_flags_88

  - blocker: ty_response_handler_exact_state_effect
    missing_relation:
      - TY: TX and expected ty: RX are confirmed, but the exact ty: handler state writes remain only partially isolated.
    affected_frame: TY: -> later core state
    next_static_target:
      - machine_ascii_dispatcher branches for lowercase `ty:`
```

Notes:

```text
core_session_required_exclusions:
  @T0: not proven required for this core-session chain
  @H1/@T1: now classified as backend/timeout path, not a required linear core-session step
  @D1: not present as original BlueFrog evidence
```

### Core Session Remaining Blockers: `ty:` and `flags_88 0x40`

Scope for this pass:

```text
primary_targets:
  - ty_response_handler_exact_state_effect
  - flags88_0x40_startup_producer
out_of_scope:
  - @T0 / @H1 / @T1 / @D1
  - state70 / GPIO button-watch / backend scheduler 0x1e8b5
  - session_tick_counter_state 0x20002788
  - @TV:81 / @TV:82 / @TS:9x
  - PMode / DFU / product commands / WLAN firmware
  - ESP runtime or TX changes
```

#### `ty:` handler exact state effect

```text
ty_response_handler_exact_state_effect:
  handler_address: 0x0001d9c8
  handler_body_address: 0x0001d9d0
  rx_prefix: ty:
  parsed_fields:
    - no payload parser call recovered in this handler
    - line content is accepted by dispatcher prefix routing before entry
  required_line_format:
    - lowercase `ty:` response line from the machine
    - expected after confirmed TY: TX branch at 0x0001bf3a
  writes_flags_88:
    - mask: 0x00000001
      operation: clear
      address: 0x0001d9e6
      condition: after handler entry
      confidence: confirmed_code
    - mask: 0x00000002
      operation: clear
      address: 0x0001d9f4
      condition: after branch over previous flag test
      confidence: confirmed_code
    - mask: 0x00000004
      operation: set
      address: 0x0001d9fc
      condition: after TY pending bits are cleared
      confidence: confirmed_code
  writes_bluefrog_state:
    - offset: 0x88
      width: word
      value_or_source: flags_88 bit operations above
      condition: handler entry
  writes_machine_cache:
    - none found in this handler
  sets_events_or_timers:
    - address: 0x0001d9d6
      effect: DAT_20002798 = 0
      confidence: confirmed_code
    - address: 0x0001d9dc
      effect: DAT_20002791 = 0
      confidence: confirmed_code
    - address: 0x0001da06 / 0x0001da0e
      effect: calls arm_state_event20_clear_event10_if_allowed()
      confidence: confirmed_code
    - address: 0x0001da00..0x0001da0c
      effect: conditionally calls app_mode_set_flag08_if_state_allows() when app_mode_state_flags condition matches
      confidence: confirmed_code
  enables_next_tx:
    - indirectly enables set_cachewriter_event40_if_session_flags_allow() by setting flags_88 0x04
  disables_next_tx:
    - clears TY pending/phase bits flags_88 0x01 and 0x02
  relationship_to_0x04:
    - directly sets flags_88 0x04
  relationship_to_0x01:
    - directly clears flags_88 0x01
  relationship_to_0x100:
    - no direct read/write found in ty_response_handler
  relationship_to_0x40:
    - no direct read/write found in ty_response_handler
    - indirect only: flags_88 0x04 is one prerequisite for set_cachewriter_event40_if_session_flags_allow()
  confidence: confirmed_code
```

This resolves the `ty:` side of the gate relation: `ty:` does not itself arm `@TR:37`; it advances the TY phase by clearing `0x01/0x02` and setting `0x04`.

#### `flags_88 0x40` writer candidates

```text
flags88_0x40_writer_candidate:
  address: 0x0001a6b2
  function: set_cachewriter_event40_if_session_flags_allow
  instruction_or_decompiled_stmt: flags_88 |= 0x40
  operation: set
  direct_or_alias: direct
  condition:
    - flags_88 0x100 set
    - flags_88 0x200 clear
    - flags_88 0x04 set
    - flags_88 0x40 clear
  caller_context:
    - machine_rx_tf_status_cache_handler
    - machine_rx_tv_progress_cache_handler
  required_rx_or_event:
    - @TF or @TV cachewriter handler execution
  relationship_to_startup_TR37:
    - does not explain first startup @TR:37 unless a cachewriter event already occurred
  relationship_to_cachewriter_TR37:
    - confirmed re-arm path for @TR:37 after cachewriter/session activity
  confidence: confirmed_code

flags88_0x40_writer_candidate:
  address: 0x0001d862
  function: machine_rx_tr37_code37_subhandler
  instruction_or_decompiled_stmt: flags_88 &= ~0x40
  operation: clear
  direct_or_alias: direct
  condition:
    - valid @tr:37 response parsed
  caller_context:
    - machine_rx_tr37_gate_handler / @tr subdispatch
  required_rx_or_event:
    - @tr:37,<hex>
  relationship_to_startup_TR37:
    - consumes/completes @TR:37 phase; not a producer
  relationship_to_cachewriter_TR37:
    - clears pending gate bit after response
  confidence: confirmed_code

flags88_0x40_writer_candidate:
  address: 0x0001c1da
  function: bluefrog_machine_state_pump
  instruction_or_decompiled_stmt: clears pending @TR:37 branch bit on retry/failover path
  operation: clear
  direct_or_alias: direct
  condition:
    - @TR:37 branch retry/failover state
  caller_context:
    - statepump
  required_rx_or_event:
    - timeout/retry condition, not a machine RX line
  relationship_to_startup_TR37:
    - prevents stale @TR:37 pending state; not a producer
  relationship_to_cachewriter_TR37:
    - prevents repeated branch after failure
  confidence: confirmed_code

flags88_0x40_writer_candidate:
  address: 0x0001d9c8
  function: ty_response_handler
  instruction_or_decompiled_stmt:
    - clears 0x01 and 0x02
    - sets 0x04
    - no 0x40 write found
  operation: test/clear/set-other-bits
  direct_or_alias: direct
  condition:
    - valid ty: response
  caller_context:
    - machine_ascii_dispatcher lowercase ty: branch
  required_rx_or_event:
    - ty:<machine type/version>
  relationship_to_startup_TR37:
    - establishes 0x04 prerequisite only
  relationship_to_cachewriter_TR37:
    - required by set_cachewriter_event40_if_session_flags_allow()
  confidence: confirmed_code
```

No non-cachewriter startup producer for `flags_88 0x40` was found in this focused pass. The confirmed set path is still the `@TF/@TV` cachewriter helper, not the pure `TY:/@T2/@T3` startup chain.

```text
flags88_0x40_startup_producer_result:
  startup_producer_found: NO
  producer_address: none found
  producer_function: none found
  producer_condition: unresolved
  producer_depends_on_ty: PARTIAL
  producer_depends_on_t2: PARTIAL
  producer_depends_on_t3: PARTIAL
  producer_depends_on_tf_tv: YES for confirmed helper path
  can_explain_first_TR37: NO
  unresolved_reason:
    - ty: directly sets 0x04 but not 0x40
    - @T2/@T3 establish core identity/session bits but no direct 0x40 producer was found
    - confirmed 0x40 writer requires @TF/@TV handler execution, so it explains cachewriter-triggered @TR:37 re-arm but not first startup @TR:37
```

#### `ty:` -> `0x04/0x100/0x40` relation

```text
ty_to_tr37_gate_relation:
  ty_sets_0x04: YES
  ty_clears_0x01: YES
  ty_interacts_with_0x100: NO
  ty_can_directly_arm_0x40: NO
  ty_required_for_cachewriter_0x40_helper: YES
  can_explain_TR37_after_TY_T2_T3: NO
  confidence: confirmed_code for direct TY effects; PARTIAL for complete first-TR37 chain
```

#### Minimal core session update after `ty:` / `0x40` pass

```text
minimal_original_core_session_after_ty_0x40_pass:
  resolved: PARTIAL
  steps:
    - step: 1
      tx: TY:
      expected_rx: ty:<machine type/version>
      state_after:
        - flags_88 0x01 cleared
        - flags_88 0x02 cleared
        - flags_88 0x04 set
        - no direct flags_88 0x40 write
      confidence: confirmed_code

    - step: 2
      tx: @t2:<payload>
      expected_rx: not directly proven by this focused pass
      state_after:
        - depends on prior @T2 handler / flags_88 0x400 path from previous section
      confidence: PARTIAL

    - step: 3
      tx: @t3
      expected_rx: not directly proven by this focused pass
      state_after:
        - depends on prior @T3 handler / flags_88 0x800 path from previous section
      confidence: PARTIAL

    - step: 4
      tx: @TR:37
      expected_rx: @tr:37,<hex>
      state_after:
        - requires flags_88 0x40 before TX
        - first-startup producer for 0x40 remains unresolved
      confidence: PARTIAL

  remaining_blockers:
    - flags88_0x40_first_startup_producer
```

Notes:

```text
@T0/@H1/@T1:
  - not required for the core TY/T2/T3/TR37 chain by current static evidence

@D1:
  - in_original_ble_firmware: NO
  - do_not_use_as_original_bluefrog_evidence: YES
```

### First `@TR:37` TX Path and `flags_88 0x40` Split

Scope for this pass:

```text
primary_target:
  - first @TR:37 TX callsite at 0x0001c164
  - literal 0x000249c0
  - concrete branch condition immediately before TX
out_of_scope:
  - @T0 / @H1 / @T1 / @D1
  - state70 / GPIO button-watch / backend scheduler
  - session_tick_counter_state
  - @TV:81 / @TV:82 / @TS:9x
  - PMode / DFU / product commands / WLAN firmware
  - ESP runtime or TX changes
```

#### `@TR:37` TX callsite detail

```text
tr37_tx_callsite_detail:
  callsite: 0x0001c164
  function: bluefrog_machine_state_pump
  basic_block_or_label:
    - branch reached from 0x0001c13e..0x0001c164
    - decompiler line: else if (*(int *)(iVar2 + 0x88) << 0x19 < 0)
  literal_addr: 0x000249c0
  literal_value: @TR:37
  sender_helper: machine_uart_send_prefix_hex_line
  sender_call:
    - machine_uart_send_prefix_hex_line(0x000249c0, NULL, 0)
  required_flags_before:
    - flags_88 bit 0x40 set
    - tested as signed `(flags_88 << 0x19) < 0`
  required_state_before:
    - state_pump_tx_in_progress_marker == 0
    - state_pump_substate_char == 0
    - passthrough_or_chunk_transport_busy() == false
    - app_mode_state_machine_tx_drain() returned 0
    - earlier priority branches in the startup/identity TX block did not fire
    - local_44 == 0, i.e. flags_88 0x01 clear
  required_cache_before:
    - none for the @TR:37 call itself
  condition_chain:
    - RX/read/dispatcher portion of statepump completes first
    - machine_passthrough_fifo_drain() must not consume TX slot
    - app_mode_state_machine_tx_drain() must not consume TX slot
    - state70 @H1/@T1 branch skipped
    - flags_88 0x800-style prefix-hex branch skipped
    - flags_88 0x1000-style line branch skipped
    - progress_control_flags_block_state_pump+4 bit3 retry branch skipped
    - flags_88 0x10 retry branch skipped
    - flags_88 0x20 retry branch skipped
    - flags_88 0x40 branch taken
  clears_or_sets_after_tx:
    - if retry_counter_b >= 2:
      - retry_counter_b decremented
      - flags_88 0x40 remains set
      - @TR:37 is sent
    - if retry_counter_b < 2:
      - retry_counter_b cleared
      - flags_88 0x40 cleared at 0x0001c1da
      - no @TR:37 TX in that final-fail path
  retry_counter_used:
    - state_pump_retry_counter_b
    - initialized to 4 if zero
    - decremented before each @TR:37 send
  expected_rx: @tr:37,<hex>
  confidence: confirmed_code
```

The callsite itself does not distinguish a first-startup `@TR:37` from a later re-arm. It only checks the pending state bit and the shared TX guards above.

#### Branch backtrace

```text
tr37_branch_backtrace:
  branch_address: 0x0001c13e..0x0001c142
  condition: signed `(flags_88 << 0x19) < 0`
  tested_register:
    - r3/r0 in the assembly block
  tested_memory:
    - bluefrog_state+0x88
  tested_mask_or_value:
    - flags_88 0x00000040
  source_of_tested_value:
    - load from bluefrog_state_ptr_literal_state_pump + 0x88
  previous_basic_block:
    - shared startup/identity TX priority chain after tx-in-progress/substate/busy guards
  producer_candidates:
    - confirmed: set_cachewriter_event40_if_session_flags_allow() at 0x0001a690
    - unresolved: no separate non-cachewriter startup producer found for first @TR:37
  confidence: confirmed_code

tr37_branch_backtrace:
  branch_address: 0x0001c150..0x0001c164
  condition:
    - retry_counter_b > 1 sends @TR:37
    - retry_counter_b <= 1 clears flags_88 0x40 and does not send
  tested_register:
    - r2 loaded from state_pump_retry_counter_b
  tested_memory:
    - state_pump_retry_counter_b
  tested_mask_or_value:
    - byte counter threshold 2
  source_of_tested_value:
    - state_pump_retry_counter_b, initialized to 4 if zero at 0x0001c146..0x0001c14e
  previous_basic_block:
    - flags_88 0x40 branch
  producer_candidates:
    - retry counter is local retry pacing, not the original producer of the @TR:37 state
  confidence: confirmed_code
```

The `@TR:37` branch is therefore a two-part gate: `flags_88 0x40` arms the branch; `retry_counter_b` decides whether the current pump iteration sends or gives up and clears `0x40`.

#### Producer of the tested state

```text
tr37_startup_state_producer:
  tested_state: flags_88_0x40
  producer_found: PARTIAL
  producer_address: 0x0001a6b2
  producer_function: set_cachewriter_event40_if_session_flags_allow
  producer_condition:
    - flags_88 0x100 set
    - flags_88 0x200 clear
    - flags_88 0x04 set
    - flags_88 0x40 clear
  depends_on_ty: YES for 0x04 prerequisite
  depends_on_T2: PARTIAL via 0x100/session-ready path from previous core-session analysis
  depends_on_T3: PARTIAL via 0x100/session-ready path from previous core-session analysis
  depends_on_t3: UNKNOWN
  depends_on_cachewriter_TF_TV: YES for the confirmed producer callsites
  can_explain_first_TR37: NO
  confidence: confirmed_code for producer and callsites; PARTIAL for first-startup relation
```

The confirmed producer is called only from:

```text
set_cachewriter_event40_if_session_flags_allow_callers:
  - machine_rx_tf_status_cache_handler @ 0x0001d8fe
  - machine_rx_tv_progress_cache_handler @ 0x0001daac
```

No additional direct reference to the `@TR:37` literal was found:

```text
tr37_literal_refs:
  literal_addr: 0x000249c0
  refs:
    - 0x0001c160 PARAM into machine_uart_send_prefix_hex_line
  other_refs_found: NO
```

#### Path split assessment

```text
tr37_path_split:
  first_startup_path_found: PARTIAL
  first_startup_callsite: 0x0001c164
  first_startup_required_state:
    - same visible state as cachewriter path: flags_88 0x40 set
    - no separate first-startup-only callsite found
  cachewriter_rearm_path_found: YES
  cachewriter_rearm_required_state:
    - set_cachewriter_event40_if_session_flags_allow() must set flags_88 0x40
    - helper is called from @TF/@TV RX handlers
  same_callsite: YES
  same_flag: YES
  confidence: confirmed_code for same callsite/flag; PARTIAL for first-startup producer
```

Interpretation:

```text
tr37_path_split_interpretation:
  - There is one visible @TR:37 TX callsite in bluefrog_machine_state_pump.
  - The callsite is shared for any path that sets flags_88 0x40.
  - The only confirmed 0x40 producer remains the @TF/@TV cachewriter helper.
  - A distinct first-startup 0x40 producer was not found in this focused pass.
  - Therefore first startup @TR:37 is not statically explained yet, even though the TX branch itself is fully mapped.
```

#### Minimal core session update after `@TR:37` callsite pass

```text
minimal_original_core_session_after_tr37_callsite_pass:
  resolved: PARTIAL
  steps:
    - step: 1
      tx: TY:
      expected_rx: ty:<machine type/version>
      required_previous_state:
        - flags_88 0x01 set
      state_after:
        - flags_88 0x01 cleared
        - flags_88 0x02 cleared
        - flags_88 0x04 set
      confidence: confirmed_code

    - step: 2
      tx: @t2:<payload>
      expected_rx: not resolved by this pass
      required_previous_state:
        - @T2-derived state / flags_88 0x400 path from previous core-session section
      state_after:
        - @t2 branch consumed by statepump
      confidence: PARTIAL

    - step: 3
      tx: @t3
      expected_rx: not resolved by this pass
      required_previous_state:
        - @T3-derived state / flags_88 0x800 path from previous core-session section
      state_after:
        - @t3 branch consumed by statepump
      confidence: PARTIAL

    - step: 4
      tx: @TR:37
      expected_rx: @tr:37,<hex>
      required_previous_state:
        - flags_88 0x40 set
        - retry_counter_b > 1
        - shared TX guards clear
      state_after:
        - if sent: retry_counter_b decremented, flags_88 0x40 remains pending until response or retry fail
        - if @tr:37 response received: @tr:37 handler clears 0x40 and sets post-gate 0x200
      confidence: confirmed_code for TX branch; PARTIAL for first-startup state producer

  remaining_blockers:
    - flags88_0x40_first_startup_producer
```

Notes:

```text
@T0/@H1/@T1:
  - not required for the core TY/T2/T3/TR37 chain by current static evidence

@D1:
  - in_original_ble_firmware: NO
  - do_not_use_as_original_bluefrog_evidence: YES
```

### Indirect table around `0x25058` -> `maybe_advance_t0_state70_substate`

Scope for this pass:

```text
primary_target: table/descriptor owner for raw Thumb pointer 0x00016295 at 0x00025058
out_of_scope:
  - @TF/@TV
  - @TV:81/@TV:82
  - @TS:9x
  - PMode / DFU / product commands
  - App-mode @mn/@mo/@me/@me1
  - WLAN firmware
  - ESP runtime changes
```

#### Raw dump `0x00025020..0x00025090`

```text
table_25058_raw_dump:
  addr: 0x00025020
  u32_le: 0x36353433
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "3456"
  comment: printf/hex-format literal tail before .data init image

  addr: 0x00025024
  u32_le: 0x41393837
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "789A"
  comment: printf/hex-format literal tail

  addr: 0x00025028
  u32_le: 0x45444342
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "BCDE"
  comment: printf/hex-format literal tail

  addr: 0x0002502c
  u32_le: 0x31300046
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "F\\x0001"
  comment: string terminator and next small literal bytes

  addr: 0x00025030
  u32_le: 0x35343332
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "2345"
  comment: second hex literal

  addr: 0x00025034
  u32_le: 0x39383736
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "6789"
  comment: second hex literal

  addr: 0x00025038
  u32_le: 0x64636261
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: "abcd"
  comment: second hex literal

  addr: 0x0002503c
  u32_le: 0x00006665
  possible_pointer: accidental low flash-looking value
  possible_thumb_func: NO
  ascii_if_any: "ef\\x00\\x00"
  comment: second hex literal terminator, not a callback entry

  addr: 0x00025040
  u32_le: 0x00000000
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: guard/one-shot value referenced by 0x000160c0 helper, before .data copy source

  addr: 0x00025044
  u32_le: 0x7fff10dc
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: unrelated constant before .data copy source

  addr: 0x00025048
  u32_le: 0x00000001
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: unrelated scalar before .data copy source

  addr: 0x0002504c
  u32_le: 0x00000000
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: start of .data init image copied to RAM 0x20002000

  addr: 0x00025050
  u32_le: 0x0000ffff
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM copy at 0x20002004, adjacent context field

  addr: 0x00025054
  u32_le: 0x00000000
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM copy at 0x20002008; first 4 bytes of gpio_button_entry[0]

  addr: 0x00025058
  u32_le: 0x00016295
  possible_pointer: YES
  possible_thumb_func: 0x00016294
  ascii_if_any: ""
  comment: RAM copy at 0x2000200c; gpio_button_entry[0].callback = maybe_advance_t0_state70_substate

  addr: 0x0002505c
  u32_le: 0x00000001
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM copy at 0x20002010, following context/DFU-adjacent data; not part of 8-byte button entry

  addr: 0x00025060
  u32_le: 0x000000ab
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: scalar in .data init image

  addr: 0x00025064
  u32_le: 0x0000001b
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: scalar in .data init image

  addr: 0x00025068
  u32_le: 0x20002ae4
  possible_pointer: RAM
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM pointer in .data init image

  addr: 0x0002506c
  u32_le: 0x20002028
  possible_pointer: RAM
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM pointer in .data init image

  addr: 0x00025070
  u32_le: 0x2000203c
  possible_pointer: RAM
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM pointer in .data init image

  addr: 0x00025074
  u32_le: 0x00011523
  possible_pointer: YES
  possible_thumb_func: 0x00011522
  ascii_if_any: ""
  comment: separate Thumb pointer in later .data region, no direct relation to 0x16294 found in this pass

  addr: 0x00025078
  u32_le: 0x00024614
  possible_pointer: FLASH
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: flash pointer in later .data region

  addr: 0x0002507c
  u32_le: 0x0000000e
  possible_pointer: scalar
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: scalar in later .data region

  addr: 0x00025080
  u32_le: 0x20002564
  possible_pointer: RAM
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: RAM pointer in later .data region

  addr: 0x00025084
  u32_le: 0x00000000
  possible_pointer: NO
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: zero field in later .data region

  addr: 0x00025088
  u32_le: 0x00011623
  possible_pointer: YES
  possible_thumb_func: 0x00011622
  ascii_if_any: ""
  comment: separate Thumb pointer in later .data region, no direct relation to 0x16294 found in this pass

  addr: 0x0002508c
  u32_le: 0x000248b0
  possible_pointer: FLASH
  possible_thumb_func: NO
  ascii_if_any: ""
  comment: flash pointer in later .data region
```

#### Layout / owner result

The bytes around `0x00025058` are not a directly consumed Flash callback table. They are part of the C runtime `.data` init image.

```text
table_25058_layout_guess:
  table_start: 0x0002504c
  table_end: 0x00025144
  entry_size: not a homogeneous table; block copy image
  entry_count: not applicable for full block
  entry_containing_0x25058:
    flash_source: 0x00025054..0x0002505b
    ram_destination: 0x20002008..0x2000200f
    interpreted_as: gpio_button_entry[0]
    entry_size: 8
    fields:
      +0x00: pin/index byte = 0
      +0x01: polarity/edge mode byte = 0
      +0x02: pin config byte = 0
      +0x03: reserved/padding byte = 0
      +0x04: callback Thumb pointer = 0x00016295 -> maybe_advance_t0_state70_substate
  previous_entry: none in this one-entry button array
  next_entry: none; 0x0002505c maps to the next RAM context field at 0x20002010
  confidence: confirmed_code for .data copy and gpio_button_watch_init consumer

layout_evidence:
  - reset/init function 0x00020798 copies from 0x0002504c to 0x20002000 up to 0x200020f8.
  - main_loop_or_scheduler callsite 0x00016460 passes 0x20002008 as button_entries to gpio_button_watch_init().
  - gpio_button_watch_init treats button_entries as 8-byte entries and reads callback from entry + 4.
  - direct code XREFs to flash address 0x00025058 are absent because code consumes the RAM copy at 0x2000200c.
```

#### Consumer candidates

```text
table_25058_consumer_candidate:
  candidate_function: reset/.data init copier at 0x00020798
  callsite_or_xref: 0x000207aa..0x000207b8
  evidence:
    - DAT_000207cc = 0x0002504c source
    - DAT_000207d0 = 0x20002000 destination
    - DAT_000207d4 = 0x200020f8 end
    - loop copies words from flash source to RAM destination
  reads_table_start: YES, as block-copy source
  reads_entry_size: NO, copies whole .data block backward by words
  indirect_call_instruction: none
  passes_param_1: none
  passes_param_2: none
  relationship_to_0x16294: materializes the Thumb pointer into RAM at 0x2000200c
  confidence: confirmed_code

table_25058_consumer_candidate:
  candidate_function: main_loop_or_scheduler
  callsite_or_xref: 0x00016460 -> gpio_button_watch_init((byte *)(gpio_button_watch_context + 4), 1, 0x32, 0)
  evidence:
    - xref to DAT_20002008 at 0x00016460
    - button_entries pointer equals RAM copy of flash source 0x00025054
  reads_table_start: YES, via RAM copy 0x20002008
  reads_entry_size: passes entry_count=1; entry size inferred by callee as 8 bytes
  indirect_call_instruction: none at callsite
  passes_param_1: button_entries=0x20002008
  passes_param_2: entry_count=1
  relationship_to_0x16294: registers the entry containing callback pointer 0x00016295
  confidence: confirmed_code

table_25058_consumer_candidate:
  candidate_function: gpio_button_watch_init
  callsite_or_xref: 0x0001f34c..0x0001f376
  evidence:
    - loops entries as button_entries + index * 8
    - reads entry[0] as pin/index
    - reads entry[2] as GPIO config bits
    - calls gpio_watch_allocate(..., gpio_button_watch_default_callback)
    - installs gpio_button_watch_dispatch as wrapper callback after successful allocation
  reads_table_start: YES, RAM copy 0x20002008
  reads_entry_size: 8 bytes
  indirect_call_instruction: none in init
  passes_param_1: out_watch_index and pin mask to gpio_watch_allocate
  passes_param_2: not applicable
  relationship_to_0x16294: stores the button entry pointer in gpio_button_watch_state; callback remains entry[+4]
  confidence: confirmed_code

table_25058_consumer_candidate:
  candidate_function: gpio_button_watch_dispatch
  callsite_or_xref: 0x0001f214..0x0001f319
  evidence:
    - reads entries from gpio_button_watch_state[0x10] with stride 8
    - loads pcVar7 = *(code **)(entry + 4)
    - invokes either pcVar7(pin, logical_state) or wrapper(pcVar7, pin, logical_state)
  reads_table_start: YES, RAM copy pointer retained in gpio_button_watch_state
  reads_entry_size: 8 bytes
  indirect_call_instruction:
    - 0x0001f2a8 / 0x0001f2ec: wrapper callback with pcVar7, pin, logical_state
    - 0x0001f2dc / 0x0001f300 / 0x0001f308 / 0x0001f314: direct pcVar7(pin, logical_state) when wrapper absent
  passes_param_1: pin/index from entry[0], initialized to 0
  passes_param_2: logical edge/state 0 or 1 after GPIO debounce/polarity handling
  relationship_to_0x16294: this is the indirect caller path for maybe_advance_t0_state70_substate
  confidence: confirmed_code

table_25058_consumer_candidate:
  candidate_function: gpio_button_watch_default_callback at 0x0001f1f8
  callsite_or_xref: registered by gpio_watch_allocate at 0x0001f374
  evidence:
    - called by lower gpio_watch layer
    - records callback/event data and arms gpio_button_watch_dispatch through state+0x10
  reads_table_start: NO
  reads_entry_size: NO
  indirect_call_instruction: none
  passes_param_1: records callback/event parameters
  passes_param_2: records callback/event parameters
  relationship_to_0x16294: schedules the higher-level dispatch that later invokes the entry callback
  confidence: confirmed_code / inferred role
```

#### Callback ABI for `0x00016294`

```text
callback_0x16294_indirect_abi:
  param_1_source:
    - gpio_button_watch_dispatch loads entry[0] as pin/index.
    - For the initialized entry copied from 0x00025054, entry[0] = 0.
  param_2_source:
    - logical edge/state value produced by gpio_button_watch_dispatch after comparing GPIO watch masks and entry[1] polarity/mode.
    - For the initialized entry, entry[1] = 0, so both logical states 0 and 1 can be delivered on opposite observed transitions.
  return_value_used:
    - If wrapper callback is active, gpio_button_watch_dispatch checks wrapper return and asserts on nonzero.
    - For direct callback path, return value is ignored.
    - maybe_advance_t0_state70_substate returns param_1 / boolean-ish status, but no protocol TX is emitted directly from the callback.
  caller_condition:
    - main_loop_or_scheduler initializes GPIO watch table and button watch.
    - gpio_watch detects a masked GPIO state transition.
    - gpio_button_watch_dispatch debounces/maps the transition and invokes entry callback.
  expected_state70_before:
    - state70 bit0 set for the substate-advance path.
    - current substate bits3/4 must be 1 for param_2=1 to advance to 2.
    - current substate bits3/4 must be 2 for param_2=0 to advance to 3.
  expected_state70_after:
    - param_2=1 with current substate 1 writes substate 2.
    - param_2=0 with current substate 2 writes substate 3.
    - substate 3 satisfies the @T0 branch in bluefrog_machine_state_pump.
  can_advance_to_substate_3: YES
  relationship_to_@T0: YES
  confidence: confirmed_code for ABI and state writes; inferred_hardware for exact physical GPIO/button meaning
```

#### Consequence for `@T0`

```text
t0_substate_caller_result:
  caller_found: YES
  caller_count: indirect path, no direct code XREF
  callers:
    - callsite: 0x0001f2a8 / 0x0001f2ec via wrapper or 0x0001f2dc / 0x0001f300 / 0x0001f308 / 0x0001f314 direct
      function: gpio_button_watch_dispatch
      param_2: logical edge/state 0 or 1
      effect: invokes maybe_advance_t0_state70_substate(pin_index, logical_state)
  can_explain_T0_substate_3: YES
  unresolved_reason:
    - direct XREF remains absent because the pointer is initialized through .data copy and consumed through RAM entry 0x2000200c.
    - physical meaning of GPIO pin/index 0 is not named here; it is not required to prove the indirect caller ABI.
```

Partial `@T0` chain after this pass:

```text
partial_T0_substate_chain:
  - prerequisite: state70 bit0 set
  - prerequisite: substate bits3/4 initially set to 1, confirmed from @HF parsed byte 0x04 path
  - event: GPIO/button logical state 1
  - callback: maybe_advance_t0_state70_substate(0, 1)
  - state_change: substate 1 -> 2
  - event: GPIO/button logical state 0
  - callback: maybe_advance_t0_state70_substate(0, 0)
  - state_change: substate 2 -> 3
  - consumer: bluefrog_machine_state_pump @T0 branch at 0x0001befe
  - tx: @T0
  - confidence: confirmed_code for callback chain; inferred_hardware for exact GPIO stimulus
```

Remaining blocker status:

```text
remaining_static_blockers_after_this_run:
  - blocker: session_tick_counter_state+4_seed_for_@H1/@T1
    why_unresolved: unchanged; no writer outside maybe_session_tick_counter_and_timeout_pump was found in the focused seed pass.
    next_static_target: BSS/init-copy aliasing or non-direct writes to 0x2000278c.
```

`@D1` remains excluded:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
```

### Focused Remaining State70 Blockers

Scope of this pass:

```text
primary_targets:
  - session_tick_counter_state+4_seed_for_@H1/@T1
  - caller_for_maybe_advance_t0_state70_substate
explicitly_out_of_scope:
  - flags88_0x40_startup_producer_for_@TR37
  - flags88_0x40000_direct_consumer
  - @TF/@TV
  - TV81/TV82
  - @TS:9x
  - PMode / DFU / product commands
  - app-mode @mn/@mo/@me/@me1
  - WLAN firmware
```

#### `session_tick_counter_state+4` XREF table

`session_tick_counter_state` is a code literal at `0x0001e888` with value `0x20002788`.
The relevant H1/T1 countdown halfword is `0x2000278c`.

```text
session_tick_seed_xref:
  address: 0x0001e828
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: if (*session_tick_counter_state < 0x32)
  access: read
  width: word
  target_offset: +0
  value_or_source: 0x20002788 tick accumulator
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: function entered
  relationship_to_state70_bit5_bit6: cadence accumulator only; does not directly arm @H1/@T1
  confidence: confirmed_code

session_tick_seed_xref:
  address: 0x0001e830
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: *session_tick_counter_state = *session_tick_counter_state + 1
  access: increment
  width: word
  target_offset: +0
  value_or_source: previous tick accumulator + 1
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: accumulator < 0x32
  relationship_to_state70_bit5_bit6: cadence accumulator only
  confidence: confirmed_code

session_tick_seed_xref:
  address: 0x0001e86c
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: *session_tick_counter_state = 0; decrement_service_channel_countdowns()
  access: clear
  width: word
  target_offset: +0
  value_or_source: zero
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: accumulator >= 0x32
  relationship_to_state70_bit5_bit6: resets cadence and ticks service countdowns
  confidence: confirmed_code

session_tick_seed_xref:
  address: 0x0001e83c
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: ((short)session_tick_counter_state[1] != 0)
  access: read
  width: halfword
  target_offset: +4
  value_or_source: 0x2000278c countdown
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: state70 bit5 is clear
  relationship_to_state70_bit5_bit6: nonzero value is mandatory before bit5/bit6 can be produced
  confidence: confirmed_code

session_tick_seed_xref:
  address: 0x0001e84c
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: sVar4 = (short)session_tick_counter_state[1] + -1
  access: read/decrement
  width: halfword
  target_offset: +4
  value_or_source: 0x2000278c countdown - 1
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: state70 bit5 clear, countdown nonzero, service_channel_any_countdown_active() false
  relationship_to_state70_bit5_bit6: when decrement reaches zero, produces state70 bit5 and clears bit6, enabling @H1
  confidence: confirmed_code

session_tick_seed_xref:
  address: 0x0001e852
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: session_tick_counter_state[1] = sVar4
  access: write
  width: halfword
  target_offset: +4
  value_or_source: decremented countdown value
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: service_channel_any_countdown_active() false
  relationship_to_state70_bit5_bit6: if written value is zero, same basic block sets bit5 and clears bit6 for @H1
  confidence: confirmed_code

session_tick_seed_xref:
  address: 0x0001e874
  function: maybe_session_tick_counter_and_timeout_pump
  instruction_or_decompiled_stmt: session_tick_counter_state[1] = 0
  access: clear
  width: halfword
  target_offset: +4
  value_or_source: zero
  caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  required_condition: state70 bit5 clear, countdown nonzero, service_channel_any_countdown_active() true
  relationship_to_state70_bit5_bit6: same basic block sets bit5 and bit6 for @T1
  confidence: confirmed_code
```

Negative XREF/search results:

```text
session_tick_counter_state_external_seed_search:
  searched_addresses:
    - 0x20002788
    - 0x2000278a
    - 0x2000278c
    - 0x2000278e
  direct_xrefs:
    - 0x20002788: only maybe_session_tick_counter_and_timeout_pump
    - 0x2000278a: none
    - 0x2000278c: only maybe_session_tick_counter_and_timeout_pump
    - 0x2000278e: none
  literal_pointer_scan:
    - raw 0x20002788 pointer occurs at 0x0001e888 and 0x0001e8b0 only
    - raw 0x2000278c pointer occurs nowhere
  block_init_or_copy:
    - no memset/copy-style decompiler hit found that clearly covers 0x20002788..0x20002790
  conclusion: external seed writer for 0x2000278c was not found
```

```text
session_tick_counter_state_seed_result:
  seed_writer_found: NO
  seed_writer_address: none
  seed_writer_function: none
  seed_value_or_source: unresolved
  condition: unresolved
  can_explain_H1_T1_timing: PARTIAL
  unresolved_reason:
    - The consumer/producer logic is fully visible: nonzero 0x2000278c gates production of state70 bit5/bit6.
    - No external writer or initializer for 0x2000278c was found via direct XREF, decompiler search, literal pointer scan, or adjacent-address XREFs.
    - The value may be seeded through an unrecognized block initialization, aliasing, startup data copy, or an indirect runtime path not represented as a direct data XREF.
```

#### Caller search for `maybe_advance_t0_state70_substate`

Known helper:

```text
maybe_advance_t0_state70_substate:
  function_address: 0x00016294
  thumb_address: 0x00016295
  behavior:
    - if state70 bit0 set and param_2 == 1 and current substate == 1: substate becomes 2
    - if state70 bit0 set and param_2 == 0 and current substate == 2: substate becomes 3
    - substate write: state70 = (state70 & 0xffffffe7) | ((uVar3 & uVar4) << 3)
```

Direct XREF result:

```text
t0_substate_advance_caller:
  callsite: none found as direct code XREF
  caller_function: none
  param_1_source: unresolved
  param_2_value_or_source: unresolved
  required_condition: unresolved
  prior_rx_or_event: unresolved
  state70_before: substate 1 or 2 required by helper
  state70_after:
    - substate 1 -> 2 if param_2 == 1
    - substate 2 -> 3 if param_2 == 0
  advances:
    - from_substate: 1
      to_substate: 2
    - from_substate: 2
      to_substate: 3
  relationship_to_T0: reaches the required substate 3 for @T0, but caller/event remains unresolved
  confidence: confirmed_code for helper behavior; unknown for direct caller
```

Raw pointer result:

```text
t0_substate_advance_caller:
  callsite: raw data pointer at 0x00025058
  caller_function: unresolved indirect descriptor/table user
  raw_entry: 95 62 01 00
  target: 0x00016295 Thumb -> 0x00016294
  surrounding_data:
    - 0x00025058: callback pointer 0x00016295
    - following words include 0x00000001, 0x000000ab, 0x0000001b, 0x20002ae4, 0x20002028, 0x2000203c
  param_1_source: unresolved table/user callback ABI
  param_2_value_or_source: unresolved table/user callback ABI
  required_condition: unresolved
  prior_rx_or_event: unresolved
  state70_before: helper still requires state70 bit0 set and substate 1/2
  state70_after:
    - potentially 1 -> 2 or 2 -> 3, depending callback argument param_2
  advances:
    - from_substate: 1
      to_substate: 2
    - from_substate: 2
      to_substate: 3
  relationship_to_T0: likely intended indirect/timer callback path for @T0 substate progression, but not enough to resolve order
  confidence: PARTIAL
```

Negative caller checks:

```text
t0_substate_advance_negative_checks:
  direct_xref_to_0x00016294: none
  direct_xref_to_thumb_0x00016295: none
  raw_pointer_scan:
    - 0x00016295 occurs once at 0x00025058
    - 0x00016294 occurs zero times as a raw word
  pointer_to_descriptor_scan:
    - no raw pointer to 0x00025058 / 0x00025020 / 0x00025044 found in the local firmware byte scan
  rx_handler_callers:
    - no direct RX-handler call to 0x00016294 found
  scheduler_tick_callers:
    - no direct scheduler/tick call to 0x00016294 found
  host_handler_callers:
    - no direct @HF/@HP/@HT call to 0x00016294 found; @HF only seeds substate 1 directly
```

```text
t0_substate_caller_result:
  caller_found: PARTIAL
  caller_count: 1 raw function-pointer entry, 0 direct code callers
  callers:
    - callsite: data entry 0x00025058
      function: unresolved indirect descriptor/table user
      param_2: unresolved
      effect: points to maybe_advance_t0_state70_substate, which can advance 1->2 or 2->3
  can_explain_T0_substate_3: PARTIAL
  unresolved_reason:
    - The helper and raw Thumb function pointer are confirmed.
    - No direct caller or referenced descriptor/table owner was found.
    - Without the indirect callback ABI, param_2 cannot be tied to the 1->2 or 2->3 transitions.
```

#### Partial sequence result for `@T0/@H1/@T1`

```text
partial_sequence_T0_H1_T1_resolved: NO

remaining_static_blockers_after_this_run:
  - blocker: session_tick_counter_state+4_seed_for_@H1/@T1
    why_unresolved: 0x2000278c is consumed and modified only inside maybe_session_tick_counter_and_timeout_pump; no external seed writer was found.
    next_static_target: investigate startup/BSS/init-copy tables and aliasing around 0x20002788; inspect any runtime allocator/zero-init path that may write RAM without direct data XREF.

  - blocker: indirect_table_owner_for_0x25058_callback_to_0x16294
    why_unresolved: raw Thumb pointer 0x00016295 is present at 0x00025058, but no XREF to that table/descriptor was found, so the callback caller and param_2 source remain unknown.
    next_static_target: identify the table structure around 0x00025044..0x00025080 and its owner; inspect startup descriptor scans or linker-section iteration code that may consume unreferenced tables.
```

`@D1` remains excluded:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
```

### Focused Producer Chain: state70 -> @T0/@H1/@T1

Scope of this focused pass:

```text
PRIMARY_TARGET: session_tick_counter_state_seed_for_state70_bit5_bit6_@H1_@T1
SECONDARY_TARGET: state70_bits3_4_substate_for_@T0
out_of_scope_for_this_pass:
  - flags88_0x40_startup_producer_for_@TR37
  - flags88_0x40000_direct_consumer
  - @TF/@TV
  - TV81/TV82
  - PMode / DFU / product paths
```

#### XREF details for `bluefrog_state+0x70` bits 0/3/4/5/6

```text
state70_xref_detail:
  address: 0x00016822..0x00016828
  function: main_loop_or_scheduler
  basic_block_or_label: LAB_00016822
  instruction_or_decompiled_stmt: if (*bluefrog_device_id_word0_cache == 2) state70 |= 0x00000001
  bit_or_mask: bit0 / 0x00000001
  operation: set
  direct_caller: reset/start path into main_loop_or_scheduler
  caller_chain_if_known: start -> main_loop_or_scheduler
  required_state_before: FICR/device-id word copied into bluefrog_device_id_word0_cache
  required_rx_before: none
  required_timer_or_counter: none
  related_global_or_state_offset: bluefrog_state+0x70 at 0x2000298c
  enables_tx_candidate: @T0 if bits3/4 substate later becomes 3
  disables_tx_candidate: none
  confidence: confirmed_code

state70_xref_detail:
  address: 0x0001d638..0x0001d642
  function: host_reply_hf_state70_control_handler (handler body at 0x0001d0a4; @HF table entry)
  basic_block_or_label: @HF payload bVar2 == 0x04 branch
  instruction_or_decompiled_stmt: state70 = (state70 & 0xffffffe7) | 0x00000008
  bit_or_mask: bits3/4 mask 0x00000018, value 0x00000008
  operation: assign substate bits to 1
  direct_caller: host_subdispatcher table entry for @HF
  caller_chain_if_known: machine_ascii_dispatcher -> host_subdispatcher -> @HF handler
  required_state_before: (*DAT_0001d408 == 0) and (state70 bit0 set or high nibble of parsed byte is 0x20)
  required_rx_before: @HF with parsed byte 0x04
  required_timer_or_counter: none
  related_global_or_state_offset: bluefrog_state+0x70 at 0x2000298c
  enables_tx_candidate: not @T0 yet; prepares substate 1
  disables_tx_candidate: previous @T0 substate value
  confidence: confirmed_code

state70_xref_detail:
  address: 0x00016294..0x00016300
  function: maybe_advance_t0_state70_substate
  basic_block_or_label: state70 bit0 set, param_2-controlled state advance
  instruction_or_decompiled_stmt: state70 = (state70 & 0xffffffe7) | ((uVar3 & uVar4) << 3)
  bit_or_mask: bits3/4 mask 0x00000018
  operation: assign/advance substate
  direct_caller: unresolved; no direct XREF to 0x00016294/0x00016295 was emitted by Ghidra
  caller_chain_if_known: likely mainloop/jumptable-adjacent helper; not statically resolved
  required_state_before:
    - state70 bit0 set
    - for param_2 == 1: current substate must be 1; writes substate 2
    - for param_2 == 0: current substate must be 2; writes substate 3
  required_rx_before: unresolved
  required_timer_or_counter: none inside helper
  related_global_or_state_offset: bluefrog_state+0x70 at 0x2000298c
  enables_tx_candidate: @T0 once substate reaches 3
  disables_tx_candidate: none
  confidence: confirmed_code for state transition; unknown for caller/event source

state70_xref_detail:
  address: 0x0001bdb2..0x0001befe
  function: bluefrog_machine_state_pump
  basic_block_or_label: first startup state70 TX branch
  instruction_or_decompiled_stmt: if ((state70 bit0 set) && (((state70 & 0x1f) >> 3) == 3)) { state70 &= 0xffffffe7; send @T0; }
  bit_or_mask: tests bit0 and bits3/4 substate; clears 0x00000019/0x00000018 region with 0xffffffe7
  operation: test/clear
  direct_caller: main_loop_or_scheduler
  caller_chain_if_known: main_loop_or_scheduler -> bluefrog_machine_state_pump
  required_state_before: state70 bit0 set and substate bits3/4 == 3
  required_rx_before: unresolved; substate 3 source is only partially mapped
  required_timer_or_counter: none at TX callsite
  related_global_or_state_offset: bluefrog_state+0x70 at 0x2000298c
  enables_tx_candidate: @T0
  disables_tx_candidate: re-sending @T0 until state70 is rearmed
  confidence: confirmed_code

state70_xref_detail:
  address: 0x0001e858..0x0001e866
  function: maybe_session_tick_counter_and_timeout_pump
  basic_block_or_label: countdown reaches zero without active service countdown
  instruction_or_decompiled_stmt: state70 |= 0x20; state70 &= 0xffffffbf
  bit_or_mask: bit5 set, bit6 clear
  operation: set/clear
  direct_caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  caller_chain_if_known: main_loop_or_scheduler timer path -> 0x0001e8b4 -> maybe_session_tick_counter_and_timeout_pump
  required_state_before:
    - state70 bit5 clear
    - session_tick_counter_state+4 halfword nonzero
    - service_channel_any_countdown_active() returns false
    - decrement makes session_tick_counter_state+4 reach zero
  required_rx_before: none directly proven
  required_timer_or_counter: session_tick_counter_state+4 halfword countdown
  related_global_or_state_offset:
    - session_tick_counter_state literal 0x0001e888 -> RAM 0x20002788
    - consumed halfword at 0x2000278c
  enables_tx_candidate: @H1
  disables_tx_candidate: @T1 selector bit
  confidence: confirmed_code for producer; seed source unresolved

state70_xref_detail:
  address: 0x0001e874..0x0001e884
  function: maybe_session_tick_counter_and_timeout_pump
  basic_block_or_label: service countdown active
  instruction_or_decompiled_stmt: session_tick_counter_state+4 = 0; state70 |= 0x20; state70 |= 0x40
  bit_or_mask: bit5 set, bit6 set
  operation: set
  direct_caller: timer/tick function body at 0x0001e8b4, callsite 0x0001e94a
  caller_chain_if_known: main_loop_or_scheduler timer path -> 0x0001e8b4 -> maybe_session_tick_counter_and_timeout_pump
  required_state_before:
    - state70 bit5 clear
    - session_tick_counter_state+4 halfword nonzero
    - service_channel_any_countdown_active() returns true
  required_rx_before: none directly proven
  required_timer_or_counter: session_tick_counter_state+4 halfword nonzero; service countdown active
  related_global_or_state_offset:
    - session_tick_counter_state literal 0x0001e888 -> RAM 0x20002788
    - consumed halfword at 0x2000278c
  enables_tx_candidate: @T1
  disables_tx_candidate: @H1 selector path
  confidence: confirmed_code for producer; seed source unresolved

state70_xref_detail:
  address: 0x0001be06..0x0001be26 / 0x0001bfd4
  function: bluefrog_machine_state_pump
  basic_block_or_label: @H1/@T1 selector branch
  instruction_or_decompiled_stmt:
    - if state70 bit5 set: clear bit5
    - if state70 bit6 set: clear bit6 and send @T1
    - else send @H1
  bit_or_mask: bit5 / 0x20, bit6 / 0x40
  operation: test/clear
  direct_caller: main_loop_or_scheduler
  caller_chain_if_known: main_loop_or_scheduler -> bluefrog_machine_state_pump
  required_state_before:
    - @H1: state70 bit5 set, bit6 clear
    - @T1: state70 bit5 set, bit6 set
  required_rx_before: none directly proven
  required_timer_or_counter: state produced by maybe_session_tick_counter_and_timeout_pump
  related_global_or_state_offset: bluefrog_state+0x70 at 0x2000298c
  enables_tx_candidate: @H1 or @T1
  disables_tx_candidate: re-send until bit5/bit6 are rearmed
  confidence: confirmed_code
```

#### Session/tick/counter analysis for state70 bit5/bit6

```text
state70_bit5_bit6_seed:
  seed_source_address: 0x0001e824 consumer; external seed writer not found
  seed_source_function: maybe_session_tick_counter_and_timeout_pump
  seed_state_offset:
    - session_tick_counter_state literal at 0x0001e888 -> 0x20002788
    - tick accumulator: 0x20002788
    - H1/T1 countdown halfword: 0x2000278c
  seed_type: timer/countdown with unresolved external seed
  bit5_set_condition:
    - state70 bit5 is currently clear
    - *(uint16_t *)0x2000278c != 0
    - either service countdown active, or the countdown decrements to zero
  bit6_set_condition:
    - service_channel_any_countdown_active() returns true while 0x2000278c != 0
  bit5_clear_condition:
    - bluefrog_machine_state_pump clears bit5 before sending @H1 or @T1
  bit6_clear_condition:
    - maybe_session_tick_counter_and_timeout_pump clears bit6 when countdown reaches zero without active service countdown
    - bluefrog_machine_state_pump clears bit6 before sending @T1
  produces_H1: PARTIAL
  produces_T1: PARTIAL
  expected_rx_after_H1: not statically proven; no @H? identity dialog is encoded at the @H1 callsite
  expected_rx_after_T1: @t1
  unresolved_reason_if_any:
    - XREFs to 0x2000278c show reads/writes only inside maybe_session_tick_counter_and_timeout_pump.
    - No direct writer that seeds the nonzero halfword countdown was found in this focused pass.
    - The caller at 0x0001e8b4 invokes the tick pump only when state70 bit0 is set and on its 10-tick cadence, but does not seed 0x2000278c itself in the visible decompilation.
  confidence: confirmed_code for bit production; unknown for countdown seed origin
```

Supporting call context:

```text
session_tick_caller:
  function: timer/tick function body at 0x0001e8b4
  callsite: 0x0001e94a
  condition:
    - periodic local counter puVar6[3] reaches a 10-tick cadence
    - state70 bit0 set
  calls: maybe_session_tick_counter_and_timeout_pump()
  notes:
    - the same function owns several timing fields at 0x20002050, 0x20002054, 0x20002056, 0x20002058
    - these are broader timer/scheduler fields; they do not directly seed 0x2000278c in the recovered code
  confidence: confirmed_code
```

#### `@T0` substate bits3/4 analysis

```text
state70_t0_substate:
  substate_bits: state70 bits3/4
  substate_mask: 0x00000018
  substate_shift: decompiler shows ((state70 & 0x1f) >> 3); equivalent effective substate is bits3/4
  required_substate_for_T0: 3
  t0_tx_callsite: 0x0001befe
  t0_literal_addr: 0x000249b0
  sender_helper: machine_uart_send_line_encoded
  substate_writer_address:
    - 0x0001d638..0x0001d642 sets substate to 1 from @HF parsed byte 0x04
    - 0x00016294..0x00016300 advances substate 1 -> 2 or 2 -> 3 depending param_2
    - 0x0001d0e0..0x0001d0e6 clears substate bits on @HF parsed byte 0xff
    - 0x0001bdb2..0x0001befe clears substate bits after @T0 TX
  substate_writer_function:
    - host_reply_hf_state70_control_handler at 0x0001d0a4 for @HF-controlled direct writes
    - maybe_advance_t0_state70_substate at 0x00016294 for substate progression
    - bluefrog_machine_state_pump for post-TX clear
  required_previous_rx_or_event:
    - @HF with parsed byte 0x04 sets substate 1
    - unknown caller/event invokes maybe_advance_t0_state70_substate(param_2=1) to move 1 -> 2
    - unknown caller/event invokes maybe_advance_t0_state70_substate(param_2=0) to move 2 -> 3
  clears_after_T0: state70 &= 0xffffffe7
  expected_rx_after_T0: @T3... and/or @t0 seen in runtime, but not directly encoded at the @T0 callsite
  unresolved_reason_if_any:
    - the helper that advances substate 1->2->3 is decompilable at 0x00016294, but Ghidra exposes no direct XREF/caller.
    - likely entry is mainloop/jumptable-adjacent, but the main_loop_or_scheduler jumptable at 0x00016506 is not fully recovered.
  confidence: PARTIAL
```

Substate progression helper:

```text
maybe_advance_t0_state70_substate:
  address: 0x00016294
  behavior_when_state70_bit0_set:
    - param_2 == 1 and current substate == 1: write substate 2
    - param_2 == 0 and current substate == 2: write substate 3
    - otherwise return without write
  behavior_when_state70_bit0_clear:
    - param_2 == 1: state+0x0c = 1
    - param_2 == 0 and 0 < state+0x0c < 0x1f: maybe_set_flag46_bit0_and_refresh_scheduler(0), then state+0x0c = 0
  direct_xrefs: none found by ReVa/Ghidra
  confidence: confirmed_code for behavior; unknown for caller/event source
```

`@HF` control handler relation:

```text
host_reply_hf_state70_control_handler:
  address: 0x0001d0a4
  table_relation: @H? subhandler table entry index 4, corresponding to @HF if indexed as line[2] - 'B'
  parsed_payload: parse_ascii_hex_byte_or_nibble(line+4)
  relevant_state70_effects:
    - parsed 0x00: write DAT_20002058 = 1; clear state70 bit1; send literal at 0x00024978
    - parsed 0x01: write DAT_20002058 = 10; set state70 bit1; send literal at 0x00024978
    - parsed 0x02: clear state70 bit2; send literal at 0x00024978
    - parsed 0x03: set state70 bit2; send literal at 0x00024978
    - parsed 0x04: state70 = (state70 & 0xffffffe7) | 0x08
    - parsed 0xff: state70 &= 0xffffffe7; send literal at 0x00024978
  role_for_T0: establishes only substate 1; does not by itself satisfy @T0 required substate 3
  confidence: confirmed_code for writes; inferred for @HF table mapping
```

#### Partial sequence for `@T0/@H1/@T1`

```text
partial_sequence_resolved: NO

partial_original_sequence_T0_H1_T1:
  - step: H1/T1 producer
    state_condition:
      - state70 bit0 set
      - session_tick_counter_state+4 halfword nonzero
      - state70 bit5 currently clear
    tx:
      - @H1 if service_channel_any_countdown_active() is false and countdown reaches zero
      - @T1 if service_channel_any_countdown_active() is true
    callsite:
      - producer: 0x0001e824
      - consumer/TX: 0x0001be06..0x0001be26 / 0x0001bfd4
    expected_rx:
      - @T1 path: @t1
      - @H1 path: not statically proven
    state_after_rx: @t1 handler clears flags_88 bit1 but does not directly seed state70 bit5/bit6
    confidence: PARTIAL

  - step: T0 producer
    state_condition:
      - state70 bit0 set
      - state70 bits3/4 reach substate 3
    tx: @T0
    callsite: 0x0001befe
    expected_rx: not statically encoded at callsite; runtime showed @T3/@t0 nearby
    state_after_rx: @T0 branch clears bit0/substate bits with state70 &= 0xffffffe7
    confidence: PARTIAL

partial_sequence_blockers:
  - missing_writer_for: session_tick_counter_state+4 seed
    affects_tx: @H1/@T1
    why_unresolved: only reads/writes found for 0x2000278c are inside maybe_session_tick_counter_and_timeout_pump; no external seeding function or RX handler was found in this focused pass.
    next_static_target: non-data-reference writes to 0x2000278c, BSS/init-copy analysis, and unresolved aliasing around 0x20002788.

  - missing_caller_for: maybe_advance_t0_state70_substate
    affects_tx: @T0
    why_unresolved: function at 0x00016294 advances substate 1->2->3, but no direct XREF/caller is exposed; likely mainloop/jumptable-adjacent relation remains unresolved.
    next_static_target: recover/annotate main_loop_or_scheduler jumptable at 0x00016506 and targets 0x00016508..0x000166a8.
```

`@D1` remains excluded:

```text
@D1:
  in_original_ble_firmware: NO
  do_not_use_as_original_bluefrog_evidence: YES
```
