# BlueFrog 0x26 Runtime Handling

The decoded Jutta UART stream can contain either plain ASCII protocol lines or BlueFrog inner transport frames
starting with byte `0x26`.

`jutta_proto` treats `0x26` frames as a global transport path:

- ASCII lines are still handled by the existing startup/statistics parser.
- Any received line or passive idle chunk beginning with `0x26` is routed to the common BlueFrog frame handler.
- The handler logs direction, timestamp, length, hex, and an ASCII preview.
- Known decoded cachewriter frames such as `@TF` and `@TV` are forwarded to the existing status/progress handlers.
- Unknown `0x26` frames are counted and logged instead of being silently discarded.

The diagnostic fields are intentionally observational. They do not add commands, change startup order, or treat
`@TF`/`@TV` as queries.

Use `tools/analyze_jutta_26_log.py` to extract and group `0x26` frames from decoded sniffer logs:

```bash
python3 tools/analyze_jutta_26_log.py jura_jutta_decoded_resync.log
```

This produces:

- `jura_jutta_decoded_resync_26_frames.csv`
- `jura_jutta_decoded_resync_26_sequences.md`

The sequence grouping is heuristic: Dongle-to-Machine `0x26` frames are marked as request/write candidates, and
nearby Machine-to-Dongle frames are grouped as possible replies or status updates.

## Current ESP runtime mode

The post-startup live idle observe delay is disabled by default, so XML statistics are no longer held for 180 seconds
after core startup. This keeps normal stats behavior available while the 0x26 runtime path is being investigated.

The captured two-frame 0x26 replay is available only as an explicit diagnostic mode:

```yaml
jutta_proto:
  enable_bluefrog_26_replay: true
```

Default is `false`. With replay disabled, the component keeps the existing classic startup path after
`@t2:818811...0000`. With replay enabled, the component sends only the two captured decoded 0x26 frames, observes for
machine 0x26 responses, and does not immediately fall through to the old `@t3` / `@TR:37` path after a successful 0x26
response.

Reverse-map status:

- The replay responses seen so far decode as core/session lines such as `@tr:37`, `@T3`, and `@t0`.
- `@TF` and `@TV` remain documented as machine-originated cachewriter frames, not as confirmed dongle queries.
- The classic `00 7F 80` BLE follow-up path does not prove a UART `@TP` live trigger.
- Non-printable 0x26 decode candidates are logged as possible binary/cache frames for later clustering.
