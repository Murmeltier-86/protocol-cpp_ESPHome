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
