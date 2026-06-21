# Original-like BlueFrog flags_88 diagnostics

`original_like_flags88_` is a diagnostic-only mirror of selected internal
BlueFrog `flags_88` bits from the original BLE dongle firmware.

It is derived only from frames the ESP already sends or receives in the existing
startup/session flow. It must not drive control flow, trigger TX, change command
order, or enable any product/PMode/DFU/transfer path.

The goal is runtime visibility: logs should show whether the ESP reaches the
same rough core-session milestones as the original BlueFrog firmware:

- `0x00000004`: `ty:` / type-session context observed
- `0x00000100`: combined/core-session latch candidate
- `0x00000200`: gate/session active candidate after `@tr:37`
- `0x00000400`: `@T2` observed
- `0x00000800`: `@T3` observed
- `0x00000040`: original `@TR:37` arm/rearm candidate, logged only as a
  condition and not used to send commands

The mirror intentionally does not synthesize missing original state. For example,
an existing ESP `@TR:37` TX logs whether original firmware would have expected
`0x40`, but it does not set `0x40` or send any extra command.

`@D1` is a legacy ESP probe and is disabled in the normal startup/stats-handshake
path because it is not confirmed in the original BlueFrog firmware. Keeping it
out of the normal sequence should make `startup_sequence_diff_original_vs_esp`
stop reporting `extra_in_esp_normal=[@D1]`.
