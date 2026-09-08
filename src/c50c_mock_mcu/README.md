# c50c_mock_mcu

`c50c_mock_mcu` is an independent byte-level peer for C50C STM32/ROS 2
Protocol V1. It does not import the production serial driver's codec, so CRC,
offset, endianness and stream-framing mistakes cannot silently agree on both
sides.

The executable creates a pseudo-terminal and a stable symlink, defaulting to
`/tmp/c50c_mock_mcu`. It refuses to place that symlink under `/dev`, and never
opens `/dev/c50c_ros` or `/dev/c50c_flash`.

```bash
ros2 run c50c_mock_mcu c50c_mock_mcu \
  --serial-link /tmp/c50c_mock_mcu \
  --feedback-rate-hz 50
```

Fault switches include `--bad-crc-every`, `--drop-feedback-every`,
`--ack-delay-frames`, `--disconnect-after-feedback`, `--fault-bits`, and
`--force-estop`. A disconnect closes the PTY and a reconnect creates a new PTY
behind the stable symlink. Each connection starts `LOCKED` with zero wheel
speed and requires a newly received zero command before becoming `READY`.

## Production codec compatibility seam

The production `roscar_base_interface.protocol` compatibility seam is:

- `CommandPayload(seq, vx_mm_s, vy_mm_s, wz_mrad_s, flags)` and
  `encode_command(CommandPayload) -> bytes`
- `FrameParser.feed(bytes)` returning complete valid frames
- decoded feedback fields `last_cmd_seq`, four signed 64-bit encoders, four
  signed 16-bit wheel speeds, `control_state`, `fault_bits`, and `battery_mv`
- CRC-16/CCITT-FALSE with initial value `0xFFFF`, polynomial `0x1021`, no
  reflection and no xorout

The header after sync is `<BBB>`: `VERSION`, `MSG_TYPE`, and one-byte
`PAYLOAD_LEN`. CRC covers those three bytes plus the payload and is appended
little-endian. Command frames are 19 bytes and feedback frames are 58 bytes.

The mock's public Python signature uses an immutable `Command` dataclass:
`encode_command(Command(...))`. This difference is deliberate; integration
tests should compare emitted bytes or decoded field values rather than sharing
implementation objects.

The frozen command and feedback golden vectors in `test/test_protocol_codec.py`
are the cross-package compatibility oracle. All integration tests communicate
through `socket.socketpair`, so they exercise actual arbitrary stream reads and
writes rather than calling the MCU state machine directly.

## Reliable one-shot control semantics

`REARM` and `CLEAR_FAULT` are zero-velocity, idempotent one-shot transactions.
After the link has completed its fresh-zero interlock and reached `READY`, the
production serial backend caches the complete frame, retries the identical
bytes with the same `seq` when an ACK is missing, and stops after three total
write attempts by default. Completion requires an exact `last_cmd_seq` match;
a stale or unrelated ACK never clears the request.

The mock implements the firmware-side half of this contract independently.
Within one connection it remembers the most recently executed one-shot
sequence. A duplicate frame is accepted and ACKed again, but REARM/CLEAR_FAULT
side effects are executed only once. Real STM32 firmware must provide the same
per-connection duplicate suppression and must make both operations
intrinsically idempotent.

One-shot requests are never replayed across a disconnect: the host completes
the full zero/feedback/fresh-zero-ACK interlock, reaches `READY`, and waits for
a new explicit service request. ESTOP is not a one-shot; it remains
level-triggered, forces zero velocity, cancels pending/in-flight REARM, and
takes wire/control priority.

Cancellation occurs synchronously on the ROS ESTOP `true` callback edge, so a
short reliable true/false pulse between two 50 Hz transmit ticks cannot leave a
delayed REARM behind. Control service requests made during active ESTOP are
rejected and must be issued again after release.
