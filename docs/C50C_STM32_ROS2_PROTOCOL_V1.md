# C50C STM32 / ROS 2 Serial Protocol V1

Status: frozen integration contract for the future STM32 firmware plugin.  The
ROS implementation is complete and testable with a byte-stream mock; the real
device has not been opened or tested.

## 1. Scope and safety boundary

Protocol V1 carries body velocity requests and measured base state.  ROS sends
`vx`, `vy`, and `wz`; it never sends PWM, direction GPIO values, timer CCR
values, or per-wheel motor duty.  Wheel mixing, motor control, current limiting,
encoder polarity normalization, and the hard command watchdog remain MCU
responsibilities.

All multibyte integers, including the CRC field, are little endian.
`PAYLOAD_LEN` is a single byte.  Signed integers use two's-complement
representation.  Reserved flag
and fault bits are transmitted as zero by ROS V1 unless explicitly defined.

## 2. Common frame

| Absolute byte offset | Bytes | Type | Field | Value / meaning |
|---:|---:|---|---|---|
| 0 | 1 | `uint8` | `SYNC_0` | `0xAA` |
| 1 | 1 | `uint8` | `SYNC_1` | `0x55` |
| 2 | 1 | `uint8` | `VERSION` | `0x01` |
| 3 | 1 | `uint8` | `MSG_TYPE` | `0x01` command or `0x81` feedback |
| 4 | 1 | `uint8` | `PAYLOAD_LEN` | Payload byte count, excluding common header and CRC |
| 5 | N | bytes | `PAYLOAD` | Message-specific payload |
| 5+N | 2 | `uint16_le` | `CRC16` | CRC-16/CCITT-FALSE |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection,
no final XOR.  CRC coverage starts at `VERSION` and ends with the final payload
byte:

```
VERSION | MSG_TYPE | PAYLOAD_LEN | PAYLOAD
```

The sync bytes are excluded.  The check value for ASCII `123456789` is
`0x29B1`.  A receiver must reject a bad CRC without publishing the payload.

Interoperability golden command vector (`seq=0x78563412`, `vx=200`, `vy=-200`,
`wz=17`, `flags=0x0009`) is 19 bytes:

```
aa5501010c12345678c80038ff1100090065a7
```

## 3. Command (`MSG_TYPE = 0x01`)

Command payload length is exactly 12 bytes (`0x0C`); total frame length is 19
bytes.

| Payload offset | Absolute frame offset | Bytes | Type | Field | Unit | Range / meaning |
|---:|---:|---:|---|---|---|---|
| 0 | 5 | 4 | `uint32_le` | `seq` | count | 0..4294967295; monotonic modulo-2^32 command sequence |
| 4 | 9 | 2 | `int16_le` | `vx_mm_s` | mm/s | Body +X forward; wire range -32768..32767; ROS production clamp -200..200 |
| 6 | 11 | 2 | `int16_le` | `vy_mm_s` | mm/s | Body +Y left; wire range -32768..32767; ROS production clamp -200..200 |
| 8 | 13 | 2 | `int16_le` | `wz_mrad_s` | mrad/s | Body +Z CCW; wire range -32768..32767; accepted safety-chain clamp -450..450 |
| 10 | 15 | 2 | `uint16_le` | `flags` | bitset | 0x0000..0xFFFF; defined command flags below, reserved bits written zero |
| - | 17 | 2 | `uint16_le` | `CRC16` | - | 0x0000..0xFFFF; common-frame CRC, little endian |

### Command flags

| Bit | Mask | Name | Meaning |
|---:|---:|---|---|
| 0 | `0x0001` | `MOTION_ENABLE` | The command watchdog may apply the requested body velocity |
| 1 | `0x0002` | `ESTOP` | Enter / retain emergency stop; velocity fields must be treated as zero |
| 2 | `0x0004` | `REARM` | One-shot request to rearm after permitted safety checks |
| 3 | `0x0008` | `CLEAR_FAULT` | One-shot request to clear clearable latched faults |
| 4..15 | - | reserved | Sender writes zero; receiver ignores for V1 |

`MOTION_ENABLE` is never permission to ignore the MCU watchdog, ESTOP input, or
motor protection.  The MCU must force zero if a valid command is not received
within its own safety timeout.

`REARM` and `CLEAR_FAULT` are reliable one-shot transactions.  ROS transmits
them only in a zero-velocity frame and never mixes them into a nonzero motion
frame.  Within one verified connection, the MCU must remember the most recently
executed one-shot `seq`: a repeated byte-identical frame with that `seq` is
accepted and ACKed again, but its REARM/CLEAR_FAULT effects are not executed
again.  Both operations must also be intrinsically idempotent.  A sender must
never reuse a `seq` with different payload bytes.  Protocol V1 does not require
this duplicate cache to survive reconnect because ROS forbids cross-connection
one-shot replay.

## 4. Feedback (`MSG_TYPE = 0x81`)

Feedback payload length is exactly 51 bytes (`0x33`); total frame length is 58
bytes.

| Payload offset | Absolute frame offset | Bytes | Type | Field | Unit | Range / meaning |
|---:|---:|---:|---|---|---|---|
| 0 | 5 | 4 | `uint32_le` | `last_cmd_seq` | count | 0..4294967295; most recent command sequence accepted by MCU |
| 4 | 9 | 8 | `int64_le` | `fl_encoder` | tick | -9223372036854775808..9223372036854775807; accumulated front-left encoder count |
| 12 | 17 | 8 | `int64_le` | `fr_encoder` | tick | -9223372036854775808..9223372036854775807; accumulated front-right encoder count |
| 20 | 25 | 8 | `int64_le` | `rl_encoder` | tick | -9223372036854775808..9223372036854775807; accumulated rear-left encoder count |
| 28 | 33 | 8 | `int64_le` | `rr_encoder` | tick | -9223372036854775808..9223372036854775807; accumulated rear-right encoder count |
| 36 | 41 | 2 | `int16_le` | `fl_speed_mm_s` | mm/s | -32768..32767; actual front-left wheel rim speed |
| 38 | 43 | 2 | `int16_le` | `fr_speed_mm_s` | mm/s | -32768..32767; actual front-right wheel rim speed |
| 40 | 45 | 2 | `int16_le` | `rl_speed_mm_s` | mm/s | -32768..32767; actual rear-left wheel rim speed |
| 42 | 47 | 2 | `int16_le` | `rr_speed_mm_s` | mm/s | -32768..32767; actual rear-right wheel rim speed |
| 44 | 49 | 1 | `uint8` | `control_state` | enum | 0..255; values 0..6 defined below, 7..255 reserved/unknown |
| 45 | 50 | 4 | `uint32_le` | `fault_bits` | bitset | 0x00000000..0xFFFFFFFF; fault table below, preserve unknown bits |
| 49 | 54 | 2 | `uint16_le` | `battery_mv` | mV | 0..65535 mV; ROS publishes volts as `battery_mv / 1000` |
| - | 56 | 2 | `uint16_le` | `CRC16` | - | 0x0000..0xFFFF; common-frame CRC, little endian |

The four wheel speeds are already normalized to the vehicle convention: a
positive value means that wheel rotates in the direction that drives the
vehicle forward when all four values are positive.  Encoder polarity is
normalized exactly once, in MCU firmware or in the serial decoder contract;
upper ROS nodes do not expose per-wheel sign parameters.

### Control states

| Value | Name | Meaning |
|---:|---|---|
| 0 | `BOOT` | MCU initialization is in progress |
| 1 | `LOCKED` | Motion interlock is locked |
| 2 | `READY` | Armed and ready, currently no motion |
| 3 | `RUNNING` | Motor control is applying a motion command |
| 4 | `STOPPED` | Controlled stop / valid zero command |
| 5 | `ESTOP` | Emergency stop active |
| 6 | `FAULT` | Fault state |

Unknown state values are published numerically and must not crash the ROS
driver or GUI.

### Fault bits

| Bit | Mask | Name |
|---:|---:|---|
| 0 | `0x00000001` | `COMMAND_TIMEOUT` |
| 1 | `0x00000002` | `ENCODER_FAULT` |
| 2 | `0x00000004` | `UNDERVOLTAGE` |
| 3 | `0x00000008` | `MOTOR_CONTROL_FAULT` |
| 4 | `0x00000010` | `COMMUNICATION_FAULT` |
| 5 | `0x00000020` | `ESTOP_ACTIVE` |
| 6..31 | - | reserved / future; preserve and display as an unknown hexadecimal mask |

Battery voltage is the only BatteryState quantity supplied by V1.  ROS sets
`sensor_msgs/BatteryState.voltage`; percentage and unsupported analog fields
are NaN, never fabricated.

Interoperability golden feedback vector (`last_cmd_seq=0x78563412`, encoders
`[-1,2,-3,4]`, wheel speeds `[100,-100,25,-25]` mm/s, state `FAULT`, faults
`0x20000004`, battery `24123` mV) is 58 bytes:

```
aa5501813312345678ffffffffffffffff0200000000000000fdffffffffffffff040000000000000064009cff1900e7ff06040000203b5e23d1
```

## 5. Rates, timeout, and acknowledgement

- ROS command transmit rate: 50 Hz by default, configurable.
- The transmit timer uses a monotonic steady clock; Gazebo `/clock` pause or
  burst behavior cannot stall or coalesce the safety I/O cadence.
- Expected MCU feedback rate: 50 Hz by default, configurable.
- ROS command freshness timeout: 0.15 s by default; stale input becomes a zero
  frame with `MOTION_ENABLE` cleared.
- ROS feedback timeout: 0.15 s by default; it drops the link into the reconnect
  interlock.
- MCU must implement its own independent hard command watchdog.
- ROS compares `last_cmd_seq` with its last transmitted `seq` modulo 2^32 and
  reports current and maximum ACK lag.  It does not alter Nav2 commands to hide
  lag.

For `REARM` and `CLEAR_FAULT`, receipt is confirmed only when a valid feedback
frame contains the exact transaction `seq` in `last_cmd_seq`.  A different or
stale ACK does not complete the request.  On missing ACK, ROS retries the exact
cached bytes with the same `seq`; the default budget is three total write
attempts.  A write exception means delivery is uncertain and immediately
terminates the transaction as failed because the link must reconnect.  Within
an otherwise healthy connection, exhaustion of the retry budget also stops
resending and reports `CONTROL_DELIVERY_FAILED`.  Both cases require a new
explicit service request.  This gives bounded at-least-once delivery within one
connection; the MCU duplicate suppression above supplies safe idempotent
execution when an ACK is lost.

## 6. Stream parser requirements

The receiver accepts arbitrary chunks: a frame split across reads, multiple
frames in one read, and garbage before/between frames.  It scans for `AA 55`,
validates version and bounded payload length, waits for the complete candidate,
and validates CRC before decoding.  On invalid version, length, or CRC it
discards one byte and rescans, retaining a final lone `AA` for the next read.
Bad frames never block, crash, or publish data.

## 7. Disconnect / reconnect interlock

The serial backend follows this state machine:

```
DISCONNECTED
  -> RECONNECTED_LOCKED
  -> clear host RX/TX buffers and parser
  -> transmit a zero command
  -> WAIT_FEEDBACK
  -> receive one valid CRC-checked feedback frame
  -> WAIT_FRESH_ZERO
  -> receive a new zero /cmd_vel_safe sample timestamped after valid feedback
  -> transmit that zero and observe last_cmd_seq ACK for its exact seq
  -> READY
```

Nonzero commands received while locked are discarded, not queued.  Therefore a
previous nonzero command cannot replay after reconnect.  ESTOP is allowed to
propagate while locked.  Any read/write exception or feedback timeout returns
to `DISCONNECTED` and repeats the full interlock.

No one-shot frame may be transmitted during `RECONNECTED_LOCKED`,
`WAIT_FEEDBACK`, or `WAIT_FRESH_ZERO`.  The complete reconnect sequence and the
exact fresh-zero ACK must establish `READY` first.  A queued or in-flight
one-shot is marked failed on any disconnect/write exception and is never
replayed into the new connection; old nonzero motion is likewise discarded.
The operator must issue a new service request after `READY`.

ESTOP has level-triggered wire priority on every transmit tick and always
forces zero velocity fields.  As soon as ESTOP is asserted, all pending or
in-flight one-shot requests—including REARM—are marked failed.  Deasserting
ESTOP never releases an earlier REARM automatically; a deliberate new REARM
service request is mandatory.

The ROS ESTOP subscription performs that cancellation synchronously on the
`true` callback edge, rather than waiting for the next 50 Hz transmit tick.
Consequently a reliable `true` then `false` pulse wholly between two ticks
still clears node-side requests and atomically fails backend one-shot state.
REARM/CLEAR_FAULT service calls made while ESTOP is active are rejected and
must be repeated after release.

The ROS node publishes standard `diagnostic_msgs/DiagnosticArray` on
`/diagnostics`.  It reports backend/link state, transmitted and acknowledged
sequence numbers, current/maximum ACK lag, feedback age, CRC/decode counts,
disconnect count, MCU fault bits, unknown fault bits, and the one-shot delivery
state/flags/sequence/attempt count.  A disconnected link, MCU fault, or failed
one-shot delivery is ERROR; lock/reconnect, queued/in-flight control delivery,
stale feedback, excessive ACK lag, or CRC errors are WARN.

## 8. Wheel odometry convention

For normalized rim speeds `(fl, fr, rl, rr)` in m/s and
`K = (wheel_spacing + axle_spacing) / 2`:

```
vx = ( fl + fr + rl + rr) / 4
vy = (-fl + fr + rl - rr) / 4
wz = (-fl + fr - rl + rr) / (4 K)
```

Current nominal parameters are `wheel_radius=0.050 m`,
`wheel_spacing=0.535 m` (full left-right spacing), and
`axle_spacing=0.410 m` (full front-rear spacing).  These support simulation and
initial integration only:

`REAL_WHEEL_ODOM_SCALE=NOT_CALIBRATED`

Future EKF input is `/wheel/odom_raw`.  Simulation truth is published only as
`/sim/ground_truth`; it must never be substituted for the production `/odom`
interface.
