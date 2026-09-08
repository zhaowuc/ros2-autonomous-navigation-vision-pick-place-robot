"""Backend implementations for the unified ROSCar base interface.

Only :class:`SerialBackend.start` / :meth:`SerialBackend.tick` can open a
serial port.  Importing or constructing this module is side-effect free, and
the ROS node defaults to :class:`SimBackend`.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Callable, List, Optional, Protocol

from .kinematics import body_to_wheel_speeds, encoder_delta_from_speed
from .protocol import (
    COMMAND_MSG_TYPE,
    FEEDBACK_MSG_TYPE,
    CommandFlag,
    CommandPayload,
    ControlState,
    FaultBit,
    FeedbackPayload,
    FrameParser,
    command_from_si,
    decode_command,
    decode_feedback,
    encode_command,
    encode_feedback,
)


@dataclass(frozen=True)
class BackendCommand:
    seq: int
    vx: float
    vy: float
    wz: float
    flags: int
    source_stamp: float

    @property
    def is_zero(self) -> bool:
        return max(abs(self.vx), abs(self.vy), abs(self.wz)) <= 1.0e-9

    def to_payload(
        self,
        *,
        max_vx: float = 0.20,
        max_vy: float = 0.20,
        max_wz: float = 0.45,
    ) -> CommandPayload:
        return command_from_si(
            self.seq,
            self.vx,
            self.vy,
            self.wz,
            self.flags,
            max_vx=max_vx,
            max_vy=max_vy,
            max_wz=max_wz,
        )


@dataclass(frozen=True)
class BackendFeedback:
    payload: FeedbackPayload
    received_at: float

    @property
    def wheel_speeds_m_s(self) -> tuple[float, float, float, float]:
        return tuple(value / 1000.0 for value in self.payload.wheel_speeds_mm_s)


class BaseBackend(ABC):
    name = 'abstract'

    @abstractmethod
    def set_command(self, command: BackendCommand) -> None:
        raise NotImplementedError

    @abstractmethod
    def tick(self, now: Optional[float] = None) -> List[BackendFeedback]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def cancel_control_transactions(self, reason: str) -> None:
        """Synchronously cancel durable one-shot requests, if supported."""

        del reason

    @property
    def control_transactions_ready(self) -> bool:
        return True

    def notify_estop_edge(self, active: bool, source_stamp: float) -> None:
        """Synchronously apply an ESTOP edge before the next control tick."""

        del source_stamp
        if active:
            self.cancel_control_transactions('cancelled by ESTOP callback edge')

    @property
    def command_for_simulator(self) -> Optional[BackendCommand]:
        return None

    @property
    def diagnostic_summary(self) -> str:
        return self.name

    def diagnostic_snapshot(self, now: Optional[float] = None) -> dict[str, object]:
        return {'backend': self.name}


class SimBackend(BaseBackend):
    """Command-driven simulated wheel sensors behind the production boundary.

    In the kinematic Gazebo fixture the executed body velocity equals the
    bounded actuator command.  Convert that value to four measured wheel
    speeds and encoder counts without consuming ``/sim/ground_truth``.  The
    latter remains an acceptance-only measurement owned by the benchmark.
    """

    name = 'sim'

    def __init__(
        self,
        *,
        wheel_radius: float,
        wheel_spacing: float,
        axle_spacing: float,
        encoder_ticks_per_revolution: int = 4096,
        battery_mv: int = 24000,
    ) -> None:
        self.wheel_radius = float(wheel_radius)
        self.wheel_spacing = float(wheel_spacing)
        self.axle_spacing = float(axle_spacing)
        self.encoder_ticks_per_revolution = int(encoder_ticks_per_revolution)
        self.battery_mv = int(battery_mv)
        self.command = BackendCommand(0, 0.0, 0.0, 0.0, 0, 0.0)
        self._encoders_float = [0.0, 0.0, 0.0, 0.0]
        self._last_tick: Optional[float] = None

    def set_command(self, command: BackendCommand) -> None:
        self.command = command

    @property
    def command_for_simulator(self) -> Optional[BackendCommand]:
        return self.command

    def tick(self, now: Optional[float] = None) -> List[BackendFeedback]:
        now = time.monotonic() if now is None else float(now)
        dt = 0.0 if self._last_tick is None else max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        flags = CommandFlag(self.command.flags)
        if flags & CommandFlag.ESTOP or not flags & CommandFlag.MOTION_ENABLE:
            vx = vy = wz = 0.0
        else:
            vx, vy, wz = self.command.vx, self.command.vy, self.command.wz
        wheel_speeds = body_to_wheel_speeds(
            vx,
            vy,
            wz,
            self.wheel_spacing,
            self.axle_spacing,
        )
        for index, speed in enumerate(wheel_speeds):
            self._encoders_float[index] += encoder_delta_from_speed(
                speed,
                dt,
                self.wheel_radius,
                self.encoder_ticks_per_revolution,
            )
        if flags & CommandFlag.ESTOP:
            state = ControlState.ESTOP
            faults = int(FaultBit.ESTOP_ACTIVE)
        elif any(abs(value) > 1.0e-6 for value in wheel_speeds):
            state = ControlState.RUNNING
            faults = 0
        else:
            state = ControlState.STOPPED
            faults = 0
        payload = FeedbackPayload(
            last_cmd_seq=self.command.seq & 0xFFFFFFFF,
            encoders=tuple(int(round(value)) for value in self._encoders_float),
            wheel_speeds_mm_s=tuple(_to_i16_mm_s(value) for value in wheel_speeds),
            control_state=int(state),
            fault_bits=faults,
            battery_mv=self.battery_mv,
        )
        return [BackendFeedback(payload, now)]

    @property
    def diagnostic_summary(self) -> str:
        return 'sim command_to_wheel_feedback state=READY'

    def diagnostic_snapshot(self, now: Optional[float] = None) -> dict[str, object]:
        return {
            'backend': self.name,
            'state': 'READY',
            'tx_seq': self.command.seq & 0xFFFFFFFF,
            'ack_seq': self.command.seq & 0xFFFFFFFF,
            'ack_lag': 0,
            'feedback_age_sec': 0.0,
            'crc_errors': 0,
            'fault_bits': 0,
            'unknown_fault_bits': 0,
        }


class LoopbackMockMcu:
    """Minimal MCU model whose only ingress/egress is Protocol V1 bytes."""

    def __init__(
        self,
        *,
        wheel_radius: float,
        wheel_spacing: float,
        axle_spacing: float,
        encoder_ticks_per_revolution: int = 4096,
        battery_mv: int = 24100,
    ) -> None:
        self.wheel_radius = float(wheel_radius)
        self.wheel_spacing = float(wheel_spacing)
        self.axle_spacing = float(axle_spacing)
        self.encoder_ticks_per_revolution = int(encoder_ticks_per_revolution)
        self.battery_mv = int(battery_mv)
        self.parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
        self.last_cmd_seq = 0
        self.last_command = CommandPayload(0, 0, 0, 0, 0)
        self.encoders_float = [0.0, 0.0, 0.0, 0.0]
        self.last_update: Optional[float] = None
        self.fault_bits = 0
        self.drop_feedback = False
        self.corrupt_next_feedback = False
        self.ack_delay_frames = 0
        self._ack_queue: list[int] = []

    def exchange(self, data: bytes, now: float) -> bytes:
        responses = bytearray()
        for frame in self.parser.feed(data):
            command = decode_command(frame)
            self._advance(float(now))
            self.last_command = command
            self._ack_queue.append(command.seq)
            if len(self._ack_queue) > self.ack_delay_frames:
                self.last_cmd_seq = self._ack_queue.pop(0)
            if self.drop_feedback:
                continue
            response = bytearray(self._make_feedback())
            if self.corrupt_next_feedback:
                response[-1] ^= 0x5A
                self.corrupt_next_feedback = False
            responses.extend(response)
        return bytes(responses)

    def _advance(self, now: float) -> None:
        dt = 0.0 if self.last_update is None else max(0.0, min(0.2, now - self.last_update))
        self.last_update = now
        for index, speed in enumerate(self._current_wheel_speeds()):
            self.encoders_float[index] += encoder_delta_from_speed(
                speed,
                dt,
                self.wheel_radius,
                self.encoder_ticks_per_revolution,
            )

    def _current_wheel_speeds(self) -> tuple[float, float, float, float]:
        command = self.last_command
        flags = CommandFlag(command.flags)
        if flags & CommandFlag.ESTOP or not flags & CommandFlag.MOTION_ENABLE:
            return (0.0, 0.0, 0.0, 0.0)
        return body_to_wheel_speeds(
            command.vx_mm_s / 1000.0,
            command.vy_mm_s / 1000.0,
            command.wz_mrad_s / 1000.0,
            self.wheel_spacing,
            self.axle_spacing,
        )

    def _make_feedback(self) -> bytes:
        speeds = self._current_wheel_speeds()
        flags = CommandFlag(self.last_command.flags)
        if flags & CommandFlag.ESTOP:
            state = ControlState.ESTOP
            faults = self.fault_bits | int(FaultBit.ESTOP_ACTIVE)
        elif self.fault_bits:
            state = ControlState.FAULT
            faults = self.fault_bits
        elif any(abs(value) > 1.0e-6 for value in speeds):
            state = ControlState.RUNNING
            faults = 0
        else:
            state = ControlState.STOPPED
            faults = 0
        return encode_feedback(
            FeedbackPayload(
                last_cmd_seq=self.last_cmd_seq,
                encoders=tuple(int(round(value)) for value in self.encoders_float),
                wheel_speeds_mm_s=tuple(_to_i16_mm_s(value) for value in speeds),
                control_state=int(state),
                fault_bits=faults,
                battery_mv=self.battery_mv,
            )
        )


class MockBackend(BaseBackend):
    """In-memory byte-loopback backend used for no-hardware integration."""

    name = 'mock'

    def __init__(
        self,
        *,
        wheel_radius: float,
        wheel_spacing: float,
        axle_spacing: float,
        encoder_ticks_per_revolution: int = 4096,
    ) -> None:
        self.command = BackendCommand(0, 0.0, 0.0, 0.0, 0, 0.0)
        self.mcu = LoopbackMockMcu(
            wheel_radius=wheel_radius,
            wheel_spacing=wheel_spacing,
            axle_spacing=axle_spacing,
            encoder_ticks_per_revolution=encoder_ticks_per_revolution,
        )
        self.feedback_parser = FrameParser(accepted_types={FEEDBACK_MSG_TYPE})
        self.last_ack_lag = 0
        self.max_ack_lag = 0

    def set_command(self, command: BackendCommand) -> None:
        self.command = command

    def tick(self, now: Optional[float] = None) -> List[BackendFeedback]:
        now = time.monotonic() if now is None else float(now)
        wire_command = encode_command(self.command.to_payload())
        wire_feedback = self.mcu.exchange(wire_command, now)
        feedbacks = [
            BackendFeedback(decode_feedback(frame), now)
            for frame in self.feedback_parser.feed(wire_feedback)
        ]
        for feedback in feedbacks:
            self.last_ack_lag = _sequence_lag(self.command.seq, feedback.payload.last_cmd_seq)
            self.max_ack_lag = max(self.max_ack_lag, self.last_ack_lag)
        return feedbacks

    @property
    def diagnostic_summary(self) -> str:
        return (
            f'mock byte_loopback ack_lag={self.last_ack_lag} '
            f'crc_errors={self.feedback_parser.crc_errors}'
        )

    def diagnostic_snapshot(self, now: Optional[float] = None) -> dict[str, object]:
        return {
            'backend': self.name,
            'state': 'READY',
            'tx_seq': self.command.seq & 0xFFFFFFFF,
            'ack_seq': self.mcu.last_cmd_seq & 0xFFFFFFFF,
            'ack_lag': self.last_ack_lag,
            'max_ack_lag': self.max_ack_lag,
            'feedback_age_sec': 0.0,
            'crc_errors': self.feedback_parser.crc_errors,
            'fault_bits': self.mcu.fault_bits & 0xFFFFFFFF,
            'unknown_fault_bits': self.mcu.fault_bits & ~0x3F & 0xFFFFFFFF,
        }


class SerialTransport(Protocol):
    @property
    def in_waiting(self) -> int:
        ...

    @property
    def is_open(self) -> bool:
        ...

    def read(self, size: int) -> bytes:
        ...

    def write(self, data: bytes) -> int:
        ...

    def reset_input_buffer(self) -> None:
        ...

    def reset_output_buffer(self) -> None:
        ...

    def close(self) -> None:
        ...


class SerialLinkState(str, Enum):
    DISCONNECTED = 'DISCONNECTED'
    RECONNECTED_LOCKED = 'RECONNECTED_LOCKED'
    WAIT_FEEDBACK = 'WAIT_FEEDBACK'
    WAIT_FRESH_ZERO = 'WAIT_FRESH_ZERO'
    WAIT_FRESH_ZERO_ACK = 'WAIT_FRESH_ZERO_ACK'
    READY = 'READY'


class ControlDeliveryState(str, Enum):
    """Delivery state for reliable one-shot MCU control requests."""

    IDLE = 'IDLE'
    QUEUED = 'QUEUED'
    INFLIGHT = 'INFLIGHT'
    ACKED = 'ACKED'
    FAILED = 'FAILED'


class SerialBackend(BaseBackend):
    """Protocol V1 serial backend with an RX-first, no-replay interlock.

    A newly opened transport is receive-only until one valid MCU feedback
    frame has been decoded.  It then accepts exactly one *new* ROS zero sample,
    freezes both the command and its encoded bytes, writes that frame once, and
    requires an exact sequence acknowledgement before entering ``READY``.
    ESTOP is the sole pre-READY transmit exception.
    """

    name = 'serial'
    _TX_KINDS = ('fresh_zero', 'estop', 'normal', 'control', 'close')

    def __init__(
        self,
        *,
        serial_device: str = '/dev/c50c_ros',
        baudrate: int = 115200,
        command_tx_rate: float = 50.0,
        command_timeout: float = 0.15,
        feedback_expected_rate: float = 50.0,
        feedback_timeout: float = 0.15,
        fresh_zero_ack_timeout: Optional[float] = None,
        reconnect_interval: float = 1.0,
        max_vx: float = 0.20,
        max_vy: float = 0.20,
        max_wz: float = 0.45,
        control_max_attempts: int = 3,
        control_retry_interval: Optional[float] = None,
        serial_factory: Optional[Callable[..., SerialTransport]] = None,
    ) -> None:
        self.serial_device = str(serial_device)
        self.baudrate = int(baudrate)
        self.command_tx_rate = float(command_tx_rate)
        self.command_timeout = float(command_timeout)
        self.feedback_expected_rate = float(feedback_expected_rate)
        self.feedback_timeout = float(feedback_timeout)
        self.fresh_zero_ack_timeout = (
            self.feedback_timeout
            if fresh_zero_ack_timeout is None
            else float(fresh_zero_ack_timeout)
        )
        if (
            not math.isfinite(self.fresh_zero_ack_timeout)
            or self.fresh_zero_ack_timeout <= 0.0
        ):
            raise ValueError('fresh_zero_ack_timeout must be positive')
        self.reconnect_interval = float(reconnect_interval)
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_wz = float(max_wz)
        self.control_max_attempts = int(control_max_attempts)
        if self.control_max_attempts <= 0:
            raise ValueError('control_max_attempts must be positive')
        self.control_retry_interval = (
            max(0.001, float(control_retry_interval))
            if control_retry_interval is not None
            else max(
                1.0 / max(1.0, self.command_tx_rate),
                1.0 / max(1.0, self.feedback_expected_rate),
            )
        )
        self.serial_factory = serial_factory
        self._state_lock = threading.RLock()

        self.state = SerialLinkState.DISCONNECTED
        self.transport: Optional[SerialTransport] = None
        self.command = BackendCommand(0, 0.0, 0.0, 0.0, 0, 0.0)
        self._safe_zero = self.command
        self._estop_asserted = False
        self._estop_command: Optional[BackendCommand] = None
        self._estop_freshness_barrier_at: Optional[float] = None
        self._fresh_zero_retry_after_source_stamp: Optional[float] = None
        self.parser = FrameParser(accepted_types={FEEDBACK_MSG_TYPE})
        self.next_reconnect_at = 0.0
        self.reconnected_at: Optional[float] = None
        self.connection_epoch = 0
        self.valid_feedback_seen = False
        self.fresh_zero_seen = False
        self.first_valid_feedback_at: Optional[float] = None
        self.fresh_zero_seq: Optional[int] = None
        self.fresh_zero_ack_baseline_seq: Optional[int] = None
        self.zero_sent_seq: Optional[int] = None
        self.ready_zero_seq: Optional[int] = None
        self._fresh_zero_command: Optional[BackendCommand] = None
        self._fresh_zero_frame: Optional[bytes] = None
        self._fresh_zero_write_attempted = False
        self._fresh_zero_tx_at: Optional[float] = None
        self.last_feedback_at: Optional[float] = None
        self.last_tx_seq = 0
        self.last_ack_seq = 0
        self.last_ack_lag = 0
        self.max_ack_lag = 0
        self.feedback_frames = 0
        self.decode_errors = 0
        self.last_fault_bits = 0
        self.disconnects = 0
        self.last_error = ''

        # Every transport.write invocation is accounted for before the call;
        # success is recorded only after a complete write.  Total counters are
        # process-lifetime, while session counters reset on each successful
        # serial open and remain available after a disconnect for forensics.
        self.tx_attempts_total = 0
        self.tx_successes_total = 0
        self._tx_attempts_by_kind_total = {kind: 0 for kind in self._TX_KINDS}
        self._tx_successes_by_kind_total = {kind: 0 for kind in self._TX_KINDS}
        self.nonzero_tx_attempts_total = 0
        self.nonzero_tx_successes_total = 0
        self.fresh_zero_ack_total = 0
        self.fresh_zero_ack_collision_rejections_total = 0
        self._reset_session_tx_diagnostics()

        # REARM and CLEAR_FAULT are reliable one-shot transactions rather than
        # ordinary level flags.  They may retry with one fixed seq only while
        # the verified connection remains READY.  Disconnect or ESTOP marks
        # them failed; neither condition permits delayed automatic replay.
        self.control_pending_flags = 0
        self.control_failed_flags = 0
        self.control_inflight_flags = 0
        self.control_inflight_seq: Optional[int] = None
        self.control_inflight_command: Optional[BackendCommand] = None
        self.control_inflight_frame: Optional[bytes] = None
        self.control_attempts = 0
        self.control_last_tx_at: Optional[float] = None
        self.control_last_acked_seq: Optional[int] = None
        self.control_last_acked_flags = 0
        self.control_delivery_state = ControlDeliveryState.IDLE
        self.control_last_error = ''

    def _reset_session_tx_diagnostics(self) -> None:
        self.tx_attempts_session = 0
        self.tx_successes_session = 0
        self._tx_attempts_by_kind_session = {kind: 0 for kind in self._TX_KINDS}
        self._tx_successes_by_kind_session = {kind: 0 for kind in self._TX_KINDS}
        self.nonzero_tx_attempts_session = 0
        self.nonzero_tx_successes_session = 0
        self.fresh_zero_ack_session = 0
        self.fresh_zero_ack_collision_rejections_session = 0
        self.fresh_zero_tx_attempt_seq: Optional[int] = None
        self.fresh_zero_tx_attempt_flags = 0
        self.fresh_zero_tx_success_seq: Optional[int] = None
        self.fresh_zero_tx_success_flags = 0
        self.last_tx_attempt_kind = 'none'
        self.last_tx_attempt_seq: Optional[int] = None
        self.last_tx_attempt_flags = 0
        self.last_tx_success_kind = 'none'
        self.last_tx_success_seq: Optional[int] = None
        self.last_tx_success_flags = 0
        self.last_tx_success_was_nonzero_motion = False

    def set_command(self, command: BackendCommand) -> None:
        with self._state_lock:
            self._set_command_locked(command)

    def _set_command_locked(self, command: BackendCommand) -> None:
        one_shot_mask = int(CommandFlag.REARM | CommandFlag.CLEAR_FAULT)
        requested_flags = command.flags & one_shot_mask
        if requested_flags:
            self._queue_control_flags(requested_flags)

        estop_requested = bool(command.flags & int(CommandFlag.ESTOP))
        normalized = BackendCommand(
            command.seq & 0xFFFFFFFF,
            command.vx,
            command.vy,
            command.wz,
            command.flags & ~one_shot_mask,
            command.source_stamp,
        )
        if estop_requested:
            # ESTOP is always encoded as a zero-velocity level command.  It
            # invalidates even an already-READY handshake and advances a
            # source-time barrier, so release cannot reuse a pre-ESTOP zero.
            self._fail_all_control_transactions('cancelled by ESTOP')
            self._estop_asserted = True
            barrier = float(command.source_stamp)
            if not math.isfinite(barrier):
                barrier = time.monotonic()
            self._estop_freshness_barrier_at = (
                barrier
                if self._estop_freshness_barrier_at is None
                else max(self._estop_freshness_barrier_at, barrier)
            )
            self._estop_command = BackendCommand(
                normalized.seq,
                0.0,
                0.0,
                0.0,
                int(CommandFlag.ESTOP),
                normalized.source_stamp,
            )
            self.command = self._estop_command
            self._invalidate_fresh_zero_handshake()
            return

        self._estop_asserted = False
        self._estop_command = None
        if self.state == SerialLinkState.READY:
            self.command = normalized
            return

        # Motion and ordinary zero samples are receive-only while interlocked.
        # Store a harmless representation so no pre-READY motion can replay.
        self._safe_zero = BackendCommand(
            normalized.seq,
            0.0,
            0.0,
            0.0,
            0,
            normalized.source_stamp,
        )
        self.command = self._safe_zero

        if not self._qualifies_as_fresh_zero(normalized, requested_flags):
            return

        # Freeze a protocol-level disarmed zero, not the ROS command's
        # MOTION_ENABLE bit.  A zero Twist is normally marked motion-enabled by
        # the node, but the one startup-authority frame must remain an exact
        # flags=0 zero.  Preserve only the sequence/source stamp that prove the
        # sample arrived after the first valid feedback.
        handshake_zero = BackendCommand(
            normalized.seq,
            0.0,
            0.0,
            0.0,
            0,
            normalized.source_stamp,
        )
        # Freeze both the semantic command and its exact wire bytes.  Command
        # churn while waiting for ACK cannot replace or resend this candidate.
        self.fresh_zero_seen = True
        self.fresh_zero_seq = handshake_zero.seq & 0xFFFFFFFF
        self.fresh_zero_ack_baseline_seq = self.last_ack_seq & 0xFFFFFFFF
        self._fresh_zero_command = handshake_zero
        self._fresh_zero_frame = self._encode_command(handshake_zero)
        self._fresh_zero_write_attempted = False
        self._fresh_zero_tx_at = None
        self.command = handshake_zero
        self.state = SerialLinkState.WAIT_FRESH_ZERO_ACK

    def _qualifies_as_fresh_zero(
        self,
        command: BackendCommand,
        requested_flags: int,
    ) -> bool:
        if self.state != SerialLinkState.WAIT_FRESH_ZERO:
            return False
        if not self.valid_feedback_seen or self.first_valid_feedback_at is None:
            return False
        if requested_flags or command.flags & int(CommandFlag.ESTOP):
            return False
        if not command.is_zero:
            return False
        cutoff = self.first_valid_feedback_at
        if self._estop_freshness_barrier_at is not None:
            cutoff = max(cutoff, self._estop_freshness_barrier_at)
        if self._fresh_zero_retry_after_source_stamp is not None:
            cutoff = max(cutoff, self._fresh_zero_retry_after_source_stamp)
        if (
            not math.isfinite(float(command.source_stamp))
            or command.source_stamp <= cutoff
        ):
            return False
        if (command.seq & 0xFFFFFFFF) == (self.last_ack_seq & 0xFFFFFFFF):
            self._record_fresh_zero_ack_collision_rejection(command.source_stamp)
            return False
        return True

    def _record_fresh_zero_ack_collision_rejection(self, source_stamp: float) -> None:
        self.fresh_zero_ack_collision_rejections_total += 1
        self.fresh_zero_ack_collision_rejections_session += 1
        stamp = float(source_stamp)
        if math.isfinite(stamp):
            self._fresh_zero_retry_after_source_stamp = (
                stamp
                if self._fresh_zero_retry_after_source_stamp is None
                else max(self._fresh_zero_retry_after_source_stamp, stamp)
            )

    def _invalidate_fresh_zero_handshake(self) -> None:
        self.fresh_zero_seen = False
        self.fresh_zero_seq = None
        self.zero_sent_seq = None
        self.ready_zero_seq = None
        self._fresh_zero_command = None
        self._fresh_zero_frame = None
        self._fresh_zero_write_attempted = False
        self._fresh_zero_tx_at = None
        if self.transport is not None:
            self.state = (
                SerialLinkState.WAIT_FRESH_ZERO
                if self.valid_feedback_seen
                else SerialLinkState.WAIT_FEEDBACK
            )

    def tick(self, now: Optional[float] = None) -> List[BackendFeedback]:
        with self._state_lock:
            return self._tick_locked(now)

    def _tick_locked(self, now: Optional[float] = None) -> List[BackendFeedback]:
        now = time.monotonic() if now is None else float(now)
        if self.transport is None:
            if now < self.next_reconnect_at:
                return []
            if not self._connect(now):
                return []

        # Once the one permitted fresh-zero write has succeeded, a late ACK is
        # not accepted after its independent deadline even if feedback remains
        # otherwise healthy.
        if (
            self.state == SerialLinkState.WAIT_FRESH_ZERO_ACK
            and self._fresh_zero_tx_at is not None
            and now - self._fresh_zero_tx_at > self.fresh_zero_ack_timeout
        ):
            self._disconnect(now, 'fresh zero ACK timeout')
            return []

        state_before_read = self.state
        feedbacks = self._read_available(now)
        if self.transport is None:
            return feedbacks
        if (
            self.last_feedback_at is not None
            and now - self.last_feedback_at > self.feedback_timeout
        ):
            self._disconnect(now, 'feedback timeout')
            return feedbacks
        if (
            self.last_feedback_at is None
            and self.reconnected_at is not None
            and now - self.reconnected_at > self.feedback_timeout
        ):
            self._disconnect(now, 'initial feedback timeout')
            return feedbacks

        # Do not emit a READY heartbeat in the same tick that consumed the
        # exact handshake ACK.  This preserves an unambiguous one-frame
        # pre-READY transcript; ordinary 50 Hz zero traffic may resume later.
        if (
            state_before_read == SerialLinkState.WAIT_FRESH_ZERO_ACK
            and self.state == SerialLinkState.READY
        ):
            return feedbacks

        selected = self._outgoing_command(now)
        if selected is None:
            return feedbacks
        outgoing, wire, kind = selected
        try:
            self._write_command(outgoing, now=now, kind=kind, wire=wire)
        except Exception as exc:  # serial implementations expose several exception classes
            self._disconnect(now, f'write error ({kind}): {exc}')
        return feedbacks

    def cancel_control_transactions(self, reason: str) -> None:
        """Atomically prevent queued/inflight one-shots from later execution."""

        with self._state_lock:
            self._fail_all_control_transactions(str(reason))

    @property
    def control_transactions_ready(self) -> bool:
        with self._state_lock:
            return self.transport is not None and self.state == SerialLinkState.READY

    def notify_estop_edge(self, active: bool, source_stamp: float) -> None:
        """Invalidate serial authority on both ESTOP assertion and release."""

        with self._state_lock:
            if active:
                self._fail_all_control_transactions('cancelled by ESTOP callback edge')
            barrier = float(source_stamp)
            if not math.isfinite(barrier):
                barrier = time.monotonic()
            self._estop_freshness_barrier_at = (
                barrier
                if self._estop_freshness_barrier_at is None
                else max(self._estop_freshness_barrier_at, barrier)
            )
            self._invalidate_fresh_zero_handshake()

    def _connect(self, now: float) -> bool:
        opened_transport: Optional[SerialTransport] = None
        try:
            factory = self.serial_factory or _default_serial_factory
            opened_transport = factory(
                port=self.serial_device,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=min(0.05, max(0.01, 1.0 / self.command_tx_rate)),
            )
            opened_transport.reset_input_buffer()
            opened_transport.reset_output_buffer()
            self.parser.reset()
        except Exception as exc:
            if opened_transport is not None:
                try:
                    opened_transport.close()
                except Exception:
                    pass
            self.transport = None
            self.last_error = f'connect error: {exc}'
            self.next_reconnect_at = now + self.reconnect_interval
            self._fail_all_control_transactions(self.last_error)
            return False
        self.transport = opened_transport
        self.connection_epoch += 1
        self._reset_session_tx_diagnostics()
        self.state = SerialLinkState.WAIT_FEEDBACK
        self.reconnected_at = now
        self.valid_feedback_seen = False
        self.fresh_zero_seen = False
        self.first_valid_feedback_at = None
        self.fresh_zero_seq = None
        self.fresh_zero_ack_baseline_seq = None
        self.zero_sent_seq = None
        self.ready_zero_seq = None
        self._fresh_zero_command = None
        self._fresh_zero_frame = None
        self._fresh_zero_write_attempted = False
        self._fresh_zero_tx_at = None
        self._fresh_zero_retry_after_source_stamp = None
        self.last_feedback_at = None
        self.last_tx_seq = 0
        self.last_ack_seq = 0
        self.last_ack_lag = 0
        self._safe_zero = BackendCommand(
            self._safe_zero.seq,
            0.0,
            0.0,
            0.0,
            0,
            now,
        )
        self.command = (
            self._estop_command
            if self._estop_asserted and self._estop_command is not None
            else self._safe_zero
        )
        self.last_error = ''
        return True

    def _read_available(self, now: float) -> List[BackendFeedback]:
        if self.transport is None:
            return []
        try:
            waiting = int(self.transport.in_waiting)
            data = self.transport.read(waiting) if waiting > 0 else b''
        except Exception as exc:
            self._disconnect(now, f'read error: {exc}')
            return []
        feedbacks: List[BackendFeedback] = []
        for frame in self.parser.feed(data):
            try:
                payload = decode_feedback(frame)
            except ValueError:
                self.decode_errors += 1
                continue
            self.valid_feedback_seen = True
            if self.first_valid_feedback_at is None:
                self.first_valid_feedback_at = now
            self.last_feedback_at = now
            self.last_ack_seq = payload.last_cmd_seq
            self.last_ack_lag = _sequence_lag(self.last_tx_seq, self.last_ack_seq)
            self.max_ack_lag = max(self.max_ack_lag, self.last_ack_lag)
            self._ack_fresh_zero(payload.last_cmd_seq)
            if (
                self.control_inflight_seq is not None
                and payload.last_cmd_seq == self.control_inflight_seq
            ):
                self._ack_control_transaction()
            self.feedback_frames += 1
            self.last_fault_bits = payload.fault_bits
            feedbacks.append(BackendFeedback(payload, now))
        if self.valid_feedback_seen and self.state in (
            SerialLinkState.WAIT_FEEDBACK,
            SerialLinkState.RECONNECTED_LOCKED,
        ):
            self.state = SerialLinkState.WAIT_FRESH_ZERO
        return feedbacks

    def _ack_fresh_zero(self, acknowledged_seq: int) -> None:
        if (
            self.state == SerialLinkState.WAIT_FRESH_ZERO_ACK
            and self.fresh_zero_seq is not None
            and self.zero_sent_seq == self.fresh_zero_seq
            and acknowledged_seq == self.fresh_zero_seq
            and self._fresh_zero_command is not None
        ):
            self.fresh_zero_ack_total += 1
            self.fresh_zero_ack_session += 1
            self.ready_zero_seq = self.fresh_zero_seq
            self.command = self._fresh_zero_command
            self.state = SerialLinkState.READY

    def _normal_outgoing_command(self, now: float) -> BackendCommand:
        if now - self.command.source_stamp > self.command_timeout:
            return BackendCommand(
                self.command.seq, 0.0, 0.0, 0.0, 0,
                self.command.source_stamp,
            )
        return self.command

    def _outgoing_command(
        self,
        now: float,
    ) -> Optional[tuple[BackendCommand, bytes, str]]:
        # ESTOP is the only command permitted before READY and always wins over
        # a queued control or an in-flight fresh-zero acknowledgement.
        if self._estop_asserted and self._estop_command is not None:
            self._fail_all_control_transactions('cancelled by ESTOP')
            return (
                self._estop_command,
                self._encode_command(self._estop_command),
                'estop',
            )

        if self.state == SerialLinkState.WAIT_FRESH_ZERO_ACK:
            if self._fresh_zero_write_attempted:
                return None
            if self._fresh_zero_command is None or self._fresh_zero_frame is None:
                self._disconnect(now, 'missing frozen fresh-zero frame')
                return None
            # Feedback is read before TX on every tick.  If the MCU already
            # reports this sequence before the first write, a later repeated
            # old status frame could masquerade as an ACK.  Reject this
            # candidate without writing and wait for a new ROS zero sequence.
            if self.last_ack_seq == self.fresh_zero_seq:
                self._record_fresh_zero_ack_collision_rejection(
                    self._fresh_zero_command.source_stamp
                )
                self._invalidate_fresh_zero_handshake()
                return None
            return self._fresh_zero_command, self._fresh_zero_frame, 'fresh_zero'

        if self.state != SerialLinkState.READY:
            return None

        normal = self._normal_outgoing_command(now)

        if self.control_inflight_command is not None:
            if self.control_attempts >= self.control_max_attempts:
                if (
                    self.control_last_tx_at is not None
                    and now - self.control_last_tx_at < self.control_retry_interval
                ):
                    return None
                self._fail_control_transaction(
                    f'no exact ACK after {self.control_attempts} attempts'
                )
                return normal, self._encode_command(normal), 'normal'
            if (
                self.control_last_tx_at is not None
                and now - self.control_last_tx_at < self.control_retry_interval
            ):
                return None
            if self.control_inflight_frame is None:
                self._fail_control_transaction('missing cached control frame')
                return None
            return (
                self.control_inflight_command,
                self.control_inflight_frame,
                'control',
            )

        if (
            self.control_pending_flags
            and (normal.seq & 0xFFFFFFFF) != self.last_tx_seq
        ):
            self._begin_control_transaction(normal, now)
            if self.control_inflight_command is None or self.control_inflight_frame is None:
                self._fail_control_transaction('failed to freeze control frame')
                return None
            return (
                self.control_inflight_command,
                self.control_inflight_frame,
                'control',
            )

        return normal, self._encode_command(normal), 'normal'

    def _encode_command(self, command: BackendCommand) -> bytes:
        return encode_command(
            command.to_payload(
                max_vx=self.max_vx,
                max_vy=self.max_vy,
                max_wz=self.max_wz,
            )
        )

    def _write_command(
        self,
        command: BackendCommand,
        *,
        now: float,
        kind: str,
        wire: Optional[bytes] = None,
    ) -> None:
        """Perform the sole transport write path with attempt-first accounting."""

        if kind not in self._TX_KINDS:
            raise ValueError(f'unknown TX kind: {kind}')
        frame = self._encode_command(command) if wire is None else bytes(wire)
        payload = command.to_payload(
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_wz=self.max_wz,
        )
        has_nonzero_velocity = bool(
            any((payload.vx_mm_s, payload.vy_mm_s, payload.wz_mrad_s))
        )
        actual_nonzero_motion = bool(
            has_nonzero_velocity
            and payload.flags & int(CommandFlag.MOTION_ENABLE)
            and not payload.flags & int(CommandFlag.ESTOP)
        )

        # Count the invocation before transport.write: an exception can be
        # raised after some or all bytes have reached the MCU.
        self.tx_attempts_total += 1
        self.tx_attempts_session += 1
        self._tx_attempts_by_kind_total[kind] += 1
        self._tx_attempts_by_kind_session[kind] += 1
        if has_nonzero_velocity:
            self.nonzero_tx_attempts_total += 1
            self.nonzero_tx_attempts_session += 1
        self.last_tx_attempt_kind = kind
        self.last_tx_attempt_seq = payload.seq & 0xFFFFFFFF
        self.last_tx_attempt_flags = payload.flags
        if kind == 'control':
            self.control_attempts += 1
            self.control_last_tx_at = now
        elif kind == 'fresh_zero':
            self.fresh_zero_tx_attempt_seq = payload.seq & 0xFFFFFFFF
            self.fresh_zero_tx_attempt_flags = payload.flags
            self._fresh_zero_write_attempted = True
            self._fresh_zero_tx_at = now

        transport = self.transport
        if transport is None:
            raise OSError('serial transport is not open')
        written = transport.write(frame)
        if written != len(frame):
            raise OSError(f'short write: {written}/{len(frame)} bytes')

        self.tx_successes_total += 1
        self.tx_successes_session += 1
        self._tx_successes_by_kind_total[kind] += 1
        self._tx_successes_by_kind_session[kind] += 1
        if has_nonzero_velocity:
            self.nonzero_tx_successes_total += 1
            self.nonzero_tx_successes_session += 1
        self.last_tx_success_kind = kind
        self.last_tx_success_seq = payload.seq & 0xFFFFFFFF
        self.last_tx_success_flags = payload.flags
        self.last_tx_success_was_nonzero_motion = actual_nonzero_motion
        self.last_tx_seq = payload.seq & 0xFFFFFFFF
        # Feedback is consumed before TX in each tick.  Keep the exported
        # tuple (tx_seq, ack_seq, ack_lag) self-consistent after this write,
        # rather than leaving the lag value from the pre-write snapshot.
        self.last_ack_lag = _sequence_lag(self.last_tx_seq, self.last_ack_seq)
        self.max_ack_lag = max(self.max_ack_lag, self.last_ack_lag)
        if kind == 'fresh_zero':
            self.fresh_zero_tx_success_seq = self.last_tx_seq
            self.fresh_zero_tx_success_flags = payload.flags
            self.zero_sent_seq = self.last_tx_seq

    def _queue_control_flags(self, requested_flags: int) -> None:
        requested_flags &= int(CommandFlag.REARM | CommandFlag.CLEAR_FAULT)
        if not requested_flags:
            return
        if self.state != SerialLinkState.READY:
            # There is no verified authority on which to bind a transaction.
            # Never defer a one-shot across the startup/reconnect handshake;
            # report failure and require a new explicit request after READY.
            self.control_failed_flags |= requested_flags
            self.control_delivery_state = ControlDeliveryState.FAILED
            self.control_last_error = (
                f'requested while serial link is {self.state.value}, not READY'
            )
            return
        # A new explicit request is allowed to retry the corresponding bits
        # after a previous bounded failure.  Bits already queued/inflight are
        # coalesced because both operations are idempotent controls.
        self.control_failed_flags &= ~requested_flags
        requested_flags &= ~self.control_inflight_flags
        self.control_pending_flags |= requested_flags
        if self.control_inflight_command is not None:
            self.control_delivery_state = ControlDeliveryState.INFLIGHT
        elif self.control_pending_flags:
            self.control_delivery_state = ControlDeliveryState.QUEUED
            self.control_last_error = ''

    def _begin_control_transaction(
        self,
        base_command: BackendCommand,
        now: float,
    ) -> None:
        flags = self.control_pending_flags
        command = BackendCommand(
            base_command.seq & 0xFFFFFFFF,
            0.0,
            0.0,
            0.0,
            flags,
            now,
        )
        self.control_pending_flags = 0
        self.control_inflight_flags = flags
        self.control_inflight_seq = command.seq
        self.control_inflight_command = command
        self.control_inflight_frame = self._encode_command(command)
        self.control_attempts = 0
        self.control_last_tx_at = None
        self.control_delivery_state = ControlDeliveryState.INFLIGHT
        self.control_last_error = ''

    def _ack_control_transaction(self) -> None:
        self.control_last_acked_seq = self.control_inflight_seq
        self.control_last_acked_flags = self.control_inflight_flags
        self.control_inflight_flags = 0
        self.control_inflight_seq = None
        self.control_inflight_command = None
        self.control_inflight_frame = None
        self.control_last_tx_at = None
        self.control_delivery_state = (
            ControlDeliveryState.QUEUED
            if self.control_pending_flags
            else ControlDeliveryState.FAILED
            if self.control_failed_flags
            else ControlDeliveryState.ACKED
        )
        self.control_last_error = ''

    def _fail_control_transaction(self, reason: str) -> None:
        self.control_failed_flags |= self.control_inflight_flags
        self.control_inflight_flags = 0
        self.control_inflight_seq = None
        self.control_inflight_command = None
        self.control_inflight_frame = None
        self.control_last_tx_at = None
        self.control_delivery_state = (
            ControlDeliveryState.QUEUED
            if self.control_pending_flags
            else ControlDeliveryState.FAILED
        )
        self.control_last_error = reason

    def _fail_all_control_transactions(self, reason: str) -> None:
        failed_flags = self.control_pending_flags | self.control_inflight_flags
        if not failed_flags:
            return
        self.control_failed_flags |= failed_flags
        self.control_pending_flags = 0
        self.control_inflight_flags = 0
        self.control_inflight_seq = None
        self.control_inflight_command = None
        self.control_inflight_frame = None
        self.control_last_tx_at = None
        self.control_delivery_state = ControlDeliveryState.FAILED
        self.control_last_error = reason

    def _disconnect(self, now: float, reason: str) -> None:
        # Delivery is uncertain across a broken connection.  Never replay a
        # delayed REARM/CLEAR_FAULT after the reconnect interlock: expose FAILED
        # and require a new service request once the link reaches READY.
        self._fail_all_control_transactions(reason)
        transport = self.transport
        self.transport = None
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        self.state = SerialLinkState.DISCONNECTED
        self.disconnects += 1
        self.last_error = reason
        self.next_reconnect_at = now + self.reconnect_interval
        self.reconnected_at = None
        self.valid_feedback_seen = False
        self.fresh_zero_seen = False
        self.first_valid_feedback_at = None
        self.fresh_zero_seq = None
        self.zero_sent_seq = None
        self.ready_zero_seq = None
        self._fresh_zero_command = None
        self._fresh_zero_frame = None
        self._fresh_zero_write_attempted = False
        self._fresh_zero_tx_at = None
        self.last_feedback_at = None
        self._safe_zero = BackendCommand(
            self.command.seq,
            0.0,
            0.0,
            0.0,
            0,
            now,
        )
        self.command = (
            self._estop_command
            if self._estop_asserted and self._estop_command is not None
            else self._safe_zero
        )
        self.parser.reset()

    def close(self) -> None:
        with self._state_lock:
            self._close_locked()

    def _close_locked(self) -> None:
        transport = self.transport
        if transport is not None:
            # A hidden teardown write is prohibited unless the most recent
            # successful wire command actually requested nonzero motion.  In
            # that one case, make a best-effort audited zero before closing.
            if self.last_tx_success_was_nonzero_motion:
                now = time.monotonic()
                zero = BackendCommand(
                    (self.last_tx_seq + 1) & 0xFFFFFFFF,
                    0.0,
                    0.0,
                    0.0,
                    0,
                    now,
                )
                try:
                    self._write_command(zero, now=now, kind='close')
                except Exception as exc:
                    self.last_error = f'close zero write error: {exc}'
            try:
                transport.close()
            except Exception:
                pass
        self._fail_all_control_transactions('backend closed')
        self.transport = None
        self.state = SerialLinkState.DISCONNECTED

    @property
    def diagnostic_summary(self) -> str:
        suffix = f' error={self.last_error}' if self.last_error else ''
        return (
            f'serial state={self.state.value} feedback={self.feedback_frames} '
            f'ack_lag={self.last_ack_lag} max_ack_lag={self.max_ack_lag} '
            f'crc_errors={self.parser.crc_errors} '
            f'tx={self.tx_attempts_session}/{self.tx_successes_session} '
            f'fresh_zero_ack={self.fresh_zero_ack_session} '
            f'control={self.control_delivery_state.value} '
            f'control_attempts={self.control_attempts}{suffix}'
        )

    def diagnostic_snapshot(self, now: Optional[float] = None) -> dict[str, object]:
        now = time.monotonic() if now is None else float(now)
        feedback_age = (
            math.inf
            if self.last_feedback_at is None
            else max(0.0, now - self.last_feedback_at)
        )
        snapshot: dict[str, object] = {
            'backend': self.name,
            'state': self.state.value,
            'connection_epoch': self.connection_epoch,
            'tx_seq': self.last_tx_seq,
            'ack_seq': self.last_ack_seq,
            'ack_lag': self.last_ack_lag,
            'max_ack_lag': self.max_ack_lag,
            'feedback_age_sec': feedback_age,
            'feedback_expected_rate_hz': self.feedback_expected_rate,
            'feedback_frames': self.feedback_frames,
            'crc_errors': self.parser.crc_errors,
            'decode_errors': self.decode_errors,
            'disconnects': self.disconnects,
            'fresh_zero_seq': 'none' if self.fresh_zero_seq is None else self.fresh_zero_seq,
            'fresh_zero_ack_baseline_seq': (
                'none'
                if self.fresh_zero_ack_baseline_seq is None
                else self.fresh_zero_ack_baseline_seq
            ),
            'zero_sent_seq': 'none' if self.zero_sent_seq is None else self.zero_sent_seq,
            'ready_zero_seq': 'none' if self.ready_zero_seq is None else self.ready_zero_seq,
            'fresh_zero_ack_timeout_sec': self.fresh_zero_ack_timeout,
            'fresh_zero_ack_total': self.fresh_zero_ack_total,
            'fresh_zero_ack_session': self.fresh_zero_ack_session,
            'fresh_zero_ack_collision_rejections_total': (
                self.fresh_zero_ack_collision_rejections_total
            ),
            'fresh_zero_ack_collision_rejections_session': (
                self.fresh_zero_ack_collision_rejections_session
            ),
            'fresh_zero_tx_attempt_seq': (
                'none'
                if self.fresh_zero_tx_attempt_seq is None
                else self.fresh_zero_tx_attempt_seq
            ),
            'fresh_zero_tx_attempt_flags': self.fresh_zero_tx_attempt_flags,
            'fresh_zero_tx_success_seq': (
                'none'
                if self.fresh_zero_tx_success_seq is None
                else self.fresh_zero_tx_success_seq
            ),
            'fresh_zero_tx_success_flags': self.fresh_zero_tx_success_flags,
            'fresh_zero_retry_after_source_stamp': (
                'none'
                if self._fresh_zero_retry_after_source_stamp is None
                else self._fresh_zero_retry_after_source_stamp
            ),
            'tx_attempts_total': self.tx_attempts_total,
            'tx_successes_total': self.tx_successes_total,
            'tx_attempts_session': self.tx_attempts_session,
            'tx_successes_session': self.tx_successes_session,
            'nonzero_tx_attempts_total': self.nonzero_tx_attempts_total,
            'nonzero_tx_successes_total': self.nonzero_tx_successes_total,
            'nonzero_tx_attempts_session': self.nonzero_tx_attempts_session,
            'nonzero_tx_successes_session': self.nonzero_tx_successes_session,
            'last_tx_attempt_kind': self.last_tx_attempt_kind,
            'last_tx_attempt_seq': (
                'none' if self.last_tx_attempt_seq is None else self.last_tx_attempt_seq
            ),
            'last_tx_attempt_flags': self.last_tx_attempt_flags,
            'last_tx_success_kind': self.last_tx_success_kind,
            'last_tx_success_seq': (
                'none' if self.last_tx_success_seq is None else self.last_tx_success_seq
            ),
            'last_tx_success_flags': self.last_tx_success_flags,
            'last_tx_success_was_nonzero_motion': (
                self.last_tx_success_was_nonzero_motion
            ),
            'fault_bits': self.last_fault_bits & 0xFFFFFFFF,
            'unknown_fault_bits': self.last_fault_bits & ~0x3F & 0xFFFFFFFF,
            'last_error': self.last_error or 'none',
            'control_delivery_state': self.control_delivery_state.value,
            'control_pending_flags': self.control_pending_flags,
            'control_inflight_flags': self.control_inflight_flags,
            'control_failed_flags': self.control_failed_flags,
            'control_inflight_seq': (
                'none' if self.control_inflight_seq is None else self.control_inflight_seq
            ),
            'control_attempts': self.control_attempts,
            'control_max_attempts': self.control_max_attempts,
            'control_last_acked_seq': (
                'none'
                if self.control_last_acked_seq is None
                else self.control_last_acked_seq
            ),
            'control_last_acked_flags': self.control_last_acked_flags,
            'control_last_error': self.control_last_error or 'none',
        }
        for kind in self._TX_KINDS:
            snapshot[f'{kind}_tx_attempts_total'] = (
                self._tx_attempts_by_kind_total[kind]
            )
            snapshot[f'{kind}_tx_successes_total'] = (
                self._tx_successes_by_kind_total[kind]
            )
            snapshot[f'{kind}_tx_attempts_session'] = (
                self._tx_attempts_by_kind_session[kind]
            )
            snapshot[f'{kind}_tx_successes_session'] = (
                self._tx_successes_by_kind_session[kind]
            )
        return snapshot


def _to_i16_mm_s(value: float) -> int:
    return max(-32768, min(32767, int(round(float(value) * 1000.0))))


def _sequence_lag(sent: int, acknowledged: int) -> int:
    delta = (int(sent) - int(acknowledged)) & 0xFFFFFFFF
    return delta if delta < 0x80000000 else 0


def _default_serial_factory(**kwargs) -> SerialTransport:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError('python3-serial is required for base_backend:=serial') from exc
    return serial.Serial(**kwargs)
