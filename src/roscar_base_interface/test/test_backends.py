import pytest

from roscar_base_interface.backends import (
    BackendCommand,
    ControlDeliveryState,
    MockBackend,
    SerialBackend,
    SerialLinkState,
    SimBackend,
)
from roscar_base_interface.protocol import (
    COMMAND_MSG_TYPE,
    CommandFlag,
    ControlState,
    FeedbackPayload,
    FrameParser,
    decode_command,
    encode_feedback,
)


GEOMETRY = {
    'wheel_radius': 0.05,
    'wheel_spacing': 0.519720,
    'axle_spacing': 0.410,
}


class FakeSerial:
    def __init__(self, *, auto_feedback=True, **_kwargs):
        self.auto_feedback = auto_feedback
        self.rx = bytearray()
        self.writes = []
        self.command_parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
        self.closed = False
        self.fail_read = False
        self.fail_next_write = False

    @property
    def is_open(self):
        return not self.closed

    @property
    def in_waiting(self):
        if self.fail_read:
            raise OSError('mock disconnect')
        return len(self.rx)

    def read(self, size):
        result = bytes(self.rx[:size])
        del self.rx[:size]
        return result

    def write(self, data):
        self.writes.append(bytes(data))
        if self.fail_next_write:
            self.fail_next_write = False
            raise OSError('mock uncertain write failure')
        if self.auto_feedback:
            for frame in self.command_parser.feed(data):
                command = decode_command(frame)
                self.rx.extend(
                    encode_feedback(
                        FeedbackPayload(
                            command.seq,
                            (0, 0, 0, 0),
                            (0, 0, 0, 0),
                            int(ControlState.STOPPED),
                            0,
                            24200,
                        )
                    )
                )
        return len(data)

    def reset_input_buffer(self):
        self.rx.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        self.closed = True


def decode_writes(fake):
    parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
    return [decode_command(frame) for wire in fake.writes for frame in parser.feed(wire)]


def queue_feedback(fake, seq, *, faults=0):
    fake.rx.extend(
        encode_feedback(
            FeedbackPayload(
                seq,
                (0, 0, 0, 0),
                (0, 0, 0, 0),
                int(ControlState.STOPPED),
                faults,
                24200,
            )
        )
    )


def drive_ready(backend, fake, *, start=0.0, zero_seq=1):
    backend.tick(start)
    queue_feedback(fake, 0)
    backend.tick(start + 0.01)
    backend.set_command(
        BackendCommand(zero_seq, 0.0, 0.0, 0.0, 0, start + 0.02)
    )
    backend.tick(start + 0.02)
    backend.tick(start + 0.03)
    assert backend.state == SerialLinkState.READY


def drive_ready_manual(backend, fake, *, start=0.0, zero_seq=1):
    backend.tick(start)
    queue_feedback(fake, 0)
    backend.tick(start + 0.01)
    backend.set_command(
        BackendCommand(zero_seq, 0.0, 0.0, 0.0, 0, start + 0.02)
    )
    backend.tick(start + 0.02)
    queue_feedback(fake, zero_seq)
    backend.tick(start + 0.03)
    assert backend.state == SerialLinkState.READY


def test_sim_backend_exposes_command_and_command_driven_wheel_feedback():
    backend = SimBackend(**GEOMETRY)
    command = BackendCommand(7, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 1.0)
    backend.set_command(command)
    feedback = backend.tick(1.01)[0]
    assert backend.command_for_simulator == command
    assert feedback.payload.last_cmd_seq == 7
    assert feedback.wheel_speeds_m_s == pytest.approx((0.2, 0.2, 0.2, 0.2))
    assert feedback.payload.control_state == int(ControlState.RUNNING)


def test_sim_backend_never_requires_or_consumes_ground_truth():
    backend = SimBackend(**GEOMETRY)
    assert not hasattr(backend, 'set_ground_truth_twist')
    feedback = backend.tick(0.0)[0]
    assert feedback.wheel_speeds_m_s == (0.0, 0.0, 0.0, 0.0)
    assert backend.diagnostic_snapshot()['state'] == 'READY'


def test_mock_backend_exercises_v1_bytes_ack_battery_and_wheels():
    backend = MockBackend(**GEOMETRY)
    backend.set_command(
        BackendCommand(1, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.0)
    )
    first = backend.tick(0.0)[0]
    backend.set_command(
        BackendCommand(2, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.02)
    )
    second = backend.tick(0.02)[0]
    assert first.payload.last_cmd_seq == 1
    assert second.payload.last_cmd_seq == 2
    assert second.payload.battery_mv == 24100
    assert second.wheel_speeds_m_s == pytest.approx((0.2, 0.2, 0.2, 0.2))
    assert any(value > 0 for value in second.payload.encoders)
    assert backend.feedback_parser.frames_ok == 2


def test_mock_backend_rejects_corrupted_feedback_without_publish():
    backend = MockBackend(**GEOMETRY)
    backend.mcu.corrupt_next_feedback = True
    backend.set_command(BackendCommand(1, 0.0, 0.0, 0.0, 0, 0.0))
    assert backend.tick(0.0) == []
    assert backend.feedback_parser.crc_errors == 1


def test_serial_reconnect_requires_feedback_and_fresh_zero_no_replay():
    fake = FakeSerial()
    backend = SerialBackend(serial_factory=lambda **kwargs: fake, reconnect_interval=0.1)

    # A nonzero command that predates connection must never be replayed.
    backend.set_command(
        BackendCommand(1, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), -1.0)
    )
    backend.tick(0.0)
    assert backend.state == SerialLinkState.WAIT_FEEDBACK
    assert fake.writes == []

    # A nonzero command received while locked is discarded.  The transport is
    # strictly RX-first and may not write anything before valid feedback.
    backend.set_command(
        BackendCommand(2, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.005)
    )
    queue_feedback(fake, 0)
    feedback = backend.tick(0.01)
    assert feedback
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO
    assert fake.writes == []

    # Only a fresh zero transitions to READY; a later new nonzero can move.
    backend.set_command(BackendCommand(3, 0.0, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.02))
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO_ACK
    backend.tick(0.02)
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO_ACK
    assert backend.fresh_zero_seq == 3
    backend.tick(0.03)
    assert backend.state == SerialLinkState.READY
    assert backend.ready_zero_seq == 3
    backend.set_command(
        BackendCommand(4, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.04)
    )
    backend.tick(0.04)
    commands = decode_writes(fake)
    assert [item.seq for item in commands] == [3, 4]
    assert [item.vx_mm_s for item in commands] == [0, 200]


def test_serial_zero_seen_before_first_feedback_does_not_unlock():
    fake = FakeSerial()
    backend = SerialBackend(serial_factory=lambda **kwargs: fake)
    backend.tick(0.0)
    backend.set_command(
        BackendCommand(2, 0.0, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.005)
    )
    backend.tick(0.01)
    assert not backend.valid_feedback_seen
    assert backend.state == SerialLinkState.WAIT_FEEDBACK
    assert fake.writes == []
    queue_feedback(fake, 0)
    backend.tick(0.02)
    assert backend.valid_feedback_seen
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO
    assert backend.fresh_zero_seq is None
    backend.tick(0.03)
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO


def test_serial_initial_feedback_timeout_disconnects():
    fake = FakeSerial(auto_feedback=False)
    backend = SerialBackend(
        serial_factory=lambda **kwargs: fake,
        feedback_timeout=0.15,
        reconnect_interval=1.0,
    )
    backend.tick(1.0)
    assert backend.state == SerialLinkState.WAIT_FEEDBACK
    backend.tick(1.16)
    assert backend.state == SerialLinkState.DISCONNECTED
    assert fake.closed
    assert 'feedback timeout' in backend.last_error


def test_serial_read_error_disconnects_and_clears_motion():
    fake = FakeSerial()
    backend = SerialBackend(serial_factory=lambda **kwargs: fake, reconnect_interval=1.0)
    backend.tick(0.0)
    fake.fail_read = True
    backend.tick(0.01)
    assert backend.state == SerialLinkState.DISCONNECTED
    assert backend.command.is_zero


def test_serial_estop_propagates_while_reconnect_locked():
    fake = FakeSerial(auto_feedback=False)
    backend = SerialBackend(serial_factory=lambda **kwargs: fake)
    backend.tick(0.0)
    backend.set_command(
        BackendCommand(2, 0.0, 0.0, 0.0, int(CommandFlag.ESTOP), 0.01)
    )
    backend.tick(0.01)
    command = decode_writes(fake)[-1]
    assert command.vx_mm_s == 0
    assert command.flags == int(CommandFlag.ESTOP)


def test_serial_one_shot_requires_explicit_retry_after_full_interlock():
    fake = FakeSerial()
    backend = SerialBackend(
        serial_factory=lambda **kwargs: fake,
        control_retry_interval=0.01,
    )
    backend.tick(0.0)
    flags = int(CommandFlag.REARM | CommandFlag.CLEAR_FAULT)
    backend.set_command(BackendCommand(2, 0.2, 0.0, 0.0, flags, 0.01))
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert backend.control_failed_flags == flags
    queue_feedback(fake, 0)
    backend.tick(0.01)
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO
    assert fake.writes == []

    backend.set_command(BackendCommand(3, 0.0, 0.0, 0.0, 0, 0.02))
    backend.tick(0.02)
    backend.tick(0.03)
    assert backend.state == SerialLinkState.READY
    assert all(command.flags == 0 for command in decode_writes(fake))

    # A deliberate new request is required after READY; the pre-READY request
    # is never replayed.
    backend.set_command(BackendCommand(4, 0.2, 0.0, 0.0, flags, 0.04))
    backend.tick(0.04)
    command = decode_writes(fake)[-1]
    assert command.vx_mm_s == 0
    assert command.flags == flags
    assert backend.control_delivery_state == ControlDeliveryState.INFLIGHT
    backend.tick(0.05)
    assert backend.control_delivery_state == ControlDeliveryState.ACKED
    assert backend.control_pending_flags == 0
    assert backend.control_inflight_flags == 0


def test_serial_control_requested_while_disconnected_requires_retry_after_ready():
    fake = FakeSerial()
    factory_calls = 0

    def factory(**_kwargs):
        nonlocal factory_calls
        factory_calls += 1
        if factory_calls == 1:
            raise OSError('port absent')
        return fake

    backend = SerialBackend(
        serial_factory=factory,
        reconnect_interval=0.10,
        control_retry_interval=0.01,
    )
    flags = int(CommandFlag.REARM)
    backend.set_command(BackendCommand(1, 0.2, 0.0, 0.0, flags, 0.0))
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert backend.control_failed_flags == flags
    backend.tick(0.0)
    backend.set_command(BackendCommand(2, 0.2, 0.0, 0.0, 0, 0.05))
    backend.tick(0.05)
    assert backend.control_pending_flags == 0
    assert fake.writes == []

    backend.tick(0.10)
    queue_feedback(fake, 0)
    backend.tick(0.11)
    backend.set_command(BackendCommand(3, 0.0, 0.0, 0.0, 0, 0.12))
    backend.tick(0.12)
    backend.tick(0.13)
    assert backend.state == SerialLinkState.READY

    backend.set_command(BackendCommand(4, 0.0, 0.0, 0.0, 0, 0.14))
    backend.tick(0.14)
    assert all(command.flags & flags == 0 for command in decode_writes(fake))

    backend.set_command(BackendCommand(5, 0.2, 0.0, 0.0, flags, 0.15))
    backend.tick(0.15)
    delivered = decode_writes(fake)[-1]
    assert delivered.seq == 5
    assert delivered.vx_mm_s == 0
    assert delivered.flags == flags
    backend.tick(0.16)
    assert backend.control_delivery_state == ControlDeliveryState.ACKED


def test_serial_uncertain_control_write_fails_and_is_not_replayed_after_reconnect():
    first = FakeSerial()
    second = FakeSerial()
    transports = iter((first, second))
    backend = SerialBackend(
        serial_factory=lambda **_kwargs: next(transports),
        reconnect_interval=0.10,
        control_retry_interval=0.01,
    )
    drive_ready(backend, first)

    first.fail_next_write = True
    backend.set_command(
        BackendCommand(10, 0.2, 0.0, 0.0, int(CommandFlag.CLEAR_FAULT), 0.04)
    )
    backend.tick(0.04)
    assert backend.state == SerialLinkState.DISCONNECTED
    assert backend.control_inflight_seq is None
    assert backend.control_failed_flags == int(CommandFlag.CLEAR_FAULT)
    assert backend.control_attempts == 1

    # A newer nonzero command received while disconnected is reduced to zero
    # and can neither replace the control transaction nor replay after connect.
    backend.set_command(
        BackendCommand(11, 0.2, 0.0, 0.0, int(CommandFlag.MOTION_ENABLE), 0.05)
    )
    backend.tick(0.14)
    assert second.writes == []
    queue_feedback(second, 0)
    backend.tick(0.15)
    backend.set_command(BackendCommand(12, 0.0, 0.0, 0.0, 0, 0.16))
    backend.tick(0.16)
    backend.tick(0.17)
    assert backend.state == SerialLinkState.READY

    backend.set_command(BackendCommand(13, 0.0, 0.0, 0.0, 0, 0.18))
    backend.tick(0.18)
    assert all(
        command.flags & int(CommandFlag.CLEAR_FAULT) == 0
        for command in decode_writes(second)
    )

    backend.set_command(
        BackendCommand(14, 0.2, 0.0, 0.0, int(CommandFlag.CLEAR_FAULT), 0.19)
    )
    backend.tick(0.19)
    assert decode_writes(second)[-1].flags == int(CommandFlag.CLEAR_FAULT)
    backend.tick(0.20)
    assert backend.control_delivery_state == ControlDeliveryState.ACKED


def test_serial_control_feedback_timeout_fails_and_never_cross_replays():
    first = FakeSerial(auto_feedback=False)
    second = FakeSerial()
    transports = iter((first, second))
    backend = SerialBackend(
        serial_factory=lambda **_kwargs: next(transports),
        feedback_timeout=0.15,
        reconnect_interval=0.10,
        control_retry_interval=0.01,
    )
    drive_ready_manual(backend, first)
    backend.set_command(
        BackendCommand(10, 0.0, 0.0, 0.0, int(CommandFlag.REARM), 0.04)
    )
    backend.tick(0.04)
    assert backend.control_inflight_seq == 10

    backend.tick(0.20)
    assert backend.state == SerialLinkState.DISCONNECTED
    assert backend.control_inflight_seq is None
    assert backend.control_failed_flags == int(CommandFlag.REARM)

    backend.set_command(BackendCommand(11, 0.2, 0.0, 0.0, 0, 0.21))
    backend.tick(0.301)
    assert second.writes == []
    queue_feedback(second, 0)
    backend.tick(0.31)
    backend.set_command(BackendCommand(12, 0.0, 0.0, 0.0, 0, 0.32))
    backend.tick(0.32)
    backend.tick(0.33)
    assert backend.state == SerialLinkState.READY
    assert all(
        command.flags & int(CommandFlag.REARM) == 0
        for command in decode_writes(second)
    )


def test_serial_control_flag_clears_only_on_exact_ack():
    fake = FakeSerial(auto_feedback=False)
    backend = SerialBackend(
        serial_factory=lambda **_kwargs: fake,
        control_retry_interval=0.05,
    )
    drive_ready_manual(backend, fake)
    backend.set_command(
        BackendCommand(2, 0.0, 0.0, 0.0, int(CommandFlag.REARM), 0.04)
    )
    backend.tick(0.04)
    assert backend.control_inflight_seq == 2

    queue_feedback(fake, 1)
    backend.set_command(BackendCommand(3, 0.0, 0.0, 0.0, 0, 0.05))
    backend.tick(0.05)
    assert backend.control_delivery_state == ControlDeliveryState.INFLIGHT
    assert backend.control_inflight_seq == 2

    queue_feedback(fake, 2)
    backend.tick(0.06)
    assert backend.control_delivery_state == ControlDeliveryState.ACKED
    assert backend.control_last_acked_seq == 2


def test_serial_control_no_ack_has_bounded_same_seq_retries_then_fails():
    fake = FakeSerial(auto_feedback=False)
    backend = SerialBackend(
        serial_factory=lambda **_kwargs: fake,
        control_max_attempts=3,
        control_retry_interval=0.01,
    )
    drive_ready_manual(backend, fake)
    flags = int(CommandFlag.REARM | CommandFlag.CLEAR_FAULT)
    backend.set_command(BackendCommand(2, 0.2, 0.0, 0.0, flags, 0.04))
    backend.tick(0.04)

    for stamp, seq in ((0.051, 3), (0.062, 4)):
        queue_feedback(fake, 1)
        backend.set_command(BackendCommand(seq, 0.2, 0.0, 0.0, 0, stamp))
        backend.tick(stamp)

    queue_feedback(fake, 1)
    backend.set_command(BackendCommand(5, 0.2, 0.0, 0.0, 0, 0.073))
    backend.tick(0.073)
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert backend.control_failed_flags == flags
    assert backend.control_attempts == 3

    flagged_wires = [
        wire
        for wire, command in zip(fake.writes, decode_writes(fake))
        if command.flags & flags
    ]
    assert len(flagged_wires) == 3
    assert len(set(flagged_wires)) == 1

    queue_feedback(fake, 5)
    backend.set_command(BackendCommand(6, 0.0, 0.0, 0.0, 0, 0.084))
    backend.tick(0.084)
    assert len([
        command for command in decode_writes(fake) if command.flags & flags
    ]) == 3


def test_serial_estop_preempts_control_ack_wait_and_remains_level_triggered():
    fake = FakeSerial(auto_feedback=False)
    backend = SerialBackend(
        serial_factory=lambda **_kwargs: fake,
        control_retry_interval=0.05,
    )
    drive_ready_manual(backend, fake)
    backend.set_command(
        BackendCommand(2, 0.0, 0.0, 0.0, int(CommandFlag.REARM), 0.04)
    )
    backend.tick(0.04)
    assert backend.control_delivery_state == ControlDeliveryState.INFLIGHT

    queue_feedback(fake, 1)
    backend.set_command(
        BackendCommand(3, 0.2, 0.0, 0.0, int(CommandFlag.ESTOP), 0.05)
    )
    backend.tick(0.05)
    backend.set_command(
        BackendCommand(4, 0.2, 0.0, 0.0, int(CommandFlag.ESTOP), 0.06)
    )
    backend.tick(0.06)

    commands = decode_writes(fake)
    assert [command.flags for command in commands[-2:]] == [
        int(CommandFlag.ESTOP),
        int(CommandFlag.ESTOP),
    ]
    assert all(command.vx_mm_s == 0 for command in commands[-2:])
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert 'ESTOP' in backend.control_last_error

    backend.set_command(BackendCommand(5, 0.0, 0.0, 0.0, 0, 0.07))
    backend.tick(0.07)
    assert decode_writes(fake)[-1].flags & int(CommandFlag.REARM) == 0

    # Release requires a new exact fresh-zero ACK before one-shot controls are
    # accepted again.
    queue_feedback(fake, 5)
    backend.tick(0.08)
    assert backend.state == SerialLinkState.READY
    backend.set_command(
        BackendCommand(6, 0.0, 0.0, 0.0, int(CommandFlag.REARM), 0.09)
    )
    backend.tick(0.09)
    assert decode_writes(fake)[-1].flags == int(CommandFlag.REARM)


def test_serial_estop_cancels_pending_rearm_during_interlock():
    fake = FakeSerial()
    backend = SerialBackend(serial_factory=lambda **_kwargs: fake)
    backend.tick(0.0)
    backend.set_command(
        BackendCommand(1, 0.0, 0.0, 0.0, int(CommandFlag.REARM), 0.005)
    )
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert backend.control_failed_flags & int(CommandFlag.REARM)
    backend.set_command(
        BackendCommand(2, 0.0, 0.0, 0.0, int(CommandFlag.ESTOP), 0.006)
    )
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert backend.control_pending_flags == 0
    assert backend.control_failed_flags & int(CommandFlag.REARM)
    backend.tick(0.01)

    backend.set_command(BackendCommand(3, 0.0, 0.0, 0.0, 0, 0.02))
    backend.tick(0.02)
    assert backend.state == SerialLinkState.WAIT_FRESH_ZERO
    backend.set_command(BackendCommand(4, 0.0, 0.0, 0.0, 0, 0.03))
    backend.tick(0.03)
    backend.tick(0.04)
    assert backend.state == SerialLinkState.READY
    backend.set_command(BackendCommand(5, 0.0, 0.0, 0.0, 0, 0.05))
    backend.tick(0.05)
    assert all(
        command.flags & int(CommandFlag.REARM) == 0
        for command in decode_writes(fake)[1:]
    )


def test_serial_ready_stale_velocity_cannot_suppress_estop():
    fake = FakeSerial()
    backend = SerialBackend(
        serial_factory=lambda **kwargs: fake,
        command_timeout=0.15,
    )
    drive_ready(backend, fake)

    backend.set_command(
        BackendCommand(3, 0.0, 0.0, 0.0, int(CommandFlag.ESTOP), 0.04)
    )
    # Keep feedback live while the command sample itself becomes stale.
    queue_feedback(fake, 1)
    backend.tick(0.20)
    command = decode_writes(fake)[-1]
    assert command.vx_mm_s == 0
    assert command.flags == int(CommandFlag.ESTOP)
