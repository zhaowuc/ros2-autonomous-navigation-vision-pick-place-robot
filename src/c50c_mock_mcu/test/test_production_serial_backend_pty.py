"""Cross-implementation test over a real Linux pseudo-terminal byte stream."""

import os
import time

import pytest

from c50c_mock_mcu.mock_mcu import MockMcuConfig, MockMcuServer, PtyEndpoint
from roscar_base_interface.backends import (
    BackendCommand,
    SerialBackend,
    SerialLinkState,
)
from roscar_base_interface.protocol import CommandFlag, ControlState


pytestmark = pytest.mark.skipif(os.name != 'posix', reason='PTY requires POSIX')


def _drive_until(backend, predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    observed = []
    while time.monotonic() < deadline:
        observed.extend(backend.tick())
        if predicate():
            return observed
        time.sleep(0.005)
    raise AssertionError(
        f'timed out waiting for serial state; snapshot={backend.diagnostic_snapshot()}'
    )


def test_production_serial_backend_talks_to_independent_mock_over_pty():
    endpoint = PtyEndpoint.create()
    server = MockMcuServer(
        endpoint,
        config=MockMcuConfig(feedback_rate_hz=50.0),
    )
    server.start()
    backend = SerialBackend(
        serial_device=endpoint.slave_path,
        command_tx_rate=50.0,
        command_timeout=0.15,
        feedback_expected_rate=50.0,
        feedback_timeout=0.30,
        reconnect_interval=0.05,
    )
    try:
        _drive_until(
            backend,
            lambda: backend.state == SerialLinkState.WAIT_FRESH_ZERO,
        )
        zero_seq = 100
        backend.set_command(
            BackendCommand(zero_seq, 0.0, 0.0, 0.0, 0, time.monotonic())
        )
        _drive_until(backend, lambda: backend.state == SerialLinkState.READY)
        assert backend.ready_zero_seq == zero_seq

        motion_seq = 101
        backend.set_command(BackendCommand(
            motion_seq,
            0.10,
            0.02,
            0.10,
            int(CommandFlag.MOTION_ENABLE),
            time.monotonic(),
        ))
        feedbacks = _drive_until(
            backend,
            lambda: backend.last_ack_seq == motion_seq,
        )
        feedback = next(
            item for item in reversed(feedbacks)
            if item.payload.last_cmd_seq == motion_seq
        )
        assert feedback.payload.control_state == int(ControlState.RUNNING)
        assert feedback.payload.battery_mv == 24400
        assert any(abs(value) > 0 for value in feedback.payload.wheel_speeds_mm_s)
        assert server.decoder.stats.valid_frames > 0
        assert backend.parser.frames_ok > 0
    finally:
        backend.close()
        server.stop()
