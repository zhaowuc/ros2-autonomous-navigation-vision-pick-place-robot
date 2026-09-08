import math
import select
import socket
import time

import pytest

from c50c_mock_mcu.mock_mcu import MockMcuConfig, MockMcuState
from c50c_mock_mcu.protocol_codec import (
    COMMAND_TIMEOUT,
    ESTOP,
    ESTOP_ACTIVE,
    MOTION_ENABLE,
    ControlState,
    Command,
    encode_command,
)
from wire_harness import WireHarness, keepalive_exchange


@pytest.fixture
def harness():
    item = WireHarness(MockMcuConfig(command_timeout_sec=0.5))
    try:
        yield item
    finally:
        item.close()


def test_01_command_and_feedback_run_at_50_hz_over_byte_stream():
    item = WireHarness(MockMcuConfig(command_timeout_sec=0.5))
    try:
        started = time.monotonic()
        records = keepalive_exchange(item, duration=0.60, first_seq=1)
        elapsed = time.monotonic() - started
        measured_hz = len(records) / elapsed
        assert len(records) >= 25
        assert 40.0 <= measured_hz <= 60.0
        assert item.server.sent_feedback_count >= len(records)
    finally:
        item.close()


def test_02_command_sequence_is_strictly_increasing_on_the_wire(harness):
    for seq in range(10, 18):
        harness.send_command(seq)
    feedback = harness.wait_feedback(lambda value: value.last_cmd_seq == 17)
    assert feedback.last_cmd_seq == 17
    assert [command.seq for command in harness.server.state.received_commands] == list(
        range(10, 18)
    )


def test_03_feedback_ack_tracks_latest_command_after_injected_delay():
    item = WireHarness(MockMcuConfig(ack_delay_frames=3, command_timeout_sec=0.5))
    try:
        item.send_command(100)
        records = item.receive_for(0.16)
        ack_values = [feedback.last_cmd_seq for _, feedback in records]
        assert ack_values[:2] == [0, 0]
        assert 100 in ack_values[2:]
    finally:
        item.close()


def test_04_bad_crc_is_rejected_in_both_stream_directions(harness):
    damaged = bytearray(encode_command(Command(11, 0, 0, 0, 0)))
    damaged[-1] ^= 0x40
    harness.send_bytes(bytes(damaged))
    before = harness.receive_for(0.08)
    assert before
    assert all(feedback.last_cmd_seq == 0 for _, feedback in before)
    assert harness.server.decoder.stats.bad_crc == 1

    harness.send_command(12)
    assert harness.wait_feedback(lambda value: value.last_cmd_seq == 12)

    corrupting_peer = WireHarness(
        MockMcuConfig(bad_crc_every=1, command_timeout_sec=0.5)
    )
    try:
        corrupting_peer.send_command(13)
        assert corrupting_peer.receive_for(0.10) == []
        assert corrupting_peer.decoder.stats.bad_crc >= 3
    finally:
        corrupting_peer.close()


def test_05_partial_frame_is_reconstructed_from_real_stream(harness):
    encoded = encode_command(Command(21, 0, 0, 0, 0))
    offsets = (1, 3, 6, 11, len(encoded))
    start = 0
    for end in offsets:
        harness.send_bytes(encoded[start:end])
        start = end
        time.sleep(0.002)
    feedback = harness.wait_feedback(lambda value: value.last_cmd_seq == 21)
    assert feedback.control_state == ControlState.READY


def test_06_multiple_frames_in_one_read_are_all_processed(harness):
    combined = encode_command(Command(30, 0, 0, 0, 0)) + encode_command(
        Command(31, 120, 0, 0, MOTION_ENABLE)
    )
    harness.send_bytes(combined)
    feedback = harness.wait_feedback(lambda value: value.last_cmd_seq == 31)
    assert feedback.control_state == ControlState.RUNNING
    assert feedback.wheel_speeds_mm_s == (120, 120, 120, 120)


def test_07_garbage_and_false_sync_are_discarded_before_valid_frame(harness):
    garbage = b'\x00\xff\xaa\x00\xaa\x55\x01\x99\x12\x00garbage'
    harness.send_bytes(garbage + encode_command(Command(41, 0, 0, 0, 0)))
    assert harness.wait_feedback(lambda value: value.last_cmd_seq == 41)
    assert harness.server.decoder.stats.discarded_bytes >= len(garbage) - 1


def test_08_dropped_feedback_produces_a_detectable_host_timeout():
    item = WireHarness(
        MockMcuConfig(drop_feedback_every=1, command_timeout_sec=0.5)
    )
    try:
        item.send_command(50)
        assert item.receive_for(0.18) == []
        assert item.server.feedback_count >= 5
        assert item.server.sent_feedback_count == 0
    finally:
        item.close()


def test_09_injected_disconnect_is_a_real_stream_eof():
    item = WireHarness(
        MockMcuConfig(disconnect_after_feedback=3, command_timeout_sec=0.5)
    )
    try:
        item.send_command(60)
        assert item.wait_closed(timeout=0.5)
        assert item.server.disconnected_event.is_set()
    finally:
        item.close()


def test_10_reconnect_requires_a_fresh_zero_before_ready():
    config = MockMcuConfig(command_timeout_sec=0.5)
    state = MockMcuState(config)
    first = WireHarness(config, state=state)
    try:
        first.send_command(70)
        first.wait_feedback(lambda value: value.control_state == ControlState.READY)
    finally:
        first.close()

    state.prepare_reconnect()
    second = WireHarness(config, state=state)
    try:
        second.send_command(71, vx_mm_s=150, flags=MOTION_ENABLE)
        locked = second.wait_feedback(lambda value: value.last_cmd_seq == 71)
        assert locked.control_state == ControlState.LOCKED
        assert locked.wheel_speeds_mm_s == (0, 0, 0, 0)

        second.send_command(72)
        ready = second.wait_feedback(lambda value: value.last_cmd_seq == 72)
        assert ready.control_state == ControlState.READY
    finally:
        second.close()


def test_11_reconnect_never_replays_the_old_nonzero_command():
    config = MockMcuConfig(command_timeout_sec=0.5)
    state = MockMcuState(config)
    first = WireHarness(config, state=state)
    try:
        first.send_command(80)
        first.wait_feedback(lambda value: value.control_state == ControlState.READY)
        first.send_command(81, vx_mm_s=200, flags=MOTION_ENABLE)
        running = first.wait_feedback(lambda value: value.control_state == ControlState.RUNNING)
        assert running.wheel_speeds_mm_s == (200, 200, 200, 200)
    finally:
        first.close()

    state.prepare_reconnect()
    second = WireHarness(config, state=state)
    try:
        passive = second.wait_feedback(lambda value: True)
        assert passive.control_state == ControlState.LOCKED
        assert passive.wheel_speeds_mm_s == (0, 0, 0, 0)
        second.send_command(82)
        zero = second.wait_feedback(lambda value: value.last_cmd_seq == 82)
        assert zero.wheel_speeds_mm_s == (0, 0, 0, 0)
    finally:
        second.close()


def test_12_estop_flag_latches_estop_state_and_fault_bit(harness):
    harness.send_command(90)
    harness.wait_feedback(lambda value: value.control_state == ControlState.READY)
    harness.send_command(91, vx_mm_s=150, flags=MOTION_ENABLE | ESTOP)
    stopped = harness.wait_feedback(lambda value: value.last_cmd_seq == 91)
    assert stopped.control_state == ControlState.ESTOP
    assert stopped.fault_bits & ESTOP_ACTIVE
    assert stopped.wheel_speeds_mm_s == (0, 0, 0, 0)


def test_13_battery_millivolts_convert_to_voltage_without_fake_percentage():
    item = WireHarness(MockMcuConfig(battery_mv=24680, command_timeout_sec=0.5))
    try:
        item.send_command(100)
        feedback = item.wait_feedback(lambda value: value.last_cmd_seq == 100)
        battery_voltage = feedback.battery_mv / 1000.0
        assert feedback.battery_mv == 24680
        assert battery_voltage == pytest.approx(24.68)
    finally:
        item.close()


def test_14_all_four_wheel_feedback_fields_cross_the_wire(harness):
    harness.send_command(110)
    harness.wait_feedback(lambda value: value.control_state == ControlState.READY)
    harness.send_command(
        111,
        vx_mm_s=100,
        vy_mm_s=50,
        wz_mrad_s=200,
        flags=MOTION_ENABLE,
    )
    feedback = harness.wait_feedback(lambda value: value.last_cmd_seq == 111)
    assert feedback.wheel_speeds_mm_s == (-44, 244, 56, 144)
    assert len(feedback.encoders) == 4
    assert len(feedback.wheel_speeds_mm_s) == 4


def test_15_encoder_feedback_supports_forward_wheel_odometry():
    item = WireHarness(MockMcuConfig(command_timeout_sec=0.15))
    try:
        item.send_command(120)
        item.wait_feedback(lambda value: value.control_state == ControlState.READY)
        records = keepalive_exchange(
            item,
            duration=0.35,
            first_seq=121,
            vx_mm_s=200,
            flags=MOTION_ENABLE,
        )
        moving = [value for _, value in records if value.control_state == ControlState.RUNNING]
        assert moving
        final = moving[-1]
        mean_ticks = sum(final.encoders) / 4.0
        metres_per_tick = 2.0 * math.pi * 0.050 / 4096
        odom_x = mean_ticks * metres_per_tick
        assert 0.045 <= odom_x <= 0.085
        assert max(final.encoders) - min(final.encoders) <= 1
    finally:
        item.close()


def test_fault_bits_include_unknown_injected_bits_without_crash():
    unknown_fault = 1 << 31
    item = WireHarness(
        MockMcuConfig(injected_fault_bits=unknown_fault, command_timeout_sec=0.5)
    )
    try:
        item.send_command(130)
        feedback = item.wait_feedback(lambda value: value.last_cmd_seq == 130)
        assert feedback.control_state == ControlState.FAULT
        assert feedback.fault_bits & unknown_fault
    finally:
        item.close()


def test_command_timeout_stops_motion_and_sets_fault_bit(harness):
    harness.send_command(140)
    harness.wait_feedback(lambda value: value.control_state == ControlState.READY)
    harness.send_command(141, vx_mm_s=100, flags=MOTION_ENABLE)
    harness.wait_feedback(lambda value: value.control_state == ControlState.RUNNING)
    feedback = harness.wait_feedback(
        lambda value: bool(value.fault_bits & COMMAND_TIMEOUT), timeout=0.8
    )
    assert feedback.control_state == ControlState.STOPPED
    assert feedback.wheel_speeds_mm_s == (0, 0, 0, 0)
