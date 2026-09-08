from c50c_mock_mcu.mock_mcu import MockMcuConfig, MockMcuState
from c50c_mock_mcu.protocol_codec import (
    CLEAR_FAULT,
    ESTOP_ACTIVE,
    REARM,
    Command,
    ControlState,
)


def control_command(seq, flags):
    return Command(seq, 0, 0, 0, flags)


def test_duplicate_one_shot_seq_is_reacked_without_reexecuting_effects():
    state = MockMcuState(MockMcuConfig())
    state.fault_bits = ESTOP_ACTIVE | 0x04
    command = control_command(41, REARM | CLEAR_FAULT)

    state.handle_command(command, 1.0)
    assert state.last_cmd_seq == 41
    assert state.rearm_effect_count == 1
    assert state.clear_fault_effect_count == 1
    assert state.fault_bits == 0

    # Reasserting ESTOP demonstrates that the duplicate REARM is not applied
    # a second time, while the frame is still accepted and ACKed again.
    state.fault_bits |= ESTOP_ACTIVE
    state.handle_command(command, 1.02)
    assert state.last_cmd_seq == 41
    assert state.rearm_effect_count == 1
    assert state.clear_fault_effect_count == 1
    assert state.fault_bits & ESTOP_ACTIVE
    assert state.control_state == ControlState.ESTOP


def test_reconnect_starts_a_new_one_shot_deduplication_window():
    state = MockMcuState(MockMcuConfig())
    command = control_command(77, CLEAR_FAULT)
    state.handle_command(command, 1.0)
    state.prepare_reconnect()
    state.handle_command(command, 2.0)

    assert state.last_cmd_seq == 77
    assert state.clear_fault_effect_count == 2
    assert [item.seq for item in state.received_commands] == [77, 77]


def test_new_sequence_executes_a_new_one_shot_transaction():
    state = MockMcuState(MockMcuConfig())
    state.handle_command(control_command(90, CLEAR_FAULT), 1.0)
    state.handle_command(control_command(91, CLEAR_FAULT), 1.02)

    assert state.last_cmd_seq == 91
    assert state.clear_fault_effect_count == 2
