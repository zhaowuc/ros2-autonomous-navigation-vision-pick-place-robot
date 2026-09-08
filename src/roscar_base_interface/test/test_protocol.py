import struct

import pytest

from roscar_base_interface.protocol import (
    COMMAND_MSG_TYPE,
    FEEDBACK_MSG_TYPE,
    CommandFlag,
    CommandPayload,
    FaultBit,
    FeedbackPayload,
    FrameParser,
    command_from_si,
    crc16_ccitt_false,
    decode_command,
    decode_feedback,
    encode_command,
    encode_feedback,
    known_fault_names,
)


def test_crc16_ccitt_false_check_vector():
    assert crc16_ccitt_false(b'123456789') == 0x29B1


def test_command_round_trip_and_exact_layout():
    command = CommandPayload(
        seq=0x78563412,
        vx_mm_s=200,
        vy_mm_s=-200,
        wz_mrad_s=17,
        flags=int(CommandFlag.MOTION_ENABLE | CommandFlag.CLEAR_FAULT),
    )
    wire = encode_command(command)
    assert wire[:2] == b'\xAA\x55'
    assert len(wire) == 19
    assert wire[2:5] == b'\x01\x01\x0C'
    assert wire[5:17] == struct.pack('<IhhhH', 0x78563412, 200, -200, 17, 0x0009)
    assert wire.hex() == 'aa5501010c12345678c80038ff1100090065a7'
    parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
    assert decode_command(parser.feed(wire)[0]) == command


def test_si_command_clamps_translation_and_accepted_rotation_limits():
    command = command_from_si(9, 0.200280, -99.0, 0.8, int(CommandFlag.MOTION_ENABLE))
    assert (command.vx_mm_s, command.vy_mm_s, command.wz_mrad_s) == (200, -200, 450)


def test_feedback_round_trip_preserves_unknown_fault_bits():
    unknown = 1 << 29
    feedback = FeedbackPayload(
        last_cmd_seq=0xFFFFFFFF,
        encoders=(-9, 8, -7, 6),
        wheel_speeds_mm_s=(100, -100, 25, -25),
        control_state=199,
        fault_bits=int(FaultBit.UNDERVOLTAGE) | unknown,
        battery_mv=24123,
    )
    wire = encode_feedback(feedback)
    assert len(wire) == 58
    parser = FrameParser(accepted_types={FEEDBACK_MSG_TYPE})
    assert decode_feedback(parser.feed(wire)[0]) == feedback
    assert known_fault_names(feedback.fault_bits) == [
        'UNDERVOLTAGE',
        'UNKNOWN_0x20000000',
    ]


def test_feedback_golden_vector_is_58_byte_little_endian_v1():
    feedback = FeedbackPayload(
        last_cmd_seq=0x78563412,
        encoders=(-1, 2, -3, 4),
        wheel_speeds_mm_s=(100, -100, 25, -25),
        control_state=6,
        fault_bits=0x20000004,
        battery_mv=24123,
    )
    assert encode_feedback(feedback).hex() == (
        'aa5501813312345678ffffffffffffffff0200000000000000'
        'fdffffffffffffff040000000000000064009cff1900e7ff06'
        '040000203b5e23d1'
    )


def test_parser_reconstructs_every_split_boundary():
    wire = encode_command(CommandPayload(3, 1, 2, 3, 4))
    for split in range(len(wire)):
        parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
        assert parser.feed(wire[:split]) == []
        frames = parser.feed(wire[split:])
        assert len(frames) == 1
        assert decode_command(frames[0]).seq == 3


def test_parser_handles_multiple_frames_and_garbage_resync():
    first = encode_command(CommandPayload(10, 1, 2, 3, 0))
    second = encode_command(CommandPayload(11, 4, 5, 6, 0))
    parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
    frames = parser.feed(b'garbage\xAA\x00' + first + b'junk' + second)
    assert [decode_command(frame).seq for frame in frames] == [10, 11]
    assert parser.discarded_bytes >= len(b'garbage\xAA\x00junk')


def test_parser_rejects_bad_crc_then_finds_next_frame():
    damaged = bytearray(encode_command(CommandPayload(20, 1, 2, 3, 0)))
    damaged[-1] ^= 0xFF
    valid = encode_command(CommandPayload(21, 4, 5, 6, 0))
    parser = FrameParser(accepted_types={COMMAND_MSG_TYPE})
    frames = parser.feed(bytes(damaged) + valid)
    assert [decode_command(frame).seq for frame in frames] == [21]
    assert parser.crc_errors == 1


def test_parser_rejects_absurd_length_and_recovers():
    bad_header = b'\xAA\x55\x01\x81\xFF'
    valid = encode_feedback(
        FeedbackPayload(1, (0, 0, 0, 0), (0, 0, 0, 0), 2, 0, 24000)
    )
    parser = FrameParser(accepted_types={FEEDBACK_MSG_TYPE}, max_payload_size=64)
    frames = parser.feed(bad_header + valid)
    assert len(frames) == 1
    assert parser.length_errors == 1


def test_parser_rejects_known_type_wrong_length_without_waiting_forever():
    truncated_bad = b'\xAA\x55\x01\x81\x20garbage'
    valid = encode_feedback(
        FeedbackPayload(2, (0, 0, 0, 0), (0, 0, 0, 0), 2, 0, 24000)
    )
    parser = FrameParser(accepted_types={FEEDBACK_MSG_TYPE})
    frames = parser.feed(truncated_bad + valid)
    assert [decode_feedback(frame).last_cmd_seq for frame in frames] == [2]
    assert parser.length_errors == 1


def test_parser_rejects_unknown_type_header_before_waiting_for_payload():
    false_sync = b'\xAA\x55\x01\x99\xFF'
    valid = encode_feedback(
        FeedbackPayload(3, (0, 0, 0, 0), (0, 0, 0, 0), 2, 0, 24000)
    )
    parser = FrameParser(accepted_types={FEEDBACK_MSG_TYPE})
    frames = parser.feed(false_sync + valid)
    assert [decode_feedback(frame).last_cmd_seq for frame in frames] == [3]
    assert parser.type_errors == 1


def test_decode_refuses_trailing_or_wrong_type_data():
    command = encode_command(CommandPayload(1, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        decode_command(command + b'junk')
    with pytest.raises(ValueError):
        decode_feedback(command)
