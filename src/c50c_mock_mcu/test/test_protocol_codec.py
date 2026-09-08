from c50c_mock_mcu.protocol_codec import (
    CLEAR_FAULT,
    ESTOP,
    MOTION_ENABLE,
    REARM,
    COMMAND_PAYLOAD_LENGTH,
    FEEDBACK_PAYLOAD_LENGTH,
    Command,
    Feedback,
    FrameDecoder,
    crc16_ccitt,
    decode_command,
    decode_feedback,
    encode_command,
    encode_feedback,
)


def test_crc16_ccitt_false_check_value():
    assert crc16_ccitt(b'123456789') == 0x29B1


def test_payload_lengths_are_frozen():
    assert COMMAND_PAYLOAD_LENGTH == 12
    assert FEEDBACK_PAYLOAD_LENGTH == 51


def test_command_golden_vector():
    encoded = encode_command(
        Command(
            seq=0x12345678,
            vx_mm_s=200,
            vy_mm_s=-50,
            wz_mrad_s=300,
            flags=MOTION_ENABLE | ESTOP | REARM | CLEAR_FAULT,
        )
    )
    assert len(encoded) == 19
    assert encoded.hex() == 'aa5501010c78563412c800ceff2c010f005f51'
    frames = FrameDecoder().feed(encoded)
    assert [decode_command(frame) for frame in frames] == [
        Command(0x12345678, 200, -50, 300, 0x000F)
    ]


def test_feedback_golden_vector():
    feedback = Feedback(
        last_cmd_seq=0x10203040,
        encoders=(1, -2, 3, -4),
        wheel_speeds_mm_s=(100, -101, 102, -103),
        control_state=3,
        fault_bits=0x80000021,
        battery_mv=24400,
    )
    encoded = encode_feedback(feedback)
    assert len(encoded) == 58
    assert encoded.hex() == (
        'aa55018133403020100100000000000000feffffffffffffff'
        '0300000000000000fcffffffffffffff64009bff660099ff0321'
        '000080505f186e'
    )
    frames = FrameDecoder().feed(encoded)
    assert [decode_feedback(frame) for frame in frames] == [feedback]


def test_decoder_rejects_bad_length_without_waiting_for_garbage_payload():
    good = encode_command(Command(7, 0, 0, 0, 0))
    fake_header = b'\xaa\x55\x01\x01\xff\x7f'
    decoder = FrameDecoder()
    frames = decoder.feed(fake_header + good)
    assert [decode_command(frame).seq for frame in frames] == [7]
    assert decoder.stats.invalid_header >= 1
