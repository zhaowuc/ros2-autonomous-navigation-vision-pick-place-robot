import math

from n300pro_imu_driver.protocol import HiPNUCDecoder


SAMPLE = bytes.fromhex(
    "5A A5 4C 00 14 BB 91 08 15 23 09 A2 C4 47 08 15 1C 00 "
    "CC E8 61 BE 9A 35 56 3E 65 EA 72 3F 31 D0 7C BD 75 DD "
    "C5 BB 6B D7 24 BC 89 88 FC 40 01 00 6A 41 AB 2A 70 C2 "
    "96 D4 50 41 ED 03 43 41 41 F4 F4 C2 CC CA F8 BE 73 6A "
    "19 BE F0 00 1C 3D 8D 37 5C 3F"
)


def test_official_hi91_sample():
    decoder = HiPNUCDecoder()
    result = decoder.feed(SAMPLE)

    assert len(SAMPLE) == 82
    assert len(result) == 1
    sample = result[0]
    assert sample.main_status == 0x1508
    assert sample.temperature_c == 35.0
    assert sample.device_time_ms == 1840392
    assert math.isclose(sample.acceleration_g[0], -0.220615, abs_tol=1e-6)
    assert math.isclose(sample.euler_deg[2], -122.477, abs_tol=1e-3)
    assert math.isclose(sample.quaternion_wxyz[3], 0.860223, abs_tol=1e-6)
    assert decoder.crc_errors == 0


def test_fragmentation_and_resynchronization():
    decoder = HiPNUCDecoder()
    assert decoder.feed(b"garbage" + SAMPLE[:17]) == []
    result = decoder.feed(SAMPLE[17:] + SAMPLE)
    assert len(result) == 2
    assert decoder.frames_ok == 2
    assert decoder.discarded_bytes == len(b"garbage")


def test_bad_crc_is_rejected_without_losing_next_frame():
    corrupt = bytearray(SAMPLE)
    corrupt[30] ^= 0x01
    decoder = HiPNUCDecoder()
    result = decoder.feed(bytes(corrupt) + SAMPLE)
    assert len(result) == 1
    assert decoder.crc_errors == 1
    assert decoder.frames_ok == 1
