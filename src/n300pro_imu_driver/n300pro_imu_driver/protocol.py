"""Decoder for the HiPNUC HI91 binary protocol used by N300Pro."""

from dataclasses import dataclass
import struct
from typing import List, Tuple


HI91_TAG = 0x91
HI91_PAYLOAD_LENGTH = 76
MAX_PAYLOAD_LENGTH = 512


@dataclass(frozen=True)
class Hi91Data:
    """One validated HI91 sample in the device's native units."""

    main_status: int
    temperature_c: float
    pressure_pa: float
    device_time_ms: int
    acceleration_g: Tuple[float, float, float]
    angular_velocity_dps: Tuple[float, float, float]
    magnetic_field_ut: Tuple[float, float, float]
    euler_deg: Tuple[float, float, float]
    quaternion_wxyz: Tuple[float, float, float, float]


def crc16_ccitt(data: bytes, crc: int = 0) -> int:
    """Calculate the protocol CRC-16/CCITT (poly 0x1021, init 0)."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_hi91_payload(payload: bytes) -> Hi91Data:
    """Parse a validated, fixed-length HI91 payload."""
    if len(payload) != HI91_PAYLOAD_LENGTH:
        raise ValueError(f"HI91 payload length is {len(payload)}, expected 76")
    if payload[0] != HI91_TAG:
        raise ValueError(f"HI91 payload tag is 0x{payload[0]:02x}, expected 0x91")

    return Hi91Data(
        main_status=struct.unpack_from("<H", payload, 1)[0],
        temperature_c=float(struct.unpack_from("<b", payload, 3)[0]),
        pressure_pa=struct.unpack_from("<f", payload, 4)[0],
        device_time_ms=struct.unpack_from("<I", payload, 8)[0],
        acceleration_g=struct.unpack_from("<3f", payload, 12),
        angular_velocity_dps=struct.unpack_from("<3f", payload, 24),
        magnetic_field_ut=struct.unpack_from("<3f", payload, 36),
        euler_deg=struct.unpack_from("<3f", payload, 48),
        quaternion_wxyz=struct.unpack_from("<4f", payload, 60),
    )


class HiPNUCDecoder:
    """Incremental decoder with framing, length, CRC and tag validation."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.invalid_lengths = 0
        self.invalid_payloads = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset_buffer(self) -> None:
        """Discard a partial stream while preserving lifetime counters."""
        self.discarded_bytes += len(self._buffer)
        self._buffer.clear()

    def feed(self, data: bytes) -> List[Hi91Data]:
        if data:
            self._buffer.extend(data)
        decoded: List[Hi91Data] = []

        while True:
            marker = self._buffer.find(b"\x5a\xa5")
            if marker < 0:
                if self._buffer and self._buffer[-1] == 0x5A:
                    self.discarded_bytes += len(self._buffer) - 1
                    del self._buffer[:-1]
                else:
                    self.discarded_bytes += len(self._buffer)
                    self._buffer.clear()
                break

            if marker:
                self.discarded_bytes += marker
                del self._buffer[:marker]

            if len(self._buffer) < 6:
                break

            payload_length = int.from_bytes(self._buffer[2:4], "little")
            if payload_length < 1 or payload_length > MAX_PAYLOAD_LENGTH:
                self.invalid_lengths += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            frame_length = 6 + payload_length
            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            expected_crc = int.from_bytes(frame[4:6], "little")
            actual_crc = crc16_ccitt(frame[:4])
            actual_crc = crc16_ccitt(frame[6:], actual_crc)
            if actual_crc != expected_crc:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            del self._buffer[:frame_length]
            try:
                sample = parse_hi91_payload(frame[6:])
            except ValueError:
                self.invalid_payloads += 1
                continue

            self.frames_ok += 1
            decoded.append(sample)

        return decoded
