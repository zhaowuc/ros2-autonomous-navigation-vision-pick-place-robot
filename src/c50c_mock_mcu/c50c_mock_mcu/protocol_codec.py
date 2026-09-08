"""Independent reference codec for C50C STM32/ROS 2 Protocol V1.

This module intentionally does not import the production serial driver.  The
mock therefore provides an independent peer for compatibility tests instead
of reproducing driver bugs through shared encode/decode code.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Iterable, List, Tuple


SYNC = b'\xAA\x55'
VERSION = 0x01
COMMAND_TYPE = 0x01
FEEDBACK_TYPE = 0x81

MOTION_ENABLE = 1 << 0
ESTOP = 1 << 1
REARM = 1 << 2
CLEAR_FAULT = 1 << 3

COMMAND_TIMEOUT = 1 << 0
ENCODER_FAULT = 1 << 1
UNDERVOLTAGE = 1 << 2
MOTOR_CONTROL_FAULT = 1 << 3
COMMUNICATION_FAULT = 1 << 4
ESTOP_ACTIVE = 1 << 5

COMMAND_STRUCT = struct.Struct('<IhhhH')
FEEDBACK_STRUCT = struct.Struct('<IqqqqhhhhBIH')
HEADER_STRUCT = struct.Struct('<BBB')
CRC_STRUCT = struct.Struct('<H')

COMMAND_PAYLOAD_LENGTH = COMMAND_STRUCT.size
FEEDBACK_PAYLOAD_LENGTH = FEEDBACK_STRUCT.size

_EXPECTED_LENGTHS = {
    COMMAND_TYPE: COMMAND_PAYLOAD_LENGTH,
    FEEDBACK_TYPE: FEEDBACK_PAYLOAD_LENGTH,
}


class ProtocolError(ValueError):
    """Raised when a complete frame violates the Protocol V1 contract."""


class ControlState(IntEnum):
    BOOT = 0
    LOCKED = 1
    READY = 2
    RUNNING = 3
    STOPPED = 4
    ESTOP = 5
    FAULT = 6


@dataclass(frozen=True)
class Command:
    seq: int
    vx_mm_s: int
    vy_mm_s: int
    wz_mrad_s: int
    flags: int


@dataclass(frozen=True)
class Feedback:
    last_cmd_seq: int
    encoders: Tuple[int, int, int, int]
    wheel_speeds_mm_s: Tuple[int, int, int, int]
    control_state: int
    fault_bits: int
    battery_mv: int


@dataclass(frozen=True)
class Frame:
    version: int
    msg_type: int
    payload: bytes


@dataclass
class DecoderStats:
    valid_frames: int = 0
    bad_crc: int = 0
    invalid_header: int = 0
    discarded_bytes: int = 0


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """Return CRC-16/CCITT-FALSE (poly 0x1021, no reflection/xorout)."""

    crc = initial & 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _require_range(name: str, value: int, minimum: int, maximum: int) -> int:
    converted = int(value)
    if converted < minimum or converted > maximum:
        raise ProtocolError(f'{name}={converted} is outside [{minimum}, {maximum}]')
    return converted


def _frame(msg_type: int, payload: bytes) -> bytes:
    if len(payload) > 0xFF:
        raise ProtocolError('Protocol V1 payload length must fit uint8')
    header_without_sync = HEADER_STRUCT.pack(VERSION, msg_type, len(payload))
    crc = crc16_ccitt(header_without_sync + payload)
    return SYNC + header_without_sync + payload + CRC_STRUCT.pack(crc)


def encode_command(command: Command) -> bytes:
    payload = COMMAND_STRUCT.pack(
        _require_range('seq', command.seq, 0, 0xFFFFFFFF),
        _require_range('vx_mm_s', command.vx_mm_s, -32768, 32767),
        _require_range('vy_mm_s', command.vy_mm_s, -32768, 32767),
        _require_range('wz_mrad_s', command.wz_mrad_s, -32768, 32767),
        _require_range('flags', command.flags, 0, 0xFFFF),
    )
    return _frame(COMMAND_TYPE, payload)


def encode_feedback(feedback: Feedback) -> bytes:
    if len(feedback.encoders) != 4:
        raise ProtocolError('feedback must contain exactly four encoders')
    if len(feedback.wheel_speeds_mm_s) != 4:
        raise ProtocolError('feedback must contain exactly four wheel speeds')
    payload = FEEDBACK_STRUCT.pack(
        _require_range('last_cmd_seq', feedback.last_cmd_seq, 0, 0xFFFFFFFF),
        *(
            _require_range(f'encoder[{index}]', value, -(1 << 63), (1 << 63) - 1)
            for index, value in enumerate(feedback.encoders)
        ),
        *(
            _require_range(f'wheel_speed[{index}]', value, -32768, 32767)
            for index, value in enumerate(feedback.wheel_speeds_mm_s)
        ),
        _require_range('control_state', feedback.control_state, 0, 0xFF),
        _require_range('fault_bits', feedback.fault_bits, 0, 0xFFFFFFFF),
        _require_range('battery_mv', feedback.battery_mv, 0, 0xFFFF),
    )
    return _frame(FEEDBACK_TYPE, payload)


def decode_command(frame: Frame) -> Command:
    if frame.version != VERSION or frame.msg_type != COMMAND_TYPE:
        raise ProtocolError('not a Protocol V1 command frame')
    if len(frame.payload) != COMMAND_PAYLOAD_LENGTH:
        raise ProtocolError('invalid command payload length')
    return Command(*COMMAND_STRUCT.unpack(frame.payload))


def decode_feedback(frame: Frame) -> Feedback:
    if frame.version != VERSION or frame.msg_type != FEEDBACK_TYPE:
        raise ProtocolError('not a Protocol V1 feedback frame')
    if len(frame.payload) != FEEDBACK_PAYLOAD_LENGTH:
        raise ProtocolError('invalid feedback payload length')
    values = FEEDBACK_STRUCT.unpack(frame.payload)
    return Feedback(
        last_cmd_seq=values[0],
        encoders=tuple(values[1:5]),
        wheel_speeds_mm_s=tuple(values[5:9]),
        control_state=values[9],
        fault_bits=values[10],
        battery_mv=values[11],
    )


class FrameDecoder:
    """Incremental parser resilient to fragmentation, concatenation and noise."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = DecoderStats()

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def clear(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> List[Frame]:
        if data:
            self._buffer.extend(data)
        frames: List[Frame] = []

        while True:
            sync_index = self._buffer.find(SYNC)
            if sync_index < 0:
                keep_prefix = bool(self._buffer and self._buffer[-1] == SYNC[0])
                discard = len(self._buffer) - (1 if keep_prefix else 0)
                if discard:
                    del self._buffer[:discard]
                    self.stats.discarded_bytes += discard
                break
            if sync_index:
                del self._buffer[:sync_index]
                self.stats.discarded_bytes += sync_index

            if len(self._buffer) < len(SYNC) + HEADER_STRUCT.size:
                break

            version, msg_type, payload_length = HEADER_STRUCT.unpack_from(
                self._buffer, len(SYNC)
            )
            expected_length = _EXPECTED_LENGTHS.get(msg_type)
            if (
                version != VERSION
                or expected_length is None
                or payload_length != expected_length
            ):
                del self._buffer[0]
                self.stats.invalid_header += 1
                self.stats.discarded_bytes += 1
                continue

            frame_length = (
                len(SYNC) + HEADER_STRUCT.size + payload_length + CRC_STRUCT.size
            )
            if len(self._buffer) < frame_length:
                break

            crc_offset = len(SYNC) + HEADER_STRUCT.size + payload_length
            received_crc = CRC_STRUCT.unpack_from(self._buffer, crc_offset)[0]
            covered = bytes(self._buffer[len(SYNC):crc_offset])
            expected_crc = crc16_ccitt(covered)
            if received_crc != expected_crc:
                del self._buffer[0]
                self.stats.bad_crc += 1
                self.stats.discarded_bytes += 1
                continue

            payload_start = len(SYNC) + HEADER_STRUCT.size
            payload = bytes(self._buffer[payload_start:crc_offset])
            frames.append(Frame(version, msg_type, payload))
            del self._buffer[:frame_length]
            self.stats.valid_frames += 1

        return frames


def decode_frames(chunks: Iterable[bytes]) -> List[Frame]:
    """Convenience helper for deterministic codec tests."""

    decoder = FrameDecoder()
    result: List[Frame] = []
    for chunk in chunks:
        result.extend(decoder.feed(chunk))
    return result
