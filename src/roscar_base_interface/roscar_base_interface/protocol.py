"""C50C STM32/ROS 2 Protocol V1 codec and incremental stream parser.

The codec in this module has no ROS or serial dependency.  Both the production
serial backend and the byte-loopback mock use it, so tests exercise the exact
wire representation that will be used by the MCU integration.
"""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
import struct
from typing import Iterable, List, Optional, Set


SYNC = b'\xAA\x55'
VERSION = 0x01
COMMAND_MSG_TYPE = 0x01
FEEDBACK_MSG_TYPE = 0x81
HEADER_STRUCT = struct.Struct('<2sBBB')
CRC_STRUCT = struct.Struct('<H')
COMMAND_STRUCT = struct.Struct('<IhhhH')
FEEDBACK_STRUCT = struct.Struct('<IqqqqhhhhBIH')
COMMAND_PAYLOAD_SIZE = COMMAND_STRUCT.size
FEEDBACK_PAYLOAD_SIZE = FEEDBACK_STRUCT.size
MAX_PAYLOAD_SIZE = 0xFF
EXPECTED_PAYLOAD_SIZES = {
    COMMAND_MSG_TYPE: COMMAND_PAYLOAD_SIZE,
    FEEDBACK_MSG_TYPE: FEEDBACK_PAYLOAD_SIZE,
}


class CommandFlag(IntFlag):
    """Command payload flags frozen by Protocol V1."""

    MOTION_ENABLE = 1 << 0
    ESTOP = 1 << 1
    REARM = 1 << 2
    CLEAR_FAULT = 1 << 3


class ControlState(IntEnum):
    BOOT = 0
    LOCKED = 1
    READY = 2
    RUNNING = 3
    STOPPED = 4
    ESTOP = 5
    FAULT = 6


class FaultBit(IntFlag):
    COMMAND_TIMEOUT = 1 << 0
    ENCODER_FAULT = 1 << 1
    UNDERVOLTAGE = 1 << 2
    MOTOR_CONTROL_FAULT = 1 << 3
    COMMUNICATION_FAULT = 1 << 4
    ESTOP_ACTIVE = 1 << 5


KNOWN_FAULT_MASK = int(
    FaultBit.COMMAND_TIMEOUT
    | FaultBit.ENCODER_FAULT
    | FaultBit.UNDERVOLTAGE
    | FaultBit.MOTOR_CONTROL_FAULT
    | FaultBit.COMMUNICATION_FAULT
    | FaultBit.ESTOP_ACTIVE
)


@dataclass(frozen=True)
class Frame:
    version: int
    msg_type: int
    payload: bytes


@dataclass(frozen=True)
class CommandPayload:
    seq: int
    vx_mm_s: int
    vy_mm_s: int
    wz_mrad_s: int
    flags: int


@dataclass(frozen=True)
class FeedbackPayload:
    last_cmd_seq: int
    encoders: tuple[int, int, int, int]
    wheel_speeds_mm_s: tuple[int, int, int, int]
    control_state: int
    fault_bits: int
    battery_mv: int


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    """Return CRC-16/CCITT-FALSE (poly=0x1021, init=0xffff)."""

    crc = 0xFFFF
    for value in bytes(data):
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def clamp(value: float, lower: float, upper: float) -> float:
    number = float(value)
    lower_bound = float(lower)
    upper_bound = float(upper)
    if not all(math.isfinite(item) for item in (number, lower_bound, upper_bound)):
        return 0.0
    if lower_bound > upper_bound:
        return 0.0
    return min(upper_bound, max(lower_bound, number))


def _validate_uint(name: str, value: int, bits: int) -> int:
    integer = int(value)
    if integer < 0 or integer > (1 << bits) - 1:
        raise ValueError(f'{name} must fit uint{bits}: {integer}')
    return integer


def _validate_int16(name: str, value: int) -> int:
    integer = int(value)
    if integer < -32768 or integer > 32767:
        raise ValueError(f'{name} must fit int16: {integer}')
    return integer


def encode_frame(msg_type: int, payload: bytes, *, version: int = VERSION) -> bytes:
    msg_type = _validate_uint('msg_type', msg_type, 8)
    version = _validate_uint('version', version, 8)
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f'payload too large: {len(payload)}')
    protected = struct.pack('<BBB', version, msg_type, len(payload)) + payload
    return SYNC + protected + CRC_STRUCT.pack(crc16_ccitt_false(protected))


def encode_command(command: CommandPayload) -> bytes:
    payload = COMMAND_STRUCT.pack(
        _validate_uint('seq', command.seq, 32),
        _validate_int16('vx_mm_s', command.vx_mm_s),
        _validate_int16('vy_mm_s', command.vy_mm_s),
        _validate_int16('wz_mrad_s', command.wz_mrad_s),
        _validate_uint('flags', command.flags, 16),
    )
    return encode_frame(COMMAND_MSG_TYPE, payload)


def command_from_si(
    seq: int,
    vx_m_s: float,
    vy_m_s: float,
    wz_rad_s: float,
    flags: int,
    *,
    max_vx: float = 0.20,
    max_vy: float = 0.20,
    max_wz: float = 0.45,
) -> CommandPayload:
    """Clamp production motion and scale SI units into the V1 payload.

    Any non-finite motion component or limit fails closed as a fully stopped,
    motion-disabled command.  This prevents NaN/Inf from turning into a clamp
    endpoint through Python's comparison semantics.
    """

    raw_motion = (float(vx_m_s), float(vy_m_s), float(wz_rad_s))
    limits = (float(max_vx), float(max_vy), float(max_wz))
    motion_valid = all(math.isfinite(item) for item in raw_motion + limits)
    wire_flags = int(flags) & 0xFFFF
    if motion_valid:
        vx = clamp(raw_motion[0], -abs(limits[0]), abs(limits[0]))
        vy = clamp(raw_motion[1], -abs(limits[1]), abs(limits[1]))
        wz = clamp(raw_motion[2], -abs(limits[2]), abs(limits[2]))
    else:
        vx = vy = wz = 0.0
        wire_flags &= ~int(CommandFlag.MOTION_ENABLE)
    return CommandPayload(
        seq=int(seq) & 0xFFFFFFFF,
        vx_mm_s=int(round(vx * 1000.0)),
        vy_mm_s=int(round(vy * 1000.0)),
        wz_mrad_s=int(round(wz * 1000.0)),
        flags=wire_flags,
    )


def decode_command(frame: Frame | bytes) -> CommandPayload:
    parsed = _coerce_single_frame(frame, COMMAND_MSG_TYPE)
    if len(parsed.payload) != COMMAND_PAYLOAD_SIZE:
        raise ValueError(f'command payload length {len(parsed.payload)} != {COMMAND_PAYLOAD_SIZE}')
    return CommandPayload(*COMMAND_STRUCT.unpack(parsed.payload))


def encode_feedback(feedback: FeedbackPayload) -> bytes:
    if len(feedback.encoders) != 4 or len(feedback.wheel_speeds_mm_s) != 4:
        raise ValueError('feedback requires exactly four encoders and four wheel speeds')
    payload = FEEDBACK_STRUCT.pack(
        _validate_uint('last_cmd_seq', feedback.last_cmd_seq, 32),
        *(int(value) for value in feedback.encoders),
        *(_validate_int16('wheel_speed_mm_s', value) for value in feedback.wheel_speeds_mm_s),
        _validate_uint('control_state', feedback.control_state, 8),
        _validate_uint('fault_bits', feedback.fault_bits, 32),
        _validate_uint('battery_mv', feedback.battery_mv, 16),
    )
    return encode_frame(FEEDBACK_MSG_TYPE, payload)


def decode_feedback(frame: Frame | bytes) -> FeedbackPayload:
    parsed = _coerce_single_frame(frame, FEEDBACK_MSG_TYPE)
    if len(parsed.payload) != FEEDBACK_PAYLOAD_SIZE:
        raise ValueError(f'feedback payload length {len(parsed.payload)} != {FEEDBACK_PAYLOAD_SIZE}')
    values = FEEDBACK_STRUCT.unpack(parsed.payload)
    return FeedbackPayload(
        last_cmd_seq=values[0],
        encoders=tuple(values[1:5]),
        wheel_speeds_mm_s=tuple(values[5:9]),
        control_state=values[9],
        fault_bits=values[10],
        battery_mv=values[11],
    )


def known_fault_names(fault_bits: int) -> List[str]:
    names = [bit.name for bit in FaultBit if int(fault_bits) & int(bit)]
    unknown = int(fault_bits) & ~KNOWN_FAULT_MASK & 0xFFFFFFFF
    if unknown:
        names.append(f'UNKNOWN_0x{unknown:08X}')
    return names


def _coerce_single_frame(frame: Frame | bytes, expected_type: int) -> Frame:
    if isinstance(frame, Frame):
        parsed = frame
    else:
        parser = FrameParser(accepted_types={expected_type})
        frames = parser.feed(frame)
        if len(frames) != 1 or parser.buffered_bytes or parser.discarded_bytes:
            raise ValueError('input must contain exactly one complete frame')
        parsed = frames[0]
    if parsed.version != VERSION:
        raise ValueError(f'unsupported protocol version: {parsed.version}')
    if parsed.msg_type != expected_type:
        raise ValueError(f'unexpected message type: 0x{parsed.msg_type:02X}')
    return parsed


class FrameParser:
    """Incrementally reconstruct V1 frames from an arbitrary byte stream."""

    def __init__(
        self,
        *,
        accepted_types: Optional[Iterable[int]] = None,
        max_payload_size: int = MAX_PAYLOAD_SIZE,
    ) -> None:
        self._buffer = bytearray()
        self.accepted_types: Optional[Set[int]] = (
            None if accepted_types is None else {int(item) & 0xFF for item in accepted_types}
        )
        self.max_payload_size = int(max_payload_size)
        self.frames_ok = 0
        self.crc_errors = 0
        self.version_errors = 0
        self.length_errors = 0
        self.type_errors = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes | bytearray | memoryview) -> List[Frame]:
        if data:
            self._buffer.extend(bytes(data))
        frames: List[Frame] = []
        minimum_size = HEADER_STRUCT.size + CRC_STRUCT.size

        while True:
            sync_index = self._buffer.find(SYNC)
            if sync_index < 0:
                keep = 1 if self._buffer.endswith(SYNC[:1]) else 0
                discard = len(self._buffer) - keep
                if discard > 0:
                    del self._buffer[:discard]
                    self.discarded_bytes += discard
                break
            if sync_index > 0:
                del self._buffer[:sync_index]
                self.discarded_bytes += sync_index
            if len(self._buffer) < minimum_size:
                break

            _, version, msg_type, payload_len = HEADER_STRUCT.unpack_from(self._buffer)
            if version != VERSION:
                self.version_errors += 1
                self._discard_one()
                continue
            if msg_type not in EXPECTED_PAYLOAD_SIZES:
                self.type_errors += 1
                self._discard_one()
                continue
            if self.accepted_types is not None and msg_type not in self.accepted_types:
                self.type_errors += 1
                self._discard_one()
                continue
            if payload_len > self.max_payload_size:
                self.length_errors += 1
                self._discard_one()
                continue
            expected_size = EXPECTED_PAYLOAD_SIZES.get(msg_type)
            if expected_size is not None and payload_len != expected_size:
                self.length_errors += 1
                self._discard_one()
                continue

            frame_size = HEADER_STRUCT.size + payload_len + CRC_STRUCT.size
            if len(self._buffer) < frame_size:
                break
            protected = bytes(self._buffer[2:HEADER_STRUCT.size + payload_len])
            received_crc = CRC_STRUCT.unpack_from(self._buffer, HEADER_STRUCT.size + payload_len)[0]
            if crc16_ccitt_false(protected) != received_crc:
                self.crc_errors += 1
                self._discard_one()
                continue

            payload = bytes(self._buffer[HEADER_STRUCT.size:HEADER_STRUCT.size + payload_len])
            del self._buffer[:frame_size]
            frames.append(Frame(version=version, msg_type=msg_type, payload=payload))
            self.frames_ok += 1

        return frames

    def _discard_one(self) -> None:
        if self._buffer:
            del self._buffer[0]
            self.discarded_bytes += 1
