"""Executable byte-stream mock for the C50C STM32 controller.

The default executable creates a Linux pseudo-terminal and publishes a stable
symlink (``/tmp/c50c_mock_mcu``).  It never opens the production serial device.
Tests use ``socket.socketpair`` so the same framing and stream parser are
exercised without ROS or hardware.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, replace
import errno
import json
import math
import os
import select
import socket
import sys
import threading
import time
from typing import Deque, List, Optional, Protocol, Tuple

from .protocol_codec import (
    CLEAR_FAULT,
    COMMAND_TIMEOUT,
    COMMUNICATION_FAULT,
    ESTOP,
    ESTOP_ACTIVE,
    MOTION_ENABLE,
    REARM,
    Command,
    ControlState,
    Feedback,
    FrameDecoder,
    decode_command,
    encode_feedback,
)


_ZERO_WHEELS = (0, 0, 0, 0)
_PROTECTED_DEVICE_PATHS = {'/dev/c50c_ros', '/dev/c50c_flash'}


class StreamEndpoint(Protocol):
    def fileno(self) -> int:
        ...

    def recv(self, size: int) -> bytes:
        ...

    def sendall(self, data: bytes) -> None:
        ...

    def close(self) -> None:
        ...


class SocketEndpoint:
    def __init__(self, stream_socket: socket.socket) -> None:
        self.socket = stream_socket

    def fileno(self) -> int:
        return self.socket.fileno()

    def recv(self, size: int) -> bytes:
        return self.socket.recv(size)

    def sendall(self, data: bytes) -> None:
        self.socket.sendall(data)

    def close(self) -> None:
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()


class PtyEndpoint:
    def __init__(self, master_fd: int, slave_fd: int, slave_path: str) -> None:
        self.master_fd = master_fd
        self.slave_fd = slave_fd
        self.slave_path = slave_path

    @classmethod
    def create(cls) -> 'PtyEndpoint':
        if os.name != 'posix':
            raise RuntimeError('pseudo-terminal mode requires a POSIX host')
        import pty
        import tty

        master_fd, slave_fd = pty.openpty()
        tty.setraw(slave_fd)
        return cls(master_fd, slave_fd, os.ttyname(slave_fd))

    def fileno(self) -> int:
        return self.master_fd

    def recv(self, size: int) -> bytes:
        return os.read(self.master_fd, size)

    def sendall(self, data: bytes) -> None:
        offset = 0
        while offset < len(data):
            written = os.write(self.master_fd, data[offset:])
            if written <= 0:
                raise OSError('zero-byte PTY write')
            offset += written

    def close(self) -> None:
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


@dataclass
class MockMcuConfig:
    feedback_rate_hz: float = 50.0
    command_timeout_sec: float = 0.15
    wheel_radius_m: float = 0.050
    # (wheel_spacing 0.535 + axle_spacing 0.410) / 2
    wheel_lever_arm_m: float = 0.4725
    encoder_ticks_per_revolution: int = 4096
    battery_mv: int = 24400
    ack_delay_frames: int = 0
    bad_crc_every: int = 0
    drop_feedback_every: int = 0
    disconnect_after_feedback: int = 0
    disconnect_duration_sec: float = 0.25
    injected_fault_bits: int = 0
    force_estop: bool = False

    def validate(self) -> None:
        if self.feedback_rate_hz <= 0.0:
            raise ValueError('feedback_rate_hz must be positive')
        if self.command_timeout_sec <= 0.0:
            raise ValueError('command_timeout_sec must be positive')
        if self.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be positive')
        if self.encoder_ticks_per_revolution <= 0:
            raise ValueError('encoder_ticks_per_revolution must be positive')
        if not 0 <= self.battery_mv <= 0xFFFF:
            raise ValueError('battery_mv must fit uint16')
        for name in (
            'ack_delay_frames',
            'bad_crc_every',
            'drop_feedback_every',
            'disconnect_after_feedback',
        ):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} cannot be negative')


class MockMcuState:
    """Deterministic MCU state machine, driven only by decoded wire frames."""

    def __init__(self, config: MockMcuConfig) -> None:
        config.validate()
        self.config = config
        self.last_cmd_seq = 0
        self.encoders_float = [0.0, 0.0, 0.0, 0.0]
        self.wheel_speeds_mm_s = _ZERO_WHEELS
        self.control_state = ControlState.LOCKED
        self.fault_bits = int(config.injected_fault_bits)
        self.last_command_time: Optional[float] = None
        self.received_commands: List[Command] = []
        self._pending_acks: Deque[Tuple[int, int]] = deque()
        # Within one verified link, the host may retry a one-shot transaction
        # with the exact same seq when its ACK was lost.  Duplicates are ACKed
        # again but their side effects are not re-executed.
        self.last_one_shot_seq: Optional[int] = None
        self.rearm_effect_count = 0
        self.clear_fault_effect_count = 0
        self._connection_locked = True
        self._lock = threading.Lock()

    def prepare_reconnect(self) -> None:
        """Enter the safe state required for a newly opened host connection."""

        with self._lock:
            self._connection_locked = True
            self.wheel_speeds_mm_s = _ZERO_WHEELS
            self.control_state = ControlState.LOCKED
            self.last_command_time = None
            self._pending_acks.clear()
            # The host contract forbids cross-connection one-shot replay and
            # requires a new explicit request after the full reconnect
            # interlock, so duplicate suppression need not persist sessions.
            self.last_one_shot_seq = None

    @staticmethod
    def _is_zero(command: Command) -> bool:
        return (
            command.vx_mm_s == 0
            and command.vy_mm_s == 0
            and command.wz_mrad_s == 0
        )

    def _set_ack(self, seq: int) -> None:
        delay = int(self.config.ack_delay_frames)
        if delay == 0:
            self.last_cmd_seq = seq
        else:
            self._pending_acks.append((seq, delay))

    def _target_wheels(self, command: Command) -> Tuple[int, int, int, int]:
        rotational_mm_s = self.config.wheel_lever_arm_m * command.wz_mrad_s
        fl = command.vx_mm_s - command.vy_mm_s - rotational_mm_s
        fr = command.vx_mm_s + command.vy_mm_s + rotational_mm_s
        rl = command.vx_mm_s + command.vy_mm_s - rotational_mm_s
        rr = command.vx_mm_s - command.vy_mm_s + rotational_mm_s
        return tuple(
            max(-32768, min(32767, int(round(value))))
            for value in (fl, fr, rl, rr)
        )

    def handle_command(self, command: Command, now: float) -> None:
        with self._lock:
            self.received_commands.append(command)
            self._set_ack(command.seq)
            self.last_command_time = now
            self.fault_bits &= ~COMMAND_TIMEOUT

            one_shot_flags = command.flags & (REARM | CLEAR_FAULT)
            execute_one_shot = bool(one_shot_flags) and (
                command.seq != self.last_one_shot_seq
            )
            if execute_one_shot:
                self.last_one_shot_seq = command.seq

            if execute_one_shot and command.flags & CLEAR_FAULT:
                self.clear_fault_effect_count += 1
                self.fault_bits &= ESTOP_ACTIVE
            self.fault_bits |= int(self.config.injected_fault_bits)

            if execute_one_shot and command.flags & REARM:
                self.rearm_effect_count += 1

            if self.config.force_estop or command.flags & ESTOP:
                self.fault_bits |= ESTOP_ACTIVE
                self.wheel_speeds_mm_s = _ZERO_WHEELS
                self.control_state = ControlState.ESTOP
                return

            if self.fault_bits & ESTOP_ACTIVE:
                if (
                    execute_one_shot
                    and command.flags & REARM
                    and self._is_zero(command)
                ):
                    self.fault_bits &= ~ESTOP_ACTIVE
                else:
                    self.wheel_speeds_mm_s = _ZERO_WHEELS
                    self.control_state = ControlState.ESTOP
                    return

            non_timeout_faults = self.fault_bits & ~COMMAND_TIMEOUT
            if non_timeout_faults:
                self.wheel_speeds_mm_s = _ZERO_WHEELS
                self.control_state = ControlState.FAULT
                return

            if self._connection_locked:
                self.wheel_speeds_mm_s = _ZERO_WHEELS
                if self._is_zero(command):
                    self._connection_locked = False
                    self.control_state = ControlState.READY
                else:
                    self.control_state = ControlState.LOCKED
                return

            if command.flags & MOTION_ENABLE and not self._is_zero(command):
                self.wheel_speeds_mm_s = self._target_wheels(command)
                self.control_state = ControlState.RUNNING
            else:
                self.wheel_speeds_mm_s = _ZERO_WHEELS
                self.control_state = ControlState.STOPPED

    def tick(self, dt: float, now: float) -> None:
        with self._lock:
            if self._pending_acks:
                advanced: Deque[Tuple[int, int]] = deque()
                while self._pending_acks:
                    seq, remaining = self._pending_acks.popleft()
                    remaining -= 1
                    if remaining <= 0:
                        self.last_cmd_seq = seq
                    else:
                        advanced.append((seq, remaining))
                self._pending_acks = advanced

            if (
                self.last_command_time is not None
                and now - self.last_command_time > self.config.command_timeout_sec
            ):
                self.fault_bits |= COMMAND_TIMEOUT
                self.wheel_speeds_mm_s = _ZERO_WHEELS
                if self.control_state not in (ControlState.ESTOP, ControlState.FAULT):
                    self.control_state = ControlState.STOPPED

            if self.config.force_estop:
                self.fault_bits |= ESTOP_ACTIVE
                self.wheel_speeds_mm_s = _ZERO_WHEELS
                self.control_state = ControlState.ESTOP
            elif self.config.injected_fault_bits:
                self.fault_bits |= int(self.config.injected_fault_bits)
                if not self.fault_bits & ESTOP_ACTIVE:
                    self.wheel_speeds_mm_s = _ZERO_WHEELS
                    self.control_state = ControlState.FAULT

            circumference = 2.0 * math.pi * self.config.wheel_radius_m
            ticks_per_metre = self.config.encoder_ticks_per_revolution / circumference
            for index, speed_mm_s in enumerate(self.wheel_speeds_mm_s):
                self.encoders_float[index] += speed_mm_s / 1000.0 * dt * ticks_per_metre

    def feedback(self) -> Feedback:
        with self._lock:
            return Feedback(
                last_cmd_seq=self.last_cmd_seq,
                encoders=tuple(int(round(value)) for value in self.encoders_float),
                wheel_speeds_mm_s=tuple(self.wheel_speeds_mm_s),
                control_state=int(self.control_state),
                fault_bits=int(self.fault_bits),
                battery_mv=int(self.config.battery_mv),
            )


class MockMcuServer:
    """Serve a MockMcuState over a real ordered byte-stream endpoint."""

    def __init__(
        self,
        endpoint: StreamEndpoint,
        config: Optional[MockMcuConfig] = None,
        state: Optional[MockMcuState] = None,
    ) -> None:
        self.config = config or MockMcuConfig()
        self.config.validate()
        self.endpoint = endpoint
        self.state = state or MockMcuState(self.config)
        self.stop_event = threading.Event()
        self.disconnected_event = threading.Event()
        self.decoder = FrameDecoder()
        self.feedback_count = 0
        self.sent_feedback_count = 0
        self._thread: Optional[threading.Thread] = None

    def start(self) -> threading.Thread:
        if self._thread and self._thread.is_alive():
            raise RuntimeError('mock MCU server is already running')
        self._thread = threading.Thread(target=self.run, name='c50c-mock-mcu', daemon=True)
        self._thread.start()
        return self._thread

    def stop(self) -> None:
        self.stop_event.set()
        self.endpoint.close()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def _send_feedback(self) -> bool:
        self.feedback_count += 1
        if (
            self.config.disconnect_after_feedback
            and self.feedback_count >= self.config.disconnect_after_feedback
        ):
            self.disconnected_event.set()
            return False

        if (
            self.config.drop_feedback_every
            and self.feedback_count % self.config.drop_feedback_every == 0
        ):
            return True

        encoded = bytearray(encode_feedback(self.state.feedback()))
        if (
            self.config.bad_crc_every
            and self.feedback_count % self.config.bad_crc_every == 0
        ):
            encoded[-1] ^= 0x01
        self.endpoint.sendall(bytes(encoded))
        self.sent_feedback_count += 1
        return True

    def run(self) -> None:
        period = 1.0 / self.config.feedback_rate_hz
        last_tick = time.monotonic()
        next_feedback = last_tick + period
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                timeout = max(0.0, min(0.05, next_feedback - now))
                try:
                    readable, _, _ = select.select([self.endpoint.fileno()], [], [], timeout)
                except (OSError, ValueError):
                    break

                if readable:
                    try:
                        data = self.endpoint.recv(4096)
                    except OSError as exc:
                        if exc.errno in (errno.EIO, errno.EBADF):
                            break
                        raise
                    if not data:
                        break
                    for frame in self.decoder.feed(data):
                        try:
                            command = decode_command(frame)
                        except ValueError:
                            continue
                        self.state.handle_command(command, time.monotonic())

                now = time.monotonic()
                if now >= next_feedback:
                    dt = max(0.0, now - last_tick)
                    self.state.tick(dt, now)
                    last_tick = now
                    if not self._send_feedback():
                        break
                    next_feedback += period
                    if next_feedback <= now:
                        next_feedback = now + period
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.endpoint.close()
            self.disconnected_event.set()


def create_socketpair_server(
    config: Optional[MockMcuConfig] = None,
    state: Optional[MockMcuState] = None,
) -> Tuple[socket.socket, MockMcuServer]:
    """Return the host side plus a started mock server for integration tests."""

    host_socket, mcu_socket = socket.socketpair()
    host_socket.setblocking(False)
    server = MockMcuServer(SocketEndpoint(mcu_socket), config=config, state=state)
    server.start()
    return host_socket, server


def _replace_symlink(link_path: str, target: str) -> None:
    absolute_link = os.path.abspath(link_path)
    if absolute_link in _PROTECTED_DEVICE_PATHS or absolute_link.startswith('/dev/'):
        raise ValueError('mock PTY link must not be placed under /dev')
    parent = os.path.dirname(absolute_link)
    os.makedirs(parent, exist_ok=True)
    temporary = f'{absolute_link}.tmp.{os.getpid()}'
    try:
        os.unlink(temporary)
    except FileNotFoundError:
        pass
    os.symlink(target, temporary)
    os.replace(temporary, absolute_link)


def _remove_owned_symlink(link_path: str, expected_target: str) -> None:
    try:
        if os.path.islink(link_path) and os.readlink(link_path) == expected_target:
            os.unlink(link_path)
    except FileNotFoundError:
        pass


def run_pty_supervisor(
    config: MockMcuConfig,
    serial_link: str,
    repeat_disconnect: bool,
) -> None:
    state = MockMcuState(config)
    inject_disconnect = bool(config.disconnect_after_feedback)
    while True:
        state.prepare_reconnect()
        endpoint = PtyEndpoint.create()
        _replace_symlink(serial_link, endpoint.slave_path)
        print(
            json.dumps(
                {
                    'serial_link': os.path.abspath(serial_link),
                    'pty': endpoint.slave_path,
                    'feedback_rate_hz': config.feedback_rate_hz,
                    'state': 'RECONNECTED_LOCKED',
                },
                sort_keys=True,
            ),
            flush=True,
        )
        connection_config = replace(
            config,
            disconnect_after_feedback=(
                config.disconnect_after_feedback if inject_disconnect else 0
            ),
        )
        server = MockMcuServer(endpoint, config=connection_config, state=state)
        try:
            server.run()
        finally:
            _remove_owned_symlink(serial_link, endpoint.slave_path)

        if not connection_config.disconnect_after_feedback:
            return
        if not repeat_disconnect:
            inject_disconnect = False
        time.sleep(config.disconnect_duration_sec)


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial-link', default='/tmp/c50c_mock_mcu')
    parser.add_argument('--feedback-rate-hz', type=float, default=50.0)
    parser.add_argument('--command-timeout-sec', type=float, default=0.15)
    parser.add_argument('--battery-mv', type=int, default=24400)
    parser.add_argument('--ack-delay-frames', type=int, default=0)
    parser.add_argument('--bad-crc-every', type=int, default=0)
    parser.add_argument('--drop-feedback-every', type=int, default=0)
    parser.add_argument('--disconnect-after-feedback', type=int, default=0)
    parser.add_argument('--disconnect-duration-sec', type=float, default=0.25)
    parser.add_argument('--repeat-disconnect', action='store_true')
    parser.add_argument('--fault-bits', type=lambda value: int(value, 0), default=0)
    parser.add_argument('--force-estop', action='store_true')
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = _argument_parser().parse_args(argv)
    config = MockMcuConfig(
        feedback_rate_hz=args.feedback_rate_hz,
        command_timeout_sec=args.command_timeout_sec,
        battery_mv=args.battery_mv,
        ack_delay_frames=args.ack_delay_frames,
        bad_crc_every=args.bad_crc_every,
        drop_feedback_every=args.drop_feedback_every,
        disconnect_after_feedback=args.disconnect_after_feedback,
        disconnect_duration_sec=args.disconnect_duration_sec,
        injected_fault_bits=args.fault_bits,
        force_estop=args.force_estop,
    )
    try:
        run_pty_supervisor(config, args.serial_link, args.repeat_disconnect)
    except KeyboardInterrupt:
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f'c50c_mock_mcu: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
