"""Reusable real-byte-stream harness for C50C Protocol V1 tests."""

from __future__ import annotations

import select
import socket
import time
from typing import Callable, List, Optional, Tuple

from c50c_mock_mcu.mock_mcu import (
    MockMcuConfig,
    MockMcuServer,
    MockMcuState,
    create_socketpair_server,
)
from c50c_mock_mcu.protocol_codec import (
    Command,
    Feedback,
    FrameDecoder,
    decode_feedback,
    encode_command,
)


FeedbackRecord = Tuple[float, Feedback]


class WireHarness:
    def __init__(
        self,
        config: Optional[MockMcuConfig] = None,
        state: Optional[MockMcuState] = None,
    ) -> None:
        self.config = config or MockMcuConfig()
        self.host, self.server = create_socketpair_server(self.config, state=state)
        self.decoder = FrameDecoder()
        self.connection_closed = False

    def close(self) -> None:
        try:
            self.host.close()
        finally:
            self.server.stop()

    def send_bytes(self, data: bytes) -> None:
        self.host.sendall(data)

    def send_command(
        self,
        seq: int,
        vx_mm_s: int = 0,
        vy_mm_s: int = 0,
        wz_mrad_s: int = 0,
        flags: int = 0,
    ) -> bytes:
        encoded = encode_command(
            Command(seq, vx_mm_s, vy_mm_s, wz_mrad_s, flags)
        )
        self.send_bytes(encoded)
        return encoded

    def receive_for(self, duration: float) -> List[FeedbackRecord]:
        deadline = time.monotonic() + duration
        records: List[FeedbackRecord] = []
        while time.monotonic() < deadline and not self.connection_closed:
            timeout = max(0.0, min(0.02, deadline - time.monotonic()))
            readable, _, _ = select.select([self.host], [], [], timeout)
            if not readable:
                continue
            try:
                data = self.host.recv(4096)
            except BlockingIOError:
                continue
            if not data:
                self.connection_closed = True
                break
            stamp = time.monotonic()
            for frame in self.decoder.feed(data):
                records.append((stamp, decode_feedback(frame)))
        return records

    def wait_feedback(
        self,
        predicate: Callable[[Feedback], bool],
        timeout: float = 1.0,
    ) -> Feedback:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _, feedback in self.receive_for(min(0.05, deadline - time.monotonic())):
                if predicate(feedback):
                    return feedback
        raise AssertionError('timed out waiting for matching feedback frame')

    def wait_closed(self, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.receive_for(min(0.05, deadline - time.monotonic()))
            if self.connection_closed:
                return True
        return False


def keepalive_exchange(
    harness: WireHarness,
    duration: float,
    first_seq: int = 1,
    vx_mm_s: int = 0,
    flags: int = 0,
    command_rate_hz: float = 50.0,
) -> List[FeedbackRecord]:
    start = time.monotonic()
    deadline = start + duration
    period = 1.0 / command_rate_hz
    next_command = start
    seq = first_seq
    records: List[FeedbackRecord] = []
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_command:
            harness.send_command(seq, vx_mm_s=vx_mm_s, flags=flags)
            seq += 1
            next_command += period
        records.extend(harness.receive_for(min(0.005, deadline - time.monotonic())))
    records.extend(harness.receive_for(0.04))
    return records
