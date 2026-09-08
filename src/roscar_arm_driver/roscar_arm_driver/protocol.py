ACTION_COMMANDS = {
    "initialize": b"$DGS:1!",
    "front_detect": b"$DGS:2!",
    "pickup_right": b"$DGT:3-7,1!",
    "pickup_left": b"$DGT:8-13,1!",
    "stop": b"$DST!",
}

MIN_PULSE_US = 500
MAX_PULSE_US = 2500
MIN_DURATION_MS = 100
MAX_DURATION_MS = 9999


def command_for(action: str) -> bytes:
    try:
        return ACTION_COMMANDS[action]
    except KeyError as exc:
        raise ValueError(f"unknown arm action: {action}") from exc


def stored_group_command(start: int, end: int, repeat: int = 1) -> bytes:
    if not all(isinstance(value, int) and not isinstance(value, bool)
               for value in (start, end, repeat)):
        raise ValueError("group indexes and repeat must be integers")
    if not 0 <= start <= end <= 999:
        raise ValueError("group range must satisfy 0 <= start <= end <= 999")
    if not 1 <= repeat <= 99:
        raise ValueError("repeat must be between 1 and 99")
    if start == end:
        return f"$DGS:{start}!".encode("ascii")
    return f"$DGT:{start}-{end},{repeat}!".encode("ascii")


def pose_command(pulses, duration_ms: int) -> bytes:
    values = tuple(pulses)
    if len(values) != 6:
        raise ValueError("a pose must contain exactly 6 servo pulses")
    if not isinstance(duration_ms, int) or isinstance(duration_ms, bool):
        raise ValueError("duration_ms must be an integer")
    if not MIN_DURATION_MS <= duration_ms <= MAX_DURATION_MS:
        raise ValueError("duration_ms must be between 100 and 9999")
    if not all(isinstance(value, int) and not isinstance(value, bool)
               and MIN_PULSE_US <= value <= MAX_PULSE_US for value in values):
        raise ValueError("servo pulses must be integers between 500 and 2500")
    return b"".join(
        f"#{channel:03d}P{pulse:04d}T{duration_ms:04d}!".encode("ascii")
        for channel, pulse in enumerate(values)
    )


def validate_sequence(frames):
    values = tuple(frames)
    if not 1 <= len(values) <= 100:
        raise ValueError("a sequence must contain between 1 and 100 poses")
    clean = []
    for frame in values:
        if not isinstance(frame, dict):
            raise ValueError("each sequence pose must be an object")
        pulses = tuple(frame.get("pulses", ()))
        duration_ms = frame.get("duration_ms")
        pose_command(pulses, duration_ms)
        clean.append({"pulses": pulses, "duration_ms": duration_ms})
    return tuple(clean)
