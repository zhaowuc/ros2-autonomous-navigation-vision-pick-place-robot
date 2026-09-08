"""Pure diagnostic policy shared by runtime publication and unit tests."""

from dataclasses import dataclass
import math
from typing import Mapping


OK = 0
WARN = 1
ERROR = 2


@dataclass(frozen=True)
class DiagnosticAssessment:
    level: int
    message: str


def assess_backend_snapshot(
    snapshot: Mapping[str, object],
    *,
    ack_warn_lag: int = 5,
) -> DiagnosticAssessment:
    """Classify ACK lag, stale feedback, link state, CRC, and MCU faults."""

    state = str(snapshot.get('state', 'UNKNOWN'))
    reasons: list[str] = []
    level = OK

    if state == 'DISCONNECTED':
        level = ERROR
        reasons.append('DISCONNECTED')
    elif state not in ('READY', 'RUNNING', 'STOPPED'):
        level = max(level, WARN)
        reasons.append(state)

    fault_bits = _as_int(snapshot.get('fault_bits', 0))
    unknown_fault_bits = _as_int(snapshot.get('unknown_fault_bits', 0))
    if fault_bits:
        level = ERROR
        reasons.append(f'MCU_FAULT_0x{fault_bits:08X}')
    if unknown_fault_bits:
        reasons.append(f'UNKNOWN_FAULT_0x{unknown_fault_bits:08X}')

    ack_lag = _as_int(snapshot.get('ack_lag', 0))
    if ack_lag > int(ack_warn_lag):
        level = max(level, WARN)
        reasons.append(f'ACK_LAG={ack_lag}')

    crc_errors = _as_int(snapshot.get('crc_errors', 0))
    if crc_errors:
        level = max(level, WARN)
        reasons.append(f'CRC_ERRORS={crc_errors}')

    feedback_age = _as_float(snapshot.get('feedback_age_sec', 0.0))
    expected_rate = _as_float(snapshot.get('feedback_expected_rate_hz', 50.0))
    stale_threshold = max(0.05, 2.0 / max(1.0, expected_rate))
    if not math.isfinite(feedback_age) or feedback_age > stale_threshold:
        level = max(level, WARN)
        reasons.append('FEEDBACK_STALE')

    control_delivery = str(snapshot.get('control_delivery_state', 'IDLE'))
    if control_delivery == 'FAILED':
        level = max(level, ERROR)
        reasons.append('CONTROL_DELIVERY_FAILED')
    elif control_delivery in ('QUEUED', 'INFLIGHT'):
        level = max(level, WARN)
        reasons.append(f'CONTROL_{control_delivery}')

    return DiagnosticAssessment(level, 'OK' if not reasons else '; '.join(reasons))


def _as_int(value: object) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _as_float(value: object) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.inf
