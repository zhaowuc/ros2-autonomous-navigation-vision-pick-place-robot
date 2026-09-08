import math

from roscar_base_interface.diagnostics import ERROR, OK, WARN, assess_backend_snapshot


def test_diagnostics_ok_for_fresh_ready_backend():
    result = assess_backend_snapshot({
        'state': 'READY',
        'ack_lag': 1,
        'feedback_age_sec': 0.02,
        'feedback_expected_rate_hz': 50.0,
        'crc_errors': 0,
        'fault_bits': 0,
    })
    assert result.level == OK
    assert result.message == 'OK'


def test_diagnostics_warns_for_ack_lag_crc_and_stale_feedback():
    result = assess_backend_snapshot({
        'state': 'READY',
        'ack_lag': 8,
        'feedback_age_sec': 0.08,
        'feedback_expected_rate_hz': 50.0,
        'crc_errors': 2,
        'fault_bits': 0,
    })
    assert result.level == WARN
    assert 'ACK_LAG=8' in result.message
    assert 'CRC_ERRORS=2' in result.message
    assert 'FEEDBACK_STALE' in result.message


def test_diagnostics_errors_on_disconnect_or_fault_and_keeps_unknown_bits():
    disconnected = assess_backend_snapshot({
        'state': 'DISCONNECTED',
        'feedback_age_sec': math.inf,
    })
    assert disconnected.level == ERROR
    fault = assess_backend_snapshot({
        'state': 'READY',
        'feedback_age_sec': 0.01,
        'fault_bits': 0x20000004,
        'unknown_fault_bits': 0x20000000,
    })
    assert fault.level == ERROR
    assert 'UNKNOWN_FAULT_0x20000000' in fault.message


def test_diagnostics_reports_control_delivery_progress_and_failure():
    inflight = assess_backend_snapshot({
        'state': 'READY',
        'feedback_age_sec': 0.01,
        'control_delivery_state': 'INFLIGHT',
    })
    assert inflight.level == WARN
    assert 'CONTROL_INFLIGHT' in inflight.message

    failed = assess_backend_snapshot({
        'state': 'READY',
        'feedback_age_sec': 0.01,
        'control_delivery_state': 'FAILED',
    })
    assert failed.level == ERROR
    assert 'CONTROL_DELIVERY_FAILED' in failed.message
