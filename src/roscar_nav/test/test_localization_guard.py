import json
import math
from types import MethodType, SimpleNamespace

import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time

import roscar_nav.localization_guard as guard_module
from roscar_nav.localization_guard import (
    LocalizationGuard,
    covariance_is_healthy,
    is_auto_recoverable_drive_fault,
    load_home_point,
    planar_errors,
    target_error_in_body,
)


def test_load_home_point_and_planar_errors(tmp_path):
    points = tmp_path / 'points.yaml'
    points.write_text(yaml.safe_dump({
        'version': 2,
        'points': [{
            'name': '起点', 'x': 1.0, 'y': -2.0,
            'yaw': math.pi - 0.02, 'frame_id': 'map', 'type': 'target',
        }],
    }, allow_unicode=True), encoding='utf-8')
    home = load_home_point(points)
    assert home['frame_id'] == 'map'
    xy_error, yaw_error = planar_errors(
        {'x': 1.03, 'y': -2.04, 'yaw': -math.pi + 0.01}, home
    )
    assert abs(xy_error - 0.05) < 1.0e-9
    assert abs(yaw_error - 0.03) < 1.0e-9


def test_covariance_health_thresholds():
    covariance = [0.0] * 36
    covariance[0] = 0.02
    covariance[7] = 0.03
    covariance[35] = 0.04
    assert covariance_is_healthy(covariance, 0.10, 0.20)
    covariance[35] = 0.21
    assert not covariance_is_healthy(covariance, 0.10, 0.20)


def test_target_error_is_expressed_in_robot_body_frame():
    current = {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2.0}
    target = {'x': 1.1, 'y': 2.0, 'yaw': math.pi}
    body_x, body_y, body_yaw = target_error_in_body(current, target)
    assert abs(body_x) < 1.0e-9
    assert abs(body_y + 0.1) < 1.0e-9
    assert abs(body_yaw - math.pi / 2.0) < 1.0e-9


def test_only_safe_drive_faults_are_auto_recoverable():
    assert is_auto_recoverable_drive_fault(True, 6, 2, False)
    assert is_auto_recoverable_drive_fault(True, 6, 8, False)
    assert is_auto_recoverable_drive_fault(True, 6, 10, False)
    assert not is_auto_recoverable_drive_fault(True, 6, 4, False)
    assert not is_auto_recoverable_drive_fault(True, 6, 16, False)
    assert not is_auto_recoverable_drive_fault(True, 6, 8, True)
    assert not is_auto_recoverable_drive_fault(False, 6, 8, False)


def test_recoverable_base_fault_keeps_current_navigation_goal():
    canceled = []
    parameters = {
        'auto_recover_drive_fault': True,
        'base_fault_max_auto_recoveries': 3,
    }
    guard = SimpleNamespace(
        base_status=SimpleNamespace(fault_bits=8, control_state=6),
        base_fault_recovery_phase=None,
        base_fault_last_recovered_at=None,
        base_fault_recovery_attempts=0,
        backend_ready=True,
        estop_active=False,
        state='',
        detail='',
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: SimpleNamespace(warning=lambda _message: None),
        _publish_zero=lambda: None,
        _cancel_navigation=lambda: canceled.append(True),
    )
    assert LocalizationGuard._handle_base_fault(guard, 10.0)
    assert guard.base_fault_recovery_phase == 'SETTLING_ZERO'
    assert canceled == []


def make_guard():
    """Exercise the real timer/state transitions without starting ROS or a car."""
    parameters = {
        'startup_grace_sec': 8.0, 'lost_grace_sec': 2.5, 'stop_settle_sec': 1.0,
        'healthy_stable_sec': 3.0, 'relocalize_retry_sec': 8.0,
        'relocalize_timeout_sec': 20.0, 'relocalize_max_attempts': 3,
        'relocalize_budget_sec': 60.0, 'prior_max_age_sec': 10.0,
        'tf_max_age_sec': 1.5,
    }
    calls = []

    def cancel(request):
        calls.append('cancel')
        calls.append(('cancel_stamp', request.goal_info.stamp.sec, request.goal_info.stamp.nanosec))

    guard = SimpleNamespace(
        map_yaml='/map.yaml', state='LOCALIZATION_LOST', detail='',
        started_at=0.0, unhealthy_since=10.0, healthy_since=None,
        recovery_started_at=10.0, next_relocalize_at=0.0,
        relocalize_process=None, relocalize_attempts=0, relocalize_deadline=0.0,
        trusted_pose=None, trusted_pose_at=0.0, last_amcl_pose=None,
        latest_pose={'x': 1.0, 'y': 2.0, 'yaw': 0.25},
        base_status=SimpleNamespace(fault_bits=0, control_state=4),
        backend_ready=True, estop_active=False, base_fault_recovery_phase=None,
        auto_return_home=False, return_pending=False, goal_active=False,
        final_active=False, home_attempts=0, next_home_attempt_at=0.0,
        next_cancel_at=0.0, nav_active=False, nav_recovery_process=None,
        nav_state_future=None, next_nav_check_at=0.0,
        last_status_at=0.0, home={}, calls=calls, parameters=parameters,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=guard_module.time.monotonic())),
        get_logger=lambda: SimpleNamespace(
            warning=lambda _message: None, info=lambda _message: None,
            error=lambda _message: None,
        ),
        _publish_zero=lambda: calls.append('zero'),
        _set_goal_active=lambda active: calls.append(('active', active)),
        _localization_health=lambda: (False, 'no pose'),
        _handle_base_fault=lambda _now: False,
        _poll_nav_recovery=lambda: None,
        _check_nav_state=lambda _now: None,
        _start_nav_recovery=lambda: calls.append('nav_recovery'),
        status_pub=SimpleNamespace(publish=lambda message: calls.append(json.loads(message.data))),
        cancel_client=SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=cancel,
        ),
    )
    for name in (
        '_cancel_navigation', '_stop_relocalize', '_relocalize_budget_exhausted',
        '_start_relocalize', '_poll_relocalize', '_base_allows_motion',
        '_publish_status', '_tick', '_initial_pose',
    ):
        setattr(guard, name, MethodType(getattr(LocalizationGuard, name), guard))
    return guard


class WorkerProcess:
    def __init__(self):
        self.result = None
        self.killed = False

    def poll(self):
        return self.result

    def kill(self):
        self.killed = True
        self.result = -9


def test_new_goals_are_cancelled_while_localization_stays_lost(monkeypatch):
    guard = make_guard()
    now = [20.0]
    monkeypatch.setattr(guard_module.time, 'monotonic', lambda: now[0])
    guard._cancel_navigation()
    now[0] = 20.5
    guard.goal_active = True  # Another client submits a new goal during the loss.
    guard._cancel_navigation()
    now[0] = 21.0
    guard._cancel_navigation()
    assert guard.calls.count('cancel') == 2
    assert ('cancel_stamp', 20, 0) in guard.calls
    assert ('cancel_stamp', 21, 0) in guard.calls
    assert not guard.goal_active


def test_timed_out_worker_is_killed_and_retry_budget_keeps_zero(monkeypatch):
    guard = make_guard()
    worker = WorkerProcess()
    guard.relocalize_process = worker
    guard.relocalize_deadline = 20.0
    guard.relocalize_attempts = 3
    monkeypatch.setattr(guard_module.time, 'monotonic', lambda: 20.0)
    guard._poll_relocalize(20.0)
    assert worker.killed and guard.state == 'RELOCALIZE_TIMEOUT'
    guard._tick()  # Reap the killed worker and retain the exhausted state.
    assert guard.relocalize_process is None
    assert guard.state == 'RELOCALIZE_FAILED'
    assert 'zero' in guard.calls and 'cancel' in guard.calls
    assert not guard.calls[-1]['healthy']
    assert not guard.calls[-1]['ready_for_navigation']
    guard.relocalize_attempts = 0
    assert guard._relocalize_budget_exhausted(70.0)  # Wall time is independently bounded.


def test_recent_prior_is_used_once_then_failure_falls_back_to_global(monkeypatch):
    guard = make_guard()
    commands = []
    monkeypatch.setattr(
        guard_module.subprocess, 'Popen',
        lambda command: commands.append(command) or WorkerProcess(),
    )
    guard.trusted_pose = dict(guard.latest_pose)
    guard.trusted_pose_at = 15.0
    guard._start_relocalize(20.0)
    assert commands[-1][:3] == [guard_module.sys.executable, '-m', 'roscar_nav.auto_localize']
    assert 'prior_pose:=[1.0, 2.0, 0.25]' in commands[-1]
    assert guard.relocalize_deadline == 40.0
    guard.relocalize_process.result = 1
    guard._poll_relocalize(21.0)
    guard._start_relocalize(29.0)
    assert not any('prior_pose:=' in argument for argument in commands[-1])
    guard.relocalize_process.result = 1
    guard._poll_relocalize(30.0)
    guard.trusted_pose = dict(guard.latest_pose)
    guard.trusted_pose_at = 15.0
    guard._start_relocalize(60.0)
    assert not any('prior_pose:=' in argument for argument in commands[-1])
    assert guard.relocalize_deadline == 70.0  # The remaining total budget wins.


def test_stable_localization_repairs_nav_without_auto_home_and_clears_budget(monkeypatch):
    guard = make_guard()
    guard.relocalize_attempts = 3
    guard._localization_health = lambda: (True, 'healthy')
    now = [20.0]
    monkeypatch.setattr(guard_module.time, 'monotonic', lambda: now[0])
    guard._tick()
    assert 'nav_recovery' not in guard.calls
    assert guard.relocalize_attempts == 3
    now[0] = 23.0
    guard._tick()
    assert 'nav_recovery' in guard.calls
    assert guard.relocalize_attempts == 0 and guard.recovery_started_at is None
    assert guard.state == 'WAITING_NAV2' and not guard.auto_return_home
    guard.calls.clear()
    guard.nav_active = True
    now[0] = 24.0
    guard._tick()
    assert guard.state == 'HEALTHY' and guard.calls[-1]['ready_for_navigation']
    assert guard.trusted_pose == guard.latest_pose
    for state in (0, 1, 5, 6):
        guard.base_status.control_state = state
        assert not guard._base_allows_motion()
    guard.base_status.control_state = 4
    guard.estop_active = True
    assert not guard._base_allows_motion()


def test_valid_new_initial_pose_releases_failed_budget_without_declaring_health(monkeypatch):
    guard = make_guard()
    guard.relocalize_attempts = 3
    guard.state = 'RELOCALIZE_FAILED'
    monkeypatch.setattr(guard_module.time, 'monotonic', lambda: 20.0)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.header.stamp.sec = 20
    message.pose.pose.orientation.w = 1.0
    message.pose.pose.position.x = math.nan
    guard._initial_pose(message)
    assert guard.relocalize_attempts == 3
    message.pose.pose.position.x = 1.0
    guard._initial_pose(message)
    assert guard.relocalize_attempts == 0 and guard.healthy_since is None
    assert guard.latest_pose is None and guard.state == 'VERIFYING'
    assert guard.next_relocalize_at == 28.0


def test_inactive_lifecycle_is_detected_even_if_action_server_exists():
    guard = make_guard()
    guard.nav_state_client = SimpleNamespace(service_is_ready=lambda: False)
    guard.nav_state_future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(current_state=SimpleNamespace(id=2)),
    )
    guard.next_nav_check_at = 30.0
    guard.nav_active = True
    LocalizationGuard._check_nav_state(guard, 20.0)
    assert not guard.nav_active
