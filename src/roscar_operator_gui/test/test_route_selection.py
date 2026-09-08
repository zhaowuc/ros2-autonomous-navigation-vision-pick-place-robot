import json
import time
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import patch

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time

from roscar_operator_gui.ros_worker import (
    RosWorker,
    build_soft_waypoint_route,
    should_resume_after_base_fault,
)


START = {'name': '起点', 'x': 0.119, 'y': -0.156, 'yaw': 0.0}
WAYPOINT_1 = {'name': '导航点1', 'x': 1.502, 'y': -0.522, 'yaw': 0.0}
WAYPOINT_2 = {'name': '展台导航', 'x': 2.842, 'y': -0.671, 'yaw': 0.0}
TARGET_101 = {'name': '101', 'x': 2.826, 'y': -0.209, 'yaw': 1.53}


def names(route):
    return [point['name'] for point in route]


def test_outbound_route_follows_corridor_toward_101():
    route = build_soft_waypoint_route(
        START, [WAYPOINT_1, WAYPOINT_2], TARGET_101
    )
    assert names(route) == ['导航点1', '展台导航']


def test_return_route_reverses_waypoints_toward_start():
    route = build_soft_waypoint_route(
        TARGET_101, [WAYPOINT_1, WAYPOINT_2], START
    )
    assert names(route) == ['展台导航', '导航点1']


def test_near_first_waypoint_does_not_drive_back_to_second_on_return():
    current = {'x': 1.222, 'y': -0.391, 'yaw': -0.18}
    route = build_soft_waypoint_route(
        current, [WAYPOINT_1, WAYPOINT_2], START
    )
    assert names(route) == ['导航点1']


def test_direct_target_when_no_waypoint_advances_route():
    current = {'x': 0.12, 'y': -0.15, 'yaw': 0.0}
    route = build_soft_waypoint_route(
        current, [WAYPOINT_1, WAYPOINT_2], START
    )
    assert route == []


def test_route_initializes_arm_before_sending_navigation_goal():
    published = []
    started = []
    worker = SimpleNamespace(
        navigation_arrived=SimpleNamespace(emit=lambda _value: None),
        error=SimpleNamespace(emit=lambda message: (_ for _ in ()).throw(
            AssertionError(message)
        )),
        _latest_map_pose=START,
        _arm_pub=SimpleNamespace(
            get_subscription_count=lambda: 1,
            publish=published.append,
        ),
        arm_status=SimpleNamespace(emit=lambda _message: None),
        _arm_initialize_wait_sec=3.5,
        _cancel_goal=lambda: None,
        _navigation_not_ready=lambda _now: '',
        _send_current_route_goal=lambda: started.append(True),
    )
    worker._publish_arm_command = lambda payload: (
        RosWorker._publish_arm_command(worker, payload)
    )
    before = time.monotonic()
    RosWorker._start_route(worker, [], TARGET_101)
    assert json.loads(published[0].data) == {
        'type': 'stored', 'start': 1, 'end': 1, 'repeat': 1,
    }
    assert worker._route_start_at >= before + 3.5
    assert started == []


def test_only_recent_fault_interruption_resumes_navigation():
    assert should_resume_after_base_fault(
        GoalStatus.STATUS_ABORTED, 10.0, 12.0, 0
    )
    assert not should_resume_after_base_fault(
        GoalStatus.STATUS_ABORTED, 10.0, 31.0, 0
    )
    assert not should_resume_after_base_fault(
        GoalStatus.STATUS_SUCCEEDED, 10.0, 12.0, 0
    )
    assert not should_resume_after_base_fault(
        GoalStatus.STATUS_ABORTED, 10.0, 12.0, 3
    )


def test_intermediate_goal_announces_final_target_to_mode_manager():
    published = []
    sent = []
    future = SimpleNamespace(add_done_callback=lambda _callback: None)
    worker = SimpleNamespace(
        _route_goals=[(WAYPOINT_1, True), (TARGET_101, False)],
        _route_position=0,
        _navigate_client=SimpleNamespace(
            server_is_ready=lambda: True,
            send_goal_async=lambda goal, feedback_callback: sent.append(goal) or future,
        ),
        _node=SimpleNamespace(
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=Time)
            )
        ),
        _goal_pub=SimpleNamespace(publish=published.append),
        _goal_active_pub=SimpleNamespace(publish=lambda _message: None),
        error=SimpleNamespace(emit=lambda message: (_ for _ in ()).throw(
            AssertionError(message)
        )),
        _active_goal_serial=0,
        _navigation_not_ready=lambda _now: '',
    )
    RosWorker._send_current_route_goal(worker)
    assert sent[0].pose.pose.position.x == WAYPOINT_1['x']
    assert published[0].pose.position.x == TARGET_101['x']


def ready_worker():
    worker = RosWorker()
    now = time.monotonic()
    worker._localization_guard = {
        'healthy': True, 'ready_for_navigation': True, 'state': 'HEALTHY',
    }
    worker._localization_guard_at = now
    worker._base_status = {
        'fault_bits': 0, 'control_state': 2, 'fault': '无故障', 'state': '已就绪',
    }
    worker._base_status_at = now
    worker._nav_lifecycle = 'ACTIVE'
    worker._nav_lifecycle_at = now
    worker._navigate_client = SimpleNamespace(server_is_ready=lambda: True)
    worker._latest_map_pose = START
    return worker


def test_unready_navigation_sends_neither_arm_command_nor_goal():
    for field, value in (
        ('_localization_guard', {}),
        ('_localization_guard', {'healthy': False, 'detail': '定位丢失'}),
        ('_localization_guard', {'healthy': True, 'ready_for_navigation': False}),
        ('_localization_guard', {'healthy': True, 'estop_active': True}),
        ('_localization_guard', {'healthy': True, 'return_pending': True}),
        ('_localization_guard_at', 0.0),
        ('_base_status_at', 0.0),
        ('_base_status', {'fault_bits': 16, 'control_state': 6, 'fault': '通信故障'}),
        ('_base_status', {'fault_bits': 0, 'control_state': 6, 'state': '故障锁定'}),
        ('_nav_lifecycle', 'INACTIVE'),
        ('_nav_lifecycle_at', 0.0),
        ('_navigate_client', SimpleNamespace(server_is_ready=lambda: False)),
    ):
        worker = ready_worker()
        arm_commands, errors = [], []
        worker._publish_arm_command = arm_commands.append
        worker.error.connect(errors.append)
        setattr(worker, field, value)
        worker._start_route([], TARGET_101)
        assert arm_commands == [] and worker._route_start_at == 0.0
        assert not worker._route_goals and errors and worker._nav_state == 'ERROR'
        # Readiness is checked again after initialization and at every segment.
        worker._route_goals = [(TARGET_101, False)]
        worker._send_current_route_goal()
        assert not worker._route_goals and worker._goal_future is None


def test_localization_loss_clears_scheduled_route_and_cancels_late_goal():
    worker = ready_worker()
    arm_commands, cancellations = [], []
    worker._publish_arm_command = lambda command: arm_commands.append(command) or True
    worker._start_route([], TARGET_101)
    serial = worker._active_goal_serial
    assert len(arm_commands) == 1 and worker._route_start_at > time.monotonic()
    worker._localization_guard['healthy'] = False
    worker._check_navigation_health(time.monotonic())
    assert not worker._route_goals and worker._route_start_at == 0.0
    accepted = Future()
    accepted.set_result(SimpleNamespace(
        accepted=True, cancel_goal_async=lambda: cancellations.append(True),
    ))
    worker._goal_response(accepted, serial)
    assert cancellations == [True] and worker._goal_handle is None
    worker._localization_guard['healthy'] = True
    worker._check_navigation_health(time.monotonic())
    assert not worker._route_goals
    worker._route_goals = [(TARGET_101, False)]
    worker._goal_handle = accepted.result()
    worker._localization_guard['state'] = 'RELOCALIZING'
    worker._check_navigation_health(time.monotonic())
    assert cancellations == [True, True] and not worker._route_goals


def test_persistent_base_fault_has_fixed_deadline_despite_fresh_feedback():
    worker = ready_worker()
    now = time.monotonic()
    cancellations = []
    worker._route_goals = [(TARGET_101, False)]
    worker._goal_handle = SimpleNamespace(cancel_goal_async=lambda: cancellations.append(True))
    fault = SimpleNamespace(fault_bits=8, control_state=6)
    worker._base_status_callback(fault)
    worker._check_navigation_health(now)
    assert cancellations == [True] and worker._goal_handle is None
    assert worker._fault_resume_deadline == now + 20.0
    for elapsed in range(1, 21):
        tick = now + elapsed
        worker._localization_guard_at = tick
        with patch('roscar_operator_gui.ros_worker.time.monotonic', return_value=tick):
            worker._base_status_callback(fault)
            worker._resume_after_base_fault(tick)
        if elapsed < 20:
            assert worker._route_goals and worker._fault_resume_deadline == now + 20.0
    assert not worker._route_goals and worker._fault_resume_at == 0.0
    assert worker._fault_resume_deadline == 0.0 and worker._nav_state == 'ERROR'


def test_fault_resume_requires_readiness_and_cancel_prevents_ghost_route():
    worker = ready_worker()
    now = time.monotonic()
    sent = []
    worker._send_current_route_goal = lambda: sent.append(worker._route_goals[0])
    worker._route_goals = [(TARGET_101, False)]
    worker._base_status_callback(SimpleNamespace(fault_bits=8, control_state=6))
    worker._check_navigation_health(now)
    worker._base_status_callback(SimpleNamespace(fault_bits=0, control_state=2))
    worker._localization_guard['ready_for_navigation'] = False
    worker._resume_after_base_fault(now + 0.5)
    assert sent == []
    worker._localization_guard['ready_for_navigation'] = True
    worker._resume_after_base_fault(now + 1.0)
    assert len(sent) == 1 and worker._fault_resume_attempts == 1
    worker._base_status_callback(SimpleNamespace(fault_bits=8, control_state=6))
    worker._check_navigation_health(now + 1.1)
    serial = worker._active_goal_serial
    worker._cancel_goal()
    worker._base_status_callback(SimpleNamespace(fault_bits=0, control_state=2))
    result = Future()
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    worker._navigation_result(result, serial)
    worker._resume_after_base_fault(now + 1.2)
    assert len(sent) == 1 and not worker._route_goals
    assert worker._fault_resume_at == worker._fault_resume_deadline == 0.0


def test_late_goal_send_exception_does_not_clear_replacement_route():
    worker = ready_worker()
    inactive, errors = [], []
    worker._publish_goal_inactive = lambda: inactive.append(True)
    worker.error.connect(errors.append)
    serial = worker._active_goal_serial
    worker._cancel_goal()
    worker._route_goals = [(TARGET_101, False)]
    failed = Future()
    failed.set_exception(RuntimeError('old goal response failed'))
    worker._goal_response(failed, serial)
    assert worker._route_goals == [(TARGET_101, False)]
    assert inactive == errors == []
    # A current goal failure must still stop the route and report its cause.
    worker._goal_response(failed, worker._active_goal_serial)
    assert not worker._route_goals and inactive == [True] and errors
