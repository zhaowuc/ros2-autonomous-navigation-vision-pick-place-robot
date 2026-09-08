#!/usr/bin/env python3
"""Run repeatable ROSCAR Gazebo acceptance suites and write per-run CSV evidence."""

import argparse
import csv
import hashlib
import json
import math
import os
import statistics
import time
from dataclasses import dataclass, field

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


PROFILES = {'low': 0.06, 'normal': 0.10, 'fast': 0.20}
ROUTE_TIMEOUTS = {'low': 240.0, 'normal': 180.0, 'fast': 180.0}
ABSOLUTE_MAX_SPEED = 0.20
ROUTE_XY_TOLERANCE = 0.03
ROUTE_YAW_TOLERANCE = 0.05
HANDOFF_MAX_GAP = 0.10
VELOCITY_EPSILON = 1.0e-6
STOP_MOTION_THRESHOLD = 0.005
RESUME_MOTION_THRESHOLD = 0.010
MIN_WAIT_INTERVAL = 0.50
POINTS = {
    'SIM_A': (-3.0, -2.0, 0.0),
    'SIM_B': (3.0, -2.0, 0.0),
    'SIM_C': (3.0, 3.0, math.pi / 2.0),
}
ROUTES = [
    ('normal', 'SIM_A', 'SIM_B'),
    ('normal', 'SIM_A', 'SIM_C'),
    ('normal', 'SIM_B', 'SIM_A'),
    ('normal', 'SIM_B', 'SIM_C'),
    ('normal', 'SIM_C', 'SIM_A'),
    ('normal', 'SIM_C', 'SIM_B'),
    ('low', 'SIM_A', 'SIM_C'),
    ('fast', 'SIM_A', 'SIM_C'),
]


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def rectangles_overlap(
        ax, ay, ayaw, ahx, ahy, bx, by, byaw, bhx, bhy):
    """Return SAT overlap for two planar oriented rectangles."""
    axes = (
        (math.cos(ayaw), math.sin(ayaw)),
        (-math.sin(ayaw), math.cos(ayaw)),
        (math.cos(byaw), math.sin(byaw)),
        (-math.sin(byaw), math.cos(byaw)),
    )
    delta_x = bx - ax
    delta_y = by - ay
    a_x = axes[0]
    a_y = axes[1]
    b_x = axes[2]
    b_y = axes[3]
    for axis_x, axis_y in axes:
        center_distance = abs(delta_x * axis_x + delta_y * axis_y)
        a_radius = (
            ahx * abs(a_x[0] * axis_x + a_x[1] * axis_y)
            + ahy * abs(a_y[0] * axis_x + a_y[1] * axis_y)
        )
        b_radius = (
            bhx * abs(b_x[0] * axis_x + b_x[1] * axis_y)
            + bhy * abs(b_y[0] * axis_x + b_y[1] * axis_y)
        )
        if center_distance > a_radius + b_radius:
            return False
    return True


@dataclass
class Capture:
    maxima: dict = field(default_factory=lambda: {
        'controller': 0.0, 'nav_raw': 0.0, 'selected': 0.0,
        'smoothed': 0.0, 'safe': 0.0,
    })
    max_cruise_vy: float = 0.0
    max_terminal_vy: float = 0.0
    max_switches: int = 0
    mode: str = 'CRUISE_DIFF'
    controller: str = 'FollowPathCruise'
    last_controller: str = 'FollowPathCruise'
    switch_time: float = 0.0
    min_scan: float = math.inf
    twist_events: dict = field(default_factory=lambda: {
        'controller': [], 'nav_raw': [], 'selected': [],
        'smoothed': [], 'safe': [],
    })
    selector_events: list = field(default_factory=list)
    mode_events: list = field(default_factory=list)
    switch_events: list = field(default_factory=list)
    plan_events: list = field(default_factory=list)
    scan_events: list = field(default_factory=list)
    pedestrian_clearance_events: list = field(default_factory=list)
    blocker_state_events: list = field(default_factory=list)
    wait_status_events: list = field(default_factory=list)
    wait_status_last: dict = field(default_factory=dict)
    wait_goals_seen_active: set = field(default_factory=set)
    controller_log_events: list = field(default_factory=list)
    recovery_log_events: list = field(default_factory=list)


class Benchmark(Node):
    def __init__(self, output_dir):
        # This executable is simulation-only.  Force its clock into Gazebo's
        # time domain so initial poses, goals, TF lookups, and evidence stamps
        # never mix system time with /clock.
        super().__init__(
            'roscar_sim_benchmark',
            parameter_overrides=[RclpyParameter('use_sim_time', value=True)],
        )
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir))
        os.makedirs(self.output_dir, exist_ok=True)
        self.odom = None
        self.odom_message = None
        self.ground_truth = None
        self.ground_truth_message = None
        self.ground_truth_received_ros_ns = 0
        self.ground_truth_received_mono = 0.0
        self.map_pose = None
        self.plan_message = None
        self.plan_received_ros_ns = 0
        self.plan_received_mono = 0.0
        self.current_mode = 'CRUISE_DIFF'
        self.current_controller = 'FollowPathCruise'
        self.current_selector = ''
        self.capture = Capture()
        self.capturing = False
        self.run_sequence = 0
        self.velocity_trace_rows = []
        self.handoff_trace_rows = []

        durable = QoSProfile(depth=1)
        durable.reliability = ReliabilityPolicy.RELIABLE
        durable.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latest_state_qos = QoSProfile(depth=1)
        latest_state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.test_pub = self.create_publisher(Twist, '/cmd_vel_test', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/roscar/active_goal', durable)
        self.goal_active_pub = self.create_publisher(Bool, '/roscar/goal_active', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', durable
        )
        self.reset_pub = self.create_publisher(PoseStamped, '/roscar_sim/reset_pose', 10)
        self.create_subscription(Odometry, '/wheel/odom_raw', self._odom, 20)
        self.create_subscription(
            Odometry, '/sim/ground_truth', self._ground_truth,
            latest_state_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose, 20
        )
        self.create_subscription(Path, '/plan', self._plan, 20)
        for topic, name in (
            ('/cmd_vel_nav_controller', 'controller'),
            ('/cmd_vel_nav_raw', 'nav_raw'),
            ('/cmd_vel_selected', 'selected'),
            ('/cmd_vel_smoothed', 'smoothed'),
            ('/cmd_vel_safe', 'safe'),
        ):
            self.create_subscription(Twist, topic, self._twist_callback(name), 20)
        self.create_subscription(
            String, '/roscar/navigation_mode/status', self._mode_status, 20
        )
        self.create_subscription(
            String, '/controller_selector', self._selector, durable
        )
        # Acceptance clearance must use the same self-filtered scan consumed by
        # the Nav2 costmaps and Collision Monitor. Raw scan minima include the
        # robot's own arm/footprint and are not an obstacle-clearance metric.
        self.create_subscription(
            LaserScan, '/scan_filtered', self._scan, qos_profile_sensor_data
        )
        # Gazebo Classic commonly exposes either name depending on its ROS
        # namespace.  Record physical geometry separation so a crossing PASS
        # cannot be inferred from action success alone.
        self.create_subscription(
            ModelStates, '/model_states', self._model_states,
            latest_state_qos,
        )
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self._model_states,
            latest_state_qos,
        )
        self.create_subscription(
            GoalStatusArray, '/wait/_action/status', self._wait_status, 20
        )
        self.create_subscription(Log, '/rosout', self._rosout, 100)

        self.entity_client = self.create_client(SetEntityState, '/set_entity_state')
        self.profile_client = self.create_client(
            SetParameters, '/roscar_command_arbiter/set_parameters'
        )
        self.pedestrian_client = self.create_client(
            SetBool, '/roscar_sim/set_pedestrian_motion'
        )
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Keep TF ingestion independent from the benchmark's high-volume
        # evidence callbacks.  Otherwise a completed action can leave the
        # single benchmark executor one simulation tick behind /tf and turn a
        # valid terminal pose into a false "future extrapolation" failure.
        self.tf_buffer = Buffer()
        self.tf_listener_node = Node(
            'roscar_sim_benchmark_tf_listener',
            context=self.context,
            use_global_arguments=False,
            enable_rosout=False,
            start_parameter_services=False,
            parameter_overrides=[
                RclpyParameter('use_sim_time', value=True),
            ],
        )
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.tf_listener_node,
            spin_thread=True,
        )

    def close_tf_listener(self):
        """Stop the dedicated TF executor before shutting down rclpy."""
        listener = getattr(self, 'tf_listener', None)
        listener_node = getattr(self, 'tf_listener_node', None)
        if listener is not None:
            executor = getattr(listener, 'executor', None)
            thread = getattr(listener, 'dedicated_listener_thread', None)
            if executor is not None:
                executor.shutdown()
            if thread is not None:
                thread.join()
            listener.unregister()
        if listener_node is not None:
            listener_node.destroy_node()

    def _odom(self, message):
        self.odom_message = message
        pose = message.pose.pose
        self.odom = (
            float(pose.position.x), float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _ground_truth(self, message):
        self.ground_truth_message = message
        self.ground_truth_received_ros_ns = self.get_clock().now().nanoseconds
        self.ground_truth_received_mono = time.monotonic()
        pose = message.pose.pose
        self.ground_truth = (
            float(pose.position.x), float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _amcl_pose(self, message):
        pose = message.pose.pose
        self.map_pose = (
            float(pose.position.x), float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    @staticmethod
    def _plan_fingerprint(message):
        values = []
        for pose_stamped in message.poses:
            pose = pose_stamped.pose
            values.append(
                f'{pose.position.x:.3f},{pose.position.y:.3f},'
                f'{yaw_from_quaternion(pose.orientation):.3f}'
            )
        return hashlib.sha256('|'.join(values).encode('ascii')).hexdigest()[:16]

    def _plan(self, message):
        self.plan_message = message
        self.plan_received_ros_ns = self.get_clock().now().nanoseconds
        self.plan_received_mono = time.monotonic()
        if not self.capturing or not message.poses:
            return
        endpoint = message.poses[-1].pose
        self.capture.plan_events.append({
            'received_ros_ns': self.plan_received_ros_ns,
            'received_mono': self.plan_received_mono,
            'stamp_ns': stamp_ns(message.header.stamp),
            'frame_id': message.header.frame_id,
            'pose_count': len(message.poses),
            'end_x': float(endpoint.position.x),
            'end_y': float(endpoint.position.y),
            'end_yaw': yaw_from_quaternion(endpoint.orientation),
            'fingerprint': self._plan_fingerprint(message),
        })

    def _twist_callback(self, name):
        def callback(message):
            if not self.capturing:
                return
            now = time.monotonic()
            values = (
                float(message.linear.x), float(message.linear.y),
                float(message.angular.z),
            )
            finite = all(math.isfinite(value) for value in values)
            speed = math.hypot(values[0], values[1]) if finite else math.inf
            self.capture.maxima[name] = max(self.capture.maxima[name], speed)
            self.capture.twist_events[name].append({
                'ros_ns': self.get_clock().now().nanoseconds,
                'monotonic': now,
                'vx': values[0],
                'vy': values[1],
                'wz': values[2],
                'finite': finite,
                'linear_abs_max': speed,
                'motion_abs_max': max(map(abs, values)) if finite else math.inf,
            })
            if name == 'nav_raw':
                if self.capture.mode == 'CRUISE_DIFF':
                    self.capture.max_cruise_vy = max(
                        self.capture.max_cruise_vy, abs(message.linear.y)
                    )
                else:
                    self.capture.max_terminal_vy = max(
                        self.capture.max_terminal_vy, abs(message.linear.y)
                    )
        return callback

    def _mode_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        observed_mono = time.monotonic()
        observed_ros_ns = self.get_clock().now().nanoseconds
        mode = str(status.get('mode', self.current_mode))
        controller = str(status.get('controller', self.current_controller))
        self.current_mode = mode
        self.current_controller = controller
        if not self.capturing:
            return
        switch_count = int(status.get('switch_count', 0))
        source_switch_mono = float(status.get('last_switch_monotonic', 0.0))
        self.capture.mode = mode
        self.capture.controller = controller
        self.capture.max_switches = max(
            self.capture.max_switches, switch_count
        )
        self.capture.switch_time = max(
            self.capture.switch_time, source_switch_mono
        )
        event = {
            'observed_ros_ns': observed_ros_ns,
            'observed_mono': observed_mono,
            'source_switch_mono': source_switch_mono,
            'mode': mode,
            'controller': controller,
            'switch_count': switch_count,
        }
        self.capture.mode_events.append(event)
        if controller != self.capture.last_controller:
            event = dict(event)
            event['from_controller'] = self.capture.last_controller
            event['to_controller'] = controller
            self.capture.switch_events.append(event)
            self.capture.last_controller = controller

    def _selector(self, message):
        observed_mono = time.monotonic()
        value = str(message.data)
        changed = value != self.current_selector
        self.current_selector = value
        if self.capturing and changed:
            self.capture.selector_events.append({
                'observed_ros_ns': self.get_clock().now().nanoseconds,
                'observed_mono': observed_mono,
                'controller': value,
            })

    def _scan(self, message):
        if not self.capturing:
            return
        finite = [value for value in message.ranges if math.isfinite(value)]
        if finite:
            minimum = min(finite)
            self.capture.min_scan = min(self.capture.min_scan, minimum)
            self.capture.scan_events.append({
                'ros_ns': self.get_clock().now().nanoseconds,
                'monotonic': time.monotonic(),
                'minimum_m': float(minimum),
            })

    def _model_states(self, message):
        if not self.capturing:
            return
        try:
            robot_pose = message.pose[message.name.index('roscar')]
        except (ValueError, IndexError):
            return
        now_mono = time.monotonic()
        now_ros_ns = self.get_clock().now().nanoseconds
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        try:
            pedestrian_pose = message.pose[message.name.index('pedestrian')]
        except (ValueError, IndexError):
            pedestrian_pose = None
        if pedestrian_pose is not None:
            # Exact 2-D circle-versus-oriented-rectangle separation. Negative
            # means physical overlap. Dimensions match the checked-in models.
            dx = pedestrian_pose.position.x - robot_pose.position.x
            dy = pedestrian_pose.position.y - robot_pose.position.y
            local_x = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
            local_y = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy
            outside_x = max(abs(local_x) - 0.275, 0.0)
            outside_y = max(abs(local_y) - 0.270, 0.0)
            clearance = math.hypot(outside_x, outside_y) - 0.220
            self.capture.pedestrian_clearance_events.append({
                'ros_ns': now_ros_ns,
                'monotonic': now_mono,
                'clearance_m': clearance,
                'robot_x': float(robot_pose.position.x),
                'robot_y': float(robot_pose.position.y),
                'pedestrian_x': float(pedestrian_pose.position.x),
                'pedestrian_y': float(pedestrian_pose.position.y),
            })

        try:
            blocker_pose = message.pose[message.name.index('route_blocker')]
        except (ValueError, IndexError):
            blocker_pose = None
        if blocker_pose is not None:
            blocker_yaw = yaw_from_quaternion(blocker_pose.orientation)
            in_route = (
                abs(float(blocker_pose.position.x)) <= 0.20
                and abs(float(blocker_pose.position.y) + 2.0) <= 0.20
            )
            overlap = in_route and rectangles_overlap(
                float(robot_pose.position.x), float(robot_pose.position.y),
                robot_yaw, 0.275, 0.270,
                float(blocker_pose.position.x), float(blocker_pose.position.y),
                blocker_yaw, 0.125, 1.850,
            )
            self.capture.blocker_state_events.append({
                'ros_ns': now_ros_ns,
                'monotonic': now_mono,
                'robot_x': float(robot_pose.position.x),
                'robot_y': float(robot_pose.position.y),
                'robot_yaw': robot_yaw,
                'blocker_x': float(blocker_pose.position.x),
                'blocker_y': float(blocker_pose.position.y),
                'in_route': in_route,
                'overlap': overlap,
            })

    def _wait_status(self, message):
        if not self.capturing:
            return
        observed_mono = time.monotonic()
        observed_ros_ns = self.get_clock().now().nanoseconds
        for status in message.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid).hex()
            status_value = int(status.status)
            if status_value in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            ):
                self.capture.wait_goals_seen_active.add(goal_id)
            elif goal_id not in self.capture.wait_goals_seen_active:
                # Ignore retained terminal statuses from earlier NavigateToPose
                # runs.  A current Wait transaction must first be observed in
                # an active state during this capture.
                continue
            # GoalStatusArray retains terminal goals.  Record transitions once
            # per capture so a historical SUCCEEDED sample cannot masquerade
            # as the Wait action belonging to the current recovery.
            if self.capture.wait_status_last.get(goal_id) == status_value:
                continue
            self.capture.wait_status_last[goal_id] = status_value
            self.capture.wait_status_events.append({
                'observed_mono': observed_mono,
                'observed_ros_ns': observed_ros_ns,
                'goal_stamp_ns': stamp_ns(status.goal_info.stamp),
                'goal_id': goal_id,
                'status': status_value,
            })

    def _rosout(self, message):
        if not self.capturing:
            return
        observed_mono = time.monotonic()
        observed_epoch = time.time()
        observed_ros_ns = self.get_clock().now().nanoseconds
        source_stamp_ns = stamp_ns(message.stamp)
        source_sec = source_stamp_ns / 1.0e9
        # Humble's rosout stamp on this host is system time even for nodes
        # using /clock.  Convert either a system-time or ROS-time source stamp
        # into the benchmark's monotonic domain before bracketing commands.
        if source_stamp_ns > 0 and abs(source_sec - observed_epoch) <= 60.0:
            source_mono = observed_mono - (observed_epoch - source_sec)
            source_clock = 'system'
        elif (
            source_stamp_ns > 0
            and abs(source_stamp_ns - observed_ros_ns) <= 60_000_000_000
        ):
            source_mono = observed_mono - (
                observed_ros_ns - source_stamp_ns
            ) / 1.0e9
            source_clock = 'ros'
        else:
            source_mono = observed_mono
            source_clock = 'receive_fallback'
        level = (
            message.level[0]
            if isinstance(message.level, (bytes, bytearray))
            else int(message.level)
        )
        event = {
            'observed_mono': observed_mono,
            'observed_epoch': observed_epoch,
            'observed_ros_ns': observed_ros_ns,
            'source_stamp_ns': source_stamp_ns,
            'source_mono': source_mono,
            'source_clock': source_clock,
            'name': str(message.name),
            'message': str(message.msg),
            'level': level,
        }
        text_lower = event['message'].lower()
        if (
            event['name'].endswith('controller_server')
            and 'selected controller:' in text_lower
        ):
            self.capture.controller_log_events.append(event)
        if 'wait' in text_lower or 'clear' in text_lower or 'recover' in text_lower:
            self.capture.recovery_log_events.append(event)

    def spin_for(self, duration, publish=None):
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if publish is not None and now >= next_publish:
                publish()
                next_publish = now + 0.04
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def wait_services(self):
        checks = (
            ('set_entity_state', self.entity_client),
            ('speed profile', self.profile_client),
            ('pedestrian control', self.pedestrian_client),
        )
        for label, client in checks:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f'{label} service unavailable')
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('NavigateToPose action unavailable')
        truth_deadline = time.monotonic() + 15.0
        while self.ground_truth is None and time.monotonic() < truth_deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
        if self.ground_truth is None:
            raise RuntimeError('/sim/ground_truth unavailable')

    def set_profile(self, profile):
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='speed_profile',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value=profile,
            ),
        )]
        future = self.profile_client.call_async(request)
        if not self.wait_future(future, 5.0):
            raise RuntimeError('speed profile request timed out')
        result = future.result()
        if not result.results or not result.results[0].successful:
            raise RuntimeError(f'speed profile rejected: {profile}')

    def set_entity(self, name, x, y, yaw, z=-0.1367):
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.z, request.state.pose.orientation.w = quaternion_from_yaw(yaw)
        request.state.reference_frame = 'world'
        future = self.entity_client.call_async(request)
        if not self.wait_future(future, 5.0) or not future.result().success:
            raise RuntimeError(f'failed to set Gazebo entity {name}')
        return time.monotonic()

    def reset_robot(self, point_name):
        x, y, yaw = POINTS[point_name]
        self.map_pose = None
        self.publish_zero(0.5)
        reset = PoseStamped()
        reset.header.frame_id = 'world'
        reset.pose.position.x = x
        reset.pose.position.y = y
        reset.pose.orientation.z, reset.pose.orientation.w = quaternion_from_yaw(yaw)
        for _ in range(5):
            reset.header.stamp = self.get_clock().now().to_msg()
            self.reset_pub.publish(reset)
            self.spin_for(0.08)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.pose.pose.position.x = x
        initial.pose.pose.position.y = y
        initial.pose.pose.orientation.z, initial.pose.pose.orientation.w = quaternion_from_yaw(yaw)
        # The reset service places the simulated model at this exact pose.
        # Seed AMCL accordingly; introducing an artificial 2 cm uncertainty
        # would test randomized initialization rather than navigation control.
        initial.pose.covariance[0] = 1.0e-8
        initial.pose.covariance[7] = 1.0e-8
        initial.pose.covariance[35] = 1.0e-8
        for _ in range(5):
            self.initial_pose_pub.publish(initial)
            self.spin_for(0.08)
        self.spin_for(1.5)
        truth_deadline = time.monotonic() + 5.0
        reset_converged = False
        tf_converged = False
        truth_tf_converged = False
        while rclpy.ok() and time.monotonic() < truth_deadline:
            if self.ground_truth is not None:
                error = math.hypot(
                    self.ground_truth[0] - x, self.ground_truth[1] - y
                )
                yaw_error = angle_error(self.ground_truth[2], yaw)
                if error <= 0.05 and yaw_error <= 0.05:
                    reset_converged = True
            transform = self._tf_snapshot('map', 'base_footprint')
            if transform['x'] != '':
                tf_error = math.hypot(transform['x'] - x, transform['y'] - y)
                tf_yaw_error = angle_error(transform['yaw'], yaw)
                tf_converged = tf_error <= 0.05 and tf_yaw_error <= 0.05
                if self.ground_truth is not None:
                    truth_tf_error = math.hypot(
                        transform['x'] - self.ground_truth[0],
                        transform['y'] - self.ground_truth[1],
                    )
                    truth_tf_yaw_error = angle_error(
                        transform['yaw'], self.ground_truth[2]
                    )
                    truth_tf_converged = (
                        truth_tf_error <= 0.01
                        and truth_tf_yaw_error <= 0.02
                    )
            if reset_converged and tf_converged and truth_tf_converged:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.ground_truth is None:
            raise RuntimeError('/sim/ground_truth lost during reset')
        if not reset_converged:
            raise RuntimeError('/sim/ground_truth did not converge to reset pose')
        if not tf_converged:
            raise RuntimeError('map->base_footprint did not converge after reset')
        if not truth_tf_converged:
            raise RuntimeError(
                'Gazebo truth and map->base_footprint did not mutually converge'
            )

    def publish_zero(self, duration=0.6):
        zero = Twist()
        self.spin_for(duration, lambda: self.test_pub.publish(zero))

    def begin_capture(self):
        self.capture = Capture(
            mode=self.current_mode,
            controller=self.current_controller,
            last_controller=self.current_controller,
        )
        self.capturing = True

    def end_capture(self):
        self.capturing = False

    def next_run_id(self, label):
        self.run_sequence += 1
        return f'{self.run_sequence:02d}_{label}'

    @staticmethod
    def _write_csv(path, fieldnames, rows):
        temporary = path + '.tmp'
        with open(temporary, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fieldnames, lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)
        os.replace(temporary, path)

    def _append_velocity_trace(self, run_id, suite, profile, capture):
        profile_limit = PROFILES[profile]
        for layer, events in capture.twist_events.items():
            absolute_layer = layer in ('controller', 'nav_raw')
            limit = ABSOLUTE_MAX_SPEED if absolute_layer else profile_limit
            rule = 'absolute_max' if absolute_layer else 'active_profile'
            for index, event in enumerate(events, start=1):
                observed = event['linear_abs_max']
                self.velocity_trace_rows.append({
                    'run_id': run_id,
                    'suite': suite,
                    'profile': profile,
                    'layer': layer,
                    'sample_index': index,
                    'ros_time_ns': event['ros_ns'],
                    'monotonic_sec': f"{event['monotonic']:.9f}",
                    'vx_mps': f"{event['vx']:.9f}",
                    'vy_mps': f"{event['vy']:.9f}",
                    'wz_radps': f"{event['wz']:.9f}",
                    'linear_abs_max_mps': f'{observed:.9f}',
                    'limit_mps': f'{limit:.9f}',
                    'limit_rule': rule,
                    'finite': event['finite'],
                    'within_limit': (
                        event['finite']
                        and observed <= limit + VELOCITY_EPSILON
                    ),
                })
        fields = [
            'run_id', 'suite', 'profile', 'layer', 'sample_index',
            'ros_time_ns', 'monotonic_sec', 'vx_mps', 'vy_mps',
            'wz_radps', 'linear_abs_max_mps', 'limit_mps',
            'limit_rule', 'finite', 'within_limit',
        ]
        self._write_csv(
            os.path.join(self.output_dir, 'velocity_limit_trace.csv'),
            fields,
            self.velocity_trace_rows,
        )

    @staticmethod
    def _velocity_contract(capture, profile, require_nav_raw=True):
        required_layers = ['selected', 'smoothed', 'safe']
        if require_nav_raw:
            required_layers.extend(('controller', 'nav_raw'))
        return (
            all(capture.twist_events[layer] for layer in required_layers)
            and
            capture.maxima['controller'] <= ABSOLUTE_MAX_SPEED + VELOCITY_EPSILON
            and
            capture.maxima['nav_raw'] <= ABSOLUTE_MAX_SPEED + VELOCITY_EPSILON
            and capture.maxima['selected'] <= PROFILES[profile] + VELOCITY_EPSILON
            and capture.maxima['smoothed'] <= PROFILES[profile] + VELOCITY_EPSILON
            and capture.maxima['safe'] <= PROFILES[profile] + VELOCITY_EPSILON
        )

    @staticmethod
    def _event_before(events, reference):
        matches = [event for event in events if event['monotonic'] <= reference]
        return matches[-1] if matches else None

    @staticmethod
    def _event_after(events, reference):
        return next(
            (event for event in events if event['monotonic'] > reference), None
        )

    @staticmethod
    def _event_before_ros(events, reference_ros_ns):
        matches = [event for event in events if event['ros_ns'] <= reference_ros_ns]
        return matches[-1] if matches else None

    @staticmethod
    def _event_after_ros(events, reference_ros_ns):
        return next(
            (event for event in events if event['ros_ns'] > reference_ros_ns),
            None,
        )

    def _append_handoff_trace(
            self, run_id, profile, capture, action_start_monotonic):
        terminal_mode = next((
            event for event in capture.switch_events
            if event['to_controller'] == 'FollowPathTerminal'
        ), None)
        terminal_selector = next((
            event for event in capture.selector_events
            if event['controller'] == 'FollowPathTerminal'
        ), None)
        if terminal_mode is not None:
            source = terminal_mode['source_switch_mono']
            request_reference = (
                source if source > 0.0 else terminal_mode['observed_mono']
            )
        elif terminal_selector is not None:
            request_reference = terminal_selector['observed_mono']
        else:
            request_reference = None

        terminal_actual = next((
            event for event in capture.controller_log_events
            if 'FollowPathTerminal' in event['message']
            and (
                request_reference is None
                or event['observed_mono'] >= request_reference - 0.10
            )
        ), None)
        switch_reference = (
            terminal_actual['source_mono']
            if terminal_actual is not None else None
        )
        reference_source = (
            'controller_server_rosout_selected_controller'
            if terminal_actual is not None else 'missing_controller_server_ack'
        )

        controller_before = controller_after = None
        raw_before = raw_after = None
        selected_before = selected_after = None
        controller_gap = raw_gap = selected_gap = None
        if switch_reference is not None and request_reference is not None:
            # Conservative envelope: last command published before the mode
            # manager requested Terminal through first command received after
            # controller_server logged that Terminal was selected.  This can
            # only overestimate the interruption and does not depend on DDS
            # callback ordering across the selector and command topics.
            controller_before = self._event_before(
                capture.twist_events['controller'], request_reference
            )
            controller_after = self._event_after(
                capture.twist_events['controller'], switch_reference
            )
            raw_before = self._event_before(
                capture.twist_events['nav_raw'], request_reference
            )
            raw_after = self._event_after(
                capture.twist_events['nav_raw'], switch_reference
            )
            selected_before = self._event_before(
                capture.twist_events['selected'], request_reference
            )
            selected_after = self._event_after(
                capture.twist_events['selected'], switch_reference
            )
            if controller_before is not None and controller_after is not None:
                controller_gap = (
                    controller_after['monotonic']
                    - controller_before['monotonic']
                )
            if raw_before is not None and raw_after is not None:
                raw_gap = raw_after['monotonic'] - raw_before['monotonic']
            if selected_before is not None and selected_after is not None:
                selected_gap = (
                    selected_after['monotonic'] - selected_before['monotonic']
                )

        def event_value(event, key, digits=9):
            if event is None:
                return ''
            value = event[key]
            return f'{value:.{digits}f}' if isinstance(value, float) else value

        gaps = (controller_gap, raw_gap, selected_gap)
        contract_gap = max(gaps) if all(value is not None for value in gaps) else None
        handoff_pass = (
            terminal_selector is not None
            and terminal_actual is not None
            and switch_reference is not None
            and request_reference is not None
            and switch_reference >= request_reference - 0.02
            and contract_gap is not None
            and contract_gap <= HANDOFF_MAX_GAP + VELOCITY_EPSILON
        )
        row = {
            'run_id': run_id,
            'profile': profile,
            'action_start_monotonic_sec': f'{action_start_monotonic:.9f}',
            'from_controller': (
                terminal_mode['from_controller'] if terminal_mode else ''
            ),
            'to_controller': 'FollowPathTerminal' if switch_reference else '',
            'switch_reference_monotonic_sec': (
                f'{switch_reference:.9f}' if switch_reference is not None else ''
            ),
            'switch_after_action_start_sec': (
                f'{switch_reference - action_start_monotonic:.9f}'
                if switch_reference is not None else ''
            ),
            'switch_reference_source': reference_source,
            'handoff_measurement': (
                'last sample before selector request to first sample after '
                'controller_server terminal acknowledgement'
            ),
            'controller_server_log_ros_ns': (
                terminal_actual['observed_ros_ns'] if terminal_actual else ''
            ),
            'controller_server_log_monotonic_sec': (
                f"{terminal_actual['observed_mono']:.9f}"
                if terminal_actual else ''
            ),
            'controller_server_log_message': (
                terminal_actual['message'] if terminal_actual else ''
            ),
            'controller_server_log_source_stamp_ns': (
                terminal_actual['source_stamp_ns'] if terminal_actual else ''
            ),
            'controller_server_log_source_clock': (
                terminal_actual['source_clock'] if terminal_actual else ''
            ),
            'controller_server_log_source_monotonic_sec': (
                f"{terminal_actual['source_mono']:.9f}"
                if terminal_actual else ''
            ),
            'selector_to_controller_server_delay_sec': (
                f'{switch_reference - request_reference:.9f}'
                if switch_reference is not None
                and request_reference is not None else ''
            ),
            'selector_source_monotonic_sec': (
                f"{terminal_mode['source_switch_mono']:.9f}"
                if terminal_mode is not None
                and terminal_mode['source_switch_mono'] > 0.0 else ''
            ),
            'selector_observed_ros_ns': (
                terminal_selector['observed_ros_ns'] if terminal_selector else ''
            ),
            'selector_observed_monotonic_sec': (
                f"{terminal_selector['observed_mono']:.9f}"
                if terminal_selector else ''
            ),
            'mode_observed_ros_ns': (
                terminal_mode['observed_ros_ns'] if terminal_mode else ''
            ),
            'mode_observed_monotonic_sec': (
                f"{terminal_mode['observed_mono']:.9f}"
                if terminal_mode else ''
            ),
            'last_controller_before_monotonic_sec': event_value(
                controller_before, 'monotonic'
            ),
            'last_controller_before_vx_mps': event_value(
                controller_before, 'vx'
            ),
            'last_controller_before_vy_mps': event_value(
                controller_before, 'vy'
            ),
            'first_controller_after_monotonic_sec': event_value(
                controller_after, 'monotonic'
            ),
            'first_controller_after_vx_mps': event_value(
                controller_after, 'vx'
            ),
            'first_controller_after_vy_mps': event_value(
                controller_after, 'vy'
            ),
            'controller_handoff_gap_sec': (
                f'{controller_gap:.9f}' if controller_gap is not None else ''
            ),
            'last_raw_before_monotonic_sec': event_value(
                raw_before, 'monotonic'
            ),
            'last_raw_before_vx_mps': event_value(raw_before, 'vx'),
            'last_raw_before_vy_mps': event_value(raw_before, 'vy'),
            'first_raw_after_monotonic_sec': event_value(
                raw_after, 'monotonic'
            ),
            'first_raw_after_vx_mps': event_value(raw_after, 'vx'),
            'first_raw_after_vy_mps': event_value(raw_after, 'vy'),
            'raw_handoff_gap_sec': (
                f'{raw_gap:.9f}' if raw_gap is not None else ''
            ),
            'last_selected_before_monotonic_sec': event_value(
                selected_before, 'monotonic'
            ),
            'last_selected_before_vx_mps': event_value(selected_before, 'vx'),
            'last_selected_before_vy_mps': event_value(selected_before, 'vy'),
            'first_selected_after_monotonic_sec': event_value(
                selected_after, 'monotonic'
            ),
            'first_selected_after_vx_mps': event_value(selected_after, 'vx'),
            'first_selected_after_vy_mps': event_value(selected_after, 'vy'),
            'selected_handoff_gap_sec': (
                f'{selected_gap:.9f}' if selected_gap is not None else ''
            ),
            'contract_handoff_gap_sec': (
                f'{contract_gap:.9f}' if contract_gap is not None else ''
            ),
            'handoff_limit_sec': f'{HANDOFF_MAX_GAP:.3f}',
            'controller_switches': capture.max_switches,
            'result': 'PASS' if handoff_pass else 'FAIL',
        }
        self.handoff_trace_rows.append(row)
        self._write_csv(
            os.path.join(self.output_dir, 'handoff_trace.csv'),
            list(row),
            self.handoff_trace_rows,
        )
        return contract_gap, handoff_pass, switch_reference

    def _ground_truth_snapshot(self, result_ros_ns, result_mono):
        message = self.ground_truth_message
        if message is None:
            return None
        pose = message.pose.pose
        message_stamp_ns = stamp_ns(message.header.stamp)
        return {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
            'stamp_ns': message_stamp_ns,
            'received_ros_ns': self.ground_truth_received_ros_ns,
            'received_mono': self.ground_truth_received_mono,
            # Freshness uses the monotonic receive-time domain so it remains
            # valid across simulated-clock pauses and jumps.
            'age_sec': result_mono - self.ground_truth_received_mono,
        }

    def _plan_snapshot(self):
        message = self.plan_message
        if message is None or not message.poses:
            return None
        pose = message.poses[-1].pose
        return {
            'frame_id': message.header.frame_id,
            'stamp_ns': stamp_ns(message.header.stamp),
            'received_ros_ns': self.plan_received_ros_ns,
            'received_mono': self.plan_received_mono,
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
            'pose_count': len(message.poses),
        }

    def _tf_snapshot(self, target, source, at_time=None):
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                at_time if at_time is not None else rclpy.time.Time(),
                timeout=Duration(seconds=0.10),
            )
            translation = transform.transform.translation
            return {
                'frame_id': transform.header.frame_id,
                'child_frame_id': transform.child_frame_id,
                'stamp_ns': stamp_ns(transform.header.stamp),
                'x': float(translation.x),
                'y': float(translation.y),
                'yaw': yaw_from_quaternion(transform.transform.rotation),
                'error': '',
            }
        except TransformException as exc:
            return {
                'frame_id': target,
                'child_frame_id': source,
                'stamp_ns': '',
                'x': '', 'y': '', 'yaw': '',
                'error': str(exc),
              }

    def _tf_snapshot_at_truth(self, at_time):
        """Read map->base at ``at_time`` without inventing a newer base pose.

        AMCL may leave map->odom unchanged until its next scan update, so tf2
        can reject the whole chain one tick into the future even though the
        localization transform is still within its configured 0.5 s validity
        window.  In that case compose the latest valid map->odom with the exact
        odom->base sample at the frozen truth timestamp.
        """
        direct = self._tf_snapshot('map', 'base_footprint', at_time)
        if direct['x'] != '' or at_time is None:
            return direct, False
        map_odom = self._tf_snapshot('map', 'odom')
        odom_base = self._tf_snapshot('odom', 'base_footprint', at_time)
        if map_odom['x'] == '' or odom_base['x'] == '':
            return direct, False
        requested_ns = at_time.nanoseconds
        localization_age = (
            abs(requested_ns - int(map_odom['stamp_ns'])) / 1.0e9
            if map_odom['stamp_ns'] != '' else math.inf
        )
        if localization_age > 0.50:
            direct['error'] += (
                f'; map->odom age {localization_age:.3f}s exceeds 0.50s'
            )
            return direct, False
        c = math.cos(map_odom['yaw'])
        s = math.sin(map_odom['yaw'])
        x = map_odom['x'] + c * odom_base['x'] - s * odom_base['y']
        y = map_odom['y'] + s * odom_base['x'] + c * odom_base['y']
        yaw = math.atan2(
            math.sin(map_odom['yaw'] + odom_base['yaw']),
            math.cos(map_odom['yaw'] + odom_base['yaw']),
        )
        return {
            'frame_id': 'map',
            'child_frame_id': 'base_footprint',
            'stamp_ns': requested_ns,
            'x': x,
            'y': y,
            'yaw': yaw,
            'error': '',
        }, True

    @staticmethod
    def _zero_segments(events, action_start, action_stop):
        segments = []
        moving_seen = False
        segment_start = None
        segment_end = None
        for event in events:
            event_time = event['monotonic']
            if event_time < action_start or event_time > action_stop:
                continue
            motion = event['motion_abs_max']
            if not moving_seen:
                if motion > RESUME_MOTION_THRESHOLD:
                    moving_seen = True
                continue
            if motion <= STOP_MOTION_THRESHOLD:
                if segment_start is None:
                    segment_start = event_time
                elif segment_end is not None and event_time - segment_end > 0.20:
                    if segment_end - segment_start >= MIN_WAIT_INTERVAL:
                        segments.append((segment_start, segment_end))
                    segment_start = event_time
                segment_end = event_time
            elif segment_start is not None:
                if segment_end - segment_start >= MIN_WAIT_INTERVAL:
                    segments.append((segment_start, segment_end))
                segment_start = None
                segment_end = None
        if segment_start is not None and segment_end is not None:
            if segment_end - segment_start >= MIN_WAIT_INTERVAL:
                segments.append((segment_start, segment_end))
        return segments

    def _dynamic_metrics(
            self, capture, action_start, action_stop, event_state, profile):
        appeared = event_state.get('appeared_mono')
        removed = event_state.get('removed_mono')
        segments = self._zero_segments(
            capture.twist_events['safe'], action_start, action_stop
        )
        eligible = [
            segment for segment in segments
            if appeared is None or segment[1] >= max(appeared, action_start)
        ]
        if removed is not None:
            before_clear = [segment for segment in eligible if segment[0] <= removed]
            wait_segment = max(
                before_clear, key=lambda value: value[1] - value[0],
                default=None,
            )
        else:
            wait_segment = max(
                eligible, key=lambda value: value[1] - value[0],
                default=None,
            )
        resume_reference = removed
        if resume_reference is None and wait_segment is not None:
            resume_reference = wait_segment[1]
        resume_event = None
        if resume_reference is not None:
            resume_event = next((
                event for event in capture.twist_events['safe']
                if event['monotonic'] > resume_reference
                and event['motion_abs_max'] > RESUME_MOTION_THRESHOLD
            ), None)

        plans_after_clear = []
        changed_plans_after_clear = 0
        if removed is not None:
            plans_after_clear = [
                event for event in capture.plan_events
                if event['received_mono'] >= removed
            ]
            plans_before_clear = [
                event for event in capture.plan_events
                if event['received_mono'] < removed
            ]
            previous = (
                plans_before_clear[-1]['fingerprint']
                if plans_before_clear else None
            )
            for event in plans_after_clear:
                if previous is not None and event['fingerprint'] != previous:
                    changed_plans_after_clear += 1
                previous = event['fingerprint']

        scan_min_event = min(
            capture.scan_events,
            key=lambda event: event['minimum_m'],
            default=None,
        )
        pedestrian_clearance_event = min(
            capture.pedestrian_clearance_events,
            key=lambda event: event['clearance_m'],
            default=None,
        )
        encounter_events = [
            event for event in capture.pedestrian_clearance_events
            if event['clearance_m'] <= 1.0
        ]
        encounter_groups = []
        for event in encounter_events:
            if (
                not encounter_groups
                or event['monotonic']
                - encounter_groups[-1][-1]['monotonic'] > 0.30
            ):
                encounter_groups.append([event])
            else:
                encounter_groups[-1].append(event)
        selected_encounter = min(
            encounter_groups,
            key=lambda group: min(event['clearance_m'] for event in group),
            default=[],
        )
        if selected_encounter:
            encounter_start = selected_encounter[0]['monotonic']
            encounter_end = selected_encounter[-1]['monotonic']
            encounter_safe = [
                event for event in capture.twist_events['safe']
                if encounter_start - 0.10 <= event['monotonic'] <= encounter_end + 0.10
            ]
            pre_encounter_safe = [
                event for event in capture.twist_events['safe']
                if encounter_start - 2.0 <= event['monotonic'] <= encounter_start - 0.15
            ]
            pre_encounter_pose = [
                event for event in capture.pedestrian_clearance_events
                if encounter_start - 2.0 <= event['monotonic'] < encounter_start
            ]
            encounter_min_speed = min(
                (event['linear_abs_max'] for event in encounter_safe),
                default=math.inf,
            )
            encounter_baseline_speed = max(
                0.0,
                statistics.median([
                    event['linear_abs_max'] for event in pre_encounter_safe
                ]) if pre_encounter_safe else 0.0,
            )
            encounter_slowdown_ratio = (
                encounter_min_speed / encounter_baseline_speed
                if encounter_baseline_speed > 1.0e-6 else math.inf
            )
            baseline_y = (
                pre_encounter_pose[-1]['robot_y']
                if pre_encounter_pose else -2.0
            )
            encounter_max_lateral = max(
                (abs(event['robot_y'] + 2.0) for event in selected_encounter),
                default=0.0,
            )
            encounter_lateral_change = max(
                (abs(event['robot_y'] - baseline_y) for event in selected_encounter),
                default=0.0,
            )
            pedestrian_start = selected_encounter[0]
            encounter_pedestrian_motion = max((
                math.hypot(
                    event['pedestrian_x'] - pedestrian_start['pedestrian_x'],
                    event['pedestrian_y'] - pedestrian_start['pedestrian_y'],
                )
                for event in selected_encounter
            ), default=0.0)
        else:
            encounter_start = encounter_end = None
            encounter_safe = []
            pre_encounter_safe = []
            encounter_min_speed = math.inf
            encounter_baseline_speed = 0.0
            encounter_slowdown_ratio = math.inf
            encounter_max_lateral = 0.0
            encounter_lateral_change = 0.0
            encounter_pedestrian_motion = 0.0
        avoidance_response = (
            len(selected_encounter) >= 3
            and bool(encounter_safe)
            and encounter_pedestrian_motion >= 0.10
            and encounter_baseline_speed >= PROFILES[profile] * 0.50
            and (
                encounter_slowdown_ratio <= 0.80
                or encounter_lateral_change >= 0.05
            )
        )
        blocker_events = [
            event for event in capture.blocker_state_events
            if action_start <= event['monotonic'] <= action_stop
        ]
        blocker_in_route = any(event['in_route'] for event in blocker_events)
        blocker_removed_observed = (
            removed is not None
            and any(
                event['monotonic'] >= removed
                and event['blocker_y'] <= -5.0
                for event in blocker_events
            )
        )
        blocker_overlap = any(event['overlap'] for event in blocker_events)
        trigger_wait_goal_id = event_state.get('trigger_wait_goal_id')
        action_start_ros_ns = event_state.get('action_start_ros_ns', 0)
        wait_succeeded_events = [
            event for event in capture.wait_status_events
            if wait_segment is not None
            and wait_segment[0] <= event['observed_mono']
            and (removed is None or event['observed_mono'] <= removed + 0.05)
            and event['status'] == GoalStatus.STATUS_SUCCEEDED
            and event['goal_stamp_ns'] >= action_start_ros_ns
            and (
                trigger_wait_goal_id is None
                or event['goal_id'] == trigger_wait_goal_id
            )
        ]
        wait_succeeded_goal_ids = {
            event['goal_id'] for event in wait_succeeded_events
        }
        clear_logs = [
            event for event in capture.recovery_log_events
            if 'clear' in event['message'].lower()
            and 'local' in event['message'].lower()
        ]
        waited_until_clear = (
            wait_segment is not None
            and removed is not None
            and wait_segment[0] <= removed
            and wait_segment[1] >= removed - 0.20
        )
        wait_removal_trigger_verified = (
            removed is not None
            and trigger_wait_goal_id is not None
            and bool(wait_succeeded_events)
            and event_state.get('trigger_stop_start_mono') is not None
            and event_state.get('trigger_stop_start_mono') <= removed
        )
        trigger_wait_succeeded = event_state.get('trigger_wait_succeeded_mono')
        causal_plans_after_clear = [
            event for event in plans_after_clear
            if trigger_wait_succeeded is not None
            and event['received_mono'] >= trigger_wait_succeeded
        ]
        causal_replan_and_resume = (
            bool(causal_plans_after_clear)
            and resume_event is not None
            and resume_event['monotonic']
            >= causal_plans_after_clear[0]['received_mono']
        )
        return {
            'obstacle_appeared_monotonic_sec': (
                appeared if appeared is not None else ''
            ),
            'obstacle_removed_monotonic_sec': (
                removed if removed is not None else ''
            ),
            'obstacle_appeared_after_start_sec': (
                appeared - action_start if appeared is not None else ''
            ),
            'obstacle_removed_after_start_sec': (
                removed - action_start if removed is not None else ''
            ),
            'scan_min_monotonic_sec': (
                scan_min_event['monotonic'] if scan_min_event else ''
            ),
            'scan_min_m': (
                scan_min_event['minimum_m'] if scan_min_event else ''
            ),
            'pedestrian_clearance_samples': len(
                capture.pedestrian_clearance_events
            ),
            'min_pedestrian_clearance_m': (
                pedestrian_clearance_event['clearance_m']
                if pedestrian_clearance_event else ''
            ),
            'collision_detected': (
                pedestrian_clearance_event is not None
                and pedestrian_clearance_event['clearance_m'] <= 0.0
            ),
            'pedestrian_encounter_detected': bool(encounter_events),
            'pedestrian_encounter_threshold_m': 1.0,
            'pedestrian_encounter_group_count': len(encounter_groups),
            'selected_encounter_samples': len(selected_encounter),
            'pedestrian_encounter_start_monotonic_sec': (
                encounter_start if encounter_start is not None else ''
            ),
            'pedestrian_encounter_end_monotonic_sec': (
                encounter_end if encounter_end is not None else ''
            ),
            'encounter_safe_samples': len(encounter_safe),
            'encounter_min_safe_speed_mps': (
                encounter_min_speed if math.isfinite(encounter_min_speed) else ''
            ),
            'encounter_pre_speed_mps': encounter_baseline_speed,
            'encounter_slowdown_ratio': (
                encounter_slowdown_ratio
                if math.isfinite(encounter_slowdown_ratio) else ''
            ),
            'encounter_max_lateral_deviation_m': encounter_max_lateral,
            'encounter_lateral_change_m': encounter_lateral_change,
            'encounter_pedestrian_motion_m': encounter_pedestrian_motion,
            'avoidance_response_detected': avoidance_response,
            'blocker_model_samples': len(blocker_events),
            'blocker_in_route_observed': blocker_in_route,
            'blocker_removed_observed': blocker_removed_observed,
            'blocker_overlap_detected': blocker_overlap,
            'blocker_appeared_after_motion': bool(
                event_state.get('motion_seen_before_appearance', False)
            ),
            'stop_wait_interval_count': len(eligible),
            'stop_wait_detected': wait_segment is not None,
            'stop_wait_start_monotonic_sec': (
                wait_segment[0] if wait_segment else ''
            ),
            'stop_wait_end_monotonic_sec': (
                wait_segment[1] if wait_segment else ''
            ),
            'stop_wait_duration_sec': (
                wait_segment[1] - wait_segment[0] if wait_segment else ''
            ),
            'waited_until_obstacle_clear': waited_until_clear,
            'resume_after_clear': resume_event is not None,
            'resume_monotonic_sec': (
                resume_event['monotonic'] if resume_event else ''
            ),
            'resume_delay_after_clear_sec': (
                resume_event['monotonic'] - removed
                if resume_event is not None and removed is not None else ''
            ),
            'plan_messages_total': len(capture.plan_events),
            'plan_messages_after_clear': len(plans_after_clear),
            'causal_plan_messages_after_clear': len(causal_plans_after_clear),
            'changed_plans_after_clear': changed_plans_after_clear,
            'replan_after_clear': causal_replan_and_resume,
            'wait_action_status_samples': len(capture.wait_status_events),
            'wait_action_success_count': len(wait_succeeded_goal_ids),
            'wait_action_observed': bool(wait_succeeded_goal_ids),
            'wait_action_goal_id': trigger_wait_goal_id or '',
            'wait_action_succeeded_monotonic_sec': (
                wait_succeeded_events[0]['observed_mono']
                if wait_succeeded_events else ''
            ),
            'wait_removal_trigger_verified': wait_removal_trigger_verified,
            'trigger_stop_start_monotonic_sec': (
                event_state.get('trigger_stop_start_mono', '')
            ),
            'stop_duration_before_removal_sec': (
                removed - event_state['trigger_stop_start_mono']
                if removed is not None
                and event_state.get('trigger_stop_start_mono') is not None
                else ''
            ),
            'physical_stop_verified': bool(
                event_state.get('physical_stop_verified', False)
            ),
            'physical_stop_samples': event_state.get('physical_stop_samples', 0),
            'physical_stop_translation_m': event_state.get(
                'physical_stop_translation_m', ''
            ),
            'physical_stop_yaw_change_rad': event_state.get(
                'physical_stop_yaw_change_rad', ''
            ),
            'clear_local_costmap_log_count': len(clear_logs),
            'clear_local_costmap_log_observed': bool(clear_logs),
        }

    def run_straight(self, selected_profile=None, trials=3):
        if selected_profile is not None and selected_profile not in PROFILES:
            raise ValueError(f'unknown straight profile: {selected_profile}')
        if trials < 1:
            raise ValueError('straight trials must be at least 1')
        rows = []
        profiles = (
            [(selected_profile, PROFILES[selected_profile])]
            if selected_profile is not None else list(PROFILES.items())
        )
        for profile, target_speed in profiles:
            self.set_profile(profile)
            for trial in range(1, trials + 1):
                self.reset_robot('SIM_A')
                start = self.ground_truth
                run_id = self.next_run_id(f'straight_{profile}_{trial}')
                command = Twist(); command.linear.x = target_speed
                # Brake conservatively, then use an allowed 0.02 m/s terminal
                # creep. This makes the stop repeatable despite Gazebo Classic
                # delivering /clock in coalesced intervals.
                brake_distance = target_speed * target_speed / (2.0 * 0.50) + 0.050
                threshold = max(0.85, 1.0 - brake_distance)
                self.begin_capture()
                wall_start = time.time(); mono_start = time.monotonic()
                deadline = mono_start + 30.0
                while rclpy.ok() and time.monotonic() < deadline:
                    self.test_pub.publish(command)
                    rclpy.spin_once(self, timeout_sec=0.04)
                    x, y, _ = self.ground_truth
                    longitudinal = ((x - start[0]) * math.cos(start[2]) +
                                    (y - start[1]) * math.sin(start[2]))
                    if longitudinal >= threshold:
                        break
                self.publish_zero(0.8)
                creep = Twist(); creep.linear.x = 0.02
                creep_deadline = time.monotonic() + 8.0
                while rclpy.ok() and time.monotonic() < creep_deadline:
                    x, y, _ = self.ground_truth
                    longitudinal = ((x - start[0]) * math.cos(start[2]) +
                                    (y - start[1]) * math.sin(start[2]))
                    if longitudinal >= 0.992:
                        break
                    self.test_pub.publish(creep)
                    rclpy.spin_once(self, timeout_sec=0.04)
                self.publish_zero(1.0)
                wall_stop = time.time(); finish = self.ground_truth
                self.end_capture()
                capture = self.capture
                self._append_velocity_trace(
                    run_id, 'straight', profile, capture
                )
                longitudinal = ((finish[0] - start[0]) * math.cos(start[2]) +
                                (finish[1] - start[1]) * math.sin(start[2]))
                lateral = (-(finish[0] - start[0]) * math.sin(start[2]) +
                           (finish[1] - start[1]) * math.cos(start[2]))
                yaw_delta = angle_error(finish[2], start[2])
                speed_pass = self._velocity_contract(
                    capture, profile, require_nav_raw=False
                )
                passed = (speed_pass and abs(longitudinal - 1.0) <= 0.02 and
                          abs(lateral) <= 0.02 and yaw_delta <= 0.05)
                rows.append({
                    'run_id': run_id,
                    'profile': profile, 'trial': trial, 'target_distance_m': '1.000',
                    'target_speed_mps': f'{target_speed:.3f}',
                    'max_cmd_vel_nav_raw': f"{capture.maxima['nav_raw']:.6f}",
                    'max_cmd_vel_selected': f"{capture.maxima['selected']:.6f}",
                    'max_cmd_vel_smoothed': f"{capture.maxima['smoothed']:.6f}",
                    'max_cmd_vel_safe': f"{capture.maxima['safe']:.6f}",
                    'velocity_contract_pass': speed_pass,
                    'measurement_source': '/sim/ground_truth',
                    'start_x': f'{start[0]:.6f}', 'start_y': f'{start[1]:.6f}',
                    'start_yaw': f'{start[2]:.6f}', 'final_x': f'{finish[0]:.6f}',
                    'final_y': f'{finish[1]:.6f}', 'final_yaw': f'{finish[2]:.6f}',
                    'longitudinal_m': f'{longitudinal:.6f}',
                    'longitudinal_error_m': f'{abs(longitudinal - 1.0):.6f}',
                    'lateral_error_m': f'{abs(lateral):.6f}',
                    'yaw_error_rad': f'{yaw_delta:.6f}',
                    'start_time_epoch': f'{wall_start:.6f}', 'stop_time_epoch': f'{wall_stop:.6f}',
                    'duration_sec': f'{time.monotonic() - mono_start:.6f}',
                    'controller_mode': capture.mode,
                    'controller_switch_time_monotonic': f'{capture.switch_time:.6f}',
                    'controller_switches': capture.max_switches,
                    'result': 'PASS' if passed else 'FAIL',
                })
                print(f"straight {profile} #{trial}: {rows[-1]['result']} distance={longitudinal:.4f} lateral={lateral:.4f} safe_max={capture.maxima['safe']:.3f}", flush=True)
        self._write_csv(os.path.join(self.output_dir, 'straight_1m_results.csv'), list(rows[0]), rows)
        return rows

    def _goal_message(self, point_name):
        x, y, yaw = POINTS[point_name]
        goal = PoseStamped(); goal.header.frame_id = 'map'; goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x; goal.pose.position.y = y
        goal.pose.orientation.z, goal.pose.orientation.w = quaternion_from_yaw(yaw)
        return goal

    def current_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            translation = transform.transform.translation
            return (
                float(translation.x), float(translation.y),
                yaw_from_quaternion(transform.transform.rotation),
            )
        except TransformException:
            return self.map_pose

    @staticmethod
    def _render_row(row):
        rendered = {}
        for key, value in row.items():
            if value is None:
                rendered[key] = ''
            elif isinstance(value, float):
                rendered[key] = f'{value:.9f}'
            else:
                rendered[key] = value
        return rendered

    def navigate(
            self, profile, start_name, goal_name, timeout=180.0, tick=None,
            scenario='route', event_state=None):
        event_state = event_state if event_state is not None else {}
        self.set_profile(profile)
        self.reset_robot(start_name)
        self.plan_message = None
        self.plan_received_ros_ns = 0
        self.plan_received_mono = 0.0
        goal_pose = self._goal_message(goal_name)
        run_id = self.next_run_id(
            f'{scenario}_{profile}_{start_name}_{goal_name}'
        )
        self.begin_capture()
        started_epoch = time.time()
        started_mono = time.monotonic()
        event_state['action_start_mono'] = started_mono
        goal_sent_ros_ns = self.get_clock().now().nanoseconds
        event_state['action_start_ros_ns'] = goal_sent_ros_ns
        self.goal_pub.publish(goal_pose)
        active = Bool(); active.data = True; self.goal_active_pub.publish(active)
        request = NavigateToPose.Goal(); request.pose = goal_pose
        send_future = self.nav_client.send_goal_async(request)
        if not self.wait_future(send_future, 10.0):
            self.end_capture()
            raise RuntimeError('NavigateToPose goal response timed out')
        handle = send_future.result()
        if not handle.accepted:
            self.end_capture()
            raise RuntimeError('NavigateToPose goal rejected')
        result_future = handle.get_result_async(); deadline = started_mono + timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            if tick is not None:
                tick(time.monotonic() - started_mono)
            rclpy.spin_once(self, timeout_sec=0.05)
        timed_out = not result_future.done()
        result_mono = time.monotonic()
        result_ros_ns = self.get_clock().now().nanoseconds
        result_epoch = time.time()
        if timed_out:
            status = GoalStatus.STATUS_ABORTED
        else:
            status = result_future.result().status

        # Freeze every acceptance input at the instant the action result is
        # observed. Cleanup publishing below must not move the scoring sample.
        # Freeze the exact observed truth message before allowing any cleanup
        # publishing or additional executor work.
        truth_message = self.ground_truth_message
        truth = self._ground_truth_snapshot(result_ros_ns, result_mono)
        plan = self._plan_snapshot()
        self.end_capture()
        capture = self.capture
        truth_time = None
        if truth_message is not None:
            truth_stamp = truth_message.header.stamp
            if stamp_ns(truth_stamp) > 0:
                truth_time = rclpy.time.Time.from_msg(truth_stamp)
        tf_at_truth, tf_query_used_latest = self._tf_snapshot_at_truth(
            truth_time
        )
        # ModelStates and TF are independent DDS streams.  On a loaded run the
        # frozen truth sample can arrive one simulation tick before its matching
        # TF. Spin only to populate the buffer, while continuing to score the
        # immutable result-time truth sample and timestamp.
        tf_wait_deadline = time.monotonic() + 0.50
        while (
            truth_time is not None
            and tf_at_truth['x'] == ''
            and time.monotonic() < tf_wait_deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.02)
            tf_at_truth, tf_query_used_latest = self._tf_snapshot_at_truth(
                truth_time
            )
        tf_latest = self._tf_snapshot('map', 'base_footprint')

        if timed_out:
            cancel = handle.cancel_goal_async(); self.wait_future(cancel, 5.0)
        inactive = Bool(); inactive.data = False; self.goal_active_pub.publish(inactive)
        self.publish_zero(0.8)

        self._append_velocity_trace(run_id, scenario, profile, capture)
        handoff_gap, handoff_pass, switch_reference = self._append_handoff_trace(
            run_id, profile, capture, started_mono
        )
        if switch_reference is None:
            cruise_vy = max(
                (abs(event['vy']) for event in capture.twist_events['nav_raw']),
                default=0.0,
            )
            terminal_vy = 0.0
        else:
            cruise_vy = max((
                abs(event['vy']) for event in capture.twist_events['nav_raw']
                if event['monotonic'] <= switch_reference
            ), default=0.0)
            terminal_vy = max((
                abs(event['vy']) for event in capture.twist_events['nav_raw']
                if event['monotonic'] > switch_reference
            ), default=0.0)
        gx, gy, gyaw = POINTS[goal_name]
        requested_gx = float(goal_pose.pose.position.x)
        requested_gy = float(goal_pose.pose.position.y)
        requested_gyaw = yaw_from_quaternion(goal_pose.pose.orientation)
        requested_to_test_target = math.hypot(
            requested_gx - gx, requested_gy - gy
        )
        requested_to_test_yaw = angle_error(requested_gyaw, gyaw)
        if truth is None:
            finish_x = finish_y = finish_yaw = math.nan
            xy_error = yaw_error = math.inf
            truth_frame_valid = False
            truth_fresh = False
        else:
            finish_x = truth['x']; finish_y = truth['y']; finish_yaw = truth['yaw']
            xy_error = math.hypot(finish_x - gx, finish_y - gy)
            yaw_error = angle_error(finish_yaw, gyaw)
            truth_frame_valid = truth['frame_id'].lstrip('/') in ('map', 'world')
            truth_fresh = abs(truth['age_sec']) <= 0.20
        plan_goal_error = (
            math.hypot(plan['x'] - requested_gx, plan['y'] - requested_gy)
            if plan is not None else math.inf
        )
        plan_goal_yaw_error = (
            angle_error(plan['yaw'], requested_gyaw)
            if plan is not None else math.inf
        )
        tf_xy_error = (
            math.hypot(tf_at_truth['x'] - gx, tf_at_truth['y'] - gy)
            if tf_at_truth['x'] != '' else math.inf
        )
        tf_yaw_error = (
            angle_error(tf_at_truth['yaw'], gyaw)
            if tf_at_truth['yaw'] != '' else math.inf
        )
        truth_to_tf_xy = (
            math.hypot(tf_at_truth['x'] - finish_x, tf_at_truth['y'] - finish_y)
            if tf_at_truth['x'] != '' and math.isfinite(finish_x) else math.inf
        )
        truth_to_tf_yaw = (
            angle_error(tf_at_truth['yaw'], finish_yaw)
            if tf_at_truth['yaw'] != '' and math.isfinite(finish_yaw) else math.inf
        )
        velocity_pass = self._velocity_contract(capture, profile)
        navigation_contract_pass = (
            status == GoalStatus.STATUS_SUCCEEDED
            and not timed_out
            and truth_frame_valid
            and truth_fresh
            and plan is not None
            and tf_latest['x'] != ''
            and tf_at_truth['x'] != ''
            and requested_to_test_target <= 1.0e-6
            and requested_to_test_yaw <= 1.0e-6
            and plan_goal_error <= ROUTE_XY_TOLERANCE
            and plan_goal_yaw_error <= ROUTE_YAW_TOLERANCE
            and xy_error <= ROUTE_XY_TOLERANCE
            and yaw_error <= ROUTE_YAW_TOLERANCE
            and truth_to_tf_xy <= ROUTE_XY_TOLERANCE
            and truth_to_tf_yaw <= ROUTE_YAW_TOLERANCE
            and capture.max_switches <= 1
            and cruise_vy <= 0.01 + VELOCITY_EPSILON
            and bool(capture.scan_events)
            and velocity_pass
            and handoff_pass
        )
        dynamic = {}
        scenario_contract_pass = navigation_contract_pass
        if scenario != 'route':
            dynamic = self._dynamic_metrics(
                capture, started_mono, result_mono, event_state, profile
            )
            if scenario == 'pedestrian_crossing':
                scenario_contract_pass = (
                    navigation_contract_pass
                    and dynamic['pedestrian_encounter_detected']
                    and dynamic['avoidance_response_detected']
                    and not dynamic['collision_detected']
                )
            if scenario == 'full_blockage_wait_resume':
                scenario_contract_pass = (
                    navigation_contract_pass
                    and dynamic['blocker_in_route_observed']
                    and dynamic['blocker_appeared_after_motion']
                    and dynamic['blocker_removed_observed']
                    and not dynamic['blocker_overlap_detected']
                    and dynamic['stop_wait_detected']
                    and dynamic['physical_stop_verified']
                    and dynamic['waited_until_obstacle_clear']
                    and dynamic['wait_action_observed']
                    and dynamic['wait_removal_trigger_verified']
                    and dynamic['resume_after_clear']
                    and dynamic['replan_after_clear']
                )

        result = {
            'run_id': run_id,
            'scenario': scenario,
            'profile': profile, 'start': start_name, 'goal': goal_name,
            'action_status': status, 'action_success': status == GoalStatus.STATUS_SUCCEEDED,
            'action_result_observed': not timed_out,
            'timed_out': timed_out, 'duration_sec': result_mono - started_mono,
            'start_time_epoch': started_epoch, 'stop_time_epoch': result_epoch,
            'action_start_monotonic_sec': started_mono,
            'action_result_monotonic_sec': result_mono,
            'goal_sent_ros_ns': goal_sent_ros_ns,
            'action_result_ros_ns': result_ros_ns,
            'requested_goal_frame': goal_pose.header.frame_id,
            'requested_goal_stamp_ns': stamp_ns(goal_pose.header.stamp),
            'requested_goal_x': requested_gx,
            'requested_goal_y': requested_gy,
            'requested_goal_yaw': requested_gyaw,
            'test_target_x': gx,
            'test_target_y': gy,
            'test_target_yaw': gyaw,
            'requested_goal_to_test_target_xy_m': requested_to_test_target,
            'requested_goal_to_test_target_yaw_rad': requested_to_test_yaw,
            'plan_frame': plan['frame_id'] if plan else '',
            'plan_stamp_ns': plan['stamp_ns'] if plan else '',
            'plan_received_ros_ns': plan['received_ros_ns'] if plan else '',
            'plan_received_monotonic_sec': plan['received_mono'] if plan else '',
            'plan_pose_count': plan['pose_count'] if plan else 0,
            'plan_end_x': plan['x'] if plan else '',
            'plan_end_y': plan['y'] if plan else '',
            'plan_end_yaw': plan['yaw'] if plan else '',
            'requested_goal_to_plan_end_xy_m': plan_goal_error,
            'requested_goal_to_plan_end_yaw_rad': plan_goal_yaw_error,
            'measurement_source': '/sim/ground_truth',
            'truth_frame': truth['frame_id'] if truth else '',
            'truth_child_frame': truth['child_frame_id'] if truth else '',
            'truth_stamp_ns': truth['stamp_ns'] if truth else '',
            'truth_received_ros_ns': truth['received_ros_ns'] if truth else '',
            'truth_age_at_result_sec': truth['age_sec'] if truth else math.inf,
            'truth_frame_valid': truth_frame_valid,
            'truth_fresh_at_result': truth_fresh,
            'final_x': finish_x, 'final_y': finish_y, 'final_yaw': finish_yaw,
            'xy_error_m': xy_error, 'yaw_error_rad': yaw_error,
            'xy_limit_m': ROUTE_XY_TOLERANCE,
            'yaw_limit_rad': ROUTE_YAW_TOLERANCE,
            'tf_query_truth_stamp_ns': (
                truth['stamp_ns'] if truth and truth['stamp_ns'] > 0 else ''
            ),
            'tf_query_used_latest': tf_query_used_latest,
            'tf_map_base_at_truth_stamp_ns': tf_at_truth['stamp_ns'],
            'tf_map_base_at_truth_x': tf_at_truth['x'],
            'tf_map_base_at_truth_y': tf_at_truth['y'],
            'tf_map_base_at_truth_yaw': tf_at_truth['yaw'],
            'tf_map_base_at_truth_xy_error_m': tf_xy_error,
            'tf_map_base_at_truth_yaw_error_rad': tf_yaw_error,
            'truth_to_tf_at_truth_xy_m': truth_to_tf_xy,
            'truth_to_tf_at_truth_yaw_rad': truth_to_tf_yaw,
            'tf_map_base_at_truth_error': tf_at_truth['error'],
            'tf_map_base_latest_stamp_ns': tf_latest['stamp_ns'],
            'tf_map_base_latest_x': tf_latest['x'],
            'tf_map_base_latest_y': tf_latest['y'],
            'tf_map_base_latest_yaw': tf_latest['yaw'],
            'tf_map_base_latest_error': tf_latest['error'],
            'max_nav_controller': capture.maxima['controller'],
            'max_nav_raw': capture.maxima['nav_raw'],
            'max_selected': capture.maxima['selected'],
            'max_smoothed': capture.maxima['smoothed'],
            'max_safe': capture.maxima['safe'],
            'raw_nav_limit_mps': ABSOLUTE_MAX_SPEED,
            'downstream_profile_limit_mps': PROFILES[profile],
            'nav_controller_samples': len(capture.twist_events['controller']),
            'nav_raw_samples': len(capture.twist_events['nav_raw']),
            'selected_samples': len(capture.twist_events['selected']),
            'smoothed_samples': len(capture.twist_events['smoothed']),
            'safe_samples': len(capture.twist_events['safe']),
            'velocity_contract_pass': velocity_pass,
            'max_cruise_vy': cruise_vy,
            'max_terminal_vy': terminal_vy,
            'controller_switches': capture.max_switches,
            'controller_switch_time_monotonic': capture.switch_time,
            'handoff_measurement': (
                'conservative selector-request to controller-server-ack envelope'
            ),
            'max_handoff_gap_sec': handoff_gap,
            'handoff_contract_pass': handoff_pass,
            'min_scan_m': capture.min_scan,
            'scan_samples': len(capture.scan_events),
            'scan_evidence_available': bool(capture.scan_events),
            'navigation_contract_pass': navigation_contract_pass,
            **dynamic,
            'result': 'PASS' if scenario_contract_pass else 'FAIL',
        }
        return result

    def run_routes(self):
        rows = []
        for profile, start, goal in ROUTES:
            # A fixed 180 s budget is not speed-profile neutral: on the long
            # A->C route the 0.06 m/s profile can legitimately spend extended
            # time at half speed near corridor walls while Collision Monitor
            # remains active.  Keep every safety polygon and endpoint contract
            # unchanged, and scale only the low-profile observation window.
            row = self.navigate(
                profile, start, goal,
                timeout=ROUTE_TIMEOUTS[profile],
            )
            rows.append(self._render_row(row))
            self._write_csv(
                os.path.join(self.output_dir, 'route_results.csv'),
                list(rows[0]), rows,
            )
            print(
                f"route {profile} {start}->{goal}: {row['result']} "
                f"status={row['action_status']} xy={row['xy_error_m']:.4f} "
                f"yaw={row['yaw_error_rad']:.4f} "
                f"tf_xy={row['tf_map_base_at_truth_xy_error_m']:.4f} "
                f"truth_tf={row['truth_to_tf_at_truth_xy_m']:.4f} "
                f"switches={row['controller_switches']}",
                flush=True,
            )
        return rows

    def pedestrian(self, enabled):
        request = SetBool.Request(); request.data = enabled
        future = self.pedestrian_client.call_async(request)
        if not self.wait_future(future, 5.0) or not future.result().success:
            raise RuntimeError('pedestrian control failed')
        return time.monotonic()

    def _write_dynamic_results(self, rows):
        rendered = [self._render_row(item) for item in rows]
        all_fields = []
        for item in rendered:
            for key in item:
                if key not in all_fields:
                    all_fields.append(key)
        self._write_csv(
            os.path.join(self.output_dir, 'dynamic_obstacle_results.csv'),
            all_fields,
            rendered,
        )
        return rendered

    def run_dynamic(self):
        rows = []
        for profile in ('normal', 'fast'):
            self.pedestrian(False)
            event_state = {'appeared_mono': None}
            trigger_x = -0.90 if profile == 'normal' else -1.10

            def manage_pedestrian(_elapsed, threshold=trigger_x):
                if event_state['appeared_mono'] is not None:
                    return
                action_start = event_state.get('action_start_mono')
                if action_start is None or self.ground_truth is None:
                    return
                motion_seen = any(
                    event['monotonic'] >= action_start
                    and event['motion_abs_max'] > RESUME_MOTION_THRESHOLD
                    for event in self.capture.twist_events['safe']
                )
                # Trigger a single crossing only after this navigation action
                # has produced real motion.  The speed-specific lead distance
                # gives both profiles the same physical reaction window.
                if motion_seen and self.ground_truth[0] >= threshold:
                    event_state['appeared_mono'] = self.pedestrian(True)
            try:
                row = self.navigate(
                    profile, 'SIM_A', 'SIM_B',
                    scenario='pedestrian_crossing', event_state=event_state,
                    tick=manage_pedestrian,
                )
            finally:
                disabled_mono = self.pedestrian(False)
            row['pedestrian_motion_disabled_after_action_monotonic_sec'] = disabled_mono
            rows.append(row)
            self._write_dynamic_results(rows)
            print(f"dynamic crossing {profile}: {row['result']}", flush=True)
        event_state = {
            'appeared_mono': None,
            'removed_mono': None,
            'motion_seen_before_appearance': False,
        }
        self.set_entity('route_blocker', 0.0, -6.0, 0.0, z=0.6)
        def manage_blocker(elapsed):
            if event_state['removed_mono'] is not None:
                return
            action_start = event_state.get('action_start_mono')
            if action_start is None:
                return
            now = time.monotonic()
            if event_state['appeared_mono'] is None:
                motion_seen = any(
                    event['monotonic'] >= action_start
                    and event['motion_abs_max'] > RESUME_MOTION_THRESHOLD
                    for event in self.capture.twist_events['safe']
                )
                # Establish an unobstructed plan and real forward motion first;
                # the blocker then appears dynamically ahead of the vehicle.
                if elapsed >= 8.0 and motion_seen:
                    event_state['motion_seen_before_appearance'] = True
                    event_state['appeared_mono'] = self.set_entity(
                        'route_blocker', 0.0, -2.0, 0.0, z=0.6
                    )
                return
            segments = self._zero_segments(
                self.capture.twist_events['safe'], action_start, now
            )
            if not segments or not self.capture.twist_events['safe']:
                return
            latest_stop = segments[-1]
            latest_safe = self.capture.twist_events['safe'][-1]
            if (
                now - latest_stop[1] > 0.25
                or
                now - latest_safe['monotonic'] > 0.25
                or latest_safe['motion_abs_max'] > STOP_MOTION_THRESHOLD
            ):
                return
            physical_events = [
                event for event in self.capture.blocker_state_events
                if max(latest_stop[0], now - 0.60)
                <= event['monotonic'] <= now
            ]
            if len(physical_events) < 5:
                return
            first_pose = physical_events[0]
            physical_translation = max((
                math.hypot(
                    event['robot_x'] - first_pose['robot_x'],
                    event['robot_y'] - first_pose['robot_y'],
                )
                for event in physical_events
            ), default=math.inf)
            physical_yaw_change = max((
                angle_error(event['robot_yaw'], first_pose['robot_yaw'])
                for event in physical_events
            ), default=math.inf)
            if (
                not physical_events[-1]['in_route']
                or physical_translation > 0.005
                or physical_yaw_change > 0.01
            ):
                return
            wait_success = next((
                event for event in reversed(self.capture.wait_status_events)
                if event['status'] == GoalStatus.STATUS_SUCCEEDED
                and event['goal_stamp_ns']
                >= event_state.get('action_start_ros_ns', 0)
                and latest_stop[0] <= event['observed_mono']
                <= latest_stop[1] + 0.20
            ), None)
            if wait_success is None:
                return
            event_state['trigger_wait_goal_id'] = wait_success['goal_id']
            event_state['trigger_wait_succeeded_mono'] = (
                wait_success['observed_mono']
            )
            event_state['trigger_stop_start_mono'] = latest_stop[0]
            event_state['physical_stop_verified'] = True
            event_state['physical_stop_samples'] = len(physical_events)
            event_state['physical_stop_translation_m'] = physical_translation
            event_state['physical_stop_yaw_change_rad'] = physical_yaw_change
            event_state['removed_mono'] = self.set_entity(
                'route_blocker', 0.0, -6.0, 0.0, z=0.6
            )
        try:
            row = self.navigate(
                'normal', 'SIM_A', 'SIM_B', timeout=190.0,
                tick=manage_blocker,
                scenario='full_blockage_wait_resume',
                event_state=event_state,
            )
        finally:
            self.set_entity('route_blocker', 0.0, -6.0, 0.0, z=0.6)
        row['blocker_removed'] = event_state['removed_mono'] is not None
        rows.append(row)
        rendered = self._write_dynamic_results(rows)
        print(f"dynamic full blockage: {row['result']}", flush=True)
        return rendered


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite', choices=['straight', 'route', 'dynamic', 'all'], default='all')
    parser.add_argument('--output-dir', default='reports/ros2_r1')
    parser.add_argument(
        '--straight-profile', choices=['all', *PROFILES], default='all',
        help='limit the straight suite without changing legacy defaults',
    )
    parser.add_argument(
        '--straight-trials', type=int, default=3,
        help='trials per selected straight profile (legacy default: 3)',
    )
    arguments = parser.parse_args()
    rclpy.init(); node = Benchmark(arguments.output_dir)
    try:
        results = []
        node.wait_services()
        if arguments.suite in ('straight', 'all'):
            results.extend(node.run_straight(
                None if arguments.straight_profile == 'all'
                else arguments.straight_profile,
                arguments.straight_trials,
            ))
        if arguments.suite in ('route', 'all'):
            results.extend(node.run_routes())
        if arguments.suite in ('dynamic', 'all'):
            results.extend(node.run_dynamic())
        failures = [
            row.get('run_id', '<unknown>')
            for row in results if row.get('result') != 'PASS'
        ]
        if failures:
            node.get_logger().error(
                'acceptance contract failures: ' + ', '.join(failures)
            )
            return 1
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc)); return 2
    finally:
        node.publish_zero(0.4)
        node.close_tf_listener()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
