import json
import math
import os
import queue
import time

import rclpy
from action_msgs.msg import GoalStatus
from c50c_interfaces.msg import BaseStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QThread, pyqtSignal
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: '未知',
    GoalStatus.STATUS_ACCEPTED: '已接受',
    GoalStatus.STATUS_EXECUTING: '执行中',
    GoalStatus.STATUS_CANCELING: '正在取消',
    GoalStatus.STATUS_SUCCEEDED: '已完成',
    GoalStatus.STATUS_CANCELED: '已取消',
    GoalStatus.STATUS_ABORTED: '已中止',
}

CONTROL_STATE_NAMES = {
    0: '启动中',
    1: '已锁定',
    2: '已就绪',
    3: '运行中',
    4: '已停止',
    5: '急停',
    6: '故障锁定',
}

FAULT_NAMES = {
    1 << 0: '指令超时',
    1 << 1: '编码器故障',
    1 << 2: '电压过低',
    1 << 3: '电机控制故障（可清除堵转；硬件故障仍需重启）',
    1 << 4: '通信故障',
    1 << 5: '急停已触发',
}


def yaw_from_quaternion(rotation):
    siny = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def planar_distance(first, second):
    return math.hypot(
        float(first['x']) - float(second['x']),
        float(first['y']) - float(second['y']),
    )


def build_soft_waypoint_route(
    current, waypoints, target,
    nearby_radius=0.80,
    min_progress=0.05,
    max_leg_detour=1.50,
):
    """Choose a direction-aware waypoint chain without requiring exact hits.

    A nearby waypoint is used as the route entry even when it is marginally
    farther from the destination.  After that entry, every waypoint must make
    meaningful progress toward the final target and must not introduce a large
    detour.  This naturally reverses a corridor's waypoint order on the return
    trip and avoids driving back to the first stored waypoint.
    """
    candidates = [dict(point) for point in waypoints]
    if not candidates or current is None or target is None:
        return candidates

    route = []
    cursor = dict(current)
    nearest = min(candidates, key=lambda point: planar_distance(cursor, point))
    nearest_distance = planar_distance(cursor, nearest)
    direct_distance = planar_distance(cursor, target)
    if nearest_distance <= float(nearby_radius):
        route.append(nearest)
        candidates.remove(nearest)
        cursor = nearest
    else:
        forward = [
            point for point in candidates
            if planar_distance(point, target) < direct_distance - float(min_progress)
            and (
                planar_distance(cursor, point) + planar_distance(point, target)
                <= direct_distance + float(max_leg_detour)
            )
        ]
        if not forward:
            return []
        entry = min(forward, key=lambda point: planar_distance(cursor, point))
        route.append(entry)
        candidates.remove(entry)
        cursor = entry

    while candidates:
        cursor_to_target = planar_distance(cursor, target)
        forward = [
            point for point in candidates
            if planar_distance(point, target) < cursor_to_target - float(min_progress)
            and (
                planar_distance(cursor, point) + planar_distance(point, target)
                <= cursor_to_target + float(max_leg_detour)
            )
        ]
        if not forward:
            break
        next_point = min(
            forward,
            key=lambda point: (
                planar_distance(cursor, point),
                planar_distance(point, target),
            ),
        )
        route.append(next_point)
        candidates.remove(next_point)
        cursor = next_point
    return route


def should_resume_after_base_fault(status, last_fault_at, now, attempts):
    return (
        status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
        and float(last_fault_at) > 0.0
        and float(now) - float(last_fault_at) <= 20.0
        and int(attempts) < 3
    )


class RosWorker(QThread):
    status_changed = pyqtSignal(dict)
    pose_snapshot = pyqtSignal(dict)
    configuration = pyqtSignal(dict)
    error = pyqtSignal(str)
    arm_status = pyqtSignal(str)
    camera_image = pyqtSignal(str, bytes)
    alignment_status = pyqtSignal(dict)
    navigation_arrived = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._commands = queue.SimpleQueue()
        self._stop_requested = False
        self._node = None
        self._executor = None
        self._manual = Twist()
        self._manual_active = False
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._route_goals = []
        self._route_position = 0
        self._route_start_at = 0.0
        self._active_goal_serial = 0
        self._active_is_intermediate = False
        self._active_goal_point = None
        self._intermediate_transition_pending = False
        self._intermediate_transition_serial = 0
        self._intermediate_transition_started_at = 0.0
        self._waypoint_tolerance = 0.45
        self._arm_initialize_wait_sec = 3.5
        self._last_recoverable_fault_at = 0.0
        self._fault_resume_at = 0.0
        self._fault_resume_deadline = 0.0
        self._fault_resume_attempts = 0
        self._latest_odom = None
        self._latest_map_pose = None
        self._base_status = None
        self._base_status_at = 0.0
        self._localization_guard = {}
        self._localization_guard_at = 0.0
        self._last_reported_fault_bits = 0
        self._arbiter = {}
        self._mode = {}
        self._collision_seen_monotonic = 0.0
        self._safe_linear_speed = 0.0
        self._nav_state = 'UNKNOWN'
        self._nav_lifecycle = 'UNKNOWN'
        self._nav_lifecycle_at = 0.0
        self._nav_query_future = None

    def set_manual(self, vx, vy, wz, active=True):
        self._commands.put(('manual', float(vx), float(vy), float(wz), bool(active)))

    def stop_manual(self):
        self._commands.put(('manual', 0.0, 0.0, 0.0, False))

    def set_speed_profile(self, profile):
        self._commands.put(('speed', str(profile)))

    def set_estop(self, enabled):
        self._commands.put(('estop', bool(enabled)))

    def recover_base(self):
        self._commands.put(('recover_base',))

    def navigate(self, point):
        self._commands.put(('navigate', dict(point)))

    def navigate_route(self, waypoints, target):
        self._commands.put((
            'navigate_route',
            [dict(point) for point in waypoints],
            dict(target),
        ))

    def update_points(self, points):
        self._commands.put(('points', [dict(point) for point in points]))

    def cancel_navigation(self):
        self._commands.put(('cancel',))

    def request_pose(self):
        self._commands.put(('pose',))

    def send_arm_command(self, payload):
        self._commands.put(('arm', dict(payload)))

    def stop_arm(self):
        self.send_arm_command({'type': 'stop'})

    def stop_worker(self):
        self._commands.put(('shutdown',))

    def run(self):
        try:
            rclpy.init(args=None)
            self._node = rclpy.create_node('roscar_operator_gui')
            self._node.declare_parameter(
                'points_file', '~/.ros/roscar_data/calibration_points.yaml'
            )
            self._node.declare_parameter(
                'map_yaml', os.path.expanduser('~/roscar_maps/active.yaml')
            )
            self._node.declare_parameter('odom_topic', '/odom')
            # Navigation points are soft corridor hints.  Advance before the
            # mode manager's 0.40 m terminal controller starts exact alignment.
            self._node.declare_parameter('waypoint_tolerance', 0.45)
            self._node.declare_parameter('arm_initialize_wait_sec', 3.5)
            self._waypoint_tolerance = float(
                self._node.get_parameter('waypoint_tolerance').value
            )
            self._arm_initialize_wait_sec = float(
                self._node.get_parameter('arm_initialize_wait_sec').value
            )
            self.configuration.emit({
                'points_file': str(self._node.get_parameter('points_file').value),
                'map_yaml': str(self._node.get_parameter('map_yaml').value),
                'odom_topic': str(self._node.get_parameter('odom_topic').value),
            })
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._setup_ros()
            last_manual = 0.0
            last_status = 0.0
            last_tf = 0.0
            last_lifecycle = 0.0
            while rclpy.ok() and not self._stop_requested:
                self._drain_commands()
                self._executor.spin_once(timeout_sec=0.015)
                now = time.monotonic()
                self._check_navigation_health(now)
                if self._route_start_at and now >= self._route_start_at:
                    self._route_start_at = 0.0
                    self._send_current_route_goal()
                if self._fault_resume_at and now >= self._fault_resume_at:
                    self._resume_after_base_fault(now)
                # Publish at 20 Hz only while a key is held. Continuously
                # publishing an inactive zero would keep the arbiter's manual
                # lease fresh and incorrectly suppress Nav2 forever. A key
                # release is handled in _drain_commands by one immediate zero;
                # the 0.25 s arbiter freshness lease then expires normally.
                if self._manual_active and now - last_manual >= 0.05:
                    self._manual_pub.publish(self._manual)
                    last_manual = now
                if now - last_tf >= 0.20:
                    self._update_map_pose()
                    self._check_intermediate_progress()
                    last_tf = now
                if now - last_lifecycle >= 1.0:
                    self._query_lifecycle()
                    last_lifecycle = now
                if now - last_status >= 0.20:
                    self._emit_status(now)
                    last_status = now
        except Exception as exc:
            self.error.emit(f'ROS 后台线程失败：{exc}')
        finally:
            self._publish_zero()
            if self._executor is not None and self._node is not None:
                self._executor.remove_node(self._node)
            if self._node is not None:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _setup_ros(self):
        durable = QoSProfile(depth=1)
        durable.reliability = ReliabilityPolicy.RELIABLE
        durable.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._manual_pub = self._node.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._goal_pub = self._node.create_publisher(PoseStamped, '/roscar/active_goal', durable)
        self._goal_active_pub = self._node.create_publisher(
            Bool, '/roscar/goal_active', durable
        )
        self._marker_pub = self._node.create_publisher(
            MarkerArray, '/roscar/calibration_markers', durable
        )
        self._arm_pub = self._node.create_publisher(String, '/roscar_arm/command', 10)
        odom_topic = str(self._node.get_parameter('odom_topic').value)
        self._node.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self._node.create_subscription(
            BaseStatus, '/c50c/status', self._base_status_callback, 10
        )
        self._node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, 10
        )
        self._node.create_subscription(
            String, '/roscar/command_arbiter/status', self._arbiter_callback, 10
        )
        self._node.create_subscription(
            String, '/roscar/navigation_mode/status', self._mode_callback, 10
        )
        self._node.create_subscription(
            String, '/roscar/localization_guard/status',
            self._localization_guard_callback, durable,
        )
        self._node.create_subscription(Twist, '/cmd_vel_safe', self._safe_cmd_callback, 10)
        self._node.create_subscription(
            String, '/roscar_arm/status', self._arm_status_callback, 10
        )
        sensor = QoSProfile(depth=1)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor.durability = DurabilityPolicy.VOLATILE
        self._node.create_subscription(
            CompressedImage,
            '/astra/color/image_raw/compressed',
            lambda message: self.camera_image.emit('astra', bytes(message.data)),
            sensor,
        )
        self._node.create_subscription(
            CompressedImage,
            '/gemini/alignment/image_jpeg',
            lambda message: self.camera_image.emit('gemini', bytes(message.data)),
            sensor,
        )
        self._node.create_subscription(
            String,
            '/gemini/alignment/status',
            self._alignment_status_callback,
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        self._navigate_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._speed_client = self._node.create_client(
            SetParameters, '/roscar_command_arbiter/set_parameters'
        )
        self._estop_client = self._node.create_client(SetBool, '/roscar/set_estop')
        self._clear_fault_client = self._node.create_client(
            Trigger, '/roscar/base/clear_fault'
        )
        self._rearm_client = self._node.create_client(
            Trigger, '/roscar/base/rearm'
        )
        self._lifecycle_client = self._node.create_client(GetState, '/bt_navigator/get_state')

    def _odom_callback(self, message):
        pose = message.pose.pose
        self._latest_odom = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
        }

    def _amcl_callback(self, message):
        pose = message.pose.pose
        self._latest_map_pose = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
            'frame_id': message.header.frame_id or 'map',
        }

    def _base_status_callback(self, message):
        self._base_status_at = time.monotonic()
        faults = int(message.fault_bits)
        names = [name for bit, name in FAULT_NAMES.items() if faults & bit]
        unknown = faults & ~sum(FAULT_NAMES)
        if unknown:
            names.append(f'未知故障 0x{unknown:08X}')
        self._base_status = {
            'fault_bits': faults,
            'control_state': int(message.control_state),
            'state': CONTROL_STATE_NAMES.get(
                int(message.control_state), f'未知状态 {int(message.control_state)}'
            ),
            'fault': '无故障' if not names else '；'.join(names),
        }
        if faults and faults != self._last_reported_fault_bits:
            self.error.emit(f'底盘已停止：{self._base_status["fault"]}')
        if faults in (2, 8, 10):
            self._last_recoverable_fault_at = time.monotonic()
        self._last_reported_fault_bits = faults

    def _arbiter_callback(self, message):
        try:
            self._arbiter = json.loads(message.data)
        except (TypeError, ValueError):
            self._arbiter = {'source': message.data}

    def _mode_callback(self, message):
        try:
            self._mode = json.loads(message.data)
        except (TypeError, ValueError):
            self._mode = {'mode': message.data}

    def _localization_guard_callback(self, message):
        try:
            status = json.loads(message.data)
            self._localization_guard = status if isinstance(status, dict) else {}
        except (TypeError, ValueError):
            self._localization_guard = {}
        self._localization_guard_at = time.monotonic()

    def _safe_cmd_callback(self, message):
        self._collision_seen_monotonic = time.monotonic()
        self._safe_linear_speed = math.hypot(
            float(message.linear.x), float(message.linear.y)
        )

    def _arm_status_callback(self, message):
        try:
            payload = json.loads(message.data)
            text = str(payload.get('message') or message.data)
        except (TypeError, ValueError):
            payload = {}
            text = message.data
        if self._route_start_at and payload.get('success') is False:
            self._route_start_at = 0.0
            self._route_goals = []
            self._nav_state = '机械臂初始化失败，导航未启动'
            self.error.emit(self._nav_state)
        self.arm_status.emit(text)

    def _alignment_status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {'state': '未知', 'message': message.data}
        self.alignment_status.emit(payload)

    def _update_map_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
            self._latest_map_pose = {
                'x': float(transform.transform.translation.x),
                'y': float(transform.transform.translation.y),
                'yaw': yaw_from_quaternion(transform.transform.rotation),
                'frame_id': 'map',
            }
        except TransformException:
            pass

    def _query_lifecycle(self):
        if self._nav_query_future is not None and not self._nav_query_future.done():
            return
        if not self._lifecycle_client.service_is_ready():
            self._nav_lifecycle = 'UNAVAILABLE'
            return
        self._nav_query_future = self._lifecycle_client.call_async(GetState.Request())
        self._nav_query_future.add_done_callback(self._lifecycle_done)

    def _lifecycle_done(self, future):
        try:
            self._nav_lifecycle = str(future.result().current_state.label).upper()
        except Exception:
            self._nav_lifecycle = 'ERROR'
        self._nav_lifecycle_at = time.monotonic()

    def _emit_status(self, now):
        node_names = set(self._node.get_node_names())
        status = {
            'graph_connected': len(node_names) > 1,
            'base_state': (
                self._base_status['state'] if self._base_status else '等待反馈'
            ),
            'base_fault': (
                self._base_status['fault'] if self._base_status else '等待反馈'
            ),
            'nav_lifecycle': self._nav_lifecycle,
            'odom': self._latest_odom,
            'map_pose': self._latest_map_pose,
            'control_source': self._arbiter.get('source', 'UNKNOWN'),
            'speed_profile': self._arbiter.get('speed_profile', 'UNKNOWN'),
            'actual_linear_speed': f'{self._safe_linear_speed:.3f} m/s',
            'controller_mode': self._mode.get('mode', 'UNKNOWN'),
            'controller_switches': self._mode.get('switch_count', 0),
            'collision_monitor': (
                'ACTIVE' if now - self._collision_seen_monotonic < 0.5 else 'NO_OUTPUT'
            ),
            'navigation': self._nav_state if self._goal_handle is None else 'GOAL_ACTIVE',
        }
        self.status_changed.emit(status)

    def _drain_commands(self):
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            kind = command[0]
            if kind == 'manual':
                self._manual = Twist()
                self._manual.linear.x = command[1]
                self._manual.linear.y = command[2]
                self._manual.angular.z = command[3]
                self._manual_active = command[4]
                if not self._manual_active:
                    self._manual_pub.publish(Twist())
            elif kind == 'speed':
                self._request_speed(command[1])
            elif kind == 'estop':
                self._request_estop(command[1])
            elif kind == 'recover_base':
                self._manual_active = False
                self._publish_zero()
                self._request_base_recovery()
            elif kind == 'navigate':
                self._start_route([], command[1])
            elif kind == 'navigate_route':
                self._start_route(command[1], command[2])
            elif kind == 'points':
                self._publish_point_markers(command[1])
            elif kind == 'cancel':
                self._cancel_goal()
            elif kind == 'pose':
                if self._latest_map_pose is None:
                    self.error.emit('当前无法获取 map → base_footprint 位姿')
                else:
                    self.pose_snapshot.emit(dict(self._latest_map_pose))
            elif kind == 'arm':
                self._publish_arm_command(command[1])
            elif kind == 'shutdown':
                self._manual_active = False
                self._cancel_goal()
                self._publish_zero()
                message = String()
                message.data = json.dumps({'type': 'stop'})
                self._arm_pub.publish(message)
                self._stop_requested = True

    def _publish_arm_command(self, payload):
        if self._arm_pub.get_subscription_count() == 0:
            self.arm_status.emit('机械臂驱动未在线')
            return False
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._arm_pub.publish(message)
        return True

    def _request_speed(self, profile):
        if profile not in ('low', 'normal', 'fast'):
            self.error.emit('速度档位无效')
            return
        if not self._speed_client.service_is_ready():
            self.error.emit('控制仲裁器参数服务不可用')
            return
        request = SetParameters.Request()
        request.parameters = [ParameterMessage(
            name='speed_profile',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value=profile,
            ),
        )]
        self._speed_client.call_async(request)

    def _request_estop(self, enabled):
        if not self._estop_client.service_is_ready():
            self.error.emit('急停服务不可用')
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        self._estop_client.call_async(request)

    def _request_base_recovery(self):
        if not self._clear_fault_client.service_is_ready():
            self.error.emit('底盘故障清除服务不可用')
            return
        future = self._clear_fault_client.call_async(Trigger.Request())
        future.add_done_callback(self._clear_fault_requested)

    def _clear_fault_requested(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.error.emit(f'底盘故障清除请求失败：{exc}')
            return
        if not response.success:
            self.error.emit(f'底盘故障清除被拒绝：{response.message}')
            return
        if not self._rearm_client.service_is_ready():
            self.error.emit('故障清除已发送，但底盘重新就绪服务不可用')
            return
        rearm = self._rearm_client.call_async(Trigger.Request())
        rearm.add_done_callback(self._rearm_requested)

    def _rearm_requested(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.error.emit(f'底盘重新就绪请求失败：{exc}')
            return
        if response.success:
            self.error.emit('已发送清故障、零速度和重新就绪指令；等待底盘反馈')
        else:
            self.error.emit(f'底盘重新就绪被拒绝：{response.message}')

    def _start_route(self, waypoints, target):
        self.navigation_arrived.emit(False)
        self._cancel_goal()
        reason = self._navigation_not_ready(time.monotonic())
        if reason:
            self._stop_unready_navigation(reason)
            return
        self._fault_resume_attempts = 0
        selected_waypoints = build_soft_waypoint_route(
            self._latest_map_pose, waypoints, target
        )
        self._route_goals = [
            (dict(point), True) for point in selected_waypoints
        ] + [(dict(target), False)]
        self._route_position = 0
        if not self._publish_arm_command({
            'type': 'stored', 'start': 1, 'end': 1, 'repeat': 1,
        }):
            self.error.emit('机械臂未初始化，导航未启动')
            self._route_goals = []
            return
        self._route_start_at = time.monotonic() + self._arm_initialize_wait_sec
        self._nav_state = '机械臂初始化中，随后开始导航'

    def _send_current_route_goal(self):
        if self._route_position >= len(self._route_goals):
            self._nav_state = '已完成'
            self._goal_handle = None
            self._publish_goal_inactive()
            return
        reason = self._navigation_not_ready(time.monotonic())
        if reason:
            self._stop_unready_navigation(reason)
            return
        point, is_intermediate = self._route_goals[self._route_position]
        pose = PoseStamped()
        pose.header.frame_id = str(point.get('frame_id') or 'map')
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        z, w = quaternion_from_yaw(float(point['yaw']))
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        announced_pose = pose
        if is_intermediate:
            final = self._route_goals[-1][0]
            announced_pose = PoseStamped()
            announced_pose.header.frame_id = str(final.get('frame_id') or 'map')
            announced_pose.header.stamp = pose.header.stamp
            announced_pose.pose.position.x = float(final['x'])
            announced_pose.pose.position.y = float(final['y'])
            z, w = quaternion_from_yaw(float(final['yaw']))
            announced_pose.pose.orientation.z = z
            announced_pose.pose.orientation.w = w
        self._goal_pub.publish(announced_pose)
        active = Bool()
        active.data = True
        self._goal_active_pub.publish(active)
        self._active_goal_serial += 1
        serial = self._active_goal_serial
        self._active_is_intermediate = bool(is_intermediate)
        self._active_goal_point = dict(point)
        self._intermediate_transition_pending = False
        if is_intermediate:
            waypoint_number = self._route_position + 1
            waypoint_total = max(0, len(self._route_goals) - 1)
            self._nav_state = f'前往导航点 {waypoint_number}/{waypoint_total}'
        else:
            self._nav_state = f'前往目标点“{point.get("name", "未命名")}”'
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._goal_future = self._navigate_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, goal_serial=serial:
                self._navigation_feedback(feedback, goal_serial),
        )
        self._goal_future.add_done_callback(
            lambda future, goal_serial=serial:
                self._goal_response(future, goal_serial)
        )

    def _goal_response(self, future, serial):
        try:
            handle = future.result()
            if serial != self._active_goal_serial:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.error.emit('导航目标被拒绝')
                self._route_goals = []
                self._publish_goal_inactive()
                return
            self._goal_handle = handle
            self._result_future = handle.get_result_async()
            self._result_future.add_done_callback(
                lambda result_future, goal_serial=serial:
                    self._navigation_result(result_future, goal_serial)
            )
        except Exception as exc:
            if serial != self._active_goal_serial:
                return
            self.error.emit(f'导航目标发送失败：{exc}')
            self._route_goals = []
            self._publish_goal_inactive()

    def _navigation_feedback(self, feedback, serial):
        if serial != self._active_goal_serial:
            return
        distance = float(feedback.feedback.distance_remaining)
        if self._active_is_intermediate:
            waypoint_total = max(0, len(self._route_goals) - 1)
            self._nav_state = (
                f'导航点 {self._route_position + 1}/{waypoint_total}，'
                f'剩余 {distance:.2f} m'
            )
        else:
            self._nav_state = f'目标点执行中，剩余 {distance:.2f} m'

    def _navigation_result(self, future, serial):
        if serial != self._active_goal_serial:
            return
        status = GoalStatus.STATUS_UNKNOWN
        try:
            status = int(future.result().status)
            self._nav_state = STATUS_NAMES.get(status, str(status))
        except Exception as exc:
            self._nav_state = 'ERROR'
            self.error.emit(f'导航结果读取失败：{exc}')
        self._goal_handle = None
        now = time.monotonic()
        if (
            self._route_goals
            and not self._localization_not_ready(now)
            and should_resume_after_base_fault(
                status,
                self._last_recoverable_fault_at,
                now,
                self._fault_resume_attempts,
            )
        ):
            self._result_future = None
            self._fault_resume_at = now + 0.5
            self._fault_resume_deadline = now + 20.0
            self._nav_state = '底盘故障恢复中；保留当前路线'
            return
        if status == GoalStatus.STATUS_SUCCEEDED and self._active_is_intermediate:
            self._intermediate_transition_pending = False
            self._route_position += 1
            self._send_current_route_goal()
            return
        self.navigation_arrived.emit(status == GoalStatus.STATUS_SUCCEEDED)
        self._route_goals = []
        self._active_goal_point = None
        self._publish_goal_inactive()

    def _localization_not_ready(self, now):
        guard = self._localization_guard
        if not guard or now - self._localization_guard_at > 1.5:
            return '定位守护反馈缺失或超过 1.5 秒未更新'
        if guard.get('estop_active', False):
            return '急停已触发'
        if guard.get('healthy') is not True:
            return '定位未就绪：' + str(guard.get('detail') or guard.get('state', '未知'))
        if str(guard.get('state', '')).startswith('RELOCALIZ'):
            return '正在重新定位：' + str(guard.get('detail') or guard['state'])
        if (
            guard.get('return_pending') or guard.get('goal_active')
            or guard.get('final_approach_active')
        ):
            return '定位守护正在返航或等待返航'
        return ''

    def _navigation_not_ready(self, now):
        reason = self._localization_not_ready(now)
        if reason:
            return reason
        base = self._base_status
        if base is None or now - self._base_status_at > 1.5:
            return '底盘反馈缺失或超过 1.5 秒未更新'
        if base['fault_bits']:
            return '底盘故障：' + base['fault']
        if base['control_state'] in (0, 5, 6):
            return '底盘未就绪：' + base['state']
        if self._localization_guard.get('ready_for_navigation') is not True:
            return '定位保护尚未解除：' + str(self._localization_guard.get('detail') or '等待稳定')
        if self._nav_lifecycle != 'ACTIVE' or now - self._nav_lifecycle_at > 3.0:
            return 'Nav2 生命周期未就绪或反馈超时：' + self._nav_lifecycle
        if not self._navigate_client.server_is_ready():
            return 'NavigateToPose 导航服务不可用'
        return ''

    def _stop_unready_navigation(self, reason):
        self._cancel_goal()
        self._nav_state = 'ERROR'
        self.error.emit('导航已停止：' + reason)

    def _check_navigation_health(self, now):
        if not self._route_goals or self._fault_resume_at:
            return
        reason = self._navigation_not_ready(now)
        if not reason:
            return
        if (
            not self._route_start_at
            and not self._localization_not_ready(now)
            and (self._base_status or {}).get('fault_bits') in (2, 8, 10)
            and self._fault_resume_attempts < 3
        ):
            self._cancel_goal(keep_route=True)
            self._fault_resume_at = now + 0.5
            self._fault_resume_deadline = now + 20.0
            self._nav_state = '底盘故障恢复中；已取消当前目标，最多等待 20 秒'
            self.error.emit(self._nav_state)
            return
        self._stop_unready_navigation(reason)

    def _resume_after_base_fault(self, now):
        if not self._route_goals:
            self._fault_resume_at = 0.0
            self._fault_resume_deadline = 0.0
            return
        reason = self._localization_not_ready(now)
        if reason:
            self._stop_unready_navigation(reason)
            return
        base = self._base_status or {}
        if base.get('fault_bits', 0) not in (0, 2, 8, 10) or base.get('control_state') == 5:
            self._stop_unready_navigation('底盘出现不可自动恢复故障或急停')
            return
        if now >= self._fault_resume_deadline:
            self._stop_unready_navigation('底盘恢复超过 20 秒')
            return
        if self._navigation_not_ready(now):
            return
        self._fault_resume_at = 0.0
        self._fault_resume_deadline = 0.0
        self._fault_resume_attempts += 1
        self._nav_state = '底盘已恢复，继续当前导航'
        self._send_current_route_goal()

    def _publish_point_markers(self, points):
        stamp = self._node.get_clock().now().to_msg()
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        marker_id = 0
        for point in points:
            is_waypoint = str(point.get('type') or 'target') == 'waypoint'
            x = float(point['x'])
            y = float(point['y'])
            yaw = float(point.get('yaw', 0.0))
            frame_id = str(point.get('frame_id') or 'map')
            name = str(point.get('name') or '未命名')
            red, green, blue = (
                (0.45, 1.0, 0.55) if is_waypoint else (0.10, 0.30, 1.0)
            )

            body = Marker()
            body.header.frame_id = frame_id
            body.header.stamp = stamp
            body.ns = 'route_points'
            body.id = marker_id
            marker_id += 1
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = 0.08
            body.pose.orientation.w = 1.0
            body.scale.x = 0.18
            body.scale.y = 0.18
            body.scale.z = 0.12
            body.color.r = red
            body.color.g = green
            body.color.b = blue
            body.color.a = 0.95
            body.frame_locked = True
            array.markers.append(body)

            if not is_waypoint:
                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.header.stamp = stamp
                arrow.ns = 'target_headings'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = x
                arrow.pose.position.y = y
                arrow.pose.position.z = 0.14
                arrow.pose.orientation.z, arrow.pose.orientation.w = (
                    quaternion_from_yaw(yaw)
                )
                arrow.scale.x = 0.32
                arrow.scale.y = 0.065
                arrow.scale.z = 0.065
                arrow.color.r = red
                arrow.color.g = green
                arrow.color.b = blue
                arrow.color.a = 1.0
                arrow.frame_locked = True
                array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = 'route_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.32
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = red
            label.color.g = green
            label.color.b = blue
            label.color.a = 1.0
            label.text = ('导航点 ' if is_waypoint else '目标点 ') + name
            label.frame_locked = True
            array.markers.append(label)
        self._marker_pub.publish(array)

    def _check_intermediate_progress(self):
        if self._intermediate_transition_pending:
            if (
                time.monotonic() - self._intermediate_transition_started_at
                >= 3.0
            ):
                # The cancel response and action result normally arrive within
                # milliseconds.  Do not leave the route permanently stopped if
                # an action result callback is lost during a Nav2 transition.
                self._finish_intermediate_transition(
                    self._intermediate_transition_serial
                )
            return
        if (
            not self._active_is_intermediate
            or self._active_goal_point is None
            or self._latest_map_pose is None
            or self._goal_handle is None
        ):
            return
        distance = math.hypot(
            float(self._active_goal_point['x']) - self._latest_map_pose['x'],
            float(self._active_goal_point['y']) - self._latest_map_pose['y'],
        )
        if distance > self._waypoint_tolerance:
            return
        self._intermediate_transition_pending = True
        previous_handle = self._goal_handle
        self._active_goal_serial += 1
        transition_serial = self._active_goal_serial
        self._intermediate_transition_serial = transition_serial
        self._intermediate_transition_started_at = time.monotonic()
        try:
            cancel_future = previous_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future, goal_serial=transition_serial:
                    self._intermediate_cancelled(future, goal_serial)
            )
        except Exception:
            self._finish_intermediate_transition(transition_serial)

    def _intermediate_cancelled(self, future, serial):
        try:
            future.result()
        except Exception:
            pass
        if serial != self._active_goal_serial:
            return
        # A CancelGoal response only confirms that cancellation was accepted;
        # bt_navigator may still own the old action.  Wait for its result
        # future before starting the next route segment to avoid losing /plan.
        if self._result_future is not None and not self._result_future.done():
            self._result_future.add_done_callback(
                lambda result_future, goal_serial=serial:
                    self._intermediate_result_finished(
                        result_future, goal_serial
                    )
            )
            return
        self._finish_intermediate_transition(serial)

    def _intermediate_result_finished(self, future, serial):
        self._finish_intermediate_transition(serial)

    def _finish_intermediate_transition(self, serial):
        if serial != self._active_goal_serial or not self._route_goals:
            return
        self._goal_handle = None
        self._result_future = None
        self._route_position += 1
        self._intermediate_transition_pending = False
        self._intermediate_transition_serial = 0
        self._intermediate_transition_started_at = 0.0
        self._send_current_route_goal()

    def _cancel_goal(self, keep_route=False):
        had_route = bool(
            self._route_goals or self._goal_handle is not None
            or self._goal_future is not None
        )
        self.navigation_arrived.emit(False)
        self._active_goal_serial += 1
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        if not keep_route:
            self._route_goals = []
            self._route_position = 0
            self._fault_resume_attempts = 0
        self._route_start_at = 0.0
        self._fault_resume_at = 0.0
        self._fault_resume_deadline = 0.0
        self._active_is_intermediate = False
        self._active_goal_point = None
        self._intermediate_transition_pending = False
        self._intermediate_transition_serial = 0
        self._intermediate_transition_started_at = 0.0
        if had_route:
            self._publish_goal_inactive()

    def _publish_goal_inactive(self):
        message = Bool()
        message.data = False
        if hasattr(self, '_goal_active_pub'):
            self._goal_active_pub.publish(message)

    def _publish_zero(self):
        if hasattr(self, '_manual_pub') and rclpy.ok():
            zero = Twist()
            for _ in range(3):
                try:
                    self._manual_pub.publish(zero)
                except Exception:
                    break
