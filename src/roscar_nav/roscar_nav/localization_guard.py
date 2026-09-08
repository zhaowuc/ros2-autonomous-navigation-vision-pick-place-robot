import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from c50c_interfaces.msg import BaseStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def load_home_point(path, name='起点'):
    value = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or {}
    rows = value.get('points', []) if isinstance(value, dict) else value
    for row in rows if isinstance(rows, list) else []:
        if str(row.get('name') or '') == str(name):
            return {
                'name': str(name),
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw']),
                'frame_id': str(row.get('frame_id') or 'map'),
            }
    raise ValueError(f'点位文件中缺少目标点“{name}”')


def covariance_is_healthy(covariance, max_xy_variance, max_yaw_variance):
    if len(covariance) != 36:
        return False
    values = (covariance[0], covariance[7], covariance[35])
    return (
        all(math.isfinite(value) and value >= 0.0 for value in values)
        and max(values[0], values[1]) <= float(max_xy_variance)
        and values[2] <= float(max_yaw_variance)
    )


def planar_errors(current, target):
    return (
        math.hypot(current['x'] - target['x'], current['y'] - target['y']),
        abs(wrap_angle(current['yaw'] - target['yaw'])),
    )


def target_error_in_body(current, target):
    dx = float(target['x']) - float(current['x'])
    dy = float(target['y']) - float(current['y'])
    yaw = float(current['yaw'])
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
        wrap_angle(float(target['yaw']) - yaw),
    )


def is_auto_recoverable_drive_fault(
    backend_ready, control_state, fault_bits, estop_active
):
    # Encoder-direction and transient motor-control latches are recoverable
    # through CLEAR_FAULT + fresh zero + REARM. Never auto-clear undervoltage,
    # communication, estop, or unknown fault bits.
    return (
        bool(backend_ready)
        and not bool(estop_active)
        and int(control_state) == 6
        and int(fault_bits) in (2, 8, 10)
    )


class LocalizationGuard(Node):
    def __init__(self):
        super().__init__('roscar_localization_guard')
        self.declare_parameter('map_yaml', str(Path.home() / 'roscar_maps/active.yaml'))
        self.declare_parameter(
            'points_file', str(Path.home() / '.ros/roscar_data/calibration_points.yaml')
        )
        self.declare_parameter('home_point_name', '起点')
        self.declare_parameter('auto_return_home', True)
        self.declare_parameter('startup_grace_sec', 8.0)
        self.declare_parameter('lost_grace_sec', 2.5)
        self.declare_parameter('stop_settle_sec', 1.0)
        self.declare_parameter('healthy_stable_sec', 3.0)
        self.declare_parameter('relocalize_retry_sec', 8.0)
        self.declare_parameter('relocalize_timeout_sec', 20.0)
        self.declare_parameter('relocalize_max_attempts', 3)
        self.declare_parameter('relocalize_budget_sec', 60.0)
        self.declare_parameter('prior_max_age_sec', 10.0)
        self.declare_parameter('tf_max_age_sec', 1.5)
        self.declare_parameter('max_xy_variance', 0.10)
        self.declare_parameter('max_yaw_variance', 0.20)
        self.declare_parameter('home_xy_tolerance', 0.019)
        self.declare_parameter('home_yaw_tolerance', 0.05)
        self.declare_parameter('home_retry_sec', 10.0)
        self.declare_parameter('home_max_attempts', 3)
        self.declare_parameter('final_entry_distance', 0.12)
        self.declare_parameter('final_timeout_sec', 20.0)
        self.declare_parameter('final_linear_kp', 0.8)
        self.declare_parameter('final_angular_kp', 1.5)
        self.declare_parameter('final_min_linear', 0.015)
        self.declare_parameter('final_max_linear', 0.04)
        self.declare_parameter('final_min_angular', 0.14)
        self.declare_parameter('final_max_angular', 0.20)
        self.declare_parameter('auto_recover_drive_fault', True)
        self.declare_parameter('base_fault_zero_settle_sec', 1.0)
        self.declare_parameter('base_fault_recovery_timeout_sec', 15.0)
        self.declare_parameter('base_fault_max_auto_recoveries', 3)
        self.declare_parameter('base_fault_counter_reset_sec', 60.0)

        self.map_yaml = str(self.get_parameter('map_yaml').value)
        self.home = load_home_point(
            str(self.get_parameter('points_file').value),
            str(self.get_parameter('home_point_name').value),
        )
        self.auto_return_home = bool(self.get_parameter('auto_return_home').value)
        self.started_at = time.monotonic()
        self.unhealthy_since = None
        self.healthy_since = None
        self.recovery_started_at = None
        self.base_fault_recovery_attempts = 0
        self.base_fault_last_recovered_at = None
        self.next_relocalize_at = 0.0
        self.relocalize_process = None
        self.relocalize_deadline = 0.0
        self.relocalize_attempts = 0
        self.last_amcl_pose = None
        self.latest_pose = None
        self.trusted_pose = None
        self.trusted_pose_at = 0.0
        self.base_status = None
        self.backend_ready = False
        self.estop_active = False
        self.base_fault_recovery_phase = None
        self.base_fault_recovery_started_at = 0.0
        self.base_fault_post_clear_zero_at = 0.0
        self.state = 'STARTING'
        self.detail = '等待 AMCL 定位'
        self.return_pending = self.auto_return_home
        self.goal_handle = None
        self.goal_future = None
        self.goal_active = False
        self.ignore_nav_result = False
        self.next_home_attempt_at = 0.0
        self.home_attempts = 0
        self.final_active = False
        self.final_started_at = 0.0
        self.nav_recovery_process = None
        self.next_nav_recovery_at = 0.0
        self.next_cancel_at = 0.0
        self.nav_active = False
        self.nav_state_future = None
        self.next_nav_check_at = 0.0
        self.last_status_at = 0.0

        durable = QoSProfile(depth=1)
        durable.reliability = ReliabilityPolicy.RELIABLE
        durable.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, '/roscar/localization_guard/status', durable
        )
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/roscar/active_goal', durable
        )
        self.goal_active_pub = self.create_publisher(
            Bool, '/roscar/goal_active', durable
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initial_pose, 10
        )
        self.create_subscription(BaseStatus, '/c50c/status', self._base_status, 10)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diagnostics, 10
        )
        self.create_subscription(Bool, '/roscar/estop_state', self._estop, durable)
        self.clear_fault_client = self.create_client(
            Trigger, '/roscar/base/clear_fault'
        )
        self.rearm_client = self.create_client(Trigger, '/roscar/base/rearm')
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal'
        )
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.10, self._tick)
        self.get_logger().info(
            f'定位守护已启动；起点=({self.home["x"]:.3f}, '
            f'{self.home["y"]:.3f}, {math.degrees(self.home["yaw"]):.1f}deg)，'
            f'自动返航={self.auto_return_home}'
        )

    def _amcl_pose(self, message):
        self.last_amcl_pose = message

    def _initial_pose(self, message):
        pose = message.pose.pose
        covariance = message.pose.covariance
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        stamp = int(message.header.stamp.sec) + int(message.header.stamp.nanosec) / 1.0e9
        age = self.get_clock().now().nanoseconds / 1.0e9 - stamp
        if (
            message.header.frame_id.lstrip('/') != 'map'
            or not -1.0 <= age <= float(self.get_parameter('tf_max_age_sec').value)
            or not all(math.isfinite(value) for value in values)
            or sum(value * value for value in values[3:]) < 1.0e-12
            or len(covariance) != 36
            or not all(math.isfinite(value) for value in covariance)
            or any(covariance[index] < 0.0 for index in (0, 7, 35))
        ):
            return
        self._stop_relocalize()
        self.last_amcl_pose = None
        self.latest_pose = None
        self.trusted_pose = None
        self.healthy_since = None
        self.recovery_started_at = None
        self.relocalize_attempts = 0
        self.next_relocalize_at = time.monotonic() + float(
            self.get_parameter('startup_grace_sec').value
        )
        self.state = 'VERIFYING'
        self.detail = '已收到新的初始位姿，保持停止并等待 AMCL 验证'
        self._publish_zero()
        self._cancel_navigation()

    def _base_status(self, message):
        self.base_status = message
        if int(message.fault_bits) or int(message.control_state) not in (2, 3, 4):
            self.trusted_pose = None

    def _diagnostics(self, message):
        for status in message.status:
            if status.name != 'roscar_base_interface/backend':
                continue
            values = {item.key: item.value for item in status.values}
            self.backend_ready = values.get('state') == 'READY'
            if not self.backend_ready:
                self.trusted_pose = None
            return

    def _estop(self, message):
        self.estop_active = bool(message.data)
        if self.estop_active:
            self.trusted_pose = None

    def _clear_fault_response(self, future):
        try:
            result = future.result()
            accepted = bool(result.success)
            detail = str(result.message)
        except Exception as error:
            accepted = False
            detail = str(error)
        if accepted:
            self.base_fault_recovery_phase = 'WAIT_CLEAR_ACK'
            self.detail = '可恢复底盘故障清除请求已发送，等待 STM32 状态确认'
        else:
            self.base_fault_recovery_phase = 'FAILED'
            self.detail = f'自动清除底盘故障失败：{detail}'
            self.get_logger().error(self.detail)

    def _rearm_response(self, future):
        try:
            result = future.result()
            accepted = bool(result.success)
            detail = str(result.message)
        except Exception as error:
            accepted = False
            detail = str(error)
        if accepted:
            self.base_fault_recovery_phase = 'WAIT_REARM_ACK'
            self.detail = '底盘 rearm 已发送，等待可运动状态确认'
        else:
            self.base_fault_recovery_phase = 'FAILED'
            self.detail = f'底盘自动 rearm 失败：{detail}'
            self.get_logger().error(self.detail)

    def _handle_base_fault(self, now):
        if self.base_status is None:
            return False
        fault_bits = int(self.base_status.fault_bits)
        control_state = int(self.base_status.control_state)
        if self.base_fault_recovery_phase is not None:
            elapsed = now - self.base_fault_recovery_started_at
            timeout = float(
                self.get_parameter('base_fault_recovery_timeout_sec').value
            )
            if elapsed > timeout and self.base_fault_recovery_phase != 'FAILED':
                self.base_fault_recovery_phase = 'FAILED'
                self.detail = '底盘故障自动复位超时；保持停止'
                self.get_logger().error(self.detail)
        if self.base_fault_recovery_phase == 'FAILED' and fault_bits == 0:
            self.base_fault_recovery_phase = None
            self.return_pending = self.auto_return_home
            return False
        if self.base_fault_recovery_phase == 'FAILED':
            self._cancel_navigation()
            return True
        if self.base_fault_recovery_phase == 'WAIT_CLEAR_ACK' and fault_bits == 0:
            self.base_fault_recovery_phase = 'WAIT_POST_CLEAR_ZERO'
            self.base_fault_post_clear_zero_at = now
            self.detail = '方向故障已清除，等待新的零速同步'
            return True
        if self.base_fault_recovery_phase == 'WAIT_POST_CLEAR_ZERO':
            # R19 CLEAR intentionally invalidates zero synchronization.  The
            # running safe-command chain supplies a fresh zero before REARM.
            if now - self.base_fault_post_clear_zero_at < 0.25:
                return True
            if self.rearm_client.service_is_ready():
                self.base_fault_recovery_phase = 'REARM_REQUESTED'
                future = self.rearm_client.call_async(Trigger.Request())
                future.add_done_callback(self._rearm_response)
            return True
        if self.base_fault_recovery_phase == 'WAIT_REARM_ACK':
            if fault_bits == 0 and control_state not in (0, 5, 6):
                self.base_fault_recovery_phase = None
                self.base_fault_last_recovered_at = now
                self.home_attempts = 0
                self.return_pending = self.auto_return_home
                self.next_home_attempt_at = now + 1.0
                self.state = 'VERIFYING'
                self.detail = '底盘故障已自动复位，继续当前导航'
                self.get_logger().info(self.detail)
                return False
            return True
        if self.base_fault_recovery_phase in (
            'CLEAR_REQUESTED', 'REARM_REQUESTED', 'FAILED'
        ):
            return True
        if self.base_fault_recovery_phase == 'SETTLING_ZERO':
            elapsed = now - self.base_fault_recovery_started_at
            settle = float(
                self.get_parameter('base_fault_zero_settle_sec').value
            )
            if elapsed >= settle and self.clear_fault_client.service_is_ready():
                self.base_fault_recovery_phase = 'CLEAR_REQUESTED'
                future = self.clear_fault_client.call_async(Trigger.Request())
                future.add_done_callback(self._clear_fault_response)
            return True

        enabled = bool(
            self.get_parameter('auto_recover_drive_fault').value
        )
        recoverable = is_auto_recoverable_drive_fault(
            self.backend_ready, control_state, fault_bits, self.estop_active
        )
        if fault_bits == 0:
            reset_sec = float(
                self.get_parameter('base_fault_counter_reset_sec').value
            )
            if (
                self.base_fault_last_recovered_at is not None
                and now - self.base_fault_last_recovered_at >= reset_sec
            ):
                self.base_fault_recovery_attempts = 0
                self.base_fault_last_recovered_at = None
            return False
        self._publish_zero()
        max_attempts = int(
            self.get_parameter('base_fault_max_auto_recoveries').value
        )
        if (
            enabled and recoverable
            and self.base_fault_recovery_attempts < max_attempts
        ):
            self.base_fault_recovery_attempts += 1
            self.base_fault_recovery_phase = 'SETTLING_ZERO'
            self.base_fault_recovery_started_at = now
            self.state = 'BASE_FAULT_RECOVERY'
            self.detail = (
                f'检测到可恢复的底盘故障 fault_bits={fault_bits}；'
                f'暂停车速并自动 clear_fault/rearm，保留当前导航 '
                f'({self.base_fault_recovery_attempts}/{max_attempts})'
            )
            self.get_logger().warning(self.detail)
        else:
            self._cancel_navigation()
            self.state = 'BASE_FAULT_BLOCKED'
            self.detail = (
                f'底盘故障禁止自动清除：control_state={control_state}，'
                f'fault_bits={fault_bits}，estop={self.estop_active}，'
                f'自动恢复次数={self.base_fault_recovery_attempts}/{max_attempts}'
            )
        return True

    def _lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.03),
            )
        except TransformException as error:
            return None, f'map→base_footprint 不可用：{error}'
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9
        max_age = float(self.get_parameter('tf_max_age_sec').value)
        if age < -1.0 or age > max_age:
            return None, f'定位 TF 已过期 {age:.2f}s'
        translation = transform.transform.translation
        pose = {
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': yaw_from_quaternion(transform.transform.rotation),
        }
        if not all(math.isfinite(value) for value in pose.values()):
            return None, '定位 TF 含无效数值'
        return pose, '定位 TF 新鲜'

    def _localization_health(self):
        if self.last_amcl_pose is None:
            return False, '尚未收到 /amcl_pose'
        covariance = self.last_amcl_pose.pose.covariance
        if not covariance_is_healthy(
            covariance,
            self.get_parameter('max_xy_variance').value,
            self.get_parameter('max_yaw_variance').value,
        ):
            return False, 'AMCL 协方差超过稳定阈值'
        pose, detail = self._lookup_pose()
        if pose is None:
            return False, detail
        self.latest_pose = pose
        return True, 'AMCL 位姿、协方差和 TF 均稳定'

    def _publish_zero(self):
        self.manual_pub.publish(Twist())

    def _set_goal_active(self, value):
        message = Bool()
        message.data = bool(value)
        self.goal_active_pub.publish(message)

    def _cancel_navigation(self):
        self.final_active = False
        self.goal_active = False
        self._set_goal_active(False)
        now = time.monotonic()
        if now < self.next_cancel_at or not self.cancel_client.service_is_ready():
            return
        # A new goal can arrive while localization remains lost. Keep cancelling
        # at a bounded rate instead of latching the first cancellation forever.
        self.next_cancel_at = now + 1.0
        request = CancelGoal.Request()
        # A delayed request must not cancel a newer goal accepted after recovery.
        request.goal_info.stamp = self.get_clock().now().to_msg()
        self.cancel_client.call_async(request)

    def _stop_relocalize(self):
        if self.relocalize_process is not None and self.relocalize_process.poll() is None:
            # This is the Python worker itself, not a ros2 wrapper with a child
            # that could survive termination. Never block the zero-speed timer.
            self.relocalize_process.kill()

    def _relocalize_budget_exhausted(self, now):
        return (
            self.relocalize_attempts >= max(
                1, int(self.get_parameter('relocalize_max_attempts').value)
            )
            or (
                self.recovery_started_at is not None
                and now - self.recovery_started_at >= max(
                    1.0, float(self.get_parameter('relocalize_budget_sec').value)
                )
            )
        )

    def _start_relocalize(self, now):
        # Discard only the pre-recovery estimate. AMCL may publish the new pose
        # before the child process exits, so clearing it in _poll_relocalize()
        # would lose the only stationary /amcl_pose sample and cause a loop.
        self.last_amcl_pose = None
        command = [
            sys.executable, '-m', 'roscar_nav.auto_localize', '--ros-args',
            '-p', f'map_yaml:={self.map_yaml}',
            '-p', 'scan_topic:=/scan_filtered',
            '-p', 'target_frames:=7',
            '-p', 'max_frame_yaw_deg:=1.2',
            '-p', 'max_frame_yaw_span_deg:=2.0',
            '-p', 'max_frame_translation_m:=0.015',
            '-p', 'max_frame_translation_span_m:=0.025',
            # Seven-frame field calibration produced a stable 0.002168 gap
            # between the best basin and a distant, reversed alternative.
            # Keep a nonzero ambiguity gate while allowing that measured case.
            '-p', 'min_distinct_score_gap:=0.0015',
        ]
        use_prior = (
            self.trusted_pose is not None
            and 0.0 <= now - self.trusted_pose_at <= float(
                self.get_parameter('prior_max_age_sec').value
            )
        )
        if use_prior:
            prior = [self.trusted_pose[key] for key in ('x', 'y', 'yaw')]
            command += ['-p', 'prior_pose:=' + json.dumps(prior)]
        self.get_logger().warning(
            '定位丢失，启动' + ('最近可信位姿附近的' if use_prior else '全局激光') + '重定位'
        )
        self.relocalize_attempts += 1
        self.relocalize_deadline = min(
            now + max(1.0, float(self.get_parameter('relocalize_timeout_sec').value)),
            self.recovery_started_at + max(
                1.0, float(self.get_parameter('relocalize_budget_sec').value)
            ),
        )
        self.next_relocalize_at = now + float(
            self.get_parameter('relocalize_retry_sec').value
        )
        try:
            self.relocalize_process = subprocess.Popen(command)
        except OSError as error:
            self.trusted_pose = None
            self.state = 'RELOCALIZE_RETRY'
            self.detail = f'无法启动重定位：{error}；保持停止'
            self.get_logger().error(self.detail)
            return
        self.state = 'RELOCALIZING'
        self.detail = f'正在重定位（第 {self.relocalize_attempts} 次，有执行时限）'

    def _poll_relocalize(self, now):
        if self.relocalize_process is None:
            return
        result = self.relocalize_process.poll()
        if result is None:
            if now >= self.relocalize_deadline:
                self._stop_relocalize()
                self.state = 'RELOCALIZE_TIMEOUT'
                self.detail = '单次重定位超时，已终止计算；保持停止'
            return
        self.relocalize_process = None
        self.healthy_since = None
        if result == 0:
            self.state = 'VERIFYING'
            self.detail = '重定位已提交，等待连续稳定确认'
            self.get_logger().info(self.detail)
        else:
            # A rejected local prior must not constrain the next global attempt.
            self.trusted_pose = None
            exhausted = self._relocalize_budget_exhausted(now)
            self.state = 'RELOCALIZE_FAILED' if exhausted else 'RELOCALIZE_RETRY'
            self.detail = (
                f'重定位失败（退出码 {result}）；'
                + ('自动重试预算已耗尽，请确认地图及实际位姿；保持停止'
                   if exhausted else '稍后重试，保持停止')
            )
            self.get_logger().warning(self.detail)
            self.next_relocalize_at = now + float(
                self.get_parameter('relocalize_retry_sec').value
            )

    def _base_allows_motion(self):
        return (
            self.backend_ready
            and not self.estop_active
            and
            self.base_status is not None
            and int(self.base_status.fault_bits) == 0
            and int(self.base_status.control_state) in (2, 3, 4)
        )

    def _send_home(self):
        if not self.auto_return_home:
            self.return_pending = False
            self.state = 'HEALTHY'
            self.detail = '定位稳定；自动返航已禁用'
            return
        if not self._base_allows_motion():
            self.state = 'WAITING_BASE'
            self.detail = (
                '定位稳定，但底盘状态不允许自动返航：'
                f'backend_ready={self.backend_ready}，'
                f'control_state={getattr(self.base_status, "control_state", "--")}，'
                f'fault_bits={getattr(self.base_status, "fault_bits", "--")}'
            )
            return
        if not self.nav_active or not self.navigate_client.server_is_ready():
            self.state = 'WAITING_NAV2'
            self.detail = '定位稳定，正在恢复 Nav2 生命周期'
            return
        if self.latest_pose is not None:
            xy_error, yaw_error = planar_errors(self.latest_pose, self.home)
            if (
                xy_error < float(self.get_parameter('home_xy_tolerance').value)
                and yaw_error <= float(self.get_parameter('home_yaw_tolerance').value)
            ):
                self.return_pending = False
                self.state = 'HOME_REACHED'
                self.detail = '机器人已在起点且车头方向正确'
                return
        pose = PoseStamped()
        pose.header.frame_id = self.home['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.home['x']
        pose.pose.position.y = self.home['y']
        pose.pose.orientation.z = math.sin(self.home['yaw'] * 0.5)
        pose.pose.orientation.w = math.cos(self.home['yaw'] * 0.5)
        self.goal_pub.publish(pose)
        self._set_goal_active(True)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_future = self.navigate_client.send_goal_async(goal)
        self.goal_future.add_done_callback(self._home_goal_response)
        self.return_pending = False
        self.next_home_attempt_at = time.monotonic() + 2.0
        self.goal_active = True
        self.ignore_nav_result = False
        self.home_attempts += 1
        self.state = 'RETURNING_HOME'
        self.detail = '定位已稳定，正在自动返回起点并对齐车头'
        self.get_logger().warning(self.detail)

    def _home_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.goal_active = False
            maximum = int(self.get_parameter('home_max_attempts').value)
            self.return_pending = self.home_attempts < maximum
            self.state = 'HOME_RETRY_WAIT' if self.return_pending else 'HOME_GOAL_ERROR'
            self.detail = (
                f'返航目标发送失败（{self.home_attempts}/{maximum}）：{error}'
            )
            self.next_home_attempt_at = time.monotonic() + float(
                self.get_parameter('home_retry_sec').value
            )
            return
        if not handle.accepted:
            self.goal_active = False
            maximum = int(self.get_parameter('home_max_attempts').value)
            self.return_pending = self.home_attempts < maximum
            self.state = (
                'HOME_RETRY_WAIT' if self.return_pending else 'HOME_GOAL_REJECTED'
            )
            self.detail = (
                f'返航目标被 Nav2 拒绝（{self.home_attempts}/{maximum}）'
            )
            self.next_home_attempt_at = time.monotonic() + float(
                self.get_parameter('home_retry_sec').value
            )
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._home_result)

    def _home_result(self, future):
        if self.ignore_nav_result:
            # The 12 cm terminal controller has already taken authority and
            # intentionally cancelled the coarse Nav2 action.
            self.ignore_nav_result = False
            self.goal_handle = None
            return
        self.goal_active = False
        self.goal_handle = None
        self._set_goal_active(False)
        try:
            status = int(future.result().status)
        except Exception as error:
            status = GoalStatus.STATUS_UNKNOWN
            self.detail = f'返航结果读取失败：{error}'
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.latest_pose is None:
                self.state = 'HOME_VERIFY_FAILED'
                self.detail = 'Nav2 已完成返航，但缺少最终定位，保持停止'
                self.get_logger().error(self.detail)
                return
            xy_error, yaw_error = planar_errors(self.latest_pose, self.home)
            xy_limit = float(self.get_parameter('home_xy_tolerance').value)
            yaw_limit = float(self.get_parameter('home_yaw_tolerance').value)
            if xy_error < xy_limit and yaw_error <= yaw_limit:
                self.state = 'HOME_REACHED'
                self.detail = (
                    f'已返回起点：位置误差 {xy_error:.3f}m，'
                    f'车头误差 {math.degrees(yaw_error):.2f}deg'
                )
                self.get_logger().info(self.detail)
            else:
                entry = float(self.get_parameter('final_entry_distance').value)
                if xy_error <= entry:
                    self._begin_final_approach(time.monotonic(), cancel_nav=False)
                else:
                    self.state = 'HOME_VERIFY_FAILED'
                    self.detail = (
                        f'Nav2 提前结束：距起点仍有 {xy_error:.3f}m；保持停止'
                    )
                    self.get_logger().error(self.detail)
        else:
            maximum = int(self.get_parameter('home_max_attempts').value)
            self.return_pending = self.home_attempts < maximum
            self.next_home_attempt_at = time.monotonic() + float(
                self.get_parameter('home_retry_sec').value
            )
            self.state = 'HOME_RETRY_WAIT' if self.return_pending else 'HOME_FAILED'
            self.detail = (
                f'自动返航未完成，状态码 {status}（{self.home_attempts}/'
                f'{maximum}）' + ('；稍后重新规划' if self.return_pending else '；保持停止')
            )
            self.get_logger().error(self.detail)

    def _begin_final_approach(self, now, cancel_nav=True):
        if self.final_active or self.latest_pose is None:
            return
        self.final_active = True
        self.final_started_at = now
        self.return_pending = False
        self.goal_active = True
        self.state = 'FINAL_APPROACH'
        self.detail = '进入最后 12cm 麦克纳姆精确控制，目标位置误差 < 2cm'
        if cancel_nav and self.cancel_client.service_is_ready():
            self.ignore_nav_result = True
            self.cancel_client.call_async(CancelGoal.Request())
        self._set_goal_active(True)
        self.get_logger().warning(self.detail)
        self._run_final_approach(now)

    def _finish_final_approach(self, success, detail):
        self._publish_zero()
        self.final_active = False
        self.goal_active = False
        self.return_pending = False
        self._set_goal_active(False)
        self.state = 'HOME_REACHED' if success else 'HOME_FINAL_FAILED'
        self.detail = detail
        (self.get_logger().info if success else self.get_logger().error)(detail)

    def _run_final_approach(self, now):
        if not self.final_active or self.latest_pose is None:
            return
        xy_error, yaw_error = planar_errors(self.latest_pose, self.home)
        xy_limit = float(self.get_parameter('home_xy_tolerance').value)
        yaw_limit = float(self.get_parameter('home_yaw_tolerance').value)
        if xy_error < xy_limit and yaw_error <= yaw_limit:
            self._finish_final_approach(
                True,
                f'已精确返回起点：位置误差 {xy_error:.3f}m，'
                f'车头误差 {math.degrees(yaw_error):.2f}deg',
            )
            return
        timeout = float(self.get_parameter('final_timeout_sec').value)
        if now - self.final_started_at > timeout:
            self._finish_final_approach(
                False,
                f'终端精确控制超时：位置误差 {xy_error:.3f}m，'
                f'车头误差 {math.degrees(yaw_error):.2f}deg；保持停止',
            )
            return
        if not self._base_allows_motion():
            self._finish_final_approach(False, '底盘状态异常，终端控制停止')
            return

        body_x, body_y, body_yaw = target_error_in_body(
            self.latest_pose, self.home
        )
        command = Twist()
        if xy_error >= xy_limit:
            linear_kp = float(self.get_parameter('final_linear_kp').value)
            command.linear.x = linear_kp * body_x
            command.linear.y = linear_kp * body_y
            magnitude = math.hypot(command.linear.x, command.linear.y)
            maximum = float(self.get_parameter('final_max_linear').value)
            minimum = float(self.get_parameter('final_min_linear').value)
            target_magnitude = min(maximum, max(minimum, magnitude))
            if magnitude > 1.0e-9:
                scale = target_magnitude / magnitude
                command.linear.x *= scale
                command.linear.y *= scale
        if abs(body_yaw) > yaw_limit:
            angular = float(self.get_parameter('final_angular_kp').value) * body_yaw
            minimum = float(self.get_parameter('final_min_angular').value)
            maximum = float(self.get_parameter('final_max_angular').value)
            command.angular.z = math.copysign(
                min(maximum, max(minimum, abs(angular))), angular
            )
        self.manual_pub.publish(command)
        self.detail = (
            f'终端精确控制中：位置误差 {xy_error:.3f}m，'
            f'车头误差 {math.degrees(yaw_error):.2f}deg'
        )

    def _check_nav_state(self, now):
        if self.nav_state_future is not None and self.nav_state_future.done():
            try:
                self.nav_active = self.nav_state_future.result().current_state.id == 3
            except Exception:
                self.nav_active = False
            self.nav_state_future = None
        if now < self.next_nav_check_at:
            return
        if self.nav_state_future is not None:
            self.nav_state_future.cancel()
            self.nav_state_future = None
            self.nav_active = False
        self.next_nav_check_at = now + 1.0
        if self.nav_state_client.service_is_ready():
            self.nav_state_future = self.nav_state_client.call_async(GetState.Request())
        else:
            self.nav_active = False

    def _start_nav_recovery(self):
        now = time.monotonic()
        if self.nav_recovery_process is not None or now < self.next_nav_recovery_at:
            return
        script = r'''
for node in velocity_smoother collision_monitor controller_server smoother_server planner_server behavior_server bt_navigator; do
  state=$(timeout 5 ros2 lifecycle get /$node 2>/dev/null || true)
  if echo "$state" | grep -q unconfigured; then
    timeout 8 ros2 lifecycle set /$node configure >/dev/null 2>&1 || true
    state=$(timeout 5 ros2 lifecycle get /$node 2>/dev/null || true)
  fi
  if echo "$state" | grep -q inactive; then
    timeout 8 ros2 lifecycle set /$node activate >/dev/null 2>&1 || true
  fi
done
'''
        self.nav_recovery_process = subprocess.Popen(['/bin/bash', '-lc', script])
        self.next_nav_recovery_at = now + 10.0

    def _poll_nav_recovery(self):
        if self.nav_recovery_process is None:
            return
        result = self.nav_recovery_process.poll()
        if result is None:
            return
        self.nav_recovery_process = None
        self.get_logger().info(
            f'Nav2 生命周期恢复流程结束，退出码 {result}'
        )

    def _publish_status(self, now, healthy):
        if now - self.last_status_at < 0.5:
            return
        message = String()
        stable = (
            healthy and self.healthy_since is not None
            and now - self.healthy_since >= float(
                self.get_parameter('healthy_stable_sec').value
            )
        )
        message.data = json.dumps({
            'state': self.state,
            'detail': self.detail,
            'healthy': bool(stable and self.relocalize_process is None),
            'ready_for_navigation': bool(
                stable and self._base_allows_motion() and self.nav_active
                and self.relocalize_process is None and self.nav_recovery_process is None
                and self.base_fault_recovery_phase is None
                and not self.return_pending and not self.goal_active and not self.final_active
            ),
            'auto_return_home': self.auto_return_home,
            'return_pending': self.return_pending,
            'goal_active': self.goal_active,
            'home_attempts': self.home_attempts,
            'relocalize_attempts': self.relocalize_attempts,
            'base_backend_ready': self.backend_ready,
            'base_fault_recovery_phase': self.base_fault_recovery_phase,
            'estop_active': self.estop_active,
            'final_approach_active': self.final_active,
            'pose': self.latest_pose,
            'home': self.home,
        }, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(message)
        self.last_status_at = now

    def _tick(self):
        now = time.monotonic()
        healthy, health_detail = self._localization_health()
        self._poll_relocalize(now)
        self._poll_nav_recovery()
        self._check_nav_state(now)

        if self._handle_base_fault(now):
            self._publish_zero()
            self._publish_status(now, healthy)
            return

        if healthy:
            self.unhealthy_since = None
            if self.healthy_since is None:
                self.healthy_since = now
            stable_for = now - self.healthy_since
            stable = stable_for >= float(self.get_parameter('healthy_stable_sec').value)
            if not stable:
                self._publish_zero()
                self._cancel_navigation()
            if stable:
                self.next_cancel_at = 0.0
                self.recovery_started_at = None
                self.relocalize_attempts = 0
                self.next_relocalize_at = 0.0
                self._stop_relocalize()
                if self._base_allows_motion():
                    self.trusted_pose = dict(self.latest_pose)
                    self.trusted_pose_at = now
                if (
                    self._base_allows_motion() and not self.nav_active
                    and not self.final_active and not self.goal_active
                ):
                    self._publish_zero()
                    self._start_nav_recovery()
            if self.final_active:
                self._run_final_approach(now)
                self._publish_status(now, healthy)
                return
            if self.state == 'RETURNING_HOME' and self.goal_active:
                xy_error, _ = planar_errors(self.latest_pose, self.home)
                if xy_error <= float(
                    self.get_parameter('final_entry_distance').value
                ):
                    self._begin_final_approach(now, cancel_nav=True)
                    self._publish_status(now, healthy)
                    return
            if self.state not in (
                'RETURNING_HOME', 'HOME_REACHED', 'HOME_FAILED',
                'HOME_VERIFY_FAILED', 'HOME_RETRY_WAIT',
            ):
                self.state = 'HEALTHY' if stable else 'VERIFYING'
                self.detail = f'{health_detail}，连续稳定 {stable_for:.1f}s'
                if stable and not self._base_allows_motion():
                    self.state = 'WAITING_BASE'
                    self.detail = '定位稳定，底盘尚未安全就绪；保持停止'
                    self._publish_zero()
                elif stable and not self.nav_active:
                    self.state = 'WAITING_NAV2'
                    self.detail = '定位稳定，正在恢复 Nav2 生命周期'
            if (
                self.return_pending
                and not self.goal_active
                and now >= self.next_home_attempt_at
                and stable
            ):
                self._send_home()
        else:
            self.healthy_since = None
            if self.unhealthy_since is None:
                self.unhealthy_since = now
            unhealthy_for = now - self.unhealthy_since
            startup_wait = now - self.started_at < float(
                self.get_parameter('startup_grace_sec').value
            )
            lost = unhealthy_for >= float(
                self.get_parameter('lost_grace_sec').value
            )
            if not startup_wait and lost:
                if self.recovery_started_at is None:
                    self.home_attempts = 0
                self.return_pending = self.auto_return_home
                self._publish_zero()
                self._cancel_navigation()
                if self.recovery_started_at is None:
                    self.recovery_started_at = now
                settled = now - self.recovery_started_at >= float(
                    self.get_parameter('stop_settle_sec').value
                )
                if self._relocalize_budget_exhausted(now) and self.relocalize_process is None:
                    self.state = 'RELOCALIZE_FAILED'
                    self.detail = '自动重定位预算已耗尽，请确认地图及实际位姿；保持停止'
                elif (
                    settled
                    and self.relocalize_process is None
                    and now >= self.next_relocalize_at
                ):
                    self._start_relocalize(now)
                elif self.relocalize_process is None:
                    self.state = 'LOCALIZATION_LOST'
                    self.detail = f'{health_detail}；底盘保持零速，等待重定位'
            else:
                self.state = 'STARTING' if startup_wait else 'UNSTABLE'
                self.detail = health_detail
        self._publish_status(now, healthy)

    def destroy_node(self):
        self._stop_relocalize()
        if self.nav_recovery_process is not None:
            self.nav_recovery_process.terminate()
        self._publish_zero()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
