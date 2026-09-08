#!/usr/bin/env python3

from collections import deque
import json
import math
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def sensor_qos(depth=20):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _all_finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _imu_valid(msg, expected_frame):
    if str(msg.header.frame_id) != expected_frame:
        return False
    orientation = msg.orientation
    values = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
        *msg.orientation_covariance,
        *msg.angular_velocity_covariance,
        *msg.linear_acceleration_covariance,
    ]
    if not _all_finite(values):
        return False
    quaternion_norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    return 0.95 <= quaternion_norm <= 1.05


def _odom_valid(msg, expected_frame, expected_child_frame):
    if (
        str(msg.header.frame_id) != expected_frame
        or str(msg.child_frame_id) != expected_child_frame
    ):
        return False
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    linear = msg.twist.twist.linear
    angular = msg.twist.twist.angular
    values = [
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
        *msg.pose.covariance,
        *msg.twist.covariance,
    ]
    if not _all_finite(values):
        return False
    quaternion_norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    return 0.95 <= quaternion_norm <= 1.05


def _scan_valid(msg):
    if not str(msg.header.frame_id):
        return False
    if not _all_finite([
        msg.angle_min,
        msg.angle_max,
        msg.angle_increment,
        msg.time_increment,
        msg.scan_time,
        msg.range_min,
        msg.range_max,
    ]):
        return False
    if msg.angle_increment <= 0.0 or msg.range_max <= msg.range_min:
        return False
    return all(
        not math.isnan(float(value)) and float(value) != -math.inf
        for value in msg.ranges
    )


class StreamState:
    def __init__(self, validator):
        self.validator = validator
        self.times = deque(maxlen=500)
        self.last_receive = None
        self.last_stamp = None
        self.last_stamp_sec = None
        self.non_increasing_stamps = 0
        self.invalid_messages = 0
        self.stamp_error_times = deque(maxlen=50)
        self.invalid_message_times = deque(maxlen=50)

    def update(self, msg):
        now = time.monotonic()
        self.last_receive = now
        self.times.append(now)
        while len(self.times) > 2 and now - self.times[0] > 2.0:
            self.times.popleft()

        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is not None:
            stamp_value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            self.last_stamp_sec = stamp_value * 1.0e-9
            if self.last_stamp is not None and stamp_value <= self.last_stamp:
                self.non_increasing_stamps += 1
                self.stamp_error_times.append(now)
            self.last_stamp = stamp_value
        try:
            valid = bool(self.validator(msg))
        except (AttributeError, TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            self.invalid_messages += 1
            self.invalid_message_times.append(now)

    @staticmethod
    def _recent_count(events, now, window_sec=2.0):
        while events and now - events[0] > window_sec:
            events.popleft()
        return len(events)

    def recent_stamp_errors(self, now):
        return self._recent_count(self.stamp_error_times, now)

    def recent_invalid_messages(self, now):
        return self._recent_count(self.invalid_message_times, now)

    def age(self, now):
        if self.last_receive is None:
            return math.inf
        return max(0.0, now - self.last_receive)

    def rate(self):
        if len(self.times) < 2:
            return 0.0
        elapsed = self.times[-1] - self.times[0]
        return 0.0 if elapsed <= 0.0 else (len(self.times) - 1) / elapsed


class NavigationHealthCheck(Node):
    def __init__(self):
        super().__init__('navigation_health_check')
        self.declare_parameter('use_fused_odom', True)
        self.declare_parameter('allow_wheel_odom_only', False)
        self.declare_parameter('wheel_odom_topic', '/wheel/odom_raw')
        self.declare_parameter('official_odom_topic', '/odom')
        self.declare_parameter('n300_imu_topic', '/imu/n300/data')
        self.declare_parameter('n300_frame', 'n300_imu_link')
        self.declare_parameter('scan_topic', '/scan_slam_filtered')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('startup_mode', 'navigation')
        self.declare_parameter('stale_sec', 0.5)
        self.declare_parameter('stable_for_sec', 2.0)
        self.declare_parameter('timeout_sec', 45.0)
        self.declare_parameter('minimum_n300_hz', 50.0)
        # The C50C firmware publishes at a nominal 20 Hz. Keep enough margin for
        # scheduler jitter so a healthy 19.9-20.0 Hz stream does not flap.
        self.declare_parameter('minimum_wheel_odom_hz', 15.0)
        self.declare_parameter('minimum_official_odom_hz', 20.0)
        self.declare_parameter('minimum_scan_hz', 3.0)
        self.declare_parameter('diagnostic_stale_sec', 3.0)

        self.use_fused_odom = bool(self.get_parameter('use_fused_odom').value)
        self.allow_wheel_odom_only = bool(
            self.get_parameter('allow_wheel_odom_only').value
        )
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.official_odom_topic = str(
            self.get_parameter('official_odom_topic').value
        )
        self.n300_imu_topic = str(self.get_parameter('n300_imu_topic').value)
        self.n300_frame = str(self.get_parameter('n300_frame').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.startup_mode = str(
            self.get_parameter('startup_mode').value
        ).strip().lower()
        if self.startup_mode not in ('navigation', 'mapping'):
            raise ValueError(
                f'startup_mode must be navigation or mapping, got {self.startup_mode!r}'
            )
        self.stale_sec = float(self.get_parameter('stale_sec').value)
        self.stable_for_sec = float(self.get_parameter('stable_for_sec').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.minimum_n300_hz = float(self.get_parameter('minimum_n300_hz').value)
        self.minimum_wheel_odom_hz = float(
            self.get_parameter('minimum_wheel_odom_hz').value
        )
        self.minimum_official_odom_hz = float(
            self.get_parameter('minimum_official_odom_hz').value
        )
        self.minimum_scan_hz = float(self.get_parameter('minimum_scan_hz').value)
        self.diagnostic_stale_sec = float(
            self.get_parameter('diagnostic_stale_sec').value
        )

        self.started = time.monotonic()
        self.healthy_since = None
        self.last_signature = None
        self.last_log = 0.0
        self.exit_code = None
        self.last_report = {}
        self.ekf_diagnostic_received = False
        self.ekf_diagnostic_last = None
        self.ekf_diagnostic_level = None
        self.ekf_diagnostic_messages = []

        self.streams = {
            'n300': StreamState(
                lambda msg: _imu_valid(msg, self.n300_frame)
            ),
            'wheel_odom': StreamState(
                lambda msg: _odom_valid(
                    msg,
                    self.odom_frame,
                    self.base_frame,
                )
            ),
            'official_odom': StreamState(
                lambda msg: _odom_valid(
                    msg,
                    self.odom_frame,
                    self.base_frame,
                )
            ),
            'scan': StreamState(_scan_valid),
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Imu,
            self.n300_imu_topic,
            lambda msg: self.streams['n300'].update(msg),
            sensor_qos(),
        )
        self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            lambda msg: self.streams['wheel_odom'].update(msg),
            sensor_qos(),
        )
        if self.official_odom_topic == self.wheel_odom_topic:
            self.streams['official_odom'] = self.streams['wheel_odom']
        else:
            self.create_subscription(
                Odometry,
                self.official_odom_topic,
                lambda msg: self.streams['official_odom'].update(msg),
                sensor_qos(),
            )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            lambda msg: self.streams['scan'].update(msg),
            sensor_qos(),
        )
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            sensor_qos(10),
        )
        self.create_timer(0.2, self.evaluate)

    @staticmethod
    def _node_names(endpoint_infos):
        names = []
        for info in endpoint_infos:
            namespace = str(info.node_namespace or '').rstrip('/')
            name = str(info.node_name or '')
            qualified = f'{namespace}/{name}' if namespace else f'/{name}'
            names.append(qualified.replace('//', '/'))
        return sorted(names)

    def publisher_names(self, topic):
        try:
            return self._node_names(self.get_publishers_info_by_topic(topic))
        except Exception:
            return []

    @staticmethod
    def _diagnostic_level(level):
        if isinstance(level, (bytes, bytearray)):
            return level[0] if level else 3
        return int(level)

    def diagnostics_callback(self, msg):
        statuses = [
            status
            for status in msg.status
            if str(status.name).lower().startswith('ekf_filter_node:')
        ]
        if not statuses:
            return
        self.ekf_diagnostic_received = True
        self.ekf_diagnostic_last = time.monotonic()
        self.ekf_diagnostic_level = max(
            self._diagnostic_level(status.level)
            for status in statuses
        )
        self.ekf_diagnostic_messages = [
            {
                'name': str(status.name),
                'level': self._diagnostic_level(status.level),
                'message': str(status.message),
            }
            for status in statuses
        ]

    def stream_check(self, key, topic, minimum_rate, required=True):
        now = time.monotonic()
        stream = self.streams[key]
        age = stream.age(now)
        rate = stream.rate()
        stamp_age = (
            math.inf
            if stream.last_stamp_sec is None
            else time.time() - stream.last_stamp_sec
        )
        recent_stamp_errors = stream.recent_stamp_errors(now)
        recent_invalid_messages = stream.recent_invalid_messages(now)
        ok = (
            not required
            or (
                age <= self.stale_sec
                and 0.0 <= stamp_age <= self.stale_sec
                and rate >= minimum_rate
                and recent_stamp_errors == 0
                and recent_invalid_messages == 0
            )
        )
        return {
            'ok': ok,
            'required': required,
            'topic': topic,
            'age_sec': None if math.isinf(age) else age,
            'stamp_age_sec': None if math.isinf(stamp_age) else stamp_age,
            'rate_hz': rate,
            'minimum_rate_hz': minimum_rate,
            'recent_non_increasing_stamps': recent_stamp_errors,
            'non_increasing_stamps_total': stream.non_increasing_stamps,
            'recent_invalid_messages': recent_invalid_messages,
            'invalid_messages_total': stream.invalid_messages,
        }

    def evaluate(self):
        now = time.monotonic()
        fused_requires_n300 = (
            self.use_fused_odom and not self.allow_wheel_odom_only
        )
        official_minimum_hz = (
            self.minimum_official_odom_hz
            if self.use_fused_odom
            else self.minimum_wheel_odom_hz
        )
        checks = {
            'n300': self.stream_check(
                'n300',
                self.n300_imu_topic,
                self.minimum_n300_hz,
                required=fused_requires_n300,
            ),
            'wheel_odom': self.stream_check(
                'wheel_odom',
                self.wheel_odom_topic,
                self.minimum_wheel_odom_hz,
            ),
            'official_odom': self.stream_check(
                'official_odom',
                self.official_odom_topic,
                official_minimum_hz,
            ),
            'scan': self.stream_check(
                'scan',
                self.scan_topic,
                self.minimum_scan_hz,
            ),
        }

        wheel_publishers = self.publisher_names(self.wheel_odom_topic)
        official_publishers = self.publisher_names(self.official_odom_topic)
        n300_publishers = self.publisher_names(self.n300_imu_topic)
        expected_official = 'ekf' if self.use_fused_odom else 'c50c'
        endpoint_checks = {
            'wheel_odom': {
                'topic': self.wheel_odom_topic,
                'publishers': wheel_publishers,
                'ok': (
                    len(wheel_publishers) == 1
                    and 'c50c' in wheel_publishers[0].lower()
                ),
            },
            'official_odom': {
                'topic': self.official_odom_topic,
                'publishers': official_publishers,
                'expected_owner': expected_official,
                'ok': (
                    len(official_publishers) == 1
                    and expected_official in official_publishers[0].lower()
                ),
            },
            'n300': {
                'topic': self.n300_imu_topic,
                'publishers': n300_publishers,
                'ok': (
                    not fused_requires_n300
                    or (
                        len(n300_publishers) == 1
                        and 'n300' in n300_publishers[0].lower()
                    )
                ),
            },
        }

        all_tf_publishers = self.publisher_names('/tf')
        odom_tf_candidates = sorted(
            name
            for name in all_tf_publishers
            if 'c50c' in name.lower() or 'ekf' in name.lower()
        )
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                self.base_frame,
                rclpy.time.Time(),
            )
            transform_available = True
        except TransformException:
            transform_available = False
        transform_age = math.inf
        transform_finite = False
        if transform is not None:
            stamp = transform.header.stamp
            stamp_sec = int(stamp.sec) + int(stamp.nanosec) * 1.0e-9
            transform_age = time.time() - stamp_sec
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transform_finite = _all_finite([
                translation.x,
                translation.y,
                translation.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ])
            quaternion_norm = math.sqrt(
                rotation.x * rotation.x
                + rotation.y * rotation.y
                + rotation.z * rotation.z
                + rotation.w * rotation.w
            )
            transform_finite = (
                transform_finite
                and 0.95 <= quaternion_norm <= 1.05
            )
        tf_check = {
            'edge': f'odom->{self.base_frame}',
            'available': transform_available,
            'age_sec': None if math.isinf(transform_age) else transform_age,
            'finite': transform_finite,
            'all_tf_publishers': all_tf_publishers,
            'publisher_nodes': odom_tf_candidates,
            'expected_owner': expected_official,
            'ok': (
                transform_available
                and 0.0 <= transform_age <= self.stale_sec
                and transform_finite
                and len(odom_tf_candidates) == 1
                and expected_official in odom_tf_candidates[0].lower()
            ),
        }

        live_names = {name for name, _namespace in self.get_node_names_and_namespaces()}
        slam_running = 'slam_toolbox' in live_names
        amcl_running = 'amcl' in live_names
        no_conflicting_localization = (
            not slam_running
            if self.startup_mode == 'navigation'
            else not slam_running and not amcl_running
        )
        localization_check = {
            'startup_mode': self.startup_mode,
            'slam_toolbox_running': slam_running,
            'amcl_running': amcl_running,
            'mutually_exclusive': not (slam_running and amcl_running),
            'expected_preflight_state': (
                'slam_toolbox absent'
                if self.startup_mode == 'navigation'
                else 'AMCL and slam_toolbox both absent'
            ),
            'ok': no_conflicting_localization,
        }

        diagnostic_age = (
            math.inf
            if self.ekf_diagnostic_last is None
            else now - self.ekf_diagnostic_last
        )
        ekf_diagnostic_required = self.use_fused_odom
        ekf_diagnostic_check = {
            'required': ekf_diagnostic_required,
            'received': self.ekf_diagnostic_received,
            'age_sec': (
                None if math.isinf(diagnostic_age) else diagnostic_age
            ),
            'stale_after_sec': self.diagnostic_stale_sec,
            'max_level': self.ekf_diagnostic_level,
            'messages': list(self.ekf_diagnostic_messages),
            'ok': (
                not ekf_diagnostic_required
                or (
                    self.ekf_diagnostic_received
                    and diagnostic_age <= self.diagnostic_stale_sec
                    and self.ekf_diagnostic_level is not None
                    and self.ekf_diagnostic_level < 2
                )
            ),
        }

        all_ok = (
            all(item['ok'] for item in checks.values())
            and all(item['ok'] for item in endpoint_checks.values())
            and tf_check['ok']
            and localization_check['ok']
            and ekf_diagnostic_check['ok']
        )
        report = {
            'generated_at_epoch': time.time(),
            'mode': 'fused' if self.use_fused_odom else 'rollback',
            'startup_mode': self.startup_mode,
            'allow_wheel_odom_only': self.allow_wheel_odom_only,
            'streams': checks,
            'endpoints': endpoint_checks,
            'tf': tf_check,
            'localization': localization_check,
            'ekf_diagnostic': ekf_diagnostic_check,
            'healthy': all_ok,
        }
        self.last_report = report
        signature = json.dumps(report, sort_keys=True, default=str)
        if signature != self.last_signature and now - self.last_log >= 1.0:
            log_message = 'Navigation preflight: ' + json.dumps(
                report,
                sort_keys=True,
            )
            # In rclpy Humble one call site cannot switch logger severity.
            if all_ok:
                self.get_logger().info(log_message)
            else:
                self.get_logger().warning(log_message)
            self.last_signature = signature
            self.last_log = now

        if all_ok:
            if self.healthy_since is None:
                self.healthy_since = now
            if now - self.healthy_since >= self.stable_for_sec:
                self.exit_code = 0
                self.get_logger().info(
                    'NAVIGATION_HEALTH_OK ' + json.dumps(report, sort_keys=True)
                )
                return
        else:
            self.healthy_since = None

        if self.timeout_sec > 0.0 and now - self.started >= self.timeout_sec:
            self.exit_code = 2
            self.get_logger().error(
                'NAVIGATION_HEALTH_TIMEOUT ' + json.dumps(report, sort_keys=True)
            )
            return


def main():
    rclpy.init()
    node = NavigationHealthCheck()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        exit_code = 2 if node.exit_code is None else node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
