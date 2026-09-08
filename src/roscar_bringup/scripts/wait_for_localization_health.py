#!/usr/bin/env python3
"""Hold Nav2 navigation startup until AMCL localization is genuinely usable.

This process is deliberately a launch gate: exit code 0 means localization has
remained healthy for the configured stability window.  While localization is
missing or stale it keeps running, so map_server, AMCL, the web UI, and manual
control can stay available without presenting Nav2 navigation as ready.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_yaw(quaternion):
    """Return planar yaw while rejecting non-finite/invalid quaternions."""
    values = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-6:
        raise ValueError('quaternion norm is zero')
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def angular_distance(left, right):
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


class LocalizationHealthGate(Node):
    def __init__(self):
        super().__init__('localization_health_check')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Humble AMCL republishes map->odom while stationary, but may publish
        # /amcl_pose only once after SetInitialPose.  Keep the stability window
        # shorter than pose freshness so a valid stationary localization can pass.
        self.declare_parameter('stable_for_sec', 1.5)
        self.declare_parameter('amcl_pose_max_age_sec', 3.0)
        self.declare_parameter('tf_max_age_sec', 1.5)
        self.declare_parameter('max_tf_position_jump_m', 0.75)
        self.declare_parameter('max_tf_yaw_jump_rad', math.radians(45.0))
        self.declare_parameter('check_period_sec', 0.10)
        self.declare_parameter('status_log_period_sec', 5.0)

        self.amcl_pose = None
        self.amcl_pose_received_at = None
        self.stable_since = None
        self.previous_transforms = None
        self.last_status = None
        self.last_log_at = 0.0

        pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._amcl_pose_callback,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            'Localization gate waiting for a fresh AMCL pose and stable '
            'map->odom/base_footprint transforms; Nav2 navigation is not active.'
        )

    def _amcl_pose_callback(self, message):
        self.amcl_pose = message
        self.amcl_pose_received_at = time.monotonic()

    @staticmethod
    def _qualified_node_names(endpoint_infos):
        names = set()
        for info in endpoint_infos:
            namespace = str(info.node_namespace or '').rstrip('/')
            name = str(info.node_name or '')
            qualified = f'{namespace}/{name}' if namespace else f'/{name}'
            names.add(qualified.replace('//', '/'))
        return sorted(names)

    def _message_age(self, stamp):
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return math.inf
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9

    def _validate_amcl_pose(self, now_monotonic):
        if self.amcl_pose is None or self.amcl_pose_received_at is None:
            return 'waiting for the first /amcl_pose message'

        max_age = float(self.get_parameter('amcl_pose_max_age_sec').value)
        receive_age = now_monotonic - self.amcl_pose_received_at
        stamp_age = self._message_age(self.amcl_pose.header.stamp)
        if receive_age > max_age:
            return f'/amcl_pose receive age {receive_age:.2f}s exceeds {max_age:.2f}s'
        if stamp_age < -0.5 or stamp_age > max_age:
            return f'/amcl_pose stamp age {stamp_age:.2f}s is not fresh'
        if self.amcl_pose.header.frame_id.lstrip('/') != str(
                self.get_parameter('map_frame').value).lstrip('/'):
            return (
                f'/amcl_pose frame is {self.amcl_pose.header.frame_id!r}, '
                'expected map'
            )

        pose = self.amcl_pose.pose.pose
        position_values = (pose.position.x, pose.position.y, pose.position.z)
        covariance = self.amcl_pose.pose.covariance
        if not all(math.isfinite(value) for value in position_values):
            return '/amcl_pose position contains a non-finite value'
        if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
            return '/amcl_pose covariance is invalid'
        if covariance[0] < 0.0 or covariance[7] < 0.0 or covariance[35] < 0.0:
            return '/amcl_pose covariance contains a negative planar variance'
        try:
            quaternion_yaw(pose.orientation)
        except ValueError as error:
            return f'/amcl_pose orientation is invalid: {error}'
        return None

    def _lookup_fresh_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            return None, f'{target_frame}->{source_frame} unavailable: {error}'

        max_age = float(self.get_parameter('tf_max_age_sec').value)
        stamp_age = self._message_age(transform.header.stamp)
        # AMCL commonly future-dates map->odom by transform_tolerance.
        if stamp_age < -1.0 or stamp_age > max_age:
            return (
                None,
                f'{target_frame}->{source_frame} stamp age '
                f'{stamp_age:.2f}s is not fresh',
            )
        translation = transform.transform.translation
        if not all(math.isfinite(value) for value in (
                translation.x, translation.y, translation.z)):
            return None, f'{target_frame}->{source_frame} translation is invalid'
        try:
            quaternion_yaw(transform.transform.rotation)
        except ValueError as error:
            return None, f'{target_frame}->{source_frame} rotation is invalid: {error}'
        return transform, None

    @staticmethod
    def _planar_transform(transform: TransformStamped):
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            quaternion_yaw(transform.transform.rotation),
        )

    def _transforms_are_continuous(self, current):
        if self.previous_transforms is None:
            self.previous_transforms = current
            return True
        max_position_jump = float(
            self.get_parameter('max_tf_position_jump_m').value)
        max_yaw_jump = float(self.get_parameter('max_tf_yaw_jump_rad').value)
        for frame_pair, pose in current.items():
            previous = self.previous_transforms[frame_pair]
            position_jump = math.hypot(
                pose[0] - previous[0],
                pose[1] - previous[1],
            )
            yaw_jump = angular_distance(pose[2], previous[2])
            if position_jump > max_position_jump or yaw_jump > max_yaw_jump:
                self.previous_transforms = current
                return False
        self.previous_transforms = current
        return True

    def check(self):
        now_monotonic = time.monotonic()
        live_nodes = {
            f'{str(namespace).rstrip("/")}/{name}'.replace('//', '/')
            for name, namespace in self.get_node_names_and_namespaces()
        }
        if '/amcl' not in live_nodes:
            return False, 'AMCL node is not running'
        if '/slam_toolbox' in live_nodes:
            return False, 'slam_toolbox and AMCL must not run simultaneously'

        try:
            tf_publishers = self._qualified_node_names(
                self.get_publishers_info_by_topic('/tf')
            )
        except Exception as error:
            return False, f'cannot inspect /tf publishers: {error}'
        map_tf_candidates = [
            name
            for name in tf_publishers
            if 'amcl' in name.lower() or 'slam_toolbox' in name.lower()
        ]
        if map_tf_candidates != ['/amcl']:
            return (
                False,
                'map->odom TF endpoint owner must be AMCL only; '
                f'candidates={map_tf_candidates}',
            )

        pose_error = self._validate_amcl_pose(now_monotonic)
        if pose_error is not None:
            self.previous_transforms = None
            return False, pose_error

        map_frame = str(self.get_parameter('map_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        transforms = {}
        for source_frame in (odom_frame, base_frame):
            transform, error = self._lookup_fresh_transform(
                map_frame,
                source_frame,
            )
            if error is not None:
                self.previous_transforms = None
                return False, error
            transforms[f'{map_frame}->{source_frame}'] = self._planar_transform(
                transform)

        if not self._transforms_are_continuous(transforms):
            return False, 'localization TF jump detected; restarting stability window'
        return True, 'AMCL pose and localization TF are fresh and continuous'

    def run(self):
        period = max(0.02, float(self.get_parameter('check_period_sec').value))
        stable_for = max(0.0, float(self.get_parameter('stable_for_sec').value))
        log_period = max(
            1.0,
            float(self.get_parameter('status_log_period_sec').value),
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=period)
            now_monotonic = time.monotonic()
            healthy, status = self.check()
            if not healthy:
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = now_monotonic

            stable_duration = (
                0.0
                if self.stable_since is None
                else now_monotonic - self.stable_since
            )
            if healthy and stable_duration >= stable_for:
                self.get_logger().info(
                    'LOCALIZATION_HEALTH_OK: fresh /amcl_pose and stable '
                    f'map->odom/base_footprint for {stable_duration:.2f}s'
                )
                return 0

            if self.last_status is None or now_monotonic - self.last_log_at >= log_period:
                suffix = (
                    f'; stable {stable_duration:.2f}/{stable_for:.2f}s'
                    if healthy
                    else ''
                )
                self.get_logger().info(
                    f'Localization gate: {status}{suffix}; '
                    'Nav2 navigation remains stopped.'
                )
                self.last_status = status
                self.last_log_at = now_monotonic
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealthGate()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    except Exception as error:  # Keep a failed checker from ever releasing Nav2.
        node.get_logger().error(f'LOCALIZATION_HEALTH_FAILED: {error}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
