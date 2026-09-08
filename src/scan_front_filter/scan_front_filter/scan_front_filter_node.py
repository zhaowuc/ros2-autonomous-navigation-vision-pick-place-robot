import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, MultiEchoLaserScan

from .scan_processing import (
    OdomFarReturnGuard,
    OdomPoseBuffer,
    Pose2D,
    compose_pose,
    deskew_ranges,
    fixed_scan_grid,
    select_multi_echo_ranges,
)


class ScanFrontFilter(Node):
    def __init__(self):
        super().__init__('scan_front_filter')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan_slam_filtered')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('multi_echo_topic', '/x10/multiecho_scan')
        self.declare_parameter('prefer_multi_echo', True)
        self.declare_parameter('output_scan_bins', 5400)
        self.output_scan_bins = int(self.get_parameter('output_scan_bins').value)
        if self.output_scan_bins <= 0:
            raise ValueError('output_scan_bins must be positive')
        self.declare_parameter('multi_echo_fallback_timeout_sec', 0.50)
        self.declare_parameter('multi_echo_min_finite', 50)
        self.declare_parameter('multi_echo_max_stamp_skew_sec', 1.0)
        self.declare_parameter('scan_max_stamp_skew_sec', 1.0)
        self.declare_parameter('lower_angle', -math.pi)
        self.declare_parameter('upper_angle', math.pi)
        self.declare_parameter('range_min_limit', 0.18)
        self.declare_parameter('range_max_limit', 4.80)

        # Legacy spatial filters are fail-open by default.  N10Plus high
        # precision LaserScan uses 5400 sparse bins for about 540 physical
        # beams, so small index windows are not physical neighborhoods.
        self.declare_parameter('median_window', 0)
        self.declare_parameter('speckle_window', 0)
        self.declare_parameter('speckle_max_delta', 0.18)
        self.declare_parameter('speckle_min_neighbors', 0)

        self.declare_parameter('enable_motion_compensation', True)
        self.declare_parameter('scan_period_fallback', 0.10)
        self.declare_parameter('scan_period_min', 0.075)
        self.declare_parameter('scan_period_max', 0.130)
        self.declare_parameter('odom_history_sec', 2.0)
        self.declare_parameter('max_odom_gap_sec', 0.060)
        self.declare_parameter('odom_reset_gap_sec', 0.25)
        self.declare_parameter('odom_jump_min_distance', 0.03)
        self.declare_parameter('odom_jump_max_speed', 0.50)
        self.declare_parameter('odom_jump_min_yaw', 0.052)
        self.declare_parameter('odom_jump_max_yaw_rate', 1.50)
        self.declare_parameter('laser_offset_x', -0.235)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('laser_offset_yaw', 0.092382808)

        self.declare_parameter('secondary_spatial_window', 2)
        self.declare_parameter('secondary_spatial_tolerance', 0.22)
        self.declare_parameter('secondary_min_neighbors', 2)

        # Keep confirmed near endpoints long enough for a slow 0.08 m/s pass.
        # A 1.2 s window allowed intermittent farther echoes to ray-trace
        # through a wall after only a few robot poses.
        self.declare_parameter('far_guard_history_scans', 50)
        self.declare_parameter('far_guard_max_age_sec', 5.0)
        self.declare_parameter('far_guard_angular_window_deg', 1.5)
        self.declare_parameter('far_guard_range_tolerance', 0.20)
        self.declare_parameter('far_guard_jump_delta', 0.25)
        self.declare_parameter('far_guard_min_confirmations', 2)
        self.declare_parameter('diagnostics_period_sec', 5.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.multi_echo_topic = str(self.get_parameter('multi_echo_topic').value)
        self.prefer_multi_echo = bool(self.get_parameter('prefer_multi_echo').value)
        self.multi_echo_fallback_timeout_sec = max(
            0.1,
            float(self.get_parameter('multi_echo_fallback_timeout_sec').value),
        )
        self.multi_echo_min_finite = max(
            1,
            int(self.get_parameter('multi_echo_min_finite').value),
        )
        self.multi_echo_max_stamp_skew_sec = max(
            0.1,
            float(self.get_parameter('multi_echo_max_stamp_skew_sec').value),
        )
        self.scan_max_stamp_skew_sec = max(
            0.1,
            float(self.get_parameter('scan_max_stamp_skew_sec').value),
        )
        self.lower_angle = float(self.get_parameter('lower_angle').value)
        self.upper_angle = float(self.get_parameter('upper_angle').value)
        self.range_min_limit = float(self.get_parameter('range_min_limit').value)
        self.range_max_limit = float(self.get_parameter('range_max_limit').value)
        self.median_window = max(0, int(self.get_parameter('median_window').value))
        if self.median_window and self.median_window % 2 == 0:
            self.median_window += 1
        self.speckle_window = max(
            0,
            int(self.get_parameter('speckle_window').value),
        )
        self.speckle_max_delta = max(
            0.0,
            float(self.get_parameter('speckle_max_delta').value),
        )
        self.speckle_min_neighbors = max(
            0,
            int(self.get_parameter('speckle_min_neighbors').value),
        )
        self.enable_motion_compensation = bool(
            self.get_parameter('enable_motion_compensation').value
        )
        self.scan_period = max(
            0.01,
            float(self.get_parameter('scan_period_fallback').value),
        )
        self.scan_period_min = max(
            0.01,
            float(self.get_parameter('scan_period_min').value),
        )
        self.scan_period_max = max(
            self.scan_period_min,
            float(self.get_parameter('scan_period_max').value),
        )
        self.max_odom_gap_sec = max(
            0.01,
            float(self.get_parameter('max_odom_gap_sec').value),
        )
        self.odom_reset_gap_sec = max(
            self.max_odom_gap_sec,
            float(self.get_parameter('odom_reset_gap_sec').value),
        )
        self.odom_jump_min_distance = max(
            0.0,
            float(self.get_parameter('odom_jump_min_distance').value),
        )
        self.odom_jump_max_speed = max(
            0.0,
            float(self.get_parameter('odom_jump_max_speed').value),
        )
        self.odom_jump_min_yaw = max(
            0.0,
            float(self.get_parameter('odom_jump_min_yaw').value),
        )
        self.odom_jump_max_yaw_rate = max(
            0.0,
            float(self.get_parameter('odom_jump_max_yaw_rate').value),
        )
        self.laser_in_base = Pose2D(
            float(self.get_parameter('laser_offset_x').value),
            float(self.get_parameter('laser_offset_y').value),
            float(self.get_parameter('laser_offset_yaw').value),
        )
        self.secondary_spatial_window = max(
            0,
            int(self.get_parameter('secondary_spatial_window').value),
        )
        self.secondary_spatial_tolerance = max(
            0.0,
            float(self.get_parameter('secondary_spatial_tolerance').value),
        )
        self.secondary_min_neighbors = max(
            0,
            int(self.get_parameter('secondary_min_neighbors').value),
        )
        self.diagnostics_period_sec = max(
            1.0,
            float(self.get_parameter('diagnostics_period_sec').value),
        )

        if self.lower_angle > self.upper_angle:
            self.lower_angle, self.upper_angle = (
                self.upper_angle,
                self.lower_angle,
            )
        self.keep_full_scan = (
            self.upper_angle - self.lower_angle
        ) >= (2.0 * math.pi - 1e-3)

        self.pose_buffer = OdomPoseBuffer(
            float(self.get_parameter('odom_history_sec').value)
        )
        self.far_guard = OdomFarReturnGuard(
            history_scans=int(
                self.get_parameter('far_guard_history_scans').value
            ),
            history_max_age_sec=float(
                self.get_parameter('far_guard_max_age_sec').value
            ),
            angular_window_rad=math.radians(float(
                self.get_parameter('far_guard_angular_window_deg').value
            )),
            range_tolerance=float(
                self.get_parameter('far_guard_range_tolerance').value
            ),
            far_jump_delta=float(
                self.get_parameter('far_guard_jump_delta').value
            ),
            min_confirmations=int(
                self.get_parameter('far_guard_min_confirmations').value
            ),
        )

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            50,
        )
        self.raw_subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.raw_scan_callback,
            10,
        )
        self.multi_echo_subscription = self.create_subscription(
            MultiEchoLaserScan,
            self.multi_echo_topic,
            self.multi_echo_callback,
            10,
        )

        now_ns = self.get_clock().now().nanoseconds
        self.start_receive_ns = now_ns
        self.last_multi_receive_ns = None
        self.last_output_stamp = None
        self.last_ros_now_seconds = None
        self.last_odom_stamp = None
        self.last_odom_pose = None
        self.last_source_stamps = {}
        self.last_diagnostics_ns = now_ns
        self.stats = {
            'raw_frames': 0,
            'multi_frames': 0,
            'finite_input': 0,
            'finite_output': 0,
            'deskew_frames': 0,
            'deskew_skipped': 0,
            'odom_lookup_failures': 0,
            'odom_resets': 0,
            'far_suppressed': 0,
            'stamp_invalid': 0,
            'stamp_out_of_order': 0,
            'clock_resets': 0,
            'primary_echoes': 0,
            'secondary_only': 0,
            'secondary_rejected': 0,
        }

        angle_mode = (
            'full 360 scan'
            if self.keep_full_scan
            else 'configured angle mask'
        )
        self.get_logger().info(
            f'N10Plus protected scan pipeline {self.input_topic} + '
            f'{self.multi_echo_topic} -> {self.output_topic}; '
            f'prefer_multi_echo={self.prefer_multi_echo}, '
            f'odom={self.odom_topic}, deskew={self.enable_motion_compensation}, '
            f'range=[{self.range_min_limit:.2f}, '
            f'{self.range_max_limit:.2f}] m, {angle_mode}, '
            f'median={self.median_window}, speckle={self.speckle_window}'
        )

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def quaternion_yaw(quaternion):
        sine = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cosine = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(sine, cosine)

    def odom_callback(self, message):
        stamp = self.stamp_seconds(message.header.stamp)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = Pose2D(
            float(position.x),
            float(position.y),
            self.quaternion_yaw(orientation),
        )
        reset = False
        if self.last_odom_stamp is not None and self.last_odom_pose is not None:
            delta_time = stamp - self.last_odom_stamp
            if delta_time <= 0.0 or delta_time > self.odom_reset_gap_sec:
                reset = True
            else:
                distance = math.hypot(
                    pose.x - self.last_odom_pose.x,
                    pose.y - self.last_odom_pose.y,
                )
                yaw_delta = abs(math.atan2(
                    math.sin(pose.yaw - self.last_odom_pose.yaw),
                    math.cos(pose.yaw - self.last_odom_pose.yaw),
                ))
                distance_limit = max(
                    self.odom_jump_min_distance,
                    self.odom_jump_max_speed * delta_time,
                )
                yaw_limit = max(
                    self.odom_jump_min_yaw,
                    self.odom_jump_max_yaw_rate * delta_time,
                )
                reset = distance > distance_limit or yaw_delta > yaw_limit
        if reset:
            self.pose_buffer.clear()
            self.far_guard.clear()
            self.stats['odom_resets'] += 1
        self.pose_buffer.add(stamp, pose)
        self.last_odom_stamp = stamp
        self.last_odom_pose = pose

    def clean_range(self, value):
        if not math.isfinite(value):
            return math.inf
        if value < self.range_min_limit or value > self.range_max_limit:
            return math.inf
        return float(value)

    def remove_speckles(self, ranges):
        if self.speckle_window <= 0 or self.speckle_min_neighbors <= 0:
            return ranges
        output = list(ranges)
        count = len(ranges)
        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue
            neighbors = 0
            for offset in range(-self.speckle_window, self.speckle_window + 1):
                if offset == 0:
                    continue
                neighbor_index = index + offset
                if self.keep_full_scan:
                    neighbor_index %= count
                elif neighbor_index < 0 or neighbor_index >= count:
                    continue
                neighbor = ranges[neighbor_index]
                if (
                    math.isfinite(neighbor)
                    and abs(neighbor - value) <= self.speckle_max_delta
                ):
                    neighbors += 1
            if neighbors < self.speckle_min_neighbors:
                output[index] = math.inf
        return output

    def median_filter(self, ranges):
        if self.median_window < 3:
            return ranges
        half = self.median_window // 2
        output = list(ranges)
        maximum_delta = max(0.20, self.speckle_max_delta * 1.5)
        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue
            values = []
            for offset in range(-half, half + 1):
                neighbor_index = index + offset
                if self.keep_full_scan:
                    neighbor_index %= len(ranges)
                elif neighbor_index < 0 or neighbor_index >= len(ranges):
                    continue
                neighbor = ranges[neighbor_index]
                if math.isfinite(neighbor):
                    values.append(neighbor)
            if len(values) < 2:
                continue
            values.sort()
            median = values[len(values) // 2]
            if abs(median - value) <= maximum_delta:
                output[index] = median
        return output

    def should_use_raw_fallback(self):
        if not self.prefer_multi_echo:
            return True
        now_ns = self.get_clock().now().nanoseconds
        reference_ns = (
            self.last_multi_receive_ns
            if self.last_multi_receive_ns is not None
            else self.start_receive_ns
        )
        return (
            now_ns - reference_ns
        ) * 1e-9 > self.multi_echo_fallback_timeout_sec

    def raw_scan_callback(self, scan):
        if not self.should_use_raw_fallback():
            return
        ranges = []
        intensities = []
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            keep_angle = (
                self.keep_full_scan
                or self.lower_angle <= angle <= self.upper_angle
            )
            ranges.append(self.clean_range(value) if keep_angle else math.inf)
            if scan.intensities:
                intensity = (
                    scan.intensities[index]
                    if index < len(scan.intensities)
                    else 0.0
                )
                intensities.append(float(intensity) if keep_angle else 0.0)
        self.stats['raw_frames'] += 1
        self.process_scan(
            scan.header,
            scan.angle_min,
            scan.angle_max,
            scan.angle_increment,
            scan.scan_time,
            ranges,
            intensities,
            'raw',
        )

    def multi_echo_callback(self, scan):
        if not self.prefer_multi_echo:
            return
        if (
            not scan.ranges
            or len(scan.ranges) < 100
            or not scan.header.frame_id
            or scan.angle_increment <= 0.0
            or scan.angle_max <= scan.angle_min
        ):
            return
        stamp = self.stamp_seconds(scan.header.stamp)
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        if (
            stamp <= 0.0
            or abs(now_seconds - stamp) > self.multi_echo_max_stamp_skew_sec
        ):
            return
        echo_values = [list(item.echoes) for item in scan.ranges]
        ranges, selected_echoes, echo_stats = select_multi_echo_ranges(
            echo_values,
            self.range_min_limit,
            self.range_max_limit,
            spatial_window=self.secondary_spatial_window,
            spatial_tolerance=self.secondary_spatial_tolerance,
            min_secondary_neighbors=self.secondary_min_neighbors,
            full_scan=self.keep_full_scan,
        )
        intensities = []
        if scan.intensities:
            for index, echo_index in enumerate(selected_echoes):
                value = 0.0
                if 0 <= echo_index and index < len(scan.intensities):
                    echoes = scan.intensities[index].echoes
                    if echo_index < len(echoes):
                        value = float(echoes[echo_index])
                intensities.append(value)

        if not self.keep_full_scan:
            for index in range(len(ranges)):
                angle = scan.angle_min + index * scan.angle_increment
                if angle < self.lower_angle or angle > self.upper_angle:
                    ranges[index] = math.inf
                    if intensities:
                        intensities[index] = 0.0

        finite_count = sum(1 for value in ranges if math.isfinite(value))
        if finite_count < self.multi_echo_min_finite:
            return
        self.stats['multi_frames'] += 1
        self.stats['primary_echoes'] += echo_stats['primary']
        self.stats['secondary_only'] += echo_stats['secondary_only']
        self.stats['secondary_rejected'] += echo_stats['secondary_rejected']
        accepted = self.process_scan(
            scan.header,
            scan.angle_min,
            scan.angle_max,
            scan.angle_increment,
            scan.scan_time,
            ranges,
            intensities,
            'multi',
        )
        if accepted:
            self.last_multi_receive_ns = self.get_clock().now().nanoseconds

    def update_scan_period(self, source, stamp, declared_scan_time):
        candidate = None
        if (
            math.isfinite(declared_scan_time)
            and self.scan_period_min <= declared_scan_time <= self.scan_period_max
        ):
            candidate = float(declared_scan_time)
        previous = self.last_source_stamps.get(source)
        self.last_source_stamps[source] = stamp
        if previous is not None:
            delta = stamp - previous
            if self.scan_period_min <= delta <= self.scan_period_max:
                candidate = delta
        if candidate is not None:
            self.scan_period = 0.85 * self.scan_period + 0.15 * candidate
        return self.scan_period

    def process_scan(
        self,
        header,
        angle_min,
        angle_max,
        angle_increment,
        declared_scan_time,
        ranges,
        intensities,
        source,
    ):
        if not ranges or angle_increment <= 0.0:
            return False
        stamp = self.stamp_seconds(header.stamp)
        now_seconds = self.get_clock().now().nanoseconds * 1.0e-9
        clock_rollback = (
            self.last_ros_now_seconds is not None
            and now_seconds < self.last_ros_now_seconds - 0.25
        )
        self.last_ros_now_seconds = now_seconds
        stamp_age = now_seconds - stamp
        if (
            not math.isfinite(stamp)
            or stamp <= 0.0
            or stamp_age < -0.25
            or stamp_age > self.scan_max_stamp_skew_sec
        ):
            self.stats['stamp_invalid'] += 1
            return False
        if self.last_output_stamp is not None:
            stamp_delta = stamp - self.last_output_stamp
            if stamp_delta < 0.0:
                if stamp_delta < -0.25 and clock_rollback:
                    # ROS time changed epoch.  Accept the first frame in the
                    # new epoch only after discarding every old-time pose and
                    # endpoint; otherwise scans remain rejected indefinitely.
                    self.far_guard.clear()
                    self.pose_buffer.clear()
                    self.last_source_stamps.clear()
                    self.last_output_stamp = None
                    self.last_odom_stamp = None
                    self.last_odom_pose = None
                    self.last_multi_receive_ns = None
                    self.stats['clock_resets'] += 1
                    stamp_delta = float('inf')
                else:
                    # Never publish an out-of-order scan or let it keep the
                    # preferred source fresh.
                    self.stats['stamp_out_of_order'] += 1
                    return False
            if stamp_delta < 0.020:
                # A healthy preferred MultiEcho frame may coincide with a
                # raw fallback frame.  Count it as accepted so subsequent raw
                # frames are disabled, without publishing a duplicate scan.
                return source == 'multi'
        self.last_output_stamp = stamp
        scan_period = self.update_scan_period(
            source,
            stamp,
            float(declared_scan_time),
        )

        ranges = self.median_filter(self.remove_speckles(ranges))
        self.stats['finite_input'] += sum(
            1 for value in ranges if math.isfinite(value)
        )

        reference_base = self.pose_buffer.lookup(stamp, self.max_odom_gap_sec)
        reference_laser = (
            compose_pose(reference_base, self.laser_in_base)
            if reference_base is not None
            else None
        )
        deskew_sources = list(range(len(ranges)))
        if self.enable_motion_compensation:
            (
                ranges,
                deskew_sources,
                deskew_applied,
                reference_laser,
                lookup_failures,
            ) = deskew_ranges(
                ranges,
                float(angle_min),
                float(angle_increment),
                stamp,
                scan_period,
                self.pose_buffer,
                self.laser_in_base,
                self.range_min_limit,
                self.range_max_limit,
                max_odom_gap=self.max_odom_gap_sec,
                full_scan=self.keep_full_scan,
            )
            if deskew_applied:
                self.stats['deskew_frames'] += 1
            else:
                self.stats['deskew_skipped'] += 1
            self.stats['odom_lookup_failures'] += lookup_failures

        corrected_intensities = []
        if intensities:
            corrected_intensities = [0.0] * len(ranges)
            for index, source_index in enumerate(deskew_sources):
                if 0 <= source_index < len(intensities):
                    corrected_intensities[index] = float(
                        intensities[source_index]
                    )

        ranges, far_suppressed = self.far_guard.filter(
            ranges,
            float(angle_min),
            float(angle_increment),
            stamp,
            reference_laser,
            self.range_min_limit,
            self.range_max_limit,
            full_scan=self.keep_full_scan,
        )
        self.stats['far_suppressed'] += far_suppressed
        self.far_guard.add_scan(
            ranges,
            float(angle_min),
            float(angle_increment),
            stamp,
            reference_laser,
        )

        if corrected_intensities:
            for index, value in enumerate(ranges):
                if not math.isfinite(value):
                    corrected_intensities[index] = 0.0

        # Raw/multi sources always share one fixed output geometry. Mapping
        # uses 540 bins so padding does not dilute Karto match confidence;
        # the fine raw bearings are retained through deskew above.
        if self.keep_full_scan:
            ranges, corrected_intensities = fixed_scan_grid(
                ranges, corrected_intensities, float(angle_min),
                float(angle_increment), self.output_scan_bins,
            )
            angle_min, angle_max = -math.pi, math.pi
            angle_increment = 2.0 * math.pi / self.output_scan_bins
        output = LaserScan()
        output.header = header
        output.angle_min = float(angle_min)
        output.angle_max = float(angle_max)
        output.angle_increment = float(angle_increment)
        # Every output point has been expressed in the sweep-end laser frame.
        output.time_increment = 0.0
        output.scan_time = float(scan_period)
        output.range_min = self.range_min_limit
        output.range_max = self.range_max_limit
        output.ranges = ranges
        if corrected_intensities:
            output.intensities = corrected_intensities
        self.publisher.publish(output)

        self.stats['finite_output'] += sum(
            1 for value in ranges if math.isfinite(value)
        )
        self.maybe_log_diagnostics(source, scan_period)
        return True

    def maybe_log_diagnostics(self, source, scan_period):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_diagnostics_ns) * 1e-9
        if elapsed < self.diagnostics_period_sec:
            return
        input_count = self.stats['finite_input']
        retention = (
            100.0 * self.stats['finite_output'] / input_count
            if input_count
            else 0.0
        )
        self.get_logger().info(
            'scan_filter_diag '
            f'source={source} period={scan_period:.4f}s '
            f'raw_frames={self.stats["raw_frames"]} '
            f'multi_frames={self.stats["multi_frames"]} '
            f'deskew={self.stats["deskew_frames"]}/'
            f'{self.stats["deskew_skipped"]} '
            f'odom_lookup_failures={self.stats["odom_lookup_failures"]} '
            f'odom_resets={self.stats["odom_resets"]} '
            f'far_suppressed={self.stats["far_suppressed"]} '
            f'stamp_invalid={self.stats["stamp_invalid"]} '
            f'stamp_out_of_order={self.stats["stamp_out_of_order"]} '
            f'clock_resets={self.stats["clock_resets"]} '
            f'secondary_only={self.stats["secondary_only"]} '
            f'secondary_rejected={self.stats["secondary_rejected"]} '
            f'finite_retention={retention:.2f}%'
        )
        for key in self.stats:
            self.stats[key] = 0
        self.last_diagnostics_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrontFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
