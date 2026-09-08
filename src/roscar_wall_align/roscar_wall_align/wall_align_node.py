import math
import random
import statistics
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def normalize_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_line_angle(angle):
    angle = normalize_pi(angle)
    if angle > math.pi / 2.0:
        angle -= math.pi
    if angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def axial_delta(angle, target):
    diff = normalize_pi(angle - target)
    if diff > math.pi / 2.0:
        diff -= math.pi
    if diff < -math.pi / 2.0:
        diff += math.pi
    return diff


def median_filter(values, window):
    if window < 3:
        return list(values)
    if window % 2 == 0:
        window += 1
    half = window // 2
    output = list(values)
    for index, value in enumerate(values):
        if not math.isfinite(value):
            output[index] = math.inf
            continue
        first = max(0, index - half)
        last = min(len(values) - 1, index + half)
        valid = sorted(v for v in values[first:last + 1] if math.isfinite(v))
        if len(valid) >= 2:
            output[index] = valid[len(valid) // 2]
    return output


class WallAlignNode(Node):
    def __init__(self):
        super().__init__('wall_align_node')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('align_scan_topic', '/scan_align_filtered')
        self.declare_parameter('done_topic', '/wall_align_done')
        self.declare_parameter('status_topic', '/wall_align/status')
        self.declare_parameter('mode', 'front')
        self.declare_parameter('target_mode', 'parallel')
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 6.0)
        self.declare_parameter('median_window', 5)
        self.declare_parameter('ransac_iterations', 160)
        self.declare_parameter('ransac_distance_threshold', 0.035)
        self.declare_parameter('min_inliers', 40)
        self.declare_parameter('min_wall_length', 0.8)
        self.declare_parameter('stable_frame_count', 10)
        self.declare_parameter('stable_angle_deg', 1.0)
        self.declare_parameter('jump_reject_deg', 5.0)
        self.declare_parameter('align_tolerance_deg', 1.5)
        self.declare_parameter('stable_done_sec', 1.0)
        self.declare_parameter('wall_search_timeout_sec', 10.0)
        self.declare_parameter('timeout_sec', 25.0)
        self.declare_parameter('no_scan_timeout_sec', 1.2)
        self.declare_parameter('min_angular_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.15)
        self.declare_parameter('hard_max_angular_speed', 0.20)
        self.declare_parameter('near_target_slow_deg', 4.0)
        self.declare_parameter('odom_angular_stop_threshold', 0.02)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.align_scan_topic = self.get_parameter('align_scan_topic').value
        self.done_topic = self.get_parameter('done_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.mode = str(self.get_parameter('mode').value).lower()
        self.target_mode = str(self.get_parameter('target_mode').value).lower()
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.median_window = int(self.get_parameter('median_window').value)
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').value)
        self.ransac_distance_threshold = float(self.get_parameter('ransac_distance_threshold').value)
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.min_wall_length = float(self.get_parameter('min_wall_length').value)
        self.stable_frame_count = int(self.get_parameter('stable_frame_count').value)
        self.stable_angle = math.radians(float(self.get_parameter('stable_angle_deg').value))
        self.jump_reject = math.radians(float(self.get_parameter('jump_reject_deg').value))
        self.align_tolerance = math.radians(float(self.get_parameter('align_tolerance_deg').value))
        self.stable_done_sec = float(self.get_parameter('stable_done_sec').value)
        self.wall_search_timeout_sec = float(self.get_parameter('wall_search_timeout_sec').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.no_scan_timeout_sec = float(self.get_parameter('no_scan_timeout_sec').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.hard_max_angular_speed = float(self.get_parameter('hard_max_angular_speed').value)
        self.near_target_slow = math.radians(float(self.get_parameter('near_target_slow_deg').value))
        self.odom_angular_stop_threshold = float(
            self.get_parameter('odom_angular_stop_threshold').value
        )

        self.angle_min, self.angle_max = self.mode_angle_range(self.mode)
        self.target_angle = 0.0 if self.target_mode == 'parallel' else math.pi / 2.0

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_wall_align', 10)
        self.scan_pub = self.create_publisher(LaserScan, self.align_scan_topic, 10)
        self.done_pub = self.create_publisher(Bool, self.done_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(0.05, self.watchdog_timer)

        self.start_time = time.monotonic()
        self.last_scan_time = 0.0
        self.last_reliable_wall_time = 0.0
        self.last_wall_angle = None
        self.recent_angles = deque(maxlen=self.stable_frame_count)
        self.stable_since = None
        self.latest_odom_wz = 0.0
        self.finished = False
        self.success = False
        self.rng = random.Random(20260630)

        self.get_logger().info(
            'wall_align_node started: '
            f'scan={self.scan_topic}, mode={self.mode}, target={self.target_mode}, '
            f'sector=[{math.degrees(self.angle_min):.1f}, {math.degrees(self.angle_max):.1f}] deg'
        )
        self.publish_done(False)

    def mode_angle_range(self, mode):
        if mode == 'left':
            return math.radians(45.0), math.radians(135.0)
        if mode == 'right':
            return math.radians(-135.0), math.radians(-45.0)
        return math.radians(-90.0), math.radians(90.0)

    def odom_callback(self, msg):
        self.latest_odom_wz = float(msg.twist.twist.angular.z)

    def select_ranges(self, scan):
        selected = []
        filtered_ranges = [math.inf] * len(scan.ranges)
        for index, value in enumerate(scan.ranges):
            angle = normalize_pi(scan.angle_min + index * scan.angle_increment)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            if not math.isfinite(value):
                selected.append((index, angle, math.inf))
                continue
            if value < self.range_min or value > self.range_max:
                selected.append((index, angle, math.inf))
                continue
            selected.append((index, angle, float(value)))

        filtered_values = median_filter([item[2] for item in selected], self.median_window)
        points = []
        for (index, angle, _value), filtered in zip(selected, filtered_values):
            if math.isfinite(filtered):
                filtered_ranges[index] = filtered
                points.append((filtered * math.cos(angle), filtered * math.sin(angle)))
        return points, filtered_ranges

    def publish_align_scan(self, scan, filtered_ranges):
        output = LaserScan()
        output.header = scan.header
        output.angle_min = scan.angle_min
        output.angle_max = scan.angle_max
        output.angle_increment = scan.angle_increment
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = max(float(scan.range_min), self.range_min)
        output.range_max = min(float(scan.range_max), self.range_max)
        output.ranges = filtered_ranges
        if scan.intensities:
            output.intensities = list(scan.intensities)
        self.scan_pub.publish(output)

    def fit_wall_ransac(self, points):
        if len(points) < self.min_inliers:
            return None

        best = None
        count = len(points)
        for _ in range(self.ransac_iterations):
            i1 = self.rng.randrange(count)
            i2 = self.rng.randrange(count)
            if i1 == i2:
                continue
            x1, y1 = points[i1]
            x2, y2 = points[i2]
            dx = x2 - x1
            dy = y2 - y1
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                continue
            ux = dx / norm
            uy = dy / norm
            nx = -uy
            ny = ux
            c = -(nx * x1 + ny * y1)
            inliers = []
            projections = []
            for point in points:
                x, y = point
                distance = abs(nx * x + ny * y + c)
                if distance <= self.ransac_distance_threshold:
                    inliers.append(point)
                    projections.append(x * ux + y * uy)
            inlier_count = len(inliers)
            if inlier_count < self.min_inliers:
                continue
            angle, length = self.refit_line(inliers)
            if length < self.min_wall_length:
                continue
            score = (length, inlier_count)
            if best is None or score > best['score']:
                best = {
                    'angle': angle,
                    'inliers': inlier_count,
                    'length': length,
                    'score': score,
                }
        return best

    def refit_line(self, points):
        count = len(points)
        mean_x = sum(point[0] for point in points) / count
        mean_y = sum(point[1] for point in points) / count
        sxx = sum((point[0] - mean_x) ** 2 for point in points) / count
        syy = sum((point[1] - mean_y) ** 2 for point in points) / count
        sxy = sum((point[0] - mean_x) * (point[1] - mean_y) for point in points) / count
        angle = normalize_line_angle(0.5 * math.atan2(2.0 * sxy, sxx - syy))
        ux = math.cos(angle)
        uy = math.sin(angle)
        projections = [point[0] * ux + point[1] * uy for point in points]
        length = max(projections) - min(projections) if projections else 0.0
        return angle, length

    def stable_wall_angle(self, angle):
        if self.last_wall_angle is not None:
            jump = abs(axial_delta(angle, self.last_wall_angle))
            if jump > self.jump_reject:
                self.publish_status(
                    wall_angle=angle,
                    error=None,
                    cmd_wz=0.0,
                    stable_frames=len(self.recent_angles),
                    note=f'discard jump {math.degrees(jump):.2f}deg',
                )
                return None
        self.last_wall_angle = angle
        self.recent_angles.append(angle)
        if len(self.recent_angles) < self.stable_frame_count:
            return None
        median_angle = statistics.median(self.recent_angles)
        max_jitter = max(abs(axial_delta(item, median_angle)) for item in self.recent_angles)
        if max_jitter > self.stable_angle:
            return None
        return median_angle

    def compute_cmd_wz(self, error):
        abs_error = abs(error)
        if abs_error <= self.align_tolerance:
            return 0.0
        if abs_error < self.near_target_slow:
            ratio = max(0.25, abs_error / self.near_target_slow)
            speed = self.min_angular_speed * ratio
        else:
            speed = max(self.min_angular_speed, min(self.max_angular_speed, 0.85 * abs_error))
        speed = min(speed, self.hard_max_angular_speed)
        return math.copysign(speed, error)

    def publish_cmd(self, wz):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_zero(self):
        self.publish_cmd(0.0)

    def publish_done(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.done_pub.publish(msg)

    def publish_status(self, wall_angle=None, error=None, cmd_wz=0.0, stable_frames=0, note=''):
        wall_text = '--' if wall_angle is None else f'{math.degrees(wall_angle):.2f}'
        error_text = '--' if error is None else f'{math.degrees(error):.2f}'
        text = (
            f'mode={self.mode} target={self.target_mode} '
            f'wall_angle_deg={wall_text} error_deg={error_text} '
            f'cmd_wz={cmd_wz:.3f} stable_frames={stable_frames}/{self.stable_frame_count} '
            f'done={self.success}'
        )
        if note:
            text += f' note={note}'
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def finish(self, success, note):
        self.success = bool(success)
        for _ in range(30):
            self.publish_zero()
            time.sleep(0.03)
        if success:
            end_time = time.monotonic() + 1.0
            while time.monotonic() < end_time:
                self.publish_zero()
                time.sleep(0.05)
            self.publish_done(True)
        else:
            self.publish_done(False)
        self.publish_status(cmd_wz=0.0, stable_frames=len(self.recent_angles), note=note)
        self.finished = True

    def scan_callback(self, scan):
        if self.finished:
            return
        self.last_scan_time = time.monotonic()
        points, filtered_ranges = self.select_ranges(scan)
        self.publish_align_scan(scan, filtered_ranges)

        model = self.fit_wall_ransac(points)
        if model is None:
            self.publish_zero()
            self.stable_since = None
            self.publish_status(
                cmd_wz=0.0,
                stable_frames=len(self.recent_angles),
                note=f'no reliable wall points={len(points)}',
            )
            return
        self.last_reliable_wall_time = time.monotonic()

        stable_angle = self.stable_wall_angle(model['angle'])
        if stable_angle is None:
            self.publish_zero()
            self.stable_since = None
            self.publish_status(
                wall_angle=model['angle'],
                cmd_wz=0.0,
                stable_frames=len(self.recent_angles),
                note=f'collecting/stabilizing inliers={model["inliers"]} length={model["length"]:.2f}',
            )
            return

        error = axial_delta(stable_angle, self.target_angle)
        cmd_wz = self.compute_cmd_wz(error)
        self.publish_cmd(cmd_wz)
        self.publish_status(
            wall_angle=stable_angle,
            error=error,
            cmd_wz=cmd_wz,
            stable_frames=len(self.recent_angles),
            note=f'inliers={model["inliers"]} length={model["length"]:.2f}',
        )

        if abs(error) <= self.align_tolerance:
            if self.stable_since is None:
                self.stable_since = time.monotonic()
            stable_time = time.monotonic() - self.stable_since
            if stable_time >= self.stable_done_sec and abs(self.latest_odom_wz) < self.odom_angular_stop_threshold:
                self.finish(True, 'aligned')
        else:
            self.stable_since = None

    def watchdog_timer(self):
        if self.finished:
            return
        now = time.monotonic()
        if self.last_scan_time and now - self.last_scan_time > self.no_scan_timeout_sec:
            self.publish_zero()
            self.stable_since = None
            self.publish_status(
                cmd_wz=0.0,
                stable_frames=len(self.recent_angles),
                note='scan timeout, zero cmd_vel',
            )
        if (
            now - self.start_time > self.wall_search_timeout_sec
            and not self.last_reliable_wall_time
        ):
            self.finish(False, 'failed: reliable wall not found within wall_search_timeout')
            return
        if now - self.start_time > self.timeout_sec:
            self.finish(False, 'failed: wall detected but not aligned within timeout')


def main(args=None):
    rclpy.init(args=args)
    node = WallAlignNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish(False, 'interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == '__main__':
    raise SystemExit(main())
