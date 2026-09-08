import fcntl
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav2_msgs.srv import SetInitialPose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .scan_match import (
    build_subcell_distance_field,
    confidence_is_high,
    distance_to_occupied,
    distinct_pose_score_gap,
    frame_consistency,
    legacy_score_batch,
    refine_seed,
    robust_score_batch,
    wrap_angle,
)


AUTO_LOCALIZE_LOCK = Path('/tmp/roscar_auto_localize.lock')
AUTO_LOCALIZE_RESULT = Path('/tmp/roscar_auto_localize.result.json')


def stage_score_passes(stage, score, min_score, refined_min_score):
    """Apply the coarse and refined confidence thresholds independently."""
    threshold = min_score if stage == 'coarse' else refined_min_score
    return float(score) >= float(threshold)


def check_calculation_deadline(deadline):
    if time.monotonic() >= deadline:
        raise TimeoutError('localization calculation exceeded its time budget')


def pose_within_prior(pose, prior):
    return not len(prior) or (
        math.hypot(pose[1] - prior[0], pose[2] - prior[1]) <= prior[3]
        and abs(wrap_angle(pose[3] - prior[2])) <= math.radians(prior[4])
    )


def search_pose_seeds(image, dist, points, resolution, origin, deadline, prior=()):
    """Generate candidates cheaply, then score them with the original kernel.

    The broad first pass only proposes poses; it never accepts a localization.
    Final scores, independent-frame checks and ambiguity gates remain strict.
    """
    height, width = image.shape
    origin_x, origin_y = origin[:2]
    candidate_rc = np.argwhere((image > 230) & (dist > 0.12))
    positions = np.column_stack((
        origin_x + (candidate_rc[:, 1] + 0.5) * resolution,
        origin_y + (height - candidate_rc[:, 0] - 0.5) * resolution,
    )).astype(np.float32)
    if len(prior):
        if len(prior) != 5 or not all(math.isfinite(float(v)) for v in prior):
            raise ValueError('prior must contain finite x, y, yaw, radius and yaw radius')
        x, y, yaw, radius, yaw_radius = map(float, prior)
        if radius <= 0.0 or not 0.0 < yaw_radius <= 180.0:
            raise ValueError('invalid local search extent')
        keep = np.hypot(positions[:, 0] - x, positions[:, 1] - y) <= radius
        positions = positions[keep]
        candidate_rc = candidate_rc[keep]
        angles = yaw + np.deg2rad(np.arange(-yaw_radius, yaw_radius + 0.1, 4.0))
        spacing = 0.10
    else:
        angles = np.deg2rad(np.arange(-180.0, 180.0, 6.0))
        spacing = 0.20
    if not len(positions):
        raise ValueError('no free candidate positions in the requested search region')
    stride = max(1, round(spacing / resolution))
    _, indices = np.unique(candidate_rc // stride, axis=0, return_index=True)
    coarse_positions = positions[indices]
    coarse_points = points[np.linspace(0, len(points) - 1, min(80, len(points))).astype(int)]
    candidates = []
    for yaw in angles:
        check_calculation_deadline(deadline)
        scores = legacy_score_batch(
            coarse_positions, yaw, coarse_points, dist,
            origin_x, origin_y, resolution, sigma=0.25,
        )
        count = min(8, len(scores))
        for index in np.argpartition(scores, -count)[-count:]:
            candidates.append((float(scores[index]), *map(float, coarse_positions[index]), float(yaw)))
    candidates.sort(reverse=True)
    seeds = []
    for candidate in candidates:
        if all(
            math.hypot(candidate[1] - other[1], candidate[2] - other[2]) >= 0.30
            or abs(wrap_angle(candidate[3] - other[3])) >= math.radians(12.0)
            for other in seeds
        ):
            seeds.append(candidate)
            if len(seeds) == 12:
                break
    fine = []
    offsets = np.arange(-0.22, 0.221, 0.02, dtype=np.float32)
    grid_offsets = np.array([(dx, dy) for dx in offsets for dy in offsets], dtype=np.float32)
    for _, x, y, yaw in seeds:
        grid = grid_offsets + (x, y)
        inside = (
            (grid[:, 0] >= origin_x) & (grid[:, 0] < origin_x + width * resolution)
            & (grid[:, 1] >= origin_y) & (grid[:, 1] < origin_y + height * resolution)
        )
        grid = grid[inside]
        cols = np.floor((grid[:, 0] - origin_x) / resolution).astype(int)
        rows = height - 1 - np.floor((grid[:, 1] - origin_y) / resolution).astype(int)
        allowed = (image[rows, cols] > 230) & (dist[rows, cols] > 0.12)
        if len(prior):
            allowed &= np.hypot(grid[:, 0] - prior[0], grid[:, 1] - prior[1]) <= prior[3]
        grid = grid[allowed]
        if not len(grid):
            continue
        for delta in np.deg2rad(np.arange(-6.0, 6.01, 1.0)):
            check_calculation_deadline(deadline)
            if len(prior) and abs(wrap_angle(yaw + delta - prior[2])) > math.radians(prior[4]):
                continue
            scores = legacy_score_batch(grid, yaw + delta, points, dist, origin_x, origin_y, resolution)
            index = int(np.argmax(scores))
            fine.append((float(scores[index]), *map(float, grid[index]), wrap_angle(yaw + delta)))
    return sorted(fine, reverse=True)


def map_yaml_is_usable(path):
    path = Path(path)
    if not path.is_file():
        return False
    try:
        image_path, _resolution, _origin = parse_map_yaml(path)
        return image_path.is_file()
    except Exception:
        return False


def parse_map_yaml(path):
    path = Path(path)
    values = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    image_path = Path(str(values.get('image') or ''))
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    origin = [float(item) for item in list(values.get('origin') or [0.0, 0.0, 0.0])[:3]]
    while len(origin) < 3:
        origin.append(0.0)
    return image_path, float(values.get('resolution', 0.05)), origin


def read_pgm(path):
    data = Path(path).read_bytes()
    index = 0

    def token():
        nonlocal index
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if index < len(data) and data[index] == ord('#'):
            while index < len(data) and data[index] not in (10, 13):
                index += 1
            return token()
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        return data[start:index].decode()

    if token() != 'P5':
        raise ValueError('only binary PGM P5 maps are supported')
    width = int(token())
    height = int(token())
    token()
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    return np.frombuffer(data[index:index + width * height], dtype=np.uint8).reshape((height, width))


class AutoLocalize(Node):
    def __init__(self):
        super().__init__('roscar_auto_localize')
        map_root = Path.home() / 'roscar_maps'
        active_map = map_root / 'active.yaml'
        current_map = map_root / 'current.yaml'
        self.declare_parameter(
            'map_yaml',
            str(active_map if map_yaml_is_usable(active_map) else current_map),
        )
        self.declare_parameter('scan_topic', '/scan_slam_filtered')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('laser_x', -0.235)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', 0.092382808)
        self.declare_parameter('min_score', 0.90)
        self.declare_parameter('refined_min_score', 0.90)
        self.declare_parameter('min_frame_score', 0.90)
        self.declare_parameter('max_frame_translation_m', 0.010)
        self.declare_parameter('max_frame_yaw_deg', 0.8)
        self.declare_parameter('max_frame_translation_span_m', 0.015)
        self.declare_parameter('max_frame_yaw_span_deg', 1.0)
        self.declare_parameter('min_distinct_score_gap', 0.010)
        self.declare_parameter('target_frames', 5)
        self.declare_parameter('min_frame_interval_sec', 0.20)
        self.declare_parameter('max_rays', 220)
        self.declare_parameter('prior_pose', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('local_search_radius_m', 0.75)
        self.declare_parameter('local_search_yaw_deg', 30.0)
        self.declare_parameter('calculation_timeout_sec', 6.0)
        self.declare_parameter('min_valid_rays', 100)
        self.declare_parameter('fine_resolution', 0.005)
        self.declare_parameter('max_stationary_translation_m', 0.005)
        self.declare_parameter('max_stationary_yaw_deg', 0.30)
        self.declare_parameter('max_observation_age_sec', 8.0)
        self.declare_parameter('max_odom_age_sec', 0.10)
        self.declare_parameter('xy_covariance', 0.0004)
        self.declare_parameter('yaw_covariance', 0.001218)
        self.declare_parameter('dry_run', False)
        self.scan = None
        self.scans = []
        self.scan_stamps = set()
        self.last_scan_stamp_ns = None
        self.odom_first = None
        self.odom_latest = None
        self.stationary_detail = 'not evaluated'
        self.refinement_consistency = None
        self.refined_candidates = []
        self.distinct_score_gap = math.inf
        self.refinement_confidence_detail = 'not evaluated'
        self.refinement_confident = False
        self.capture_after_ns = 0
        self.match_field = None
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, 10)
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.client = self.create_client(SetInitialPose, '/set_initial_pose')

    def scan_callback(self, msg):
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        if stamp_ns < self.capture_after_ns:
            return
        if stamp in self.scan_stamps:
            return
        if self.last_scan_stamp_ns is not None:
            interval_ns = int(
                float(self.get_parameter('min_frame_interval_sec').value)
                * 1_000_000_000
            )
            if stamp_ns <= self.last_scan_stamp_ns or (
                stamp_ns - self.last_scan_stamp_ns < interval_ns
            ):
                return
        target_frames = max(1, int(self.get_parameter('target_frames').value))
        if len(self.scans) >= target_frames:
            return
        self.scan_stamps.add(stamp)
        self.scans.append(msg)
        self.scan = msg
        self.last_scan_stamp_ns = stamp_ns

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        sample = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec),
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
        )
        if self.odom_first is None:
            self.odom_first = sample
        self.odom_latest = sample

    def wait_for_scan(self, timeout_sec=12.0):
        """Wait for five uniquely stamped frames (legacy method name retained)."""
        deadline = time.monotonic() + timeout_sec
        target_frames = max(1, int(self.get_parameter('target_frames').value))
        while (
            rclpy.ok()
            and (len(self.scans) < target_frames or self.odom_latest is None)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.scans) >= target_frames and self.odom_latest is not None

    def verify_stationary_observation(self, previous_odom_stamp_ns):
        """Fail closed if the robot moved while scans were matched."""
        deadline = time.monotonic() + 1.0
        max_odom_age = float(self.get_parameter('max_odom_age_sec').value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.odom_latest is not None
                and self.odom_latest[0] > int(previous_odom_stamp_ns)
            ):
                candidate_age = (
                    int(self.get_clock().now().nanoseconds)
                    - self.odom_latest[0]
                ) / 1_000_000_000.0
                if -0.05 <= candidate_age <= max_odom_age:
                    break
        if self.odom_first is None or self.odom_latest is None:
            raise RuntimeError('no odometry available for stationary localization gate')
        if self.odom_latest[0] <= int(previous_odom_stamp_ns):
            raise RuntimeError('odometry did not update after localization computation')
        translation = math.hypot(
            self.odom_latest[1] - self.odom_first[1],
            self.odom_latest[2] - self.odom_first[2],
        )
        yaw_delta_deg = abs(math.degrees(wrap_angle(
            self.odom_latest[3] - self.odom_first[3]
        )))
        max_translation = float(
            self.get_parameter('max_stationary_translation_m').value
        )
        max_yaw = float(self.get_parameter('max_stationary_yaw_deg').value)
        if translation > max_translation or yaw_delta_deg > max_yaw:
            raise RuntimeError(
                'robot moved during localization: '
                f'odom delta={translation:.4f}m/{yaw_delta_deg:.3f}deg'
            )
        if self.last_scan_stamp_ns is None:
            raise RuntimeError('last scan timestamp is unavailable')
        observation_age = (
            int(self.get_clock().now().nanoseconds) - self.last_scan_stamp_ns
        ) / 1_000_000_000.0
        max_age = float(self.get_parameter('max_observation_age_sec').value)
        if observation_age < -0.05 or observation_age > max_age:
            raise RuntimeError(
                f'localization observation age {observation_age:.3f}s '
                f'is outside [-0.05,{max_age:.2f}]s'
            )
        odom_age = (
            int(self.get_clock().now().nanoseconds) - self.odom_latest[0]
        ) / 1_000_000_000.0
        if odom_age < -0.05 or odom_age > max_odom_age:
            raise RuntimeError(
                f'odometry age {odom_age:.3f}s '
                f'is outside [-0.05,{max_odom_age:.2f}]s'
            )
        self.stationary_detail = (
            f'odom delta={translation:.4f}m/{yaw_delta_deg:.3f}deg, '
            f'observation age={observation_age:.3f}s, '
            f'odom age={odom_age:.3f}s'
        )
        return {
            'translation_m': translation,
            'yaw_deg': yaw_delta_deg,
            'observation_age_sec': observation_age,
            'odom_age_sec': odom_age,
        }

    def scan_points(self, scan=None):
        scan = scan or self.scan
        if scan is None:
            raise ValueError('no scan frame is available')
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        upper = min(float(scan.range_max), 4.8) - 0.05
        mask = np.isfinite(ranges) & (ranges > 0.18) & (ranges < upper)
        indices = np.where(mask)[0]
        max_rays = int(self.get_parameter('max_rays').value)
        if len(indices) > max_rays:
            indices = indices[np.linspace(0, len(indices) - 1, max_rays).astype(int)]
        ranges = ranges[indices]
        angles = angles[indices]
        laser_x = float(self.get_parameter('laser_x').value)
        laser_y = float(self.get_parameter('laser_y').value)
        laser_yaw = float(self.get_parameter('laser_yaw').value)
        base_angles = angles + laser_yaw
        return np.stack([
            laser_x + ranges * np.cos(base_angles),
            laser_y + ranges * np.sin(base_angles),
        ], axis=1).astype(np.float32)

    def score_batch(self, positions, yaw, points, dist, origin_x, origin_y, resolution):
        return legacy_score_batch(
            positions,
            yaw,
            points,
            dist,
            origin_x,
            origin_y,
            resolution,
        )

    @staticmethod
    def distinct_seeds(candidates, limit=3):
        selected = []
        for candidate in candidates:
            if all(
                math.hypot(candidate[1] - other[1], candidate[2] - other[2]) >= 0.10
                or abs(wrap_angle(candidate[3] - other[3])) >= math.radians(3.0)
                for other in selected
            ):
                selected.append(candidate)
                if len(selected) >= int(limit):
                    break
        return selected or list(candidates[:1])

    def estimate_pose(self):
        deadline = time.monotonic() + float(self.get_parameter('calculation_timeout_sec').value)
        map_yaml = Path(str(self.get_parameter('map_yaml').value)).expanduser()
        image_path, resolution, origin = parse_map_yaml(map_yaml)
        image = read_pgm(image_path)
        dist = distance_to_occupied(image, resolution)
        frames = [self.scan_points(scan) for scan in self.scans]
        if len(frames) < max(1, int(self.get_parameter('target_frames').value)):
            raise ValueError(f'not enough unique scan frames: {len(frames)}')
        for index, points in enumerate(frames):
            min_valid_rays = int(self.get_parameter('min_valid_rays').value)
            if len(points) < min_valid_rays:
                raise ValueError(
                    f'not enough valid scan rays in frame {index}: '
                    f'{len(points)} < {min_valid_rays}'
                )
        # The centre frame makes the legacy global place-recognition pass less
        # sensitive to a startup transient than the very first lidar revolution.
        points = frames[len(frames) // 2]

        origin_x, origin_y = float(origin[0]), float(origin[1])
        prior = list(self.get_parameter_or('prior_pose').value or [])
        if prior:
            if len(prior) != 3:
                raise ValueError('prior_pose must be empty or [x, y, yaw]')
            prior += [float(self.get_parameter('local_search_radius_m').value),
                      float(self.get_parameter('local_search_yaw_deg').value)]
        fine = search_pose_seeds(image, dist, points, resolution, origin, deadline, prior)
        coarse_min_score = float(self.get_parameter('min_score').value)
        if not fine or not stage_score_passes(
            'coarse', fine[0][0], coarse_min_score,
            float(self.get_parameter('refined_min_score').value),
        ):
            score = fine[0][0] if fine else 0.0
            raise ValueError(
                f'coarse localization score too low: {score:.3f} < {coarse_min_score:.3f}'
            )

        seeds = self.distinct_seeds(fine, limit=3)
        check_calculation_deadline(deadline)
        fine_resolution = float(self.get_parameter('fine_resolution').value)
        subcell_dist, fine_resolution = build_subcell_distance_field(
            image,
            resolution,
            origin_x,
            origin_y,
            fine_resolution=fine_resolution,
        )
        refined = []
        for seed in seeds:
            check_calculation_deadline(deadline)
            refined.append(refine_seed(
                seed,
                frames,
                subcell_dist,
                origin_x,
                origin_y,
                fine_resolution,
            ))
        height, width = image.shape
        accepted = []
        for candidate in refined:
            col = math.floor((candidate[1] - origin_x) / resolution)
            row = height - 1 - math.floor((candidate[2] - origin_y) / resolution)
            if (0 <= row < height and 0 <= col < width
                    and image[row, col] > 230 and dist[row, col] > 0.12
                    and pose_within_prior(candidate, prior)):
                accepted.append(candidate)
        refined = accepted
        if not refined:
            raise ValueError('refinement left the permitted free search region')
        refined.sort(reverse=True, key=lambda item: item[0])
        self.refined_candidates = [tuple(item) for item in refined]
        pose = refined[0]
        self.distinct_score_gap = distinct_pose_score_gap(refined)
        self.match_field = (subcell_dist, origin_x, origin_y, fine_resolution)
        self.check_frame_confidence(pose, frames)
        check_calculation_deadline(deadline)
        return pose

    def check_frame_confidence(self, pose, frames):
        subcell_dist, origin_x, origin_y, fine_resolution = self.match_field
        consistency = frame_consistency(
            pose,
            frames,
            subcell_dist,
            origin_x,
            origin_y,
            fine_resolution,
        )
        min_score = float(self.get_parameter('refined_min_score').value)
        confident, confidence_detail = confidence_is_high(
            pose,
            consistency,
            min_score=min_score,
            min_frame_score=float(self.get_parameter('min_frame_score').value),
            max_translation_m=float(
                self.get_parameter('max_frame_translation_m').value
            ),
            max_yaw_deg=float(self.get_parameter('max_frame_yaw_deg').value),
            max_translation_span_m=float(
                self.get_parameter('max_frame_translation_span_m').value
            ),
            max_yaw_span_deg=float(
                self.get_parameter('max_frame_yaw_span_deg').value
            ),
        )
        min_distinct_gap = float(
            self.get_parameter('min_distinct_score_gap').value
        )
        if self.distinct_score_gap < min_distinct_gap:
            confident = False
            gap_detail = (
                f'distinct pose score gap {self.distinct_score_gap:.3f} '
                f'< {min_distinct_gap:.3f}'
            )
            confidence_detail = '; '.join(
                part for part in (confidence_detail, gap_detail) if part
            )
        self.refinement_consistency = consistency
        self.refinement_confident = confident
        self.refinement_confidence_detail = confidence_detail
        if not confident:
            raise ValueError(f'localization confidence rejected: {confidence_detail}')

    def refresh_observation(self, pose):
        """Revalidate with new scans without relabelling an old result as fresh."""
        self.capture_after_ns = self.get_clock().now().nanoseconds
        self.scans = []
        self.scan_stamps.clear()
        self.last_scan_stamp_ns = None
        if not self.wait_for_scan(timeout_sec=3.0):
            raise RuntimeError('fresh scan validation timed out')
        frames = [self.scan_points(scan) for scan in self.scans]
        if any(len(frame) < int(self.get_parameter('min_valid_rays').value) for frame in frames):
            raise ValueError('not enough valid rays in fresh validation scans')
        field, origin_x, origin_y, resolution = self.match_field
        fresh_candidates = []
        for candidate in self.refined_candidates:
            score = float(robust_score_batch(
                [[candidate[1], candidate[2]]], candidate[3], frames,
                field, origin_x, origin_y, resolution,
            )[0])
            fresh_candidates.append((score, *candidate[1:]))
        fresh_candidates.sort(reverse=True)
        winner = fresh_candidates[0]
        if (math.hypot(winner[1] - pose[1], winner[2] - pose[2]) >= 0.10
                or abs(wrap_angle(winner[3] - pose[3])) >= math.radians(3.0)):
            raise ValueError('fresh observations favour a different localization candidate')
        self.distinct_score_gap = distinct_pose_score_gap(fresh_candidates)
        self.refined_candidates = fresh_candidates
        score = float(robust_score_batch(
            [[pose[1], pose[2]]], pose[3], frames, field, origin_x, origin_y, resolution,
        )[0])
        pose = (score, *pose[1:])
        self.check_frame_confidence(pose, frames)
        return pose

    def publish_estimate(self, pose):
        score, x, y, yaw = pose
        if not self.refinement_confident:
            raise ValueError(
                'localization confidence was not established: '
                + self.refinement_confidence_detail
            )
        if not stage_score_passes(
            'refined', score,
            float(self.get_parameter('min_score').value),
            float(self.get_parameter('refined_min_score').value),
        ):
            raise ValueError(f'localization score too low: {score:.3f}')
        if not self.client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError('/set_initial_pose service unavailable')
        # The service may appear late during a cold start.  Re-check both robot
        # motion and observation freshness immediately before publishing so an
        # old scan can never cross the TF cache window unnoticed.
        if self.odom_latest is None:
            raise RuntimeError('odometry unavailable before initial pose request')
        self.verify_stationary_observation(self.odom_latest[0])
        xy_covariance = float(self.get_parameter('xy_covariance').value)
        yaw_covariance = float(self.get_parameter('yaw_covariance').value)
        request = SetInitialPose.Request()
        request.pose.header.frame_id = 'map'
        # Use the observation time rather than node-clock "now".  The latter is
        # normally 10--20 ms newer than odom TF and makes AMCL report future
        # extrapolation before falling back to an untransformed pose.
        last_stamp = self.scans[-1].header.stamp
        request.pose.header.stamp.sec = int(last_stamp.sec)
        request.pose.header.stamp.nanosec = int(last_stamp.nanosec)
        request.pose.pose.pose.position.x = x
        request.pose.pose.pose.position.y = y
        request.pose.pose.pose.orientation.z = math.sin(yaw * 0.5)
        request.pose.pose.pose.orientation.w = math.cos(yaw * 0.5)
        request.pose.pose.covariance[0] = xy_covariance
        request.pose.pose.covariance[7] = xy_covariance
        request.pose.pose.covariance[35] = yaw_covariance
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done():
            raise RuntimeError('/set_initial_pose timed out')
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        self.get_logger().info(
            f'auto localized on fixed map from {len(self.scans)} frames: '
            f'score={score:.3f}, x={x:.3f}, y={y:.3f}, '
            f'yaw={math.degrees(yaw):.2f}deg; '
            f'distinct gap={self.distinct_score_gap:.3f}; '
            f'{self.refinement_confidence_detail}; {self.stationary_detail}'
        )


def current_boot_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text(
            encoding='utf-8'
        ).strip()
    except Exception:
        return ''


def acquire_single_flight(
    node,
    started_monotonic,
    timeout_sec=2.0,
):
    """Serialize launch/web invocations and reuse an overlapping success.

    ``navigation.launch.py`` and the local GUI can both invoke this executable.
    A second overlapping process waits for the first and reuses only a same-map
    result completed after the second process began.  The result check is also
    made after an immediate lock acquisition, closing the race where the first
    process completes while rclpy constructs the second node.
    """
    lock_file = AUTO_LOCALIZE_LOCK.open('a+')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                time.sleep(0.10)
        else:
            lock_file.close()
            raise RuntimeError('timed out waiting for concurrent auto localization')

    reuse = None
    try:
        result = json.loads(AUTO_LOCALIZE_RESULT.read_text(encoding='utf-8'))
        requested_map = str(
            Path(str(node.get_parameter('map_yaml').value)).expanduser().resolve()
        )
        if (
            str(result.get('map_yaml') or '') == requested_map
            and str(result.get('boot_id') or '') == current_boot_id()
            and float(result.get('completed_monotonic', 0.0))
            >= float(started_monotonic)
        ):
            # Check even when this process acquired the lock immediately: a
            # peer may have completed while rclpy was constructing this node.
            reuse = result
    except Exception:
        reuse = None
    return lock_file, reuse


def record_single_flight_success(node, pose):
    score, x, y, yaw = pose
    result = {
        'map_yaml': str(
            Path(str(node.get_parameter('map_yaml').value)).expanduser().resolve()
        ),
        'completed_at': time.time(),
        'completed_monotonic': time.monotonic(),
        'boot_id': current_boot_id(),
        'score': float(score),
        'x': float(x),
        'y': float(y),
        'yaw': float(yaw),
        'frames': len(node.scans),
    }
    AUTO_LOCALIZE_RESULT.write_text(
        json.dumps(result, ensure_ascii=False, separators=(',', ':')),
        encoding='utf-8',
    )


def main(args=None):
    started_monotonic = time.monotonic()
    rclpy.init(args=args)
    node = AutoLocalize()
    exit_code = 0
    lock_file = None
    try:
        lock_file, reused = acquire_single_flight(node, started_monotonic)
        if reused is not None:
            node.get_logger().info(
                'reused concurrent auto localization result: '
                f'score={float(reused["score"]):.3f}, '
                f'x={float(reused["x"]):.3f}, y={float(reused["y"]):.3f}, '
                f'yaw={math.degrees(float(reused["yaw"])):.2f}deg'
            )
            return 0
        if not node.wait_for_scan():
            raise RuntimeError('timed out waiting for five unique scan frames')
        previous_odom_stamp_ns = node.odom_latest[0]
        pose = node.estimate_pose()
        pose = node.refresh_observation(pose)
        stationary = node.verify_stationary_observation(
            previous_odom_stamp_ns
        )
        if bool(node.get_parameter('dry_run').value):
            score, x, y, yaw = pose
            print(
                'ROSCAR_AUTO_LOCALIZE_DRY_RUN '
                + json.dumps(
                    {
                        'score': float(score),
                        'x': float(x),
                        'y': float(y),
                        'yaw': float(yaw),
                        'yaw_deg': math.degrees(float(yaw)),
                        'frames': len(node.scans),
                        'confidence': node.refinement_consistency,
                        'distinct_score_gap': node.distinct_score_gap,
                        'refined_candidates': [
                            {
                                'score': float(candidate[0]),
                                'x': float(candidate[1]),
                                'y': float(candidate[2]),
                                'yaw': float(candidate[3]),
                            }
                            for candidate in node.refined_candidates
                        ],
                        'stationary': stationary,
                    },
                    ensure_ascii=False,
                    separators=(',', ':'),
                )
            )
            return 0
        node.publish_estimate(pose)
        record_single_flight_success(node, pose)
    except Exception as exc:
        exit_code = 1
        node.get_logger().error(f'auto localization failed: {exc}')
    finally:
        if lock_file is not None:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            finally:
                lock_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
