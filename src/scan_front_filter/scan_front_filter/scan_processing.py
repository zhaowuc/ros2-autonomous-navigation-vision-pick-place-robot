"""Pure 2-D scan processing helpers used by the ROS scan filter node.

The N10Plus driver publishes one revolution with a sweep-end timestamp and
without per-ray timing.  These helpers keep the geometry and temporal
filtering independent from rclpy so they can be unit tested off the robot.
"""

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math


TAU = 2.0 * math.pi


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_angle(first, second, ratio):
    return normalize_angle(first + normalize_angle(second - first) * ratio)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def compose_pose(parent, child):
    cosine = math.cos(parent.yaw)
    sine = math.sin(parent.yaw)
    return Pose2D(
        parent.x + cosine * child.x - sine * child.y,
        parent.y + sine * child.x + cosine * child.y,
        normalize_angle(parent.yaw + child.yaw),
    )


def transform_point(pose, x_value, y_value):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return (
        pose.x + cosine * x_value - sine * y_value,
        pose.y + sine * x_value + cosine * y_value,
    )


def inverse_transform_point(pose, x_value, y_value):
    delta_x = x_value - pose.x
    delta_y = y_value - pose.y
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


class OdomPoseBuffer:
    """Small timestamp-ordered odometry pose buffer with bounded extrapolation."""

    def __init__(self, history_sec=2.0):
        self.history_sec = max(0.5, float(history_sec))
        self._times = []
        self._poses = []

    def __len__(self):
        return len(self._times)

    def clear(self):
        self._times.clear()
        self._poses.clear()

    @property
    def first_stamp(self):
        return self._times[0] if self._times else None

    @property
    def last_stamp(self):
        return self._times[-1] if self._times else None

    def add(self, stamp, pose):
        stamp = float(stamp)
        if not math.isfinite(stamp):
            return
        if self._times and stamp < self._times[-1]:
            index = bisect_left(self._times, stamp)
            if index < len(self._times) and abs(self._times[index] - stamp) < 1e-9:
                self._poses[index] = pose
            else:
                self._times.insert(index, stamp)
                self._poses.insert(index, pose)
        elif self._times and abs(stamp - self._times[-1]) < 1e-9:
            self._poses[-1] = pose
        else:
            self._times.append(stamp)
            self._poses.append(pose)

        cutoff = self._times[-1] - self.history_sec
        first_kept = bisect_left(self._times, cutoff)
        if first_kept > 0:
            del self._times[:first_kept]
            del self._poses[:first_kept]

    def lookup(self, stamp, max_gap=0.06):
        if not self._times:
            return None
        stamp = float(stamp)
        max_gap = max(0.0, float(max_gap))
        index = bisect_left(self._times, stamp)

        if index == 0:
            if self._times[0] - stamp <= max_gap:
                return self._poses[0]
            return None

        if index == len(self._times):
            gap = stamp - self._times[-1]
            if gap > max_gap:
                return None
            if len(self._times) < 2 or self._times[-1] <= self._times[-2]:
                return self._poses[-1]
            duration = self._times[-1] - self._times[-2]
            ratio = gap / duration
            previous = self._poses[-2]
            latest = self._poses[-1]
            return Pose2D(
                latest.x + (latest.x - previous.x) * ratio,
                latest.y + (latest.y - previous.y) * ratio,
                interpolate_angle(
                    latest.yaw,
                    latest.yaw + normalize_angle(latest.yaw - previous.yaw),
                    ratio,
                ),
            )

        first_time = self._times[index - 1]
        second_time = self._times[index]
        if stamp - first_time > max_gap or second_time - stamp > max_gap:
            return None
        duration = second_time - first_time
        if duration <= 1e-9:
            return self._poses[index]
        ratio = (stamp - first_time) / duration
        first = self._poses[index - 1]
        second = self._poses[index]
        return Pose2D(
            first.x + (second.x - first.x) * ratio,
            first.y + (second.y - first.y) * ratio,
            interpolate_angle(first.yaw, second.yaw, ratio),
        )


def acquisition_phase_for_n10plus(angle):
    """Return [0, 1) revolution phase for azimuth increasing 0 -> 2*pi.

    The driver uses x=range*cos(azimuth), y=-range*sin(azimuth), therefore
    LaserScan angle is -azimuth.  A scan is cut at the 0-degree wrap.
    """

    return ((-float(angle)) % TAU) / TAU


def angle_to_index(angle, angle_min, angle_increment, count, full_scan=True):
    if count <= 0 or angle_increment <= 0.0:
        return None
    angle = float(angle)
    if full_scan:
        while angle < angle_min:
            angle += TAU
        upper_sample = angle_min + (count - 1) * angle_increment
        while angle > upper_sample + 0.5 * angle_increment:
            angle -= TAU
    index = int(round((angle - angle_min) / angle_increment))
    if full_scan:
        index %= count
    if 0 <= index < count:
        return index
    return None


def fixed_scan_grid(ranges, intensities, angle_min, angle_increment, count):
    """Place measured returns on a fixed circle; unobserved bins stay empty."""
    if count <= 0 or not math.isfinite(angle_increment) or angle_increment <= 0:
        raise ValueError('Invalid scan grid')
    step = TAU / count
    output = [math.inf] * count
    levels = [0.0] * count if intensities else []
    for i, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0:
            continue
        index = angle_to_index(angle_min + i * angle_increment, -math.pi, step, count)
        if value < output[index]:
            output[index] = value
            if levels and i < len(intensities):
                levels[index] = intensities[i]
    return output, levels


def deskew_ranges(
    ranges,
    angle_min,
    angle_increment,
    sweep_end_stamp,
    scan_period,
    pose_buffer,
    laser_in_base,
    range_min,
    range_max,
    max_odom_gap=0.06,
    full_scan=True,
):
    """Reproject all finite beams into the laser frame at sweep end."""

    count = len(ranges)
    identity_sources = list(range(count))
    reference_base = pose_buffer.lookup(sweep_end_stamp, max_odom_gap)
    start_base = pose_buffer.lookup(sweep_end_stamp - scan_period, max_odom_gap)
    if reference_base is None or start_base is None:
        reference_laser = (
            compose_pose(reference_base, laser_in_base)
            if reference_base is not None
            else None
        )
        return list(ranges), identity_sources, False, reference_laser, 0

    reference_laser = compose_pose(reference_base, laser_in_base)
    output = [math.inf] * count
    sources = [-1] * count
    lookup_failures = 0

    for source_index, range_value in enumerate(ranges):
        if not math.isfinite(range_value):
            continue
        source_angle = angle_min + source_index * angle_increment
        phase = acquisition_phase_for_n10plus(source_angle)
        beam_stamp = sweep_end_stamp - scan_period * (1.0 - phase)
        beam_base = pose_buffer.lookup(beam_stamp, max_odom_gap)
        if beam_base is None:
            beam_base = reference_base
            lookup_failures += 1
        beam_laser = compose_pose(beam_base, laser_in_base)
        local_x = range_value * math.cos(source_angle)
        local_y = range_value * math.sin(source_angle)
        world_x, world_y = transform_point(beam_laser, local_x, local_y)
        reference_x, reference_y = inverse_transform_point(
            reference_laser,
            world_x,
            world_y,
        )
        corrected_range = math.hypot(reference_x, reference_y)
        if corrected_range < range_min or corrected_range > range_max:
            continue
        corrected_angle = math.atan2(reference_y, reference_x)
        output_index = angle_to_index(
            corrected_angle,
            angle_min,
            angle_increment,
            count,
            full_scan,
        )
        if output_index is None:
            continue
        if corrected_range < output[output_index]:
            output[output_index] = corrected_range
            sources[output_index] = source_index

    return output, sources, True, reference_laser, lookup_failures


def select_multi_echo_ranges(
    range_echoes,
    range_min,
    range_max,
    spatial_window=2,
    spatial_tolerance=0.22,
    min_secondary_neighbors=2,
    full_scan=True,
):
    """Select conservative near/first echoes from physical N10Plus beams.

    A valid primary echo is accepted immediately.  A secondary-only echo must
    be supported by nearby physical beams; later odom-frame history filtering
    still guards a spatially coherent far wall behind an intermittently seen
    nearer wall.
    """

    primary = []
    secondary = []
    all_candidates = []
    selected_echo_indices = []
    primary_count = 0
    secondary_only_count = 0

    for echoes in range_echoes:
        cleaned = [
            float(value)
            for value in echoes
            if math.isfinite(value) and range_min <= value <= range_max
        ]
        primary_value = math.inf
        if echoes:
            first = float(echoes[0])
            if math.isfinite(first) and range_min <= first <= range_max:
                primary_value = first
        later_values = []
        for value in echoes[1:]:
            value = float(value)
            if math.isfinite(value) and range_min <= value <= range_max:
                later_values.append(value)
        secondary_value = min(later_values) if later_values else math.inf
        primary.append(primary_value)
        secondary.append(secondary_value)
        if math.isfinite(primary_value):
            all_candidates.append(primary_value)
        else:
            all_candidates.append(secondary_value)

    output = [math.inf] * len(range_echoes)
    rejected_secondary = 0
    for index in range(len(range_echoes)):
        if math.isfinite(primary[index]):
            # Echo 0 is the driver's first return.  Keep it when valid;
            # later returns are allowed only as a structurally supported
            # fallback when the first return is invalid.
            output[index] = primary[index]
            selected_echo_indices.append(0)
            primary_count += 1
            continue
        if not math.isfinite(secondary[index]):
            selected_echo_indices.append(-1)
            continue

        secondary_only_count += 1
        neighbors = 0
        for offset in range(-spatial_window, spatial_window + 1):
            if offset == 0:
                continue
            neighbor_index = index + offset
            if full_scan:
                neighbor_index %= len(range_echoes)
            elif neighbor_index < 0 or neighbor_index >= len(range_echoes):
                continue
            neighbor = all_candidates[neighbor_index]
            if (
                math.isfinite(neighbor)
                and abs(neighbor - secondary[index]) <= spatial_tolerance
            ):
                neighbors += 1
        if neighbors >= min_secondary_neighbors:
            output[index] = secondary[index]
            selected_echo_indices.append(1)
        else:
            selected_echo_indices.append(-1)
            rejected_secondary += 1

    stats = {
        'primary': primary_count,
        'secondary_only': secondary_only_count,
        'secondary_rejected': rejected_secondary,
    }
    return output, selected_echo_indices, stats


def _confirmed_cluster(values, required, tolerance):
    if len(values) < required:
        return False
    values = sorted(values)
    first = 0
    for last in range(len(values)):
        while values[last] - values[first] > tolerance:
            first += 1
        if last - first + 1 >= required:
            return True
    return False


class OdomFarReturnGuard:
    """Reject transient farther rays behind recently confirmed endpoints."""

    def __init__(
        self,
        history_scans=50,
        history_max_age_sec=5.0,
        angular_window_rad=math.radians(1.5),
        range_tolerance=0.20,
        far_jump_delta=0.25,
        min_confirmations=2,
    ):
        self.history_scans = max(0, int(history_scans))
        self.history_max_age_sec = max(0.0, float(history_max_age_sec))
        self.angular_window_rad = max(0.0, float(angular_window_rad))
        self.range_tolerance = max(0.0, float(range_tolerance))
        self.far_jump_delta = max(0.0, float(far_jump_delta))
        self.min_confirmations = max(1, int(min_confirmations))
        self._history = deque(maxlen=max(1, self.history_scans))
        self._last_stamp = None

    def clear(self):
        self._history.clear()
        self._last_stamp = None

    def _prepare_stamp(self, stamp):
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            self.clear()
            return None
        if not math.isfinite(stamp):
            self.clear()
            return None
        if (
            self._last_stamp is not None
            and stamp < self._last_stamp - 1.0e-9
        ):
            # A ROS clock/recording epoch rollback invalidates every endpoint
            # stored in the old odom-time epoch.
            self.clear()
        self._last_stamp = stamp
        return stamp

    def _prune(self, stamp):
        while (
            self._history
            and stamp - self._history[0][0] > self.history_max_age_sec
        ):
            self._history.popleft()

    def filter(
        self,
        ranges,
        angle_min,
        angle_increment,
        stamp,
        reference_laser,
        range_min,
        range_max,
        full_scan=True,
    ):
        stamp = self._prepare_stamp(stamp)
        if stamp is None:
            return list(ranges), 0
        if self.history_scans <= 0 or reference_laser is None:
            return list(ranges), 0
        self._prune(stamp)
        if len(self._history) < self.min_confirmations:
            return list(ranges), 0

        count = len(ranges)
        window_bins = max(0, int(math.ceil(
            self.angular_window_rad / max(angle_increment, 1e-9)
        )))
        projected_histories = []
        for _history_stamp, world_points in self._history:
            projected = {}
            for world_x, world_y in world_points:
                local_x, local_y = inverse_transform_point(
                    reference_laser,
                    world_x,
                    world_y,
                )
                projected_range = math.hypot(local_x, local_y)
                if projected_range < range_min or projected_range > range_max:
                    continue
                projected_angle = math.atan2(local_y, local_x)
                index = angle_to_index(
                    projected_angle,
                    angle_min,
                    angle_increment,
                    count,
                    full_scan,
                )
                if index is None:
                    continue
                previous = projected.get(index, math.inf)
                if projected_range < previous:
                    projected[index] = projected_range
            projected_histories.append(projected)

        output = list(ranges)
        rejected = 0
        for index, current_range in enumerate(ranges):
            if not math.isfinite(current_range):
                continue
            nearer_ranges = []
            for projected in projected_histories:
                # Prefer the exact projected beam.  Only fall back to the
                # nearest angular neighbor when that beam is absent.  Taking
                # the minimum over the full window falsely treated a stable
                # far wall beside a close wall edge as a through-wall jump.
                nearest = projected.get(index, math.inf)
                if not math.isfinite(nearest):
                    for radius in range(1, window_bins + 1):
                        candidates = []
                        for offset in (-radius, radius):
                            candidate_index = index + offset
                            if full_scan:
                                candidate_index %= count
                            elif (
                                candidate_index < 0
                                or candidate_index >= count
                            ):
                                continue
                            candidate = projected.get(
                                candidate_index,
                                math.inf,
                            )
                            if math.isfinite(candidate):
                                candidates.append(candidate)
                        if candidates:
                            nearest = min(candidates)
                            break
                if nearest + self.far_jump_delta < current_range:
                    nearer_ranges.append(nearest)
            if _confirmed_cluster(
                nearer_ranges,
                self.min_confirmations,
                self.range_tolerance,
            ):
                output[index] = math.inf
                rejected += 1
        return output, rejected

    def add_scan(
        self,
        ranges,
        angle_min,
        angle_increment,
        stamp,
        reference_laser,
    ):
        stamp = self._prepare_stamp(stamp)
        if stamp is None:
            return
        if self.history_scans <= 0 or reference_laser is None:
            return
        self._prune(stamp)
        world_points = []
        for index, range_value in enumerate(ranges):
            if not math.isfinite(range_value):
                continue
            angle = angle_min + index * angle_increment
            local_x = range_value * math.cos(angle)
            local_y = range_value * math.sin(angle)
            world_points.append(transform_point(
                reference_laser,
                local_x,
                local_y,
            ))
        entry = (stamp, tuple(world_points))
        if (
            self._history
            and abs(self._history[-1][0] - stamp) <= 1.0e-9
        ):
            # A duplicated source frame must not count as another independent
            # confirmation of a wall endpoint.
            self._history[-1] = entry
        else:
            self._history.append(entry)
