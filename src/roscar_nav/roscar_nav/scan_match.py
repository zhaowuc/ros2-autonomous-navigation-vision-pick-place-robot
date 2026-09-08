"""Numerical helpers for robust, sub-cell laser-to-map matching.

This module intentionally has no ROS imports.  Keeping the numerical matcher
separate makes it possible to regression-test centimetre-scale localization
without starting a ROS graph or creating a pose publisher/service client.
"""

import math

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def distance_to_occupied(image, resolution, occupied_threshold=80):
    """Return a map-resolution distance field in metres.

    ``image`` follows map_server's PGM convention: darker pixels are occupied.
    This is the legacy field used by the global search, kept deliberately so
    the new local refinement cannot change global place recognition behaviour.
    """
    occupied = np.asarray(image) < occupied_threshold
    if not np.any(occupied):
        raise ValueError('map has no occupied cells')
    resolution = float(resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('map resolution must be finite and positive')
    return distance_transform_edt(~occupied, sampling=resolution).astype(np.float32)


def build_subcell_distance_field(
    image,
    resolution,
    origin_x,
    origin_y,
    fine_resolution=0.005,
    occupied_threshold=80,
):
    """Build a 5 mm likelihood field around occupied-cell centres.

    The saved map remains a 5 cm occupancy grid.  Interpolating a finer field
    does not invent walls; it removes the old floor-to-cell score plateau so
    several wall returns can jointly resolve a sub-cell pose.
    """
    occupied_rc = np.argwhere(image < occupied_threshold)
    if len(occupied_rc) == 0:
        raise ValueError('map has no occupied cells')
    resolution = float(resolution)
    fine_resolution = float(fine_resolution)
    if (
        not math.isfinite(resolution) or not math.isfinite(fine_resolution)
        or resolution <= 0.0 or fine_resolution <= 0.0
    ):
        raise ValueError('map and fine resolutions must be finite and positive')

    height, width = image.shape
    fine_width = int(math.ceil(width * resolution / fine_resolution)) + 1
    fine_height = int(math.ceil(height * resolution / fine_resolution)) + 1
    half_cell = resolution / (2.0 * fine_resolution)
    half_steps = int(round(half_cell))
    if half_steps > 0 and math.isclose(half_cell, half_steps, rel_tol=0.0, abs_tol=1e-9):
        # EDT is exact only when occupied centres lie on the fine grid.
        empty = np.ones((fine_height, fine_width), dtype=bool)
        xs = (2 * occupied_rc[:, 1] + 1) * half_steps
        ys = (2 * (height - occupied_rc[:, 0]) - 1) * half_steps
        empty[ys, xs] = False
        field = distance_transform_edt(empty, sampling=fine_resolution).astype(np.float32)
    else:
        # Work relative to the map origin; rounding centres onto this grid
        # would shift walls for odd or non-integral resolution ratios.
        centres = np.column_stack((
            (height - occupied_rc[:, 0] - 0.5) * resolution,
            (occupied_rc[:, 1] + 0.5) * resolution,
        ))
        tree = cKDTree(centres)
        field = np.empty((fine_height, fine_width), dtype=np.float32)
        for start in range(0, fine_height, 128):
            stop = min(start + 128, fine_height)
            grid = np.indices((stop - start, fine_width), dtype=np.float64)
            grid[0] += start
            distances, _ = tree.query(grid.reshape(2, -1).T * fine_resolution)
            field[start:stop] = distances.reshape(stop - start, fine_width)
    return field, fine_resolution


def bilinear_distances(
    field,
    world_x,
    world_y,
    origin_x,
    origin_y,
    resolution,
    outside_distance=0.8,
):
    """Sample a world-aligned distance field with bilinear interpolation."""
    gx = (np.asarray(world_x, dtype=np.float32) - float(origin_x)) / float(resolution)
    gy = (np.asarray(world_y, dtype=np.float32) - float(origin_y)) / float(resolution)
    ix = np.floor(gx).astype(np.int32)
    iy = np.floor(gy).astype(np.int32)
    valid = (
        (ix >= 0)
        & (iy >= 0)
        & (ix + 1 < field.shape[1])
        & (iy + 1 < field.shape[0])
    )
    distances = np.full(gx.shape, float(outside_distance), dtype=np.float32)
    if np.any(valid):
        vx = gx[valid] - ix[valid]
        vy = gy[valid] - iy[valid]
        d00 = field[iy[valid], ix[valid]]
        d10 = field[iy[valid], ix[valid] + 1]
        d01 = field[iy[valid] + 1, ix[valid]]
        d11 = field[iy[valid] + 1, ix[valid] + 1]
        distances[valid] = (
            d00 * (1.0 - vx) * (1.0 - vy)
            + d10 * vx * (1.0 - vy)
            + d01 * (1.0 - vx) * vy
            + d11 * vx * vy
        )
    return distances, valid


def legacy_score_batch(
    positions,
    yaw,
    points,
    distance_field,
    origin_x,
    origin_y,
    resolution,
    sigma=0.10,
):
    """The original global matcher score, retained for place recognition."""
    height, width = distance_field.shape
    positions = np.asarray(positions, dtype=np.float32)
    points = np.asarray(points, dtype=np.float32)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    point_x = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
    point_y = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
    end_x = positions[:, 0:1] + point_x[None, :]
    end_y = positions[:, 1:2] + point_y[None, :]
    map_x = np.floor((end_x - origin_x) / resolution).astype(np.int32)
    map_y = np.floor((end_y - origin_y) / resolution).astype(np.int32)
    row = height - 1 - map_y
    col = map_x
    valid = (row >= 0) & (row < height) & (col >= 0) & (col < width)
    distances = np.full(row.shape, 0.8, dtype=np.float32)
    distances[valid] = distance_field[row[valid], col[valid]]
    hits = np.exp(-((distances / float(sigma)) ** 2))
    return hits.mean(axis=1) - 0.6 * (~valid).mean(axis=1)


def robust_score_batch(
    positions,
    yaw,
    frames,
    distance_field,
    origin_x,
    origin_y,
    resolution,
    trim_fraction=0.80,
    sigma=0.050,
):
    """Score poses with equal-weighted frames and worst-return trimming.

    The worst 20 percent of returns in every frame are excluded from the
    likelihood mean, so a person or another object not present in the fixed map
    cannot pull a single scan's optimum away from the static walls.  Out-of-map
    rays are still penalised before frame scores are averaged.
    """
    positions = np.asarray(positions, dtype=np.float32)
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError('positions must have shape (N, 2)')
    if not frames:
        raise ValueError('at least one scan frame is required')
    if not 0.5 <= float(trim_fraction) <= 1.0:
        raise ValueError('trim_fraction must be in [0.5, 1.0]')
    scores = np.zeros(len(positions), dtype=np.float32)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for frame in frames:
        points = np.asarray(frame, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
            raise ValueError('each frame must have shape (N, 2) and be non-empty')
        point_x = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
        point_y = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
        end_x = positions[:, 0:1] + point_x[None, :]
        end_y = positions[:, 1:2] + point_y[None, :]
        distances, valid = bilinear_distances(
            distance_field,
            end_x,
            end_y,
            origin_x,
            origin_y,
            resolution,
        )
        keep = max(1, int(math.floor(points.shape[0] * float(trim_fraction))))
        trimmed = np.partition(distances, keep - 1, axis=1)[:, :keep]
        likelihood = np.exp(-0.5 * (trimmed / float(sigma)) ** 2).mean(axis=1)
        scores += likelihood - 0.6 * (~valid).mean(axis=1)
    return scores / float(len(frames))


def _inclusive_offsets(radius, step):
    radius = float(radius)
    step = float(step)
    count = int(round((2.0 * radius) / step))
    return np.linspace(-radius, radius, count + 1, dtype=np.float32)


def search_local_stage(
    center,
    frames,
    distance_field,
    origin_x,
    origin_y,
    resolution,
    xy_radius,
    xy_step,
    yaw_radius_deg,
    yaw_step_deg,
):
    """Run one deterministic local grid-search stage around ``center``."""
    center = tuple(float(value) for value in center)
    xy_offsets = _inclusive_offsets(xy_radius, xy_step)
    positions = np.array(
        [
            (center[0] + float(dx), center[1] + float(dy))
            for dx in xy_offsets
            for dy in xy_offsets
        ],
        dtype=np.float32,
    )
    yaw_offsets = _inclusive_offsets(
        math.radians(float(yaw_radius_deg)),
        math.radians(float(yaw_step_deg)),
    )
    best = None
    for yaw_offset in yaw_offsets:
        yaw = wrap_angle(center[2] + float(yaw_offset))
        scores = robust_score_batch(
            positions,
            yaw,
            frames,
            distance_field,
            origin_x,
            origin_y,
            resolution,
        )
        index = int(np.argmax(scores))
        candidate = (
            float(scores[index]),
            float(positions[index, 0]),
            float(positions[index, 1]),
            yaw,
        )
        if best is None or candidate[0] > best[0]:
            best = candidate
    return best


def refine_seed(
    seed,
    frames,
    distance_field,
    origin_x,
    origin_y,
    resolution,
):
    """Search the full seed neighbourhood, then refine at 5 mm and 2 mm."""
    center = (float(seed[1]), float(seed[2]), float(seed[3]))
    coarse = search_local_stage(
        center,
        frames,
        distance_field,
        origin_x,
        origin_y,
        resolution,
        xy_radius=0.120,
        xy_step=0.020,
        yaw_radius_deg=5.0,
        yaw_step_deg=0.5,
    )
    middle = search_local_stage(
        (coarse[1], coarse[2], coarse[3]),
        frames,
        distance_field,
        origin_x,
        origin_y,
        resolution,
        xy_radius=0.020,
        xy_step=0.005,
        yaw_radius_deg=0.6,
        yaw_step_deg=0.2,
    )
    return search_local_stage(
        (middle[1], middle[2], middle[3]),
        frames,
        distance_field,
        origin_x,
        origin_y,
        resolution,
        xy_radius=0.006,
        xy_step=0.002,
        yaw_radius_deg=0.2,
        yaw_step_deg=0.1,
    )


def frame_consistency(
    pose,
    frames,
    distance_field,
    origin_x,
    origin_y,
    resolution,
):
    """Independently refine each frame near the joint pose and report spread."""
    center = (float(pose[1]), float(pose[2]), float(pose[3]))
    frame_poses = []
    frame_scores = []
    for frame in frames:
        coarse = search_local_stage(
            center,
            [frame],
            distance_field,
            origin_x,
            origin_y,
            resolution,
            # Search materially beyond the acceptance window.  Keeping the
            # search radius equal to the gate would clip every bad frame at
            # the boundary and make the confidence test unable to fail.
            xy_radius=0.030,
            xy_step=0.006,
            yaw_radius_deg=1.5,
            yaw_step_deg=0.3,
        )
        best = search_local_stage(
            (coarse[1], coarse[2], coarse[3]),
            [frame],
            distance_field,
            origin_x,
            origin_y,
            resolution,
            xy_radius=0.006,
            xy_step=0.002,
            yaw_radius_deg=0.3,
            yaw_step_deg=0.1,
        )
        frame_scores.append(best[0])
        frame_poses.append((best[1], best[2], best[3]))
    array = np.asarray(frame_poses, dtype=np.float64)
    yaw_errors = np.asarray(
        [wrap_angle(value - center[2]) for value in array[:, 2]],
        dtype=np.float64,
    )
    translations = np.hypot(array[:, 0] - center[0], array[:, 1] - center[1])
    return {
        'frame_scores': [float(value) for value in frame_scores],
        'frame_poses': [tuple(float(value) for value in row) for row in array],
        'translation_max_m': float(np.max(translations)),
        'translation_span_m': float(math.hypot(np.ptp(array[:, 0]), np.ptp(array[:, 1]))),
        'yaw_max_abs_deg': float(math.degrees(np.max(np.abs(yaw_errors)))),
        'yaw_span_deg': float(math.degrees(np.ptp(yaw_errors))),
    }


def confidence_is_high(
    pose,
    consistency,
    min_score=0.90,
    min_frame_score=0.90,
    max_translation_m=0.010,
    max_yaw_deg=0.8,
    max_translation_span_m=0.015,
    max_yaw_span_deg=1.0,
):
    """Fail closed unless the joint result and every frame agree tightly."""
    reasons = []
    if float(pose[0]) < float(min_score):
        reasons.append(f'joint score {pose[0]:.3f} < {min_score:.3f}')
    frame_scores = list(consistency.get('frame_scores') or [])
    if not frame_scores or min(frame_scores) < float(min_frame_score):
        actual = min(frame_scores) if frame_scores else float('-inf')
        reasons.append(f'min frame score {actual:.3f} < {min_frame_score:.3f}')
    if float(consistency.get('translation_max_m', math.inf)) > float(max_translation_m):
        reasons.append(
            'frame translation disagreement '
            f'{consistency.get("translation_max_m", math.inf):.3f}m '
            f'> {max_translation_m:.3f}m'
        )
    if float(consistency.get('yaw_max_abs_deg', math.inf)) > float(max_yaw_deg):
        reasons.append(
            'frame yaw disagreement '
            f'{consistency.get("yaw_max_abs_deg", math.inf):.2f}deg '
            f'> {max_yaw_deg:.2f}deg'
        )
    if (
        float(consistency.get('translation_span_m', math.inf))
        > float(max_translation_span_m)
    ):
        reasons.append(
            'frame translation span '
            f'{consistency.get("translation_span_m", math.inf):.3f}m '
            f'> {max_translation_span_m:.3f}m'
        )
    if (
        float(consistency.get('yaw_span_deg', math.inf))
        > float(max_yaw_span_deg)
    ):
        reasons.append(
            'frame yaw span '
            f'{consistency.get("yaw_span_deg", math.inf):.2f}deg '
            f'> {max_yaw_span_deg:.2f}deg'
        )
    return not reasons, '; '.join(reasons) if reasons else 'multi-frame confidence passed'


def distinct_pose_score_gap(
    candidates,
    min_translation_m=0.10,
    min_yaw_deg=3.0,
):
    """Return the score gap to the best genuinely different pose basin.

    Several global seeds may converge to the same local optimum.  Those are
    not ambiguous; only a candidate separated in position or heading counts.
    ``math.inf`` means that no distinct alternative survived refinement.
    """
    candidates = list(candidates or [])
    if not candidates:
        return 0.0
    best = candidates[0]
    for candidate in candidates[1:]:
        translation = math.hypot(
            float(candidate[1]) - float(best[1]),
            float(candidate[2]) - float(best[2]),
        )
        yaw_delta = abs(math.degrees(wrap_angle(
            float(candidate[3]) - float(best[3])
        )))
        if (
            translation >= float(min_translation_m)
            or yaw_delta >= float(min_yaw_deg)
        ):
            return float(best[0]) - float(candidate[0])
    return math.inf
