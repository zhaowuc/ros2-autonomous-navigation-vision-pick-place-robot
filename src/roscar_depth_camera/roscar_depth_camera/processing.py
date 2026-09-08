"""Pure image, target, and point-cloud processing helpers."""

import math

import cv2
import numpy as np


FLOAT32_DATATYPE = 7


def detect_red_target(bgr_image, min_area_ratio=0.02):
    """Return the dominant red rectangle despite global brightness shifts."""
    image = np.asarray(bgr_image, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError('bgr_image must be a non-empty HxWx3 uint8 image')

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    mean_value = float(np.mean(value))
    clipped_ratio = float(np.mean(value >= 250))
    if float(np.percentile(value, 95)) < 30.0 or clipped_ratio > 0.65:
        return None

    # Hue and saturation carry the target colour.  Value is deliberately
    # adaptive so a room-wide light change does not move the segmentation.
    value_floor = max(25, int(np.percentile(value, 20) * 0.55))
    red_hue = (hsv[:, :, 0] <= 18) | (hsv[:, :, 0] >= 165)
    mask = (
        red_hue
        & (hsv[:, :, 1] >= 45)
        & (value >= value_floor)
    ).astype(np.uint8) * 255
    kernel = np.ones((7, 7), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None

    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    height, width = image.shape[:2]
    frame_area = float(height * width)
    contour = contours[0]
    area = float(cv2.contourArea(contour))
    if area / frame_area < float(min_area_ratio):
        return None
    if len(contours) > 1 and cv2.contourArea(contours[1]) > area * 0.35:
        return None

    x, y, box_width, box_height = cv2.boundingRect(contour)
    rectangularity = area / float(max(1, box_width * box_height))
    hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
    solidity = area / max(1.0, hull_area)
    moments = cv2.moments(contour)
    if rectangularity < 0.55 or solidity < 0.85 or moments['m00'] <= 0.0:
        return None
    return {
        'center_x': float(moments['m10'] / moments['m00']) / width,
        'center_y': float(moments['m01'] / moments['m00']) / height,
        'area_ratio': area / frame_area,
        'bbox': (int(x), int(y), int(box_width), int(box_height)),
        'contour': contour,
        'mean_brightness': mean_value,
    }


def visual_alignment_velocity(
    center_x,
    center_y,
    reference_center_x,
    reference_center_y,
    center_tolerance=0.025,
    vertical_tolerance=0.035,
    min_forward_speed=0.040,
    max_forward_speed=0.050,
    min_lateral_speed=0.050,
    max_lateral_speed=0.060,
):
    """Return bounded base-frame vx/vy and whether the reference is matched."""
    x_error = float(center_x) - float(reference_center_x)
    y_error = float(center_y) - float(reference_center_y)
    aligned = (
        abs(x_error) <= float(center_tolerance)
        and abs(y_error) <= float(vertical_tolerance)
    )
    if aligned:
        return 0.0, 0.0, True
    vx = 0.0
    if abs(y_error) > float(vertical_tolerance):
        raw_vx = -y_error * 0.22
        vx = math.copysign(
            min(float(max_forward_speed), max(float(min_forward_speed), abs(raw_vx))),
            raw_vx,
        )
    vy = 0.0
    if abs(x_error) > float(center_tolerance):
        raw_vy = -x_error * 0.20
        vy = math.copysign(
            min(float(max_lateral_speed), max(float(min_lateral_speed), abs(raw_vy))),
            raw_vy,
        )
    return vx, vy, False


def stepped_alignment_velocity(
    now,
    desired,
    previous,
    move_until,
    settle_until,
    move_seconds=0.30,
    settle_seconds=0.45,
):
    """Pulse the drivetrain, then stop long enough for a stable camera frame."""
    current_time = float(now)
    if current_time < float(move_until):
        if any(old != 0.0 and old * new <= 0.0 for old, new in zip(previous, desired)):
            return (0.0, 0.0), 0.0, current_time + float(settle_seconds)
        return tuple(previous), float(move_until), float(settle_until)
    if current_time < float(settle_until):
        return (0.0, 0.0), 0.0, float(settle_until)
    end = current_time + float(move_seconds)
    return tuple(desired), end, end + float(settle_seconds)


def depth_to_meters(depth, encoding):
    """Convert a REP-118 depth image to float32 metres."""
    array = np.asarray(depth)
    normalized_encoding = str(encoding or '').upper()
    if normalized_encoding in ('16UC1', 'MONO16') or array.dtype == np.uint16:
        return array.astype(np.float32) * np.float32(0.001)
    if normalized_encoding == '32FC1' or array.dtype == np.float32:
        return array.astype(np.float32, copy=False)
    raise ValueError(
        f'unsupported depth encoding {encoding!r} with dtype {array.dtype}'
    )


def depth_to_palette_indices(depth_m, near_m, far_m):
    """Map valid metric depths to all 256 uint8 LUT indices.

    Near pixels map to 255 (warm end of Turbo), far pixels map to 0
    (cool end). Invalid pixels are returned separately so index 0 remains
    available for a valid far-range sample.
    """
    near = float(near_m)
    far = float(far_m)
    if not np.isfinite(near) or not np.isfinite(far) or far <= near:
        raise ValueError('far_m must be finite and greater than near_m')

    depth = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0)
    safe_depth = np.where(valid, depth, far)
    clipped = np.clip(safe_depth, near, far)
    normalized = (clipped - near) / (far - near)
    indices = np.rint((1.0 - normalized) * 255.0).astype(np.uint8)
    indices[~valid] = 0
    return indices, valid


def xyz_from_pointcloud2(message, sample_stride=1):
    """Extract little/big-endian XYZ float32 fields from PointCloud2."""
    fields = {field.name: field for field in message.fields}
    missing = {'x', 'y', 'z'} - fields.keys()
    if missing:
        raise ValueError(f'PointCloud2 is missing fields: {sorted(missing)}')
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype != FLOAT32_DATATYPE or field.count != 1:
            raise ValueError(f'PointCloud2 field {name} must be FLOAT32 count=1')

    point_step = int(message.point_step)
    if point_step <= 0:
        raise ValueError('PointCloud2 point_step must be positive')
    count = int(message.width) * int(message.height)
    available = len(message.data) // point_step
    count = min(count, available)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float32)

    endian = '>' if bool(message.is_bigendian) else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
        'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
        'itemsize': point_step,
    })
    records = np.frombuffer(memoryview(message.data), dtype=dtype, count=count)
    stride = max(1, int(sample_stride))
    records = records[::stride]
    points = np.column_stack((records['x'], records['y'], records['z']))
    return points.astype(np.float32, copy=False)


def filter_and_voxelize(points, min_range_m, max_range_m, voxel_size_m):
    """Remove invalid/range-excluded samples and retain one point per voxel."""
    xyz = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    finite = np.isfinite(xyz).all(axis=1)
    range_squared = np.einsum('ij,ij->i', xyz, xyz)
    min_squared = float(min_range_m) ** 2
    max_squared = float(max_range_m) ** 2
    mask = finite & (range_squared >= min_squared) & (range_squared <= max_squared)
    xyz = xyz[mask]
    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    voxel_size = float(voxel_size_m)
    if voxel_size <= 0.0:
        return np.ascontiguousarray(xyz, dtype=np.float32)
    voxel_keys = np.floor(xyz / voxel_size).astype(np.int32)
    _, first_indices = np.unique(voxel_keys, axis=0, return_index=True)
    first_indices.sort()
    return np.ascontiguousarray(xyz[first_indices], dtype=np.float32)
