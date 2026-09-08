"""ROS orientation conversions used by the N300Pro driver."""

import math
from typing import Sequence, Tuple


def quaternion_wxyz_to_rpy(
    quaternion_wxyz: Sequence[float],
) -> Tuple[float, float, float]:
    """Convert a normalized w,x,y,z quaternion to ROS roll,pitch,yaw radians."""
    w, x, y, z = quaternion_wxyz

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)

    return roll, pitch, yaw
