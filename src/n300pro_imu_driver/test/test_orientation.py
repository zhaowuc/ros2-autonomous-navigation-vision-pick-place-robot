import math

import pytest

from n300pro_imu_driver.orientation import quaternion_wxyz_to_rpy


def rpy_to_quaternion_wxyz(roll, pitch, yaw):
    """Create a normalized ROS quaternion for test inputs."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def test_identity_quaternion():
    assert quaternion_wxyz_to_rpy((1.0, 0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)


def test_ros_roll_and_pitch_are_not_swapped():
    expected = tuple(
        math.radians(value) for value in (0.0610, -1.1745, 1.8120)
    )
    quaternion = rpy_to_quaternion_wxyz(*expected)

    actual = quaternion_wxyz_to_rpy(quaternion)

    assert actual == pytest.approx(expected, abs=1.0e-12)


def test_pitch_input_is_clamped_for_roundoff():
    # This deliberately tiny norm error makes the raw asin input exceed one.
    roll, pitch, yaw = quaternion_wxyz_to_rpy(
        (math.sqrt(0.5) + 1.0e-15, 0.0, math.sqrt(0.5) + 1.0e-15, 0.0)
    )

    assert math.isfinite(roll)
    assert pitch == pytest.approx(math.pi / 2.0, abs=1.0e-12)
    assert math.isfinite(yaw)
