import math

import pytest

from roscar_base_interface.kinematics import (
    GroundTruthResetDetector,
    WheelOdometry,
    body_to_wheel_speeds,
    encoder_delta_from_speed,
    wheel_speeds_to_body,
)


@pytest.mark.parametrize(
    'body',
    [
        (0.2, 0.0, 0.0),
        (0.0, -0.15, 0.0),
        (0.0, 0.0, 0.2),
        (0.12, -0.07, 0.11),
    ],
)
def test_mecanum_forward_inverse_round_trip(body):
    wheels = body_to_wheel_speeds(*body, wheel_spacing=0.535, axle_spacing=0.410)
    recovered = wheel_speeds_to_body(wheels, wheel_spacing=0.535, axle_spacing=0.410)
    assert (recovered.vx, recovered.vy, recovered.wz) == pytest.approx(body)


def test_odometry_integrates_body_velocity_in_world_frame():
    odom = WheelOdometry(wheel_spacing=0.535, axle_spacing=0.410)
    wheels = body_to_wheel_speeds(0.1, 0.0, math.pi / 4, 0.535, 0.410)
    odom.update(wheels, 10.0)
    velocity = odom.update(wheels, 11.0)
    assert velocity.vx == pytest.approx(0.1)
    assert odom.pose.x == pytest.approx(0.1)
    assert odom.pose.y == pytest.approx(0.0)
    assert odom.pose.yaw == pytest.approx(math.pi / 4)


def test_encoder_delta_uses_rim_speed_radius_and_ticks():
    circumference = 2.0 * math.pi * 0.05
    assert encoder_delta_from_speed(circumference, 1.0, 0.05, 4096) == pytest.approx(4096)


def test_invalid_geometry_is_rejected():
    with pytest.raises(ValueError):
        wheel_speeds_to_body([0, 0, 0, 0], 0.0, 0.4)
    with pytest.raises(ValueError):
        wheel_speeds_to_body([0, 0, 0], 0.5, 0.4)


def test_ground_truth_only_realigns_first_pose_and_reset_jump():
    detector = GroundTruthResetDetector(
        position_jump_threshold=0.5,
        yaw_jump_threshold=0.75,
    )
    assert detector.observe(-3.0, -2.0, 0.0)
    assert not detector.observe(-2.998, -2.0, 0.001)
    assert not detector.observe(-2.990, -2.0, 0.002)
    assert detector.observe(1.0, 2.0, 1.0)
    assert not detector.observe(1.01, 2.0, 1.01)


def test_ground_truth_yaw_wrap_is_not_a_false_reset():
    detector = GroundTruthResetDetector(yaw_jump_threshold=0.2)
    assert detector.observe(0.0, 0.0, math.pi - 0.01)
    assert not detector.observe(0.0, 0.0, -math.pi + 0.01)
