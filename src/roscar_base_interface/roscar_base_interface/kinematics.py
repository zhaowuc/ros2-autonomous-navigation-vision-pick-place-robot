"""Four-wheel mecanum inverse kinematics and raw wheel odometry."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class BodyVelocity:
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


def _lever_arm(wheel_spacing: float, axle_spacing: float) -> float:
    """Return half-track plus half-wheelbase for the mecanum equations."""

    spacing = float(wheel_spacing)
    axle = float(axle_spacing)
    if spacing <= 0.0 or axle <= 0.0:
        raise ValueError('wheel_spacing and axle_spacing must both be positive')
    return 0.5 * (spacing + axle)


def body_to_wheel_speeds(
    vx: float,
    vy: float,
    wz: float,
    wheel_spacing: float,
    axle_spacing: float,
) -> tuple[float, float, float, float]:
    """Map body velocity to wheel rim speeds in each wheel's forward direction.

    Wheel order is front-left, front-right, rear-left, rear-right.  The formula
    assumes an X roller arrangement and ROS body axes (+x forward, +y left,
    +z counter-clockwise).  The wire protocol carries rim speed in m/s, not
    motor angular speed, so wheel_radius does not appear here.
    """

    k = _lever_arm(wheel_spacing, axle_spacing)
    return (
        float(vx) - float(vy) - k * float(wz),
        float(vx) + float(vy) + k * float(wz),
        float(vx) + float(vy) - k * float(wz),
        float(vx) - float(vy) + k * float(wz),
    )


def wheel_speeds_to_body(
    wheel_speeds: Sequence[float],
    wheel_spacing: float,
    axle_spacing: float,
) -> BodyVelocity:
    """Convert four normalized wheel rim speeds to a body velocity."""

    if len(wheel_speeds) != 4:
        raise ValueError('exactly four wheel speeds are required')
    fl, fr, rl, rr = (float(value) for value in wheel_speeds)
    k = _lever_arm(wheel_spacing, axle_spacing)
    return BodyVelocity(
        vx=(fl + fr + rl + rr) * 0.25,
        vy=(-fl + fr + rl - rr) * 0.25,
        wz=(-fl + fr - rl + rr) / (4.0 * k),
    )


class WheelOdometry:
    """Integrate feedback-derived body velocity without publishing TF."""

    def __init__(self, wheel_spacing: float, axle_spacing: float) -> None:
        _lever_arm(wheel_spacing, axle_spacing)
        self.wheel_spacing = float(wheel_spacing)
        self.axle_spacing = float(axle_spacing)
        self.pose = PlanarPose(0.0, 0.0, 0.0)
        self.last_time: float | None = None

    def reset(self, *, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.pose = PlanarPose(float(x), float(y), _normalize_angle(yaw))
        self.last_time = None

    def update(self, wheel_speeds: Sequence[float], stamp_seconds: float) -> BodyVelocity:
        velocity = wheel_speeds_to_body(
            wheel_speeds,
            self.wheel_spacing,
            self.axle_spacing,
        )
        stamp = float(stamp_seconds)
        if self.last_time is not None:
            dt = stamp - self.last_time
            if 0.0 < dt <= 1.0:
                yaw = self.pose.yaw
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                self.pose = PlanarPose(
                    x=self.pose.x + (velocity.vx * cos_yaw - velocity.vy * sin_yaw) * dt,
                    y=self.pose.y + (velocity.vx * sin_yaw + velocity.vy * cos_yaw) * dt,
                    yaw=_normalize_angle(yaw + velocity.wz * dt),
                )
        self.last_time = stamp
        return velocity


class GroundTruthResetDetector:
    """Detect the first simulator pose and discontinuous reset jumps.

    Continuous ground truth is not copied into wheel odometry.  It is used only
    to establish the initial odometry origin and to recognize an explicit
    simulator reset; all intermediate motion remains integrated from the four
    wheel speeds.
    """

    def __init__(
        self,
        *,
        position_jump_threshold: float = 0.50,
        yaw_jump_threshold: float = 0.75,
    ) -> None:
        self.position_jump_threshold = float(position_jump_threshold)
        self.yaw_jump_threshold = float(yaw_jump_threshold)
        if self.position_jump_threshold <= 0.0 or self.yaw_jump_threshold <= 0.0:
            raise ValueError('ground-truth jump thresholds must be positive')
        self.last_pose: PlanarPose | None = None

    def observe(self, x: float, y: float, yaw: float) -> bool:
        current = PlanarPose(float(x), float(y), _normalize_angle(yaw))
        previous = self.last_pose
        self.last_pose = current
        if previous is None:
            return True
        position_jump = math.hypot(current.x - previous.x, current.y - previous.y)
        yaw_jump = abs(_normalize_angle(current.yaw - previous.yaw))
        return (
            position_jump > self.position_jump_threshold
            or yaw_jump > self.yaw_jump_threshold
        )


def encoder_delta_from_speed(
    speed_m_s: float,
    dt: float,
    wheel_radius: float,
    encoder_ticks_per_revolution: int,
) -> float:
    radius = float(wheel_radius)
    ticks = int(encoder_ticks_per_revolution)
    if radius <= 0.0 or ticks <= 0:
        raise ValueError('wheel_radius and encoder_ticks_per_revolution must be positive')
    revolutions = float(speed_m_s) * float(dt) / (2.0 * math.pi * radius)
    return revolutions * ticks


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))
