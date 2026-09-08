import math

from geometry_msgs.msg import Twist

from roscar_command_arbiter.nav_command_limiter import limit_nav_twist


def test_nav_linear_commands_are_hard_clamped():
    message = Twist()
    message.linear.x = 0.200280
    message.linear.y = -0.30
    message.angular.z = 0.60
    output = limit_nav_twist(message)
    assert math.isclose(math.hypot(output.linear.x, output.linear.y), 0.20)
    assert math.isclose(
        output.linear.x / output.linear.y,
        message.linear.x / message.linear.y,
    )
    assert output.angular.z == 0.45


def test_non_finite_command_becomes_zero():
    message = Twist()
    message.linear.x = math.inf
    message.linear.y = math.nan
    output = limit_nav_twist(message)
    assert output.linear.x == 0.0
    assert output.linear.y == 0.0


def test_in_place_rotation_is_not_amplified_across_the_goal():
    message = Twist()
    message.angular.z = -0.026
    output = limit_nav_twist(message)
    assert output.angular.z == -0.026


def test_small_steering_correction_while_translating_is_not_boosted():
    message = Twist()
    message.linear.x = 0.05
    message.angular.z = 0.026
    output = limit_nav_twist(message)
    assert output.linear.x == 0.05
    assert output.angular.z == 0.026
