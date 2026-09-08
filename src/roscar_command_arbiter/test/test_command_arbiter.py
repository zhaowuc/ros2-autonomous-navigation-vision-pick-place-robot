import math

from geometry_msgs.msg import Twist

from roscar_command_arbiter.command_arbiter import SPEED_PROFILES, sanitize_twist


def test_speed_contract_is_frozen():
    assert SPEED_PROFILES == {'low': 0.06, 'normal': 0.10, 'fast': 0.20}


def test_clamps_linear_and_angular_values():
    message = Twist()
    message.linear.x = 2.0
    message.linear.y = -1.0
    message.angular.z = 3.0
    output = sanitize_twist(message, 0.20)
    assert math.isclose(math.hypot(output.linear.x, output.linear.y), 0.20)
    assert math.isclose(output.linear.x / output.linear.y, -2.0)
    assert output.angular.z == 0.45


def test_non_finite_input_fails_closed():
    message = Twist()
    message.linear.x = math.nan
    output = sanitize_twist(message, 0.20)
    assert output.linear.x == 0.0
    assert output.angular.z == 0.0
