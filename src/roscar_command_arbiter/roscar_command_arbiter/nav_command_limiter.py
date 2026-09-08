"""Hard production boundary for Nav2 controller commands."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value, limit):
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return min(max(value, -float(limit)), float(limit))


def limit_nav_twist(
    message,
    linear_limit=0.20,
    angular_limit=0.45,
):
    output = Twist()
    values = (message.linear.x, message.linear.y, message.angular.z)
    if not all(math.isfinite(float(value)) for value in values):
        return output
    linear_x = float(message.linear.x)
    linear_y = float(message.linear.y)
    magnitude = math.hypot(linear_x, linear_y)
    scale = min(1.0, linear_limit / magnitude) if magnitude > 0.0 else 1.0
    output.linear.x = linear_x * scale
    output.linear.y = linear_y * scale
    output.angular.z = clamp(message.angular.z, angular_limit)
    return output


class NavCommandLimiter(Node):
    """Make `/cmd_vel_nav_raw` a bounded Nav output, not an optimizer leak."""

    def __init__(self):
        super().__init__('roscar_nav_command_limiter')
        self.declare_parameter('linear_limit', 0.20)
        self.declare_parameter('angular_limit', 0.45)
        self.linear_limit = min(
            0.20, max(0.0, float(self.get_parameter('linear_limit').value))
        )
        self.angular_limit = max(
            0.0, float(self.get_parameter('angular_limit').value)
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel_nav_raw', 20)
        self.create_subscription(
            Twist, '/cmd_vel_nav_controller', self._command, 20
        )

    def _command(self, message):
        self.publisher.publish(
            limit_nav_twist(
                message,
                self.linear_limit,
                self.angular_limit,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavCommandLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
