import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class ArenaBoundary(Node):
    def __init__(self):
        super().__init__('arena_boundary')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('length', 3.875)
        self.declare_parameter('width', 2.140)
        self.declare_parameter('line_width', 0.04)
        self.declare_parameter('publish_rate', 1.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.length = float(self.get_parameter('length').value)
        self.width = float(self.get_parameter('width').value)
        self.line_width = float(self.get_parameter('line_width').value)
        publish_rate = max(0.2, float(self.get_parameter('publish_rate').value))

        self.publisher = self.create_publisher(Marker, '/arena_boundary', 1)
        self.create_timer(1.0 / publish_rate, self.publish_marker)
        self.get_logger().info(
            f'Publishing arena boundary {self.length:.2f}m x {self.width:.2f}m in {self.frame_id}'
        )

    def point(self, x, y):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.03
        return point

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'arena'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = 0.0
        marker.color.g = 0.78
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.points = [
            self.point(0.0, 0.0),
            self.point(self.length, 0.0),
            self.point(self.length, self.width),
            self.point(0.0, self.width),
            self.point(0.0, 0.0),
        ]
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaBoundary()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
