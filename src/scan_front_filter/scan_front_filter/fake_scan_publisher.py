import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('frame_id', 'laser_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('samples', 721)
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.samples = max(3, int(self.get_parameter('samples').value))
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.publisher = self.create_publisher(LaserScan, self.topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_scan)
        self.get_logger().info(
            f'Publishing fake 360 degree LaserScan on {self.topic}, frame={self.frame_id}'
        )

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (self.samples - 1)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 25.0
        ranges = []
        intensities = []
        for index in range(self.samples):
            angle = msg.angle_min + index * msg.angle_increment
            ranges.append(2.0 + 0.25 * math.cos(2.0 * angle))
            intensities.append(100.0)
        msg.ranges = ranges
        msg.intensities = intensities
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
