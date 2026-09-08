#!/usr/bin/env python3
"""Remove Gazebo self-returns while preserving the frozen laser TF."""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('roscar_sim_scan_filter')
        # These are read-only copies of the frozen base_footprint->laser_link
        # transform and body envelope. They transform endpoints for filtering;
        # this node publishes no TF and changes no URDF geometry.
        self.laser_x = -0.235
        self.laser_y = 0.0
        self.laser_yaw = 0.092382808
        self.publisher = self.create_publisher(
            LaserScan, '/scan_filtered', qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, '/scan_raw', self._filter, qos_profile_sensor_data
        )

    def _filter(self, message):
        output = copy.deepcopy(message)
        cosine = math.cos(self.laser_yaw)
        sine = math.sin(self.laser_yaw)
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance):
                continue
            angle = message.angle_min + index * message.angle_increment
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            base_x = self.laser_x + cosine * laser_x - sine * laser_y
            base_y = self.laser_y + sine * laser_x + cosine * laser_y
            inside_base = -0.290 <= base_x <= 0.290 and -0.285 <= base_y <= 0.285
            # The uncontrolled demo arm can sweep farther than the chassis,
            # but it only occludes a narrow central band from this rear-mounted
            # lidar. Obstacles wider than that band remain visible on both
            # sides to the Collision Monitor.
            inside_arm_occlusion = -0.600 <= base_x <= 0.600 and abs(base_y) <= 0.150
            if inside_base or inside_arm_occlusion:
                output.ranges[index] = math.inf
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
