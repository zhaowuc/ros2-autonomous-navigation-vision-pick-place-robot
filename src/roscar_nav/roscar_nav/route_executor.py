import argparse
import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    half = float(yaw) * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


def load_points(path):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or []
    points = {}
    for item in data:
        name = str(item.get('name', '')).strip()
        if name and name not in points:
            points[name] = item
    return points


class RouteExecutor(Node):
    def __init__(self, route_config):
        super().__init__('roscar_route_executor')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.route_config = route_config

    def make_pose(self, point, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = str(point.get('frame_id') or frame_id)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        pose.pose.position.z = 0.0
        quat = yaw_to_quaternion(float(point.get('yaw', 0.0)))
        pose.pose.orientation.x = quat['x']
        pose.pose.orientation.y = quat['y']
        pose.pose.orientation.z = quat['z']
        pose.pose.orientation.w = quat['w']
        return pose

    def go_to_pose(self, pose, name):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f'导航到 {name}: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{name} 目标被 Nav2 拒绝')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error(f'{name} 没有返回结果')
            return False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{name} 已到达')
            return True
        self.get_logger().error(f'{name} 导航失败，status={result.status}')
        return False

    def execute(self):
        points = load_points(self.route_config['points_file'])
        route = self.route_config.get('route') or []
        frame_id = self.route_config.get('frame_id', 'map')
        pause_seconds = float(self.route_config.get('pause_seconds', 3.0))
        missing = [name for name in route if name not in points]
        if missing:
            raise RuntimeError('缺少标定点: ' + ', '.join(missing))
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Nav2 navigate_to_pose action server 未就绪')
        for index, name in enumerate(route, start=1):
            pose = self.make_pose(points[name], frame_id)
            if not self.go_to_pose(pose, f'{index}/{len(route)} {name}'):
                return False
            self.get_logger().info(f'停顿 {pause_seconds:.1f}s')
            time.sleep(pause_seconds)
        return True


def parse_args():
    parser = argparse.ArgumentParser(description='Execute ROSCAR named waypoint route through Nav2.')
    parser.add_argument(
        '--route',
        default=str(
            Path(get_package_share_directory('roscar_nav')) / 'config/route_main.yaml'
        ),
        help='route yaml path',
    )
    return parser.parse_args()


def main(args=None):
    parsed = parse_args()
    config = yaml.safe_load(Path(parsed.route).read_text(encoding='utf-8')) or {}
    rclpy.init(args=args)
    node = RouteExecutor(config)
    try:
        ok = node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 2)


if __name__ == '__main__':
    main()
