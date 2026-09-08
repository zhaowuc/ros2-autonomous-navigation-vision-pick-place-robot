import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


CRUISE_CONTROLLER = 'FollowPathCruise'
TERMINAL_CONTROLLER = 'FollowPathTerminal'


def controller_for_distance(distance, terminal_distance, already_terminal):
    if already_terminal or distance <= terminal_distance:
        return TERMINAL_CONTROLLER
    return CRUISE_CONTROLLER


class NavigationModeManager(Node):
    """Latch Terminal Omni at most once for each announced goal."""

    def __init__(self):
        super().__init__('roscar_navigation_mode_manager')
        self.declare_parameter('terminal_distance', 0.40)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.terminal_distance = float(self.get_parameter('terminal_distance').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.selector_pub = self.create_publisher(String, '/controller_selector', qos)
        self.status_pub = self.create_publisher(String, '/roscar/navigation_mode/status', 10)
        self.create_subscription(PoseStamped, '/roscar/active_goal', self._goal, qos)
        self.create_subscription(Bool, '/roscar/goal_active', self._goal_active, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal = None
        self.goal_active = False
        self.terminal_latched = False
        self.switch_count = 0
        self.mode = CRUISE_CONTROLLER
        self.goal_started_monotonic = 0.0
        self.last_switch_monotonic = 0.0
        self.create_timer(0.05, self._update)
        self._publish_selector()

    def _goal(self, message):
        self.goal = message
        self.goal_active = True
        self.terminal_latched = False
        self.switch_count = 0
        self.mode = CRUISE_CONTROLLER
        self.goal_started_monotonic = time.monotonic()
        self.last_switch_monotonic = self.goal_started_monotonic
        self._publish_selector()

    def _goal_active(self, message):
        if bool(message.data):
            self.goal_active = True
            return
        self.goal_active = False
        self.goal = None
        self.terminal_latched = False
        self.switch_count = 0
        self.mode = CRUISE_CONTROLLER
        self._publish_selector()

    def _publish_selector(self):
        message = String()
        message.data = self.mode
        self.selector_pub.publish(message)

    def _update(self):
        distance = None
        if self.goal_active and self.goal is not None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.03),
                )
                dx = float(self.goal.pose.position.x) - float(transform.transform.translation.x)
                dy = float(self.goal.pose.position.y) - float(transform.transform.translation.y)
                distance = math.hypot(dx, dy)
            except TransformException:
                distance = None

        if distance is not None:
            selected = controller_for_distance(
                distance, self.terminal_distance, self.terminal_latched
            )
            if selected == TERMINAL_CONTROLLER and not self.terminal_latched:
                self.terminal_latched = True
                self.switch_count += 1
                self.mode = TERMINAL_CONTROLLER
                self.last_switch_monotonic = time.monotonic()
                self._publish_selector()

        status = String()
        status.data = json.dumps({
            'mode': 'TERMINAL_OMNI' if self.terminal_latched else 'CRUISE_DIFF',
            'controller': self.mode,
            'goal_active': self.goal_active,
            'distance': distance,
            'switch_count': self.switch_count,
            'terminal_distance': self.terminal_distance,
            'last_switch_monotonic': self.last_switch_monotonic,
        }, sort_keys=True)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationModeManager()
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
