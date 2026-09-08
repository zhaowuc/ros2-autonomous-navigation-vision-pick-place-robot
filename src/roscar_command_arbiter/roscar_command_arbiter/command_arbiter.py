import json
import math
import threading
import time
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


SPEED_PROFILES = {
    'low': 0.06,
    'normal': 0.10,
    'fast': 0.20,
}
FRESHNESS_SEC = 0.25


def clamp(value, lower, upper):
    return max(lower, min(upper, float(value)))


def sanitize_twist(msg, linear_limit):
    """Return a finite, bounded Twist without modifying the input message."""
    output = Twist()
    values = (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return output
    linear_x = float(msg.linear.x)
    linear_y = float(msg.linear.y)
    magnitude = math.hypot(linear_x, linear_y)
    scale = min(1.0, linear_limit / magnitude) if magnitude > 0.0 else 1.0
    output.linear.x = linear_x * scale
    output.linear.y = linear_y * scale
    output.linear.z = 0.0
    output.angular.x = 0.0
    output.angular.y = 0.0
    output.angular.z = clamp(msg.angular.z, -0.45, 0.45)
    return output


@dataclass
class SourceState:
    message: Twist = field(default_factory=Twist)
    received_monotonic: float = 0.0

    def fresh(self, now):
        return self.received_monotonic > 0.0 and now - self.received_monotonic <= FRESHNESS_SEC


class CommandArbiter(Node):
    """Select one command source and enforce the permanent speed contract."""

    def __init__(self):
        super().__init__('roscar_command_arbiter')
        self.declare_parameter('speed_profile', 'normal')
        self.declare_parameter('publish_rate', 50.0)
        self._lock = threading.Lock()
        self._sources = {
            'manual': SourceState(),
            'wall_align': SourceState(),
            'test': SourceState(),
            'nav': SourceState(),
        }
        self._estop = False
        self._last_source = 'ZERO'
        self._profile = str(self.get_parameter('speed_profile').value).lower()
        if self._profile not in SPEED_PROFILES:
            self._profile = 'normal'

        self._publisher = self.create_publisher(Twist, '/cmd_vel_selected', 10)
        self._status_publisher = self.create_publisher(String, '/roscar/command_arbiter/status', 10)
        estop_qos = QoSProfile(depth=1)
        estop_qos.reliability = ReliabilityPolicy.RELIABLE
        estop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._estop_publisher = self.create_publisher(
            Bool, '/roscar/estop_state', estop_qos
        )
        self.create_subscription(Twist, '/cmd_vel_manual', self._callback('manual'), 10)
        self.create_subscription(Twist, '/cmd_vel_wall_align', self._callback('wall_align'), 10)
        self.create_subscription(Twist, '/cmd_vel_test', self._callback('test'), 10)
        self.create_subscription(Twist, '/cmd_vel_nav_raw', self._callback('nav'), 10)
        self.create_service(SetBool, '/roscar/set_estop', self._set_estop)
        # Do not shadow rclpy.node.Node._set_parameters(): TimeSource invokes
        # that internal method while applying the use_sim_time override.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        # Authority freshness already uses monotonic time.  A steady clock also
        # prevents Gazebo /clock bursts from stretching a 20 ms publish period
        # beyond the 100 ms handoff contract.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            1.0 / rate, self._publish_selected, clock=self._steady_clock
        )
        self._publish_estop_state()
        self.get_logger().info(
            f'command authority ready: profile={self._profile} max={SPEED_PROFILES[self._profile]:.2f} m/s'
        )

    def _callback(self, source):
        def receive(message):
            now = time.monotonic()
            with self._lock:
                self._sources[source] = SourceState(message=message, received_monotonic=now)
        return receive

    def _set_estop(self, request, response):
        with self._lock:
            self._estop = bool(request.data)
        self._publish_estop_state()
        response.success = True
        response.message = 'E-stop latched' if request.data else 'E-stop cleared'
        self._publisher.publish(Twist())
        return response

    def _publish_estop_state(self):
        state = Bool()
        state.data = self._estop
        self._estop_publisher.publish(state)

    def _on_set_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'speed_profile':
                profile = str(parameter.value).lower()
                if profile not in SPEED_PROFILES:
                    return SetParametersResult(
                        successful=False,
                        reason='speed_profile must be low, normal, or fast',
                    )
                self._profile = profile
        return SetParametersResult(successful=True)

    def _choose_locked(self, now):
        if self._estop:
            return 'E_STOP', Twist()
        if self._sources['manual'].fresh(now):
            return 'manual', self._sources['manual'].message
        wall = self._sources['wall_align']
        test = self._sources['test']
        if wall.fresh(now) or test.fresh(now):
            if test.received_monotonic > wall.received_monotonic:
                return 'test', test.message
            return 'wall_align', wall.message
        if self._sources['nav'].fresh(now):
            return 'nav', self._sources['nav'].message
        return 'ZERO', Twist()

    def _publish_selected(self):
        now = time.monotonic()
        with self._lock:
            source, message = self._choose_locked(now)
            profile = self._profile
            estop = self._estop
        output = sanitize_twist(message, SPEED_PROFILES[profile])
        if source in ('E_STOP', 'ZERO'):
            output = Twist()
        self._publisher.publish(output)
        status = String()
        status.data = json.dumps({
            'source': source,
            'speed_profile': profile,
            'max_linear': SPEED_PROFILES[profile],
            'estop': estop,
            'stamp_monotonic': now,
        }, sort_keys=True)
        self._status_publisher.publish(status)
        self._last_source = source


def main(args=None):
    rclpy.init(args=args)
    node = CommandArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
