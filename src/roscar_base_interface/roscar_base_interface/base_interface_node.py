"""ROS 2 node exposing one stable base interface across all backends."""

import math
import threading
import time

import rclpy
from c50c_interfaces.msg import BaseStatus, WheelState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .backends import (
    BackendCommand,
    BackendFeedback,
    BaseBackend,
    MockBackend,
    SerialBackend,
    SimBackend,
)
from .diagnostics import assess_backend_snapshot
from .kinematics import WheelOdometry
from .protocol import CommandFlag, ControlState, FaultBit


def latest_command_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def estop_state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def clamp(value: float, limit: float) -> float:
    number = float(value)
    bound = abs(float(limit))
    if not math.isfinite(number) or not math.isfinite(bound):
        return 0.0
    return min(bound, max(-bound, number))


def diagnostics_follow_control_tick(
    diagnostic_rate: float, command_tx_rate: float
) -> bool:
    """Return whether diagnostics must be emitted once per control callback."""
    return max(0.1, float(diagnostic_rate)) >= max(1.0, float(command_tx_rate))


class BaseInterfaceNode(Node):
    """Translate ``/cmd_vel_safe`` into one selected base backend."""

    def __init__(self) -> None:
        super().__init__('roscar_base_interface')
        self._declare_parameters()

        self.backend_name = str(self.get_parameter('base_backend').value).strip().lower()
        self.command_tx_rate = float(self.get_parameter('command_tx_rate').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.max_vx = min(0.20, abs(float(self.get_parameter('max_vx').value)))
        self.max_vy = min(0.20, abs(float(self.get_parameter('max_vy').value)))
        self.max_wz = min(0.45, abs(float(self.get_parameter('max_wz').value)))
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_spacing = float(self.get_parameter('wheel_spacing').value)
        self.axle_spacing = float(self.get_parameter('axle_spacing').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)

        self.backend = self._make_backend()
        self.odometry = WheelOdometry(self.wheel_spacing, self.axle_spacing)
        self.odom_lock = threading.Lock()
        self.lock = threading.Lock()
        self.latest_twist = Twist()
        self.latest_command_at = 0.0
        self.latest_command_valid = False
        self.invalid_command_samples = 0
        self.estop_active = False
        self.pending_flags = 0
        self.sequence = 0
        self.last_feedback: BackendFeedback | None = None
        self.last_status_publish = 0.0
        self.last_diagnostic_log = 0.0
        self.last_diagnostics_publish = 0.0
        self.diagnostic_rate = max(
            0.1,
            float(self.get_parameter('diagnostic_rate').value),
        )
        self.diagnostic_period = 1.0 / self.diagnostic_rate
        # Diagnostics are emitted from the control timer, so a requested rate
        # at or above that timer's rate means exactly one publication per
        # control tick.  Re-applying an equal 20 ms elapsed-time threshold here
        # aliases against normal timer jitter (and float rounding), otherwise
        # dropping the measured diagnostic stream to roughly 25--35 Hz.
        self.diagnostics_every_control_tick = diagnostics_follow_control_tick(
            self.diagnostic_rate,
            self.command_tx_rate,
        )
        self.ack_warn_lag = int(self.get_parameter('ack_warn_lag').value)

        self.wheel_pub = self.create_publisher(WheelState, '/c50c/wheel_states', 20)
        self.status_pub = self.create_publisher(BaseStatus, '/c50c/status', 20)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 20)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom_raw', 20)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.sim_command_pub = None
        if self.backend_name == 'sim':
            sim_command_topic = str(self.get_parameter('sim_command_topic').value)
            self.sim_command_pub = self.create_publisher(Twist, sim_command_topic, 20)

        self.create_subscription(Twist, '/cmd_vel_safe', self._command_callback, latest_command_qos())
        self.create_subscription(
            Bool, '/roscar/estop_state', self._estop_callback,
            estop_state_qos(),
        )
        self.create_service(Trigger, '/roscar/base/rearm', self._rearm_service)
        self.create_service(Trigger, '/roscar/base/clear_fault', self._clear_fault_service)
        # Safety I/O must not inherit Gazebo's paused/bursty /clock.  ROS stamps
        # still use the node clock, while the 50 Hz control cadence is steady.
        self.control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.control_timer = self.create_timer(
            1.0 / max(1.0, self.command_tx_rate),
            self._control_tick,
            clock=self.control_clock,
        )

        self.get_logger().info(
            'unified base interface ready: '
            f'base_backend={self.backend_name}, tx_rate={self.command_tx_rate:.1f}Hz, '
            f'cmd_timeout={self.command_timeout:.3f}s, '
            f'limits=({self.max_vx:.3f},{self.max_vy:.3f},{self.max_wz:.3f}), '
            'REAL_WHEEL_ODOM_SCALE=FIRMWARE_ALIGNED_R16_GROUND_CHECKED'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('base_backend', 'sim')
        self.declare_parameter('serial_device', '/dev/c50c_ros')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('command_tx_rate', 50.0)
        self.declare_parameter('command_timeout', 0.15)
        self.declare_parameter('feedback_expected_rate', 50.0)
        self.declare_parameter('feedback_timeout', 0.15)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('max_vx', 0.20)
        self.declare_parameter('max_vy', 0.20)
        self.declare_parameter('max_wz', 0.45)
        self.declare_parameter('wheel_radius', 0.050)
        self.declare_parameter('wheel_spacing', 0.519720)
        self.declare_parameter('axle_spacing', 0.410)
        self.declare_parameter('encoder_ticks_per_revolution', 4096)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('sim_command_topic', '/sim/cmd_vel')
        self.declare_parameter('sim_battery_mv', 24000)
        self.declare_parameter('diagnostic_rate', 2.0)
        self.declare_parameter('ack_warn_lag', 5)

    def _make_backend(self) -> BaseBackend:
        common = {
            'wheel_radius': self.wheel_radius,
            'wheel_spacing': self.wheel_spacing,
            'axle_spacing': self.axle_spacing,
            'encoder_ticks_per_revolution': int(
                self.get_parameter('encoder_ticks_per_revolution').value
            ),
        }
        if self.backend_name == 'sim':
            return SimBackend(
                **common,
                battery_mv=int(self.get_parameter('sim_battery_mv').value),
            )
        if self.backend_name == 'mock':
            return MockBackend(**common)
        if self.backend_name == 'serial':
            return SerialBackend(
                serial_device=str(self.get_parameter('serial_device').value),
                baudrate=int(self.get_parameter('baudrate').value),
                command_tx_rate=self.command_tx_rate,
                command_timeout=self.command_timeout,
                feedback_expected_rate=float(
                    self.get_parameter('feedback_expected_rate').value
                ),
                feedback_timeout=float(self.get_parameter('feedback_timeout').value),
                reconnect_interval=float(self.get_parameter('reconnect_interval').value),
                max_vx=self.max_vx,
                max_vy=self.max_vy,
                max_wz=self.max_wz,
            )
        raise ValueError(
            f'unsupported base_backend={self.backend_name!r}; expected sim, mock, or serial'
        )

    def _command_callback(self, message: Twist) -> None:
        now = time.monotonic()
        safe = Twist()
        motion = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        valid = all(math.isfinite(value) for value in motion)
        if valid:
            safe.linear.x = clamp(motion[0], self.max_vx)
            safe.linear.y = clamp(motion[1], self.max_vy)
            safe.angular.z = clamp(motion[2], self.max_wz)
        else:
            # One malformed component invalidates the whole velocity sample;
            # the control tick will emit a motion-disabled zero.
            safe.linear.x = 0.0
            safe.linear.y = 0.0
            safe.angular.z = 0.0
        safe.linear.z = 0.0
        safe.angular.x = 0.0
        safe.angular.y = 0.0
        with self.lock:
            self.latest_twist = safe
            self.latest_command_at = now
            self.latest_command_valid = valid
            if not valid:
                self.invalid_command_samples += 1

    def _estop_callback(self, message: Bool) -> None:
        active = bool(message.data)
        now = time.monotonic()
        with self.lock:
            previous = self.estop_active
            self.estop_active = active
            if active != previous:
                # Apply real assertion and release edges synchronously.  A
                # complete pulse can occur between 50 Hz control ticks, but a
                # repeated latched Bool sample must not continuously revoke a
                # healthy READY handshake.
                self.backend.notify_estop_edge(active, now)
            if active:
                self.pending_flags = 0

    def _rearm_service(self, _request: Trigger.Request, response: Trigger.Response):
        with self.lock:
            if self.estop_active:
                response.success = False
                response.message = 'Release ESTOP, then issue a new REARM request'
                return response
            if not self.backend.control_transactions_ready:
                response.success = False
                response.message = 'Base link is not READY; retry REARM after handshake'
                return response
            self.pending_flags |= int(CommandFlag.REARM)
        response.success = True
        response.message = (
            'REARM accepted; serial sends only in READY and reports ACK/FAILED in diagnostics'
        )
        return response

    def _clear_fault_service(self, _request: Trigger.Request, response: Trigger.Response):
        with self.lock:
            if self.estop_active:
                response.success = False
                response.message = 'Release ESTOP, then issue a new CLEAR_FAULT request'
                return response
            if not self.backend.control_transactions_ready:
                response.success = False
                response.message = (
                    'Base link is not READY; retry CLEAR_FAULT after handshake'
                )
                return response
            self.pending_flags |= int(CommandFlag.CLEAR_FAULT)
        response.success = True
        response.message = (
            'CLEAR_FAULT accepted; serial sends only in READY and reports ACK/FAILED in diagnostics'
        )
        return response

    def _control_tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            command_age = now - self.latest_command_at if self.latest_command_at else math.inf
            command_fresh = (
                self.latest_command_valid
                and command_age <= self.command_timeout
            )
            # Supply the serial fresh-zero handshake before Nav2 is active;
            # no external velocity source should be required to prove stop.
            startup_zero = self.latest_command_at == 0.0
            estop = self.estop_active
            pending_flags = self.pending_flags
            self.pending_flags = 0
            if command_fresh and not estop:
                vx = self.latest_twist.linear.x
                vy = self.latest_twist.linear.y
                wz = self.latest_twist.angular.z
                flags = int(CommandFlag.MOTION_ENABLE) | pending_flags
            elif estop:
                vx = vy = wz = 0.0
                flags = int(CommandFlag.ESTOP) | pending_flags
            else:
                vx = vy = wz = 0.0
                flags = pending_flags
            # ESTOP is a safety input of its own, not part of the velocity
            # command freshness contract.  Timestamp it now so a stale
            # /cmd_vel_safe can never suppress an asserted ESTOP frame.
            source_stamp = (
                now
                if estop or pending_flags or startup_zero
                else self.latest_command_at
                if command_fresh
                else 0.0
            )
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            command = BackendCommand(
                self.sequence,
                clamp(vx, self.max_vx),
                clamp(vy, self.max_vy),
                clamp(wz, self.max_wz),
                flags,
                source_stamp,
            )
            # Keep command construction/handoff in the same critical section
            # as the ESTOP edge callback.  Otherwise cancellation could run
            # first and this pre-edge one-shot could be enqueued afterward.
            self.backend.set_command(command)

        self._publish_sim_command()
        feedbacks = self.backend.tick(now)
        for feedback in feedbacks:
            self.last_feedback = feedback
            self._publish_feedback(feedback)
        if not feedbacks and now - self.last_status_publish >= 0.5:
            self._publish_no_feedback_status()
        if (
            getattr(self, 'diagnostics_every_control_tick', False)
            or now - self.last_diagnostics_publish >= self.diagnostic_period
        ):
            self._publish_diagnostics(now)
        if now - self.last_diagnostic_log >= 5.0:
            self.last_diagnostic_log = now
            self.get_logger().info(self.backend.diagnostic_summary)

    def _publish_sim_command(self) -> None:
        command = self.backend.command_for_simulator
        if command is None or self.sim_command_pub is None:
            return
        message = Twist()
        message.linear.x = clamp(command.vx, self.max_vx)
        message.linear.y = clamp(command.vy, self.max_vy)
        message.angular.z = clamp(command.wz, self.max_wz)
        self.sim_command_pub.publish(message)

    def _publish_feedback(self, feedback: BackendFeedback) -> None:
        stamp = self.get_clock().now().to_msg()
        payload = feedback.payload
        speeds = feedback.wheel_speeds_m_s

        wheels = WheelState()
        wheels.header.stamp = stamp
        wheels.header.frame_id = self.base_frame_id
        (
            wheels.fl_encoder,
            wheels.fr_encoder,
            wheels.rl_encoder,
            wheels.rr_encoder,
        ) = payload.encoders
        (
            wheels.fl_speed,
            wheels.fr_speed,
            wheels.rl_speed,
            wheels.rr_speed,
        ) = speeds
        self.wheel_pub.publish(wheels)

        status = BaseStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.base_frame_id
        status.last_cmd_seq = payload.last_cmd_seq
        status.control_state = payload.control_state
        status.fault_bits = payload.fault_bits
        status.battery_mv = payload.battery_mv
        self.status_pub.publish(status)
        self.last_status_publish = feedback.received_at

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.header.frame_id = self.base_frame_id
        battery.voltage = payload.battery_mv / 1000.0
        battery.temperature = math.nan
        battery.current = math.nan
        battery.charge = math.nan
        battery.capacity = math.nan
        battery.design_capacity = math.nan
        battery.percentage = math.nan
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery.present = True
        self.battery_pub.publish(battery)

        with self.odom_lock:
            velocity = self.odometry.update(speeds, feedback.received_at)
            pose = self.odometry.pose
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = math.sin(pose.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(pose.yaw * 0.5)
        odom.twist.twist.linear.x = velocity.vx
        odom.twist.twist.linear.y = velocity.vy
        odom.twist.twist.angular.z = velocity.wz
        odom.pose.covariance[0] = 0.04
        odom.pose.covariance[7] = 0.04
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.04
        odom.twist.covariance[7] = 0.04
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

    def _publish_no_feedback_status(self) -> None:
        now = time.monotonic()
        status = BaseStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.base_frame_id
        status.last_cmd_seq = (
            self.last_feedback.payload.last_cmd_seq
            if self.last_feedback is not None else 0
        )
        status.control_state = int(ControlState.LOCKED)
        status.fault_bits = int(FaultBit.COMMUNICATION_FAULT)
        status.battery_mv = 0
        self.status_pub.publish(status)
        self.last_status_publish = now

    def _publish_diagnostics(self, now: float) -> None:
        snapshot = self.backend.diagnostic_snapshot(now)
        with self.lock:
            snapshot['latest_command_valid'] = self.latest_command_valid
            snapshot['invalid_command_samples'] = self.invalid_command_samples
        assessment = assess_backend_snapshot(
            snapshot,
            ack_warn_lag=self.ack_warn_lag,
        )
        status = DiagnosticStatus()
        # diagnostic_msgs/DiagnosticStatus.level is ROS ``byte`` in Humble;
        # its generated Python setter requires one byte, not an int.
        status.level = bytes((assessment.level,))
        status.name = 'roscar_base_interface/backend'
        status.hardware_id = (
            str(self.get_parameter('serial_device').value)
            if self.backend_name == 'serial'
            else f'c50c-{self.backend_name}'
        )
        status.message = assessment.message
        status.values = [
            KeyValue(key=str(key), value=_diagnostic_text(value))
            for key, value in sorted(snapshot.items())
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)
        self.last_diagnostics_publish = now

    def destroy_node(self):
        self.backend.close()
        return super().destroy_node()


def _diagnostic_text(value: object) -> str:
    if isinstance(value, float):
        return f'{value:.6f}' if math.isfinite(value) else 'inf'
    if isinstance(value, int) and value >= 0:
        return str(value)
    return str(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseInterfaceNode()
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
