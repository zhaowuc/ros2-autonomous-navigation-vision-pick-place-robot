#!/usr/bin/env python3
"""Kinematic Gazebo actuator plus acceptance-only observed model truth."""

import math
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def clamp(value, limit):
    return max(-limit, min(limit, float(value)))


class SimBaseController(Node):
    def __init__(self):
        super().__init__('roscar_sim_base')
        self.declare_parameter('entity_name', 'roscar')
        self.declare_parameter('update_rate', 50.0)
        self.entity_name = str(self.get_parameter('entity_name').value)
        self.x = -3.0
        self.y = -2.0
        self.yaw = 0.0
        self.command = Twist()
        self.command_monotonic = 0.0
        self.last_update_monotonic = time.monotonic()
        self.pending_state = None
        self.state_request_failures = 0
        self.entity_observed = False
        self.state_client = self.create_client(SetEntityState, '/set_entity_state')
        latest_state_qos = QoSProfile(depth=1)
        latest_state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        # This topic is sourced only from Gazebo's observed ModelStates and is
        # consumed only by acceptance tooling. Production odom/TF never reads
        # it; roscar_base_interface alone owns /wheel/odom_raw.
        self.truth_pub = self.create_publisher(
            Odometry, '/sim/ground_truth', latest_state_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/sim/cmd_vel', self._command, 20)
        self.create_subscription(PoseStamped, '/roscar_sim/reset_pose', self._reset, 10)
        self.create_subscription(
            ModelStates, '/model_states', self._model_states,
            latest_state_qos,
        )
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self._model_states,
            latest_state_qos,
        )
        rate = max(10.0, float(self.get_parameter('update_rate').value))
        # Gazebo's simulated clock is intentionally bursty on this host.  A
        # steady timer keeps actuator steps bounded even when /clock catches up.
        self.control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / rate, self._update, clock=self.control_clock)

    def _command(self, message):
        self.command = message
        self.command_monotonic = time.monotonic()

    def _reset(self, message):
        self.x = float(message.pose.position.x)
        self.y = float(message.pose.position.y)
        q = message.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.command = Twist()
        self.command_monotonic = 0.0
        self.last_update_monotonic = time.monotonic()

    def _update(self):
        now = time.monotonic()
        # Never integrate a second target while Gazebo is still acknowledging
        # the previous SetEntityState request.  Accumulating those targets used
        # to create a late post-goal jump between Nav2's TF and ModelStates.
        dt = min(0.04, max(0.0, now - self.last_update_monotonic))
        self.last_update_monotonic = now
        if self.pending_state is not None and not self.pending_state.done():
            self._publish_tf()
            return
        if now - self.command_monotonic <= 0.25:
            vx = clamp(self.command.linear.x, 0.20)
            vy = clamp(self.command.linear.y, 0.20)
            wz = clamp(self.command.angular.z, 0.45)
        else:
            vx = vy = wz = 0.0
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        self.x += (vx * cy - vy * sy) * dt
        self.y += (vx * sy + vy * cy) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        self._publish_tf()
        self._set_gazebo_pose()

    def _publish_tf(self):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _model_states(self, message):
        try:
            index = message.name.index(self.entity_name)
            pose = message.pose[index]
            twist_world = message.twist[index]
        except (ValueError, IndexError):
            return
        self.entity_observed = True
        yaw = math.atan2(
            2.0 * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y
            ),
            1.0 - 2.0 * (
                pose.orientation.y * pose.orientation.y
                + pose.orientation.z * pose.orientation.z
            ),
        )
        # Gazebo ModelStates linear twist is in world axes; Odometry twist is
        # expressed in child/body axes.
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        body_vx = cy * twist_world.linear.x + sy * twist_world.linear.y
        body_vy = -sy * twist_world.linear.x + cy * twist_world.linear.y
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose = pose
        odom.twist.twist.linear.x = body_vx
        odom.twist.twist.linear.y = body_vy
        odom.twist.twist.angular.z = twist_world.angular.z
        odom.pose.covariance[0] = 1.0e-8
        odom.pose.covariance[7] = 1.0e-8
        odom.pose.covariance[35] = 1.0e-8
        self.truth_pub.publish(odom)

    def _set_gazebo_pose(self):
        if not self.entity_observed or not self.state_client.service_is_ready():
            return
        if self.pending_state is not None and not self.pending_state.done():
            return
        request = SetEntityState.Request()
        request.state.name = self.entity_name
        request.state.pose.position.x = self.x
        request.state.pose.position.y = self.y
        request.state.pose.position.z = -0.1367
        request.state.pose.orientation.z = math.sin(self.yaw / 2.0)
        request.state.pose.orientation.w = math.cos(self.yaw / 2.0)
        request.state.reference_frame = 'world'
        requested_pose = (self.x, self.y, self.yaw)
        self.pending_state = self.state_client.call_async(request)
        self.pending_state.add_done_callback(
            lambda future, pose=requested_pose: self._state_response(future, pose)
        )

    def _state_response(self, future, requested_pose):
        try:
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError('Gazebo SetEntityState returned failure')
        except Exception as exc:
            self.state_request_failures += 1
            self.get_logger().error(
                f'SetEntityState failure #{self.state_request_failures}: {exc}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimBaseController()
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
