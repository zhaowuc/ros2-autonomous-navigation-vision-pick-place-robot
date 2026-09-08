#!/usr/bin/env python3
import math
import time

import rclpy
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_srvs.srv import SetBool


class PedestrianController(Node):
    def __init__(self):
        super().__init__('pedestrian_controller')
        self.declare_parameter('entity_name', 'pedestrian')
        self.declare_parameter('enabled', False)
        self.entity_name = str(self.get_parameter('entity_name').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.started_monotonic = time.monotonic()
        self._pending_future = None
        self._parked = False
        self._state_failures = 0
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.create_service(SetBool, '/roscar_sim/set_pedestrian_motion', self._set_enabled)
        self.create_timer(0.05, self._update)

    def _set_enabled(self, request, response):
        self.enabled = bool(request.data)
        self.started_monotonic = time.monotonic()
        self._parked = False
        response.success = True
        response.message = 'pedestrian motion enabled' if self.enabled else 'pedestrian parked'
        return response

    def _update(self):
        if not self.client.service_is_ready():
            return
        if self._pending_future is not None and not self._pending_future.done():
            return
        if not self.enabled and self._parked:
            return
        elapsed = time.monotonic() - self.started_monotonic
        y = -3.5
        if self.enabled:
            # One deterministic crossing is a realistic encounter.  Repeatedly
            # walking back through a stopped vehicle would turn the pedestrian
            # itself into the collision source and invalidate the safety test.
            progress = min(elapsed * 0.35, 1.0)
            y = -3.5 + 3.0 * progress
        request = SetEntityState.Request()
        request.state.name = self.entity_name
        request.state.pose.position.x = 0.0
        request.state.pose.position.y = y
        request.state.pose.position.z = 0.85
        request.state.pose.orientation.w = 1.0
        request.state.reference_frame = 'world'
        self._pending_future = self.client.call_async(request)
        self._pending_future.add_done_callback(self._state_response)
        if not self.enabled:
            # A disabled pedestrian needs one deterministic parking request,
            # not a 20 Hz stream that can starve other Gazebo state clients.
            self._parked = True

    def _state_response(self, future):
        try:
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError('Gazebo SetEntityState returned failure')
        except Exception as exc:
            self._state_failures += 1
            self.get_logger().error(
                f'pedestrian SetEntityState failure #{self._state_failures}: {exc}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianController()
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
