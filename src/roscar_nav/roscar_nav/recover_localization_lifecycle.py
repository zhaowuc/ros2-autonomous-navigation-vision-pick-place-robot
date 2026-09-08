import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LocalizationLifecycleRecovery(Node):
    """Repair a localization lifecycle-manager response race without motion."""

    def __init__(self):
        super().__init__('localization_lifecycle_recovery')
        self.declare_parameter('timeout_sec', 30.0)
        self.timeout_sec = max(
            5.0,
            float(self.get_parameter('timeout_sec').value),
        )
        self._lifecycle_clients = {}
        for name in ('map_server', 'amcl'):
            self._lifecycle_clients[name] = {
                'get': self.create_client(GetState, f'/{name}/get_state'),
                'change': self.create_client(
                    ChangeState,
                    f'/{name}/change_state',
                ),
            }

    def _call(self, client, request, timeout_sec=3.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.exception() is not None:
            return None
        return future.result()

    def state(self, name):
        response = self._call(
            self._lifecycle_clients[name]['get'],
            GetState.Request(),
        )
        if response is None:
            return None
        return int(response.current_state.id)

    def transition(self, name, transition_id):
        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        response = self._call(
            self._lifecycle_clients[name]['change'],
            request,
        )
        return bool(response and response.success)

    def ensure_active(self, name):
        current = self.state(name)
        if current is None:
            return False
        if current == State.PRIMARY_STATE_ACTIVE:
            return True
        if current == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().warning(
                f'{name} remained unconfigured; retrying configure transition'
            )
            self.transition(name, Transition.TRANSITION_CONFIGURE)
            time.sleep(0.2)
            current = self.state(name)
        if current == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().warning(
                f'{name} remained inactive; retrying activate transition'
            )
            self.transition(name, Transition.TRANSITION_ACTIVATE)
            time.sleep(0.2)
            current = self.state(name)
        return current == State.PRIMARY_STATE_ACTIVE

    def run(self):
        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            # map_server must publish /map before AMCL can be configured.
            map_ready = self.ensure_active('map_server')
            amcl_ready = map_ready and self.ensure_active('amcl')
            if map_ready and amcl_ready:
                self.get_logger().info(
                    'Localization lifecycle nodes are active'
                )
                return True
            time.sleep(0.8)
        self.get_logger().error(
            'Localization lifecycle recovery timed out; navigation stays off'
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationLifecycleRecovery()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
