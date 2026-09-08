import threading
import unittest

from geometry_msgs.msg import Twist

from c50c_base_driver.c50c_base_driver_node import C50CBaseDriver


class EmptySerial:
    is_open = True
    in_waiting = 0

    def read(self, _size):
        raise AssertionError('empty serial input must never perform a read')


class SingleSerialWriterTests(unittest.TestCase):
    def test_empty_serial_input_is_non_blocking(self):
        node = object.__new__(C50CBaseDriver)
        node.serial_port = EmptySerial()
        node.connected = True
        node.read_available_frames()

    def test_command_callback_only_replaces_target(self):
        node = object.__new__(C50CBaseDriver)
        node.lock = threading.Lock()
        node.max_vx = 0.08
        node.max_vy = 0.08
        node.max_wz = 0.20
        node.last_motion_mode = 'zero'
        node.mode_switch_stop_seconds = 0.0
        node.mode_switch_zero_until = 0.0
        node.zero_settle_until = 0.0
        node.target_vx = 0.0
        node.target_vy = 0.0
        node.target_wz = 0.0
        node.last_cmd_time = 0.0
        node.write_velocity = lambda *_args: (
            _ for _ in ()
        ).throw(AssertionError('subscription callback must not write serial'))

        message = Twist()
        message.linear.x = 0.08
        node.cmd_callback(message)

        self.assertEqual(node.target_vx, 0.08)
        self.assertEqual(node.target_vy, 0.0)
        self.assertEqual(node.target_wz, 0.0)
        self.assertGreater(node.last_cmd_time, 0.0)


if __name__ == '__main__':
    unittest.main()
