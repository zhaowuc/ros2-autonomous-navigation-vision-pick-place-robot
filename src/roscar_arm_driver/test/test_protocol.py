import unittest

from roscar_arm_driver.protocol import (
    command_for,
    pose_command,
    stored_group_command,
    validate_sequence,
)


class ProtocolTest(unittest.TestCase):
    def test_uploaded_action_commands_are_exact(self):
        self.assertEqual(command_for("initialize"), b"$DGS:1!")
        self.assertEqual(command_for("pickup_right"), b"$DGT:3-7,1!")
        self.assertEqual(command_for("pickup_left"), b"$DGT:8-13,1!")
        with self.assertRaises(ValueError):
            command_for("raw_serial")

    def test_gui_commands_are_bounded(self):
        self.assertEqual(stored_group_command(8, 13), b"$DGT:8-13,1!")
        packet = pose_command([1500] * 6, 1000)
        self.assertEqual(packet.count(b"!"), 6)
        self.assertEqual(
            validate_sequence([{"pulses": [1500] * 6, "duration_ms": 1000}])[0][
                "duration_ms"
            ],
            1000,
        )
        with self.assertRaises(ValueError):
            pose_command([499] * 6, 1000)


if __name__ == "__main__":
    unittest.main()
