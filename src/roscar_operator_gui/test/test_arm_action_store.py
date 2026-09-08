import tempfile
import unittest
from pathlib import Path

from roscar_operator_gui.arm_action_store import ArmActionStore


class ArmActionStoreTest(unittest.TestCase):
    def test_builtin_and_recorded_actions_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ArmActionStore(Path(directory) / "actions.yaml")
            self.assertEqual(
                [action["name"] for action in store.load()[:5]],
                ["初始化", "前方检测", "右侧药包拾取", "左侧药包拾取", "药包放置"],
            )
            added = store.add(
                "测试动作",
                [{"pulses": [1500] * 6, "duration_ms": 1000}],
            )
            self.assertEqual(store.load()[-1]["frames"][0]["pulses"], [1500] * 6)
            store.delete(added["id"])
            self.assertEqual(len(store.load()), 5)


if __name__ == "__main__":
    unittest.main()
