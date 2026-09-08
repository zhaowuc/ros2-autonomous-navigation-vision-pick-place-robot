import os
import tempfile
import uuid
from pathlib import Path

import yaml


BUILTIN_ACTIONS = (
    {"id": "builtin:initialize", "name": "初始化", "kind": "stored", "start": 1, "end": 1, "repeat": 1},
    {"id": "builtin:front_detect", "name": "前方检测", "kind": "stored", "start": 2, "end": 2, "repeat": 1},
    {"id": "builtin:pickup_right", "name": "右侧药包拾取", "kind": "stored", "start": 3, "end": 7, "repeat": 1},
    {"id": "builtin:pickup_left", "name": "左侧药包拾取", "kind": "stored", "start": 8, "end": 13, "repeat": 1},
    {"id": "builtin:place", "name": "药包放置", "kind": "stored", "start": 14, "end": 17, "repeat": 1},
)


def validate_frames(frames):
    values = []
    if not 1 <= len(frames) <= 100:
        raise ValueError("动作组必须包含 1–100 帧")
    for frame in frames:
        pulses = list(frame.get("pulses", ()))
        duration_ms = frame.get("duration_ms")
        if (
            len(pulses) != 6
            or not all(
                isinstance(value, int)
                and not isinstance(value, bool)
                and 500 <= value <= 2500
                for value in pulses
            )
        ):
            raise ValueError("每帧必须包含 6 个 500–2500 μs 脉宽")
        if (
            not isinstance(duration_ms, int)
            or isinstance(duration_ms, bool)
            or not 100 <= duration_ms <= 9999
        ):
            raise ValueError("动作时间必须在 100–9999 ms")
        values.append({"pulses": pulses, "duration_ms": duration_ms})
    return values


class ArmActionStore:
    def __init__(self, path="~/.ros/roscar_data/arm_actions.yaml"):
        self.path = Path(path).expanduser()

    def load(self):
        custom = []
        if self.path.is_file():
            payload = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
            rows = payload.get("actions", [])
            if not isinstance(rows, list):
                raise ValueError("机械臂动作文件必须包含 actions 列表")
            seen = {action["name"].casefold() for action in BUILTIN_ACTIONS}
            for row in rows:
                if not isinstance(row, dict):
                    continue
                name = str(row.get("name") or "").strip()
                action_id = str(row.get("id") or uuid.uuid4())
                if not name or name.casefold() in seen:
                    continue
                custom.append({
                    "id": action_id,
                    "name": name,
                    "kind": "sequence",
                    "frames": validate_frames(row.get("frames", [])),
                })
                seen.add(name.casefold())
        return [dict(action) for action in BUILTIN_ACTIONS] + custom

    def add(self, name, frames):
        name = str(name).strip()
        if not name:
            raise ValueError("动作组名称不能为空")
        actions = self.load()
        if any(action["name"].casefold() == name.casefold() for action in actions):
            raise ValueError("动作组名称不能重复")
        action = {
            "id": str(uuid.uuid4()),
            "name": name,
            "kind": "sequence",
            "frames": validate_frames(frames),
        }
        self._write([value for value in actions if value["kind"] == "sequence"] + [action])
        return action

    def delete(self, action_id):
        if str(action_id).startswith("builtin:"):
            raise ValueError("板内动作不能删除")
        custom = [value for value in self.load() if value["kind"] == "sequence"]
        updated = [value for value in custom if value["id"] != action_id]
        if len(updated) == len(custom):
            raise KeyError(action_id)
        self._write(updated)

    def _write(self, actions):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = yaml.safe_dump(
            {"version": 1, "actions": actions},
            allow_unicode=True,
            sort_keys=False,
        )
        descriptor, temporary = tempfile.mkstemp(
            prefix="." + self.path.name + ".",
            suffix=".tmp",
            dir=str(self.path.parent),
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
