import hashlib
import os
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .models import POINT_TYPE_TARGET, StoredPoint, normalize_point_type


class PointStore:
    def __init__(self, path='~/.ros/roscar_data/calibration_points.yaml'):
        self.path = Path(path).expanduser()

    @staticmethod
    def map_sha256(map_path):
        path = Path(map_path).expanduser()
        if not path.is_file():
            return ''
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        return digest.hexdigest()

    def load(self):
        if not self.path.is_file():
            return []
        value = yaml.safe_load(self.path.read_text(encoding='utf-8')) or {}
        rows = value.get('points', []) if isinstance(value, dict) else value
        if not isinstance(rows, list):
            raise ValueError('点位文件必须包含列表或 points 列表')
        points = []
        seen_ids = set()
        seen_names = set()
        for row in rows:
            if not isinstance(row, dict):
                continue
            point = StoredPoint.from_mapping(row)
            point_id = point.point_id or str(uuid.uuid4())
            name = point.name.strip()
            if not name or point_id in seen_ids or name.casefold() in seen_names:
                continue
            point = StoredPoint(
                point_id=point_id,
                name=name,
                x=point.x,
                y=point.y,
                yaw=point.yaw,
                frame_id=point.frame_id,
                map_sha256=point.map_sha256,
                saved_at=point.saved_at,
                point_type=normalize_point_type(point.point_type),
            )
            points.append(point)
            seen_ids.add(point.point_id)
            seen_names.add(point.name.casefold())
        return points

    def _atomic_write(self, points):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = yaml.safe_dump(
            {'version': 2, 'points': [point.to_mapping() for point in points]},
            allow_unicode=True,
            sort_keys=False,
        )
        fd, temporary = tempfile.mkstemp(
            prefix='.' + self.path.name + '.', suffix='.tmp', dir=str(self.path.parent)
        )
        try:
            with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory_fd = os.open(str(self.path.parent), os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def add(
        self, name, x, y, yaw, frame_id, map_path,
        point_type=POINT_TYPE_TARGET,
    ):
        name = str(name).strip()
        if not name:
            raise ValueError('点位名称不能为空')
        points = self.load()
        if any(point.name.casefold() == name.casefold() for point in points):
            raise ValueError('点位名称不能重复')
        point = StoredPoint(
            point_id=str(uuid.uuid4()),
            name=name,
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            frame_id=str(frame_id or 'map'),
            map_sha256=self.map_sha256(map_path),
            saved_at=datetime.now(timezone.utc).isoformat(),
            point_type=normalize_point_type(point_type),
        )
        points.append(point)
        self._atomic_write(points)
        return point

    def rename(self, point_id, new_name):
        point = next((value for value in self.load() if value.point_id == point_id), None)
        if point is None:
            raise KeyError(point_id)
        self.edit(
            point_id, new_name, point.x, point.y, point.yaw,
            point_type=point.point_type,
        )

    def edit(self, point_id, new_name, x, y, yaw, point_type=None):
        new_name = str(new_name).strip()
        if not new_name:
            raise ValueError('点位名称不能为空')
        points = self.load()
        if any(p.point_id != point_id and p.name.casefold() == new_name.casefold() for p in points):
            raise ValueError('点位名称不能重复')
        updated = []
        found = False
        for point in points:
            if point.point_id == point_id:
                point = StoredPoint(
                    point_id=point.point_id, name=new_name,
                    x=float(x), y=float(y), yaw=float(yaw),
                    frame_id=point.frame_id,
                    map_sha256=point.map_sha256, saved_at=point.saved_at,
                    point_type=normalize_point_type(
                        point.point_type if point_type is None else point_type
                    ),
                )
                found = True
            updated.append(point)
        if not found:
            raise KeyError(point_id)
        self._atomic_write(updated)

    def delete(self, point_id):
        points = self.load()
        updated = [point for point in points if point.point_id != point_id]
        if len(updated) == len(points):
            raise KeyError(point_id)
        self._atomic_write(updated)

    def move(self, point_id, offset):
        points = self.load()
        index = next(
            (index for index, point in enumerate(points) if point.point_id == point_id),
            None,
        )
        if index is None:
            raise KeyError(point_id)
        destination = max(0, min(len(points) - 1, index + int(offset)))
        if destination != index:
            point = points.pop(index)
            points.insert(destination, point)
            self._atomic_write(points)
        return destination
