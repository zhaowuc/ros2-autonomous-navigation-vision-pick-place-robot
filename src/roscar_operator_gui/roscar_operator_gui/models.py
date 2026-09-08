from dataclasses import dataclass


POINT_TYPE_TARGET = 'target'
POINT_TYPE_WAYPOINT = 'waypoint'
VALID_POINT_TYPES = {POINT_TYPE_TARGET, POINT_TYPE_WAYPOINT}


def normalize_point_type(value):
    value = str(value or POINT_TYPE_TARGET).strip().lower()
    return value if value in VALID_POINT_TYPES else POINT_TYPE_TARGET


@dataclass(frozen=True)
class StoredPoint:
    point_id: str
    name: str
    x: float
    y: float
    yaw: float
    frame_id: str
    map_sha256: str
    saved_at: str
    point_type: str = POINT_TYPE_TARGET

    @classmethod
    def from_mapping(cls, value):
        return cls(
            point_id=str(value.get('id') or value.get('point_id') or ''),
            name=str(value.get('name') or ''),
            x=float(value.get('x', 0.0)),
            y=float(value.get('y', 0.0)),
            yaw=float(value.get('yaw', 0.0)),
            frame_id=str(value.get('frame_id') or 'map'),
            map_sha256=str(value.get('map_sha256') or ''),
            saved_at=str(value.get('saved_at') or value.get('timestamp') or ''),
            # Version-1 files had no type. They are real destinations and must
            # remain strict target points after the schema upgrade.
            point_type=normalize_point_type(value.get('type') or value.get('point_type')),
        )

    def to_mapping(self):
        return {
            'id': self.point_id,
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'frame_id': self.frame_id,
            'map_sha256': self.map_sha256,
            'saved_at': self.saved_at,
            'type': normalize_point_type(self.point_type),
        }
