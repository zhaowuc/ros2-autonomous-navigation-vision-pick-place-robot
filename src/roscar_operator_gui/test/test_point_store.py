from pathlib import Path

import pytest

from roscar_operator_gui.point_store import PointStore
from roscar_operator_gui.models import POINT_TYPE_TARGET, POINT_TYPE_WAYPOINT


def test_atomic_crud_and_uniqueness(tmp_path):
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n', encoding='utf-8')
    store = PointStore(tmp_path / 'points.yaml')
    point = store.add('SIM_A', 1.0, 2.0, 0.3, 'map', map_file)
    assert len(store.load()) == 1
    assert len(store.load()[0].map_sha256) == 64
    assert store.load()[0].point_type == POINT_TYPE_TARGET
    with pytest.raises(ValueError):
        store.add('sim_a', 0.0, 0.0, 0.0, 'map', map_file)
    store.rename(point.point_id, 'SIM_B')
    assert store.load()[0].name == 'SIM_B'
    original = store.load()[0]
    store.edit(
        point.point_id, 'SIM_C', -1.25, 3.5, -1.57,
        point_type=POINT_TYPE_WAYPOINT,
    )
    edited = store.load()[0]
    assert edited.name == 'SIM_C'
    assert (edited.x, edited.y, edited.yaw) == (-1.25, 3.5, -1.57)
    assert edited.map_sha256 == original.map_sha256
    assert edited.saved_at == original.saved_at
    assert edited.point_type == POINT_TYPE_WAYPOINT
    store.delete(point.point_id)
    assert store.load() == []


def test_legacy_point_defaults_to_target_and_waypoints_can_be_ordered(tmp_path):
    points_file = tmp_path / 'points.yaml'
    points_file.write_text(
        'version: 1\npoints:\n- id: old\n  name: 起点\n  x: 1\n  y: 2\n  yaw: 0.5\n',
        encoding='utf-8',
    )
    store = PointStore(points_file)
    assert store.load()[0].point_type == POINT_TYPE_TARGET

    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n', encoding='utf-8')
    first = store.add(
        '导航一', 2.0, 2.0, 0.0, 'map', map_file,
        point_type=POINT_TYPE_WAYPOINT,
    )
    second = store.add(
        '导航二', 3.0, 2.0, 0.0, 'map', map_file,
        point_type=POINT_TYPE_WAYPOINT,
    )
    store.move(second.point_id, -1)
    assert [point.name for point in store.load()] == ['起点', '导航二', '导航一']
    assert store.load()[1].point_type == POINT_TYPE_WAYPOINT
