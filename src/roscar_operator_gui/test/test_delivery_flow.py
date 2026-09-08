from types import SimpleNamespace

import pytest

from roscar_operator_gui.models import POINT_TYPE_TARGET, POINT_TYPE_WAYPOINT
from roscar_operator_gui.operator_gui import (
    DELIVERY_PICKUPS,
    DELIVERY_PLACE,
    OperatorWindow,
    resolve_delivery_targets,
)


def point(name, point_type=POINT_TYPE_TARGET):
    return SimpleNamespace(name=name, point_type=point_type)


def test_delivery_requires_exact_targets_and_orders_stops():
    points = [point('起点'), point('102'), point('V1', POINT_TYPE_WAYPOINT), point('101')]
    assert [value.name for value in resolve_delivery_targets(points)] == [
        '101', '102', '起点',
    ]
    with pytest.raises(ValueError, match='102'):
        resolve_delivery_targets([point('起点'), point('101')])


def test_intentional_gemini_stop_is_not_treated_as_delivery_failure():
    window = SimpleNamespace(_alignment_stopping=True)
    OperatorWindow._gemini_error(window, None)


def test_delivery_uses_burned_pickup_and_place_groups():
    assert DELIVERY_PICKUPS['101'][:2] == (8, 13)
    assert DELIVERY_PICKUPS['102'][:2] == (3, 7)
    assert DELIVERY_PLACE[:2] == (14, 17)
