import math
import unittest

from scan_front_filter.map_wall_filter import (
    OccupiedGridIndex,
    Transform2D,
    suppress_static_wall_endpoints,
)


def make_grid(
    occupied_cells,
    width=60,
    height=40,
    resolution=0.05,
    origin=(0.0, 0.0, 0.0),
    proximity=0.11,
):
    data = [0] * (width * height)
    for grid_x, grid_y, value in occupied_cells:
        data[grid_y * width + grid_x] = value
    return OccupiedGridIndex(
        width,
        height,
        resolution,
        origin[0],
        origin[1],
        origin[2],
        data,
        proximity=proximity,
    )


class MapWallFilterTests(unittest.TestCase):
    def test_mapped_wall_endpoint_is_suppressed(self):
        # Cell x=[2.00, 2.05], y=[1.00, 1.05].  Endpoint is 0.10 m before it.
        grid = make_grid([(40, 20, 100)])
        filtered, suppressed = suppress_static_wall_endpoints(
            [1.90],
            0.0,
            1.0,
            Transform2D(0.0, 1.025, 0.0),
            grid,
        )
        self.assertEqual(suppressed, 1)
        self.assertTrue(math.isinf(filtered[0]))

    def test_dynamic_obstacle_farther_than_threshold_is_retained(self):
        # The mapped wall begins at x=2.00, while a live object at x=1.80 is
        # 0.20 m away.  It must remain available to global replanning.
        grid = make_grid([(40, 20, 100)])
        filtered, suppressed = suppress_static_wall_endpoints(
            [1.80],
            0.0,
            1.0,
            Transform2D(0.0, 1.025, 0.0),
            grid,
        )
        self.assertEqual(suppressed, 0)
        self.assertEqual(filtered[0], 1.80)

    def test_unknown_cell_does_not_hide_live_obstacle(self):
        grid = make_grid([(40, 20, -1)])
        filtered, suppressed = suppress_static_wall_endpoints(
            [2.025],
            0.0,
            1.0,
            Transform2D(0.0, 1.025, 0.0),
            grid,
        )
        self.assertEqual(suppressed, 0)
        self.assertEqual(filtered[0], 2.025)

    def test_map_origin_rotation_is_respected(self):
        # In grid coordinates the occupied cell is centred at (1.025, 0.525).
        # A +90 degree origin rotation places that centre at (-0.525, 1.025).
        grid = make_grid(
            [(20, 10, 100)],
            origin=(0.0, 0.0, math.pi / 2.0),
        )
        filtered, suppressed = suppress_static_wall_endpoints(
            [math.hypot(0.525, 1.025)],
            math.atan2(1.025, -0.525),
            1.0,
            Transform2D(0.0, 0.0, 0.0),
            grid,
        )
        self.assertEqual(suppressed, 1)
        self.assertTrue(math.isinf(filtered[0]))

    def test_scan_transform_is_respected(self):
        grid = make_grid([(40, 20, 100)])
        filtered, suppressed = suppress_static_wall_endpoints(
            [1.0],
            0.0,
            1.0,
            Transform2D(2.025, 0.025, math.pi / 2.0),
            grid,
        )
        self.assertEqual(suppressed, 1)
        self.assertTrue(math.isinf(filtered[0]))

    def test_nonfinite_ranges_are_preserved(self):
        grid = make_grid([(40, 20, 100)])
        filtered, suppressed = suppress_static_wall_endpoints(
            [math.inf, math.nan],
            0.0,
            0.1,
            Transform2D(0.0, 0.0, 0.0),
            grid,
        )
        self.assertEqual(suppressed, 0)
        self.assertTrue(math.isinf(filtered[0]))
        self.assertTrue(math.isnan(filtered[1]))


if __name__ == '__main__':
    unittest.main()
