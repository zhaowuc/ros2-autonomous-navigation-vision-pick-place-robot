"""Pure geometry used by the global-costmap-only scan filter.

This module deliberately has no ROS imports so the suppression rule can be
unit-tested without a running graph.  Occupied cells are treated as closed
axis-aligned squares in the occupancy grid's own (possibly rotated) frame.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Transform2D:
    """Rigid transform from a LaserScan frame into the map frame."""

    x: float
    y: float
    yaw: float


class OccupiedGridIndex:
    """Small-radius occupied-cell query for a static OccupancyGrid."""

    def __init__(
        self,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        origin_yaw,
        occupancy_data,
        occupied_threshold=65,
        proximity=0.11,
    ):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self.proximity = max(0.0, float(proximity))
        self.occupied_threshold = int(occupied_threshold)
        if self.width <= 0 or self.height <= 0:
            raise ValueError('occupancy grid dimensions must be positive')
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError('occupancy grid resolution must be positive')
        expected_size = self.width * self.height
        if len(occupancy_data) != expected_size:
            raise ValueError(
                f'occupancy data has {len(occupancy_data)} cells, '
                f'expected {expected_size}'
            )
        # Unknown (-1) cells are intentionally not static walls.  Suppressing
        # returns near unknown space could hide a real, previously unseen
        # obstacle from the global planner.
        self.occupied = frozenset(
            index
            for index, value in enumerate(occupancy_data)
            if int(value) >= self.occupied_threshold
        )
        self._origin_cos = math.cos(self.origin_yaw)
        self._origin_sin = math.sin(self.origin_yaw)
        self._search_cells = int(
            math.ceil(self.proximity / self.resolution)
        ) + 1
        # The live scan can contain thousands of angular bins.  Build the
        # tiny neighbourhood lookup once per map update instead of testing an
        # O(radius^2) window for every beam at lidar rate.
        candidate_cells = {}
        for occupied_index in self.occupied:
            occupied_x = occupied_index % self.width
            occupied_y = occupied_index // self.width
            for query_y in range(
                max(0, occupied_y - self._search_cells),
                min(self.height, occupied_y + self._search_cells + 1),
            ):
                row = query_y * self.width
                for query_x in range(
                    max(0, occupied_x - self._search_cells),
                    min(self.width, occupied_x + self._search_cells + 1),
                ):
                    candidate_cells.setdefault(row + query_x, []).append(
                        (occupied_x, occupied_y)
                    )
        self._candidate_cells = {
            index: tuple(cells)
            for index, cells in candidate_cells.items()
        }

    def _map_to_grid_coordinates(self, map_x, map_y):
        delta_x = float(map_x) - self.origin_x
        delta_y = float(map_y) - self.origin_y
        # Inverse of the OccupancyGrid origin pose.
        local_x = self._origin_cos * delta_x + self._origin_sin * delta_y
        local_y = -self._origin_sin * delta_x + self._origin_cos * delta_y
        return local_x, local_y

    def is_near_occupied(self, map_x, map_y):
        """Return true if a point is within proximity of an occupied cell.

        Distance is measured to each occupied cell's square, not merely its
        centre.  The configured 0.11 m therefore means 0.11 m beyond the
        mapped cell boundary and is independent of grid resolution.
        """

        local_x, local_y = self._map_to_grid_coordinates(map_x, map_y)
        centre_x = math.floor(local_x / self.resolution)
        centre_y = math.floor(local_y / self.resolution)
        proximity_squared = self.proximity * self.proximity
        if (
            centre_x < 0
            or centre_x >= self.width
            or centre_y < 0
            or centre_y >= self.height
        ):
            return False
        query_index = centre_y * self.width + centre_x
        for grid_x, grid_y in self._candidate_cells.get(query_index, ()):
            cell_min_y = grid_y * self.resolution
            cell_max_y = cell_min_y + self.resolution
            delta_y = max(
                cell_min_y - local_y,
                0.0,
                local_y - cell_max_y,
            )
            if delta_y > self.proximity:
                continue
            cell_min_x = grid_x * self.resolution
            cell_max_x = cell_min_x + self.resolution
            delta_x = max(
                cell_min_x - local_x,
                0.0,
                local_x - cell_max_x,
            )
            if delta_x * delta_x + delta_y * delta_y <= proximity_squared:
                return True
        return False


def suppress_static_wall_endpoints(
    ranges,
    angle_min,
    angle_increment,
    map_from_scan,
    occupied_grid,
):
    """Set only static-wall-near finite endpoints to infinity.

    The result is intended for a *marking-only* global-costmap observation
    source.  It must not replace the original scan used for raytrace clearing.
    """

    output = list(ranges)
    transform_cos = math.cos(map_from_scan.yaw)
    transform_sin = math.sin(map_from_scan.yaw)
    suppressed = 0
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance < 0.0:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        scan_x = distance * math.cos(angle)
        scan_y = distance * math.sin(angle)
        map_x = (
            map_from_scan.x
            + transform_cos * scan_x
            - transform_sin * scan_y
        )
        map_y = (
            map_from_scan.y
            + transform_sin * scan_x
            + transform_cos * scan_y
        )
        if occupied_grid.is_near_occupied(map_x, map_y):
            output[index] = math.inf
            suppressed += 1
    return output, suppressed
