# Global-costmap static-wall residual filter

## Scope

`/scan_slam_filtered` is unchanged and remains the localization, RViz, local
costmap, and Collision Monitor scan.  Navigation starts one additional node:

- input: `/scan_slam_filtered`
- static reference: `/map`
- marking-only output: `/scan_global_obstacles`

A finite scan endpoint is changed to positive infinity only when its map-frame
position is within 0.11 m of a static occupied-cell boundary.  Occupancy values
below 65 and unknown cells are not walls.  If `/map` or the time-correct TF is
unavailable, the node relays the scan unchanged (fail-open), so it cannot hide
a real obstacle.

## Humble marking/clearing safety

Do **not** use `/scan_global_obstacles` as the only source with both `marking`
and `clearing` enabled.  In Nav2 Humble's ObstacleLayer, marking observations
and clearing observations are buffered independently.  Clearing ray-traces to
the projected points; with the default `inf_is_valid: false`, a beam changed to
infinity has no point and therefore no clearing ray.  A dynamic obstacle that
moves away in front of a mapped wall could then remain stuck in the obstacle
layer.  Setting `inf_is_valid: true` is not equivalent: Humble replaces positive
infinity by `range_max`, which can clear through and behind the known wall.

The global obstacle layer therefore uses two sources:

1. `/scan_global_obstacles`: `marking: true`, `clearing: false`.
2. `/scan_slam_filtered`: `marking: false`, `clearing: true`.

The second source preserves the sensor's real measured endpoint, so a wall
revealed after an object moves clears the object's old cells only along the
observed free ray.  The static layer precedes the obstacle layer and maximum
combination keeps the fixed-map wall in the master costmap.

The relevant Humble implementation is in Nav2's
[ObstacleLayer 1.1.20 source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/obstacle_layer.cpp),
and the public parameter semantics are documented in
[Obstacle Layer Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html).

## On-robot acceptance checks

Before motion, confirm a single publisher on `/scan_global_obstacles`, a fresh
rate near the source scan rate, and diagnostics with no sustained TF fail-open.
In RViz/costmap snapshots verify:

- wall-coincident endpoints disappear only from the new global marking topic;
- a person/object more than 0.11 m from a wall remains marked globally;
- after that object leaves, its cost clears when the original scan sees through
  to the wall;
- `/scan_slam_filtered`, AMCL, local avoidance, and Collision Monitor are
  unchanged.

Automated geometry tests cover wall suppression, dynamic-object retention,
unknown cells, rotated map origins, scan transforms, and non-finite ranges.
