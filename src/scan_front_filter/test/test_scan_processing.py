import math
import unittest

from scan_front_filter.scan_processing import (
    OdomFarReturnGuard,
    OdomPoseBuffer,
    Pose2D,
    compose_pose,
    deskew_ranges,
    fixed_scan_grid,
    inverse_transform_point,
    select_multi_echo_ranges,
    transform_point,
)


class ScanProcessingTests(unittest.TestCase):
    def test_compact_mapping_grid_preserves_nearest_returns_and_intensity(self):
        ranges = [math.inf] * 5400
        levels = [0.0] * 5400
        for i in range(540):
            ranges[i*10] = 2.0
        ranges[3], levels[3] = 1.0, 42.0
        ranges[100], ranges[200], ranges[300] = math.nan, 0.0, -1.0
        output, intensity = fixed_scan_grid(ranges, levels, -math.pi, 2*math.pi/5400, 540)
        self.assertEqual(len(output), 540)
        self.assertEqual(sum(math.isfinite(v) for v in output), 537)
        self.assertEqual((output[0], intensity[0]), (1.0, 42.0))
        self.assertTrue(math.isinf(output[10]))

    def test_raw_and_multi_echo_keep_one_geometry_without_inventing_returns(self):
        for count in (540, 5400):
            ranges = [math.inf] * count
            ranges[0], ranges[count // 2] = 1.0, 2.0
            levels = [10.0] * count
            output, intensity = fixed_scan_grid(ranges, levels, -math.pi, 2*math.pi/count, 5400)
            self.assertEqual(len(output), 5400)
            self.assertEqual(sum(math.isfinite(v) for v in output), 2)
            self.assertEqual((output[0], output[2700]), (1.0, 2.0))
            self.assertEqual((intensity[0], intensity[2700]), (10.0, 10.0))
        output, _ = fixed_scan_grid([2.0, 1.0, math.nan], [], -math.pi, 0.00001, 5400)
        self.assertEqual(output[0], 1.0)

    def test_pose_buffer_interpolates_wrapped_yaw(self):
        poses = OdomPoseBuffer()
        poses.add(0.0, Pose2D(0.0, 0.0, math.radians(179.0)))
        poses.add(1.0, Pose2D(1.0, 0.0, math.radians(-179.0)))
        midpoint = poses.lookup(0.5, max_gap=0.6)
        self.assertAlmostEqual(midpoint.x, 0.5)
        self.assertAlmostEqual(abs(midpoint.yaw), math.pi, places=6)

    def test_deskew_reprojects_beam_to_sweep_end(self):
        poses = OdomPoseBuffer()
        for index in range(11):
            stamp = index * 0.01
            poses.add(stamp, Pose2D(stamp, 0.0, 0.0))
        count = 360
        angle_min = -math.pi
        increment = 2.0 * math.pi / count
        source_index = 180
        source_angle = angle_min + source_index * increment
        # Angle 0 is acquired at phase 0, one period before the sweep-end
        # stamp.  A 2 m hit then appears at 1.9 m in the end frame because
        # the platform translated +0.1 m during the revolution.
        ranges = [math.inf] * count
        ranges[source_index] = 2.0
        corrected, _sources, applied, _reference, failures = deskew_ranges(
            ranges,
            angle_min,
            increment,
            0.1,
            0.1,
            poses,
            Pose2D(0.0, 0.0, 0.0),
            0.1,
            10.0,
            max_odom_gap=0.02,
        )
        self.assertTrue(applied)
        self.assertEqual(failures, 0)
        self.assertAlmostEqual(min(corrected), 1.9, places=6)

    def test_multi_echo_secondary_only_requires_structure(self):
        echoes = [
            [1.0, 2.0],
            [math.inf, 2.0],
            [math.inf, 2.02],
            [math.inf, 2.01],
            [math.inf, 3.5],
        ]
        selected, indices, stats = select_multi_echo_ranges(
            echoes,
            0.1,
            5.0,
            spatial_window=2,
            min_secondary_neighbors=2,
            full_scan=False,
        )
        self.assertEqual(selected[0], 1.0)
        self.assertTrue(math.isfinite(selected[1]))
        self.assertTrue(math.isfinite(selected[2]))
        self.assertTrue(math.isfinite(selected[3]))
        self.assertTrue(math.isinf(selected[4]))
        self.assertEqual(indices[4], -1)
        self.assertEqual(stats['secondary_rejected'], 1)

    def test_multi_echo_keeps_valid_first_return(self):
        selected, indices, _stats = select_multi_echo_ranges(
            [[1.5, 0.4]],
            0.1,
            5.0,
            min_secondary_neighbors=0,
        )
        self.assertEqual(selected[0], 1.5)
        self.assertEqual(indices[0], 0)

    def test_far_return_guard_rejects_far_but_accepts_near(self):
        guard = OdomFarReturnGuard(
            history_scans=4,
            history_max_age_sec=1.0,
            angular_window_rad=0.02,
            range_tolerance=0.1,
            far_jump_delta=0.25,
            min_confirmations=2,
        )
        pose = Pose2D(0.0, 0.0, 0.0)
        angle_min = -math.pi
        increment = 2.0 * math.pi / 360
        near_scan = [math.inf] * 360
        near_scan[180] = 1.0
        guard.add_scan(near_scan, angle_min, increment, 0.0, pose)
        guard.add_scan(near_scan, angle_min, increment, 0.1, pose)

        far_scan = [math.inf] * 360
        far_scan[180] = 2.0
        filtered, rejected = guard.filter(
            far_scan,
            angle_min,
            increment,
            0.2,
            pose,
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 1)
        self.assertTrue(math.isinf(filtered[180]))

        closer_scan = [math.inf] * 360
        closer_scan[180] = 0.8
        filtered, rejected = guard.filter(
            closer_scan,
            angle_min,
            increment,
            0.2,
            pose,
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 0)
        self.assertEqual(filtered[180], 0.8)

    def test_far_guard_does_not_cross_wall_edge_when_exact_beam_exists(self):
        guard = OdomFarReturnGuard(
            history_scans=4,
            history_max_age_sec=1.0,
            angular_window_rad=0.04,
            range_tolerance=0.1,
            far_jump_delta=0.25,
            min_confirmations=2,
        )
        pose = Pose2D(0.0, 0.0, 0.0)
        angle_min = -math.pi
        increment = 2.0 * math.pi / 360
        stable_scan = [math.inf] * 360
        stable_scan[179] = 0.5
        stable_scan[180] = 2.0
        guard.add_scan(stable_scan, angle_min, increment, 0.0, pose)
        guard.add_scan(stable_scan, angle_min, increment, 0.1, pose)
        filtered, rejected = guard.filter(
            stable_scan,
            angle_min,
            increment,
            0.2,
            pose,
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 0)
        self.assertEqual(filtered[180], 2.0)

    def test_far_guard_keeps_confirmed_wall_during_slow_robot_pass(self):
        guard = OdomFarReturnGuard(
            history_scans=50,
            history_max_age_sec=5.0,
            angular_window_rad=math.radians(1.5),
            range_tolerance=0.20,
            far_jump_delta=0.25,
            min_confirmations=2,
        )
        angle_min = -math.pi
        increment = 2.0 * math.pi / 360
        near_scan = [math.inf] * 360
        near_scan[180] = 1.0
        guard.add_scan(
            near_scan,
            angle_min,
            increment,
            0.0,
            Pose2D(0.0, 0.0, 0.0),
        )
        guard.add_scan(
            near_scan,
            angle_min,
            increment,
            0.1,
            Pose2D(0.0, 0.0, 0.0),
        )

        # After the robot advances 0.20 m, the same world wall projects to
        # 0.80 m.  A spurious 2.0 m return would ray-trace through it.
        far_scan = [math.inf] * 360
        far_scan[180] = 2.0
        filtered, rejected = guard.filter(
            far_scan,
            angle_min,
            increment,
            3.5,
            Pose2D(0.20, 0.0, 0.0),
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 1)
        self.assertTrue(math.isinf(filtered[180]))

    def test_far_guard_remains_bounded_after_configured_age(self):
        guard = OdomFarReturnGuard(
            history_scans=50,
            history_max_age_sec=5.0,
            angular_window_rad=math.radians(1.5),
            range_tolerance=0.20,
            far_jump_delta=0.25,
            min_confirmations=2,
        )
        angle_min = -math.pi
        increment = 2.0 * math.pi / 360
        near_scan = [math.inf] * 360
        near_scan[180] = 1.0
        pose = Pose2D(0.0, 0.0, 0.0)
        guard.add_scan(near_scan, angle_min, increment, 0.0, pose)
        guard.add_scan(near_scan, angle_min, increment, 0.1, pose)

        far_scan = [math.inf] * 360
        far_scan[180] = 2.0
        filtered, rejected = guard.filter(
            far_scan,
            angle_min,
            increment,
            5.2,
            pose,
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 0)
        self.assertEqual(filtered[180], 2.0)

    def test_far_guard_clears_old_epoch_on_stamp_rollback(self):
        guard = OdomFarReturnGuard(
            history_scans=10,
            history_max_age_sec=5.0,
            angular_window_rad=0.02,
            range_tolerance=0.1,
            far_jump_delta=0.25,
            min_confirmations=2,
        )
        angle_min = -math.pi
        increment = 2.0 * math.pi / 360
        near_scan = [math.inf] * 360
        near_scan[180] = 1.0
        pose = Pose2D(0.0, 0.0, 0.0)
        guard.add_scan(near_scan, angle_min, increment, 100.0, pose)
        guard.add_scan(near_scan, angle_min, increment, 100.1, pose)

        far_scan = [math.inf] * 360
        far_scan[180] = 2.0
        filtered, rejected = guard.filter(
            far_scan,
            angle_min,
            increment,
            1.0,
            pose,
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 0)
        self.assertEqual(filtered[180], 2.0)

    def test_far_guard_duplicate_stamp_is_not_a_confirmation(self):
        guard = OdomFarReturnGuard(
            history_scans=10,
            history_max_age_sec=5.0,
            angular_window_rad=0.02,
            range_tolerance=0.1,
            far_jump_delta=0.25,
            min_confirmations=2,
        )
        angle_min = -math.pi
        increment = 2.0 * math.pi / 360
        near_scan = [math.inf] * 360
        near_scan[180] = 1.0
        pose = Pose2D(0.0, 0.0, 0.0)
        guard.add_scan(near_scan, angle_min, increment, 0.0, pose)
        guard.add_scan(near_scan, angle_min, increment, 0.0, pose)

        far_scan = [math.inf] * 360
        far_scan[180] = 2.0
        filtered, rejected = guard.filter(
            far_scan,
            angle_min,
            increment,
            0.1,
            pose,
            0.1,
            5.0,
        )
        self.assertEqual(rejected, 0)
        self.assertEqual(filtered[180], 2.0)

    def test_pose_transforms_round_trip(self):
        pose = compose_pose(
            Pose2D(1.0, 2.0, 0.4),
            Pose2D(-0.235, 0.0, 0.092382808),
        )
        world = transform_point(pose, 0.8, -0.2)
        local = inverse_transform_point(pose, *world)
        self.assertAlmostEqual(local[0], 0.8, places=9)
        self.assertAlmostEqual(local[1], -0.2, places=9)


if __name__ == '__main__':
    unittest.main()
