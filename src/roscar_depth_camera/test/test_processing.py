import numpy as np
import pytest

from roscar_depth_camera.processing import (
    detect_red_target,
    depth_to_meters,
    depth_to_palette_indices,
    filter_and_voxelize,
    stepped_alignment_velocity,
    visual_alignment_velocity,
)


def test_rep118_uint16_depth_is_millimetres():
    depth = np.array([[0, 600, 4000]], dtype=np.uint16)
    result = depth_to_meters(depth, '16UC1')
    np.testing.assert_allclose(result, [[0.0, 0.6, 4.0]], atol=1e-6)


def test_rep118_float_depth_is_metres():
    depth = np.array([[np.nan, 0.6, 4.0]], dtype=np.float32)
    result = depth_to_meters(depth, '32FC1')
    assert np.isnan(result[0, 0])
    np.testing.assert_allclose(result[0, 1:], [0.6, 4.0], atol=1e-6)


def test_fixed_palette_uses_full_256_indices_and_masks_invalid():
    depth = np.array([[0.6, 2.3, 4.0, 0.0, np.nan]], dtype=np.float32)
    indices, valid = depth_to_palette_indices(depth, 0.6, 4.0)
    assert indices.tolist() == [[255, 128, 0, 0, 0]]
    assert valid.tolist() == [[True, True, True, False, False]]


def test_palette_rejects_invalid_range():
    with pytest.raises(ValueError):
        depth_to_palette_indices(np.ones((1, 1), dtype=np.float32), 1.0, 1.0)


def test_voxel_filter_keeps_range_and_one_point_per_voxel():
    points = np.array([
        [0.0, 0.0, 0.10],
        [0.0, 0.0, 0.60],
        [0.01, 0.01, 0.61],
        [0.0, 0.0, 2.70],
        [0.0, 0.0, 3.00],
        [np.nan, 0.0, 1.0],
    ], dtype=np.float32)
    result = filter_and_voxelize(points, 0.55, 2.80, 0.03)
    assert result.shape == (2, 3)
    np.testing.assert_allclose(result[:, 2], [0.60, 2.70], atol=1e-6)


def test_red_target_survives_brightness_change_and_alignment_is_bounded():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    image[60:210, 55:275] = (45, 65, 155)
    normal = detect_red_target(image)
    dark = detect_red_target((image.astype(np.float32) * 0.55).astype(np.uint8))
    assert normal is not None and dark is not None
    assert abs(normal['center_x'] - dark['center_x']) < 0.01
    assert abs(normal['center_y'] - dark['center_y']) < 0.01
    assert abs(normal['area_ratio'] - dark['area_ratio']) < 0.01
    vx, vy, aligned = visual_alignment_velocity(0.80, 0.72, 0.50, 0.50)
    assert vx == pytest.approx(-0.0484)
    assert vy == -0.06
    assert not aligned
    assert visual_alignment_velocity(0.51, 0.51, 0.50, 0.50)[2]


def test_visual_alignment_overcomes_static_friction_without_moving_aligned_axis():
    vx, vy, aligned = visual_alignment_velocity(0.526, 0.50, 0.50, 0.50)
    assert not aligned
    assert vx == 0.0
    assert vy == -0.05

    vx, vy, aligned = visual_alignment_velocity(0.50, 0.464, 0.50, 0.50)
    assert not aligned
    assert vx == pytest.approx(0.04)
    assert vy == 0.0


def test_visual_alignment_moves_in_steps_and_stops_before_reversing():
    command, move_until, settle_until = stepped_alignment_velocity(
        10.0, (0.0, 0.05), (0.0, 0.0), 0.0, 0.0
    )
    assert command == (0.0, 0.05)
    assert move_until == pytest.approx(10.30)
    assert settle_until == pytest.approx(10.75)

    command, move_until, settle_until = stepped_alignment_velocity(
        10.1, (0.0, -0.05), (0.0, 0.05), move_until, settle_until
    )
    assert command == (0.0, 0.0)
    assert move_until == 0.0
    assert settle_until == pytest.approx(10.55)

    command, _, _ = stepped_alignment_velocity(
        10.2, (0.0, -0.05), (0.0, 0.05), move_until, settle_until
    )
    assert command == (0.0, 0.0)
