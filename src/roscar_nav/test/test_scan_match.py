import math
import sys
from pathlib import Path

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roscar_nav.scan_match import (  # noqa: E402
    bilinear_distances,
    build_subcell_distance_field,
    confidence_is_high,
    distance_to_occupied,
    distinct_pose_score_gap,
    frame_consistency,
    legacy_score_batch,
    refine_seed,
    wrap_angle,
)


def test_map_distance_field_matches_brute_force():
    image = np.full((5, 7), 255, dtype=np.uint8)
    image[0, 6] = image[2, 3] = image[4, 0] = 0
    grid = np.indices(image.shape).reshape(2, -1).T
    occupied = np.argwhere(image < 80)
    expected = np.sqrt(((grid[:, None] - occupied) ** 2).sum(axis=2).min(axis=1)) * 0.05
    actual = distance_to_occupied(image, 0.05)
    np.testing.assert_allclose(actual, expected.reshape(image.shape), rtol=1e-6, atol=1e-8)


def test_distance_fields_reject_missing_walls_and_invalid_resolution():
    for builder in (
        lambda image, resolution: distance_to_occupied(image, resolution),
        lambda image, resolution: build_subcell_distance_field(image, resolution, 0.0, 0.0),
    ):
        with pytest.raises(ValueError, match='no occupied'):
            builder(np.full((2, 2), 255, dtype=np.uint8), 0.05)
        for resolution in (0.0, -0.05, math.inf, math.nan):
            with pytest.raises(ValueError, match='finite and positive'):
                builder(np.zeros((2, 2), dtype=np.uint8), resolution)


def test_global_score_retains_default_kernel_and_allows_wider_candidates():
    arguments = (
        np.array([[0.5, 0.5]], dtype=np.float32), 0.0,
        np.array([[0.0, 0.0]], dtype=np.float32),
        np.full((2, 2), 0.1, dtype=np.float32), 0.0, 0.0, 1.0,
    )
    assert legacy_score_batch(*arguments)[0] == pytest.approx(math.exp(-1.0))
    assert legacy_score_batch(*arguments, sigma=0.25)[0] > legacy_score_batch(*arguments)[0]


@pytest.mark.parametrize('fine_resolution', [0.005, 0.010, 0.013])
@pytest.mark.parametrize('origin', [(0.0, 0.0), (-0.137, 0.219), (123.456, -987.654)])
def test_subcell_distance_field_preserves_occupied_centres(fine_resolution, origin):
    image = np.full((3, 4), 255, dtype=np.uint8)
    image[0, 0] = image[1, 2] = image[2, 3] = 0
    resolution = 0.05
    field, actual_resolution = build_subcell_distance_field(
        image, resolution, *origin, fine_resolution=fine_resolution,
    )
    ys, xs = np.indices(field.shape, dtype=np.float64) * fine_resolution
    xs += origin[0]
    ys += origin[1]
    occupied = np.argwhere(image < 80)
    ox = origin[0] + (occupied[:, 1] + 0.5) * resolution
    oy = origin[1] + (image.shape[0] - occupied[:, 0] - 0.5) * resolution
    expected = np.sqrt(np.min(
        (xs[..., None] - ox) ** 2 + (ys[..., None] - oy) ** 2, axis=2,
    ))
    assert actual_resolution == fine_resolution
    np.testing.assert_allclose(field, expected, rtol=1e-6, atol=1e-8)
    if fine_resolution == 0.005:
        assert field[25, 5] == 0.0  # First occupied cell centre, not its corner.


def _synthetic_map_and_frames(true_pose=None):
    image = np.full((70, 80), 255, dtype=np.uint8)
    image[3, 3:72] = 0
    image[65, 8:77] = 0
    image[3:66, 3] = 0
    image[12:66, 76] = 0
    image[12:52, 23] = 0
    image[51, 23:60] = 0
    image[20:45, 59] = 0
    resolution = 0.05
    origin_x, origin_y = -0.4, -0.3
    field, fine_resolution = build_subcell_distance_field(
        image,
        resolution,
        origin_x,
        origin_y,
        fine_resolution=0.005,
    )

    occupied_rc = np.argwhere(image < 80)
    occupied_world = np.column_stack(
        (
            origin_x + (occupied_rc[:, 1] + 0.5) * resolution,
            origin_y + (image.shape[0] - occupied_rc[:, 0] - 0.5) * resolution,
        )
    )
    true_pose = np.array(
        [1.137, 1.263, math.radians(7.4)] if true_pose is None else true_pose,
        dtype=float,
    )
    cosine, sine = math.cos(true_pose[2]), math.sin(true_pose[2])
    delta = occupied_world - true_pose[:2]
    base_points = np.column_stack(
        (
            cosine * delta[:, 0] + sine * delta[:, 1],
            -sine * delta[:, 0] + cosine * delta[:, 1],
        )
    )
    ranges = np.hypot(base_points[:, 0], base_points[:, 1])
    base_points = base_points[(ranges > 0.20) & (ranges < 3.2)]
    indices = np.linspace(0, len(base_points) - 1, 180).astype(int)
    base_points = base_points[indices]

    frames = []
    for frame_index in range(5):
        phase = np.arange(len(base_points), dtype=float) + frame_index * 0.7
        noise = np.column_stack(
            (0.0007 * np.sin(phase), 0.0007 * np.cos(phase * 0.8))
        )
        frame = base_points + noise
        # Simulate a moving foreground object in fewer than 20% of the beams.
        frame[-24:, 0] = 0.45 + frame_index * 0.01
        frame[-24:, 1] = np.linspace(-0.25, 0.25, 24)
        frames.append(frame.astype(np.float32))
    return field, fine_resolution, origin_x, origin_y, true_pose, frames


def test_bilinear_distance_sampling_is_subcell_continuous():
    field = np.array([[0.0, 1.0], [2.0, 3.0]], dtype=np.float32)
    distances, valid = bilinear_distances(
        field,
        np.array([0.25]),
        np.array([0.50]),
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
    )
    assert valid.tolist() == [True]
    assert distances[0] == pytest.approx(1.25, abs=1e-6)


@pytest.mark.parametrize('true_pose, bias', [
    ((1.137, 1.263, 7.4), (0.047, -0.041, 2.1)),
    ((1.131, 1.259, -4.35), (0.103, 0.097, -4.8)),
    ((1.141, 1.269, 12.05), (-0.111, 0.031, 4.35)),
])
def test_five_frame_refinement_recovers_centimetre_biased_seed(true_pose, bias):
    field, fine_resolution, origin_x, origin_y, true_pose, frames = (
        _synthetic_map_and_frames((*true_pose[:2], math.radians(true_pose[2])))
    )
    seed = (
        0.0,
        true_pose[0] + bias[0],
        true_pose[1] + bias[1],
        true_pose[2] + math.radians(bias[2]),
    )
    pose = refine_seed(
        seed,
        frames,
        field,
        origin_x,
        origin_y,
        fine_resolution,
    )
    translation_error = math.hypot(pose[1] - true_pose[0], pose[2] - true_pose[1])
    yaw_error = abs(math.degrees(wrap_angle(pose[3] - true_pose[2])))
    assert translation_error <= 0.006
    assert yaw_error <= 0.20

    consistency = frame_consistency(
        pose,
        frames,
        field,
        origin_x,
        origin_y,
        fine_resolution,
    )
    confident, detail = confidence_is_high(pose, consistency)
    assert confident, detail


def test_confidence_gate_rejects_frame_disagreement():
    pose = (0.91, 1.0, 2.0, 0.1)
    consistency = {
        'frame_scores': [0.9, 0.88, 0.91, 0.87, 0.89],
        'translation_max_m': 0.031,
        'translation_span_m': 0.031,
        'yaw_max_abs_deg': 0.2,
        'yaw_span_deg': 0.2,
    }
    confident, detail = confidence_is_high(pose, consistency)
    assert not confident
    assert 'translation disagreement' in detail


@pytest.mark.parametrize('perturbation', ['translation', 'yaw'])
def test_real_frame_consistency_search_can_exceed_production_gate(perturbation):
    field, fine_resolution, origin_x, origin_y, true_pose, frames = (
        _synthetic_map_and_frames()
    )
    pose = (0.95, true_pose[0], true_pose[1], true_pose[2])
    shifted = list(frames)
    if perturbation == 'translation':
        shifted[-1] = shifted[-1] + np.array([0.024, 0.0], dtype=np.float32)
    else:
        angle = math.radians(1.2)
        rotation = np.array([
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ], dtype=np.float32)
        shifted[-1] = shifted[-1] @ rotation.T
    consistency = frame_consistency(
        pose,
        shifted,
        field,
        origin_x,
        origin_y,
        fine_resolution,
    )
    confident, detail = confidence_is_high(
        pose,
        consistency,
        min_score=0.0,
        min_frame_score=0.0,
    )
    assert not confident
    assert (
        consistency['translation_max_m'] > 0.010
        or consistency['translation_span_m'] > 0.015
        or consistency['yaw_max_abs_deg'] > 0.8
        or consistency['yaw_span_deg'] > 1.0
    )
    assert perturbation in detail


def test_distinct_pose_gap_ignores_same_basin_and_finds_alternative():
    candidates = [
        (0.95, 1.0, 2.0, 0.10),
        (0.949, 1.002, 2.001, 0.11),
        (0.91, 1.20, 2.0, 0.10),
    ]
    assert distinct_pose_score_gap(candidates) == pytest.approx(0.04)
