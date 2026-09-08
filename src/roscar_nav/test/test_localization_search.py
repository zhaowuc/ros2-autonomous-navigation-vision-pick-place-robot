import math
import time
from types import SimpleNamespace

import numpy as np
import pytest
import roscar_nav.auto_localize as auto_localize

from roscar_nav.auto_localize import AutoLocalize, pose_within_prior, search_pose_seeds
from roscar_nav.scan_match import distance_to_occupied, wrap_angle


def asymmetric_scene():
    image = np.full((70, 80), 255, dtype=np.uint8)
    image[3, 3:72] = 0
    image[65, 8:77] = 0
    image[3:66, 3] = 0
    image[12:66, 76] = 0
    image[12:52, 23] = 0
    image[51, 23:60] = 0
    image[20:45, 59] = 0
    rc = np.argwhere(image < 80)[::2]
    world = np.column_stack(((rc[:, 1] + 0.5) * .05, (69.5 - rc[:, 0]) * .05))
    pose = (1.537, 1.563, math.radians(7.4))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    points = (world - pose[:2]) @ np.array([[c, -s], [s, c]])
    return image, points.astype(np.float32), pose


@pytest.mark.parametrize('local', [False, True])
def test_hierarchical_search_keeps_a_correct_seed(local):
    image, points, pose = asymmetric_scene()
    prior = (*pose[:2], pose[2] + .1, .75, 30.) if local else ()
    seeds = search_pose_seeds(
        image, distance_to_occupied(image, .05), points, .05, (0., 0.),
        time.monotonic() + 15., prior,
    )
    best = seeds[0]
    assert best[0] >= .9
    assert math.hypot(best[1] - pose[0], best[2] - pose[1]) < .08
    assert abs(wrap_angle(best[3] - pose[2])) < math.radians(3.)


def test_search_deadline_and_bad_prior_fail_closed():
    image, points, _ = asymmetric_scene()
    args = (image, distance_to_occupied(image, .05), points, .05, (0., 0.))
    with pytest.raises(TimeoutError):
        search_pose_seeds(*args, time.monotonic() - 1.)
    with pytest.raises(ValueError):
        search_pose_seeds(*args, time.monotonic() + 10., [float('nan')] * 5)
    with pytest.raises(ValueError, match='no free candidate'):
        search_pose_seeds(*args, time.monotonic() + 10., [100., 100., 0., .5, 30.])


def test_local_fine_search_and_final_acceptance_remain_inside_prior():
    image, points, pose = asymmetric_scene()
    prior = (pose[0] + .15, pose[1], pose[2], .10, 3.)
    seeds = search_pose_seeds(
        image, distance_to_occupied(image, .05), points, .05, (0., 0.),
        time.monotonic() + 15., prior,
    )
    assert seeds
    assert all(pose_within_prior(candidate, prior) for candidate in seeds)
    assert not pose_within_prior((1., *pose), prior)


def test_refresh_discards_queued_scans_from_before_validation():
    node = SimpleNamespace(
        capture_after_ns=20_000_000_000, last_scan_stamp_ns=None,
        scan_stamps=set(), scans=[], scan=None,
        get_parameter=lambda name: SimpleNamespace(value={'target_frames': 2, 'min_frame_interval_sec': .2}[name]),
    )
    def scan(sec):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)))
    AutoLocalize.scan_callback(node, scan(19))
    assert not node.scans
    AutoLocalize.scan_callback(node, scan(20))
    AutoLocalize.scan_callback(node, scan(20))
    AutoLocalize.scan_callback(node, scan(21))
    assert len(node.scans) == 2
    assert node.last_scan_stamp_ns == 21_000_000_000


def test_fresh_observations_cannot_reuse_an_old_ambiguity_gap(monkeypatch):
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000)),
        scan_stamps=set(), scans=[], get_parameter=lambda _: SimpleNamespace(value=1),
        scan_points=lambda scan: scan, match_field=(np.zeros((2, 2)), 0., 0., 1.),
        refined_candidates=[(.99, 1., 1., 0.), (.91, 2., 2., 0.)], distinct_score_gap=.08,
        check_frame_confidence=lambda *args: None,
    )
    def collect(timeout_sec):
        node.scans.append(np.ones((2, 2)))
        return True
    node.wait_for_scan = collect
    monkeypatch.setattr(auto_localize, 'robust_score_batch', lambda *args: np.array([.95]))
    with pytest.raises(ValueError, match='different localization candidate'):
        AutoLocalize.refresh_observation(node, (.99, 1., 1., 0.))
