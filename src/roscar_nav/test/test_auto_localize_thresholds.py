from roscar_nav.auto_localize import stage_score_passes


def test_coarse_threshold_does_not_use_refined_threshold():
    assert stage_score_passes('coarse', 0.82, 0.80, 0.95)


def test_refined_threshold_does_not_use_coarse_threshold():
    assert stage_score_passes('refined', 0.82, 0.95, 0.80)


def test_each_stage_can_reject_independently():
    assert not stage_score_passes('coarse', 0.79, 0.80, 0.70)
    assert not stage_score_passes('refined', 0.79, 0.70, 0.80)
