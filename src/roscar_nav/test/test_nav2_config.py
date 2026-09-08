from pathlib import Path

import yaml


def test_goal_checker_latches_xy_without_relaxing_precision():
    path = Path(__file__).parents[1] / 'config' / 'nav2_params_roscar.yaml'
    goal = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'controller_server'
    ]['ros__parameters']['goal_checker']
    assert goal['stateful'] is True
    assert goal['xy_goal_tolerance'] == 0.02
    assert goal['yaw_goal_tolerance'] == 0.05


def test_cruise_goal_and_path_follow_costs_overlap_at_soft_waypoint():
    path = Path(__file__).parents[1] / 'config' / 'nav2_params_roscar.yaml'
    cruise = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'controller_server'
    ]['ros__parameters']['FollowPathCruise']
    assert cruise['GoalCritic']['threshold_to_consider'] > (
        cruise['PathFollowCritic']['threshold_to_consider']
    ) >= 0.45
