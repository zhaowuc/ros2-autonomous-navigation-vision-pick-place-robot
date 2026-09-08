from roscar_navigation_mode_manager.mode_manager import (
    CRUISE_CONTROLLER,
    TERMINAL_CONTROLLER,
    controller_for_distance,
)


def test_cruise_outside_terminal_window():
    assert controller_for_distance(0.41, 0.40, False) == CRUISE_CONTROLLER


def test_terminal_at_boundary():
    assert controller_for_distance(0.40, 0.40, False) == TERMINAL_CONTROLLER


def test_terminal_is_latched():
    assert controller_for_distance(2.0, 0.40, True) == TERMINAL_CONTROLLER
