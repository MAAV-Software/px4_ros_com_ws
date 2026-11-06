"""Unit tests for the pre-arm checks."""
from types import SimpleNamespace

import pytest

from milestone1_singlevehicle.preflight import arm_blocker, position_problem

FIRST = (0.0, 0.0, -5.0)


def pos(x=0.0, y=0.0, xy_valid=True, z_valid=True):
    return SimpleNamespace(x=x, y=y, xy_valid=xy_valid, z_valid=z_valid)


def test_no_position_blocks():
    assert 'no vehicle_local_position' in position_problem(None)
    assert arm_blocker(None, FIRST)


@pytest.mark.parametrize('kwargs', [{'xy_valid': False}, {'z_valid': False}])
def test_invalid_estimate_blocks(kwargs):
    assert 'valid' in arm_blocker(pos(**kwargs), FIRST)


def test_nan_position_blocks():
    assert 'finite' in arm_blocker(pos(x=float('nan')), FIRST)


def test_at_origin_is_clear():
    assert arm_blocker(pos(), FIRST) == ''
    assert position_problem(pos()) == ''


def test_offset_limit_is_inclusive():
    assert arm_blocker(pos(x=1.2, y=1.6), FIRST, max_start_offset_m=2.0) == ''
    assert 'EKF origin' in arm_blocker(pos(x=3.0, y=0.0), FIRST, max_start_offset_m=2.0)


def test_offset_measured_from_first_waypoint_not_origin():
    assert arm_blocker(pos(x=10.0, y=10.0), (10.0, 10.5, -5.0)) == ''


def test_negative_limit_disables_distance_check_only():
    assert arm_blocker(pos(x=500.0), FIRST, max_start_offset_m=-1) == ''
    assert arm_blocker(pos(xy_valid=False), FIRST, max_start_offset_m=-1)


def test_target_position_absolute_is_unchanged():
    from milestone1_singlevehicle.preflight import target_position
    assert target_position((0.0, 0.0, -5.0), False, (9.0, 9.0, 9.0), None) == (0.0, 0.0, -5.0)


def test_target_position_relative_adds_start_offset():
    from milestone1_singlevehicle.preflight import target_position
    # Drone armed at x=2.4, y=-1.3, z=-6.5 (estimator's own frame): a "-5 m" climb is 5 m above that.
    assert target_position((0.0, 0.0, -5.0), True, (2.4, -1.3, -6.5), None) == (2.4, -1.3, -11.5)
    assert target_position((0.0, 0.0, 0.0), True, (2.4, -1.3, -6.5), None) == (2.4, -1.3, -6.5)


def test_target_position_relative_holds_current_before_start_is_captured():
    from types import SimpleNamespace
    from milestone1_singlevehicle.preflight import target_position
    here = SimpleNamespace(x=1.0, y=2.0, z=-3.0)
    assert target_position((0.0, 0.0, -5.0), True, None, here) == (1.0, 2.0, -3.0)
    assert target_position((0.0, 0.0, -5.0), True, None, None) == (0.0, 0.0, -5.0)
    nan = SimpleNamespace(x=float('nan'), y=0.0, z=0.0)
    assert target_position((0.0, 0.0, -5.0), True, None, nan) == (0.0, 0.0, -5.0)
