"""Hand-laid realistic minefields (mine_cases): each one is valid, and the oracle planner
still finds a clear, complete path through it. No ROS."""
import pytest

from milestone4_multivehicle import mine_cases as mc
from milestone4_multivehicle import mines as m


def _oracle(mines):
    plan = m.plan_path(mines)
    assert plan is not None, 'no complete path through this case'
    return plan


@pytest.mark.parametrize('case_id', mc.case_ids())
def test_case_is_valid_and_has_a_clear_complete_path(case_id):
    mines = mc.case(case_id)
    stats = mc.layout_stats(mines)

    # A realistic amount of mines, all on the field.
    assert 60 <= stats['count'] <= 200
    assert all(0 <= col < m.COLS and 0 <= row < m.ROWS for col, row in mines)

    plan = _oracle(mines)
    # Starts on the south edge, reaches the north edge.
    assert plan.cells[0][1] == 0 and plan.complete and plan.cells[-1][1] == m.ROWS - 1
    # One step at a time, no repeats.
    assert len(set(plan.cells)) == len(plan.cells)
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(plan.cells, plan.cells[1:]))
    # Neither the path nor its green zone touches a mine.
    assert not plan.corridor() & mines
    # The green zone stays on the field.
    assert all(plan.green <= col <= m.COLS - 1 - plan.green for col, _ in plan.cells)


@pytest.mark.parametrize('case_id', mc.case_ids())
def test_map_overlay_calls_the_oracle_path_optimal(case_id):
    mines = mc.case(case_id)
    oracle = _oracle(mines)
    overlay = m.map_overlay(mines, set(), oracle, oracle, 1.0, final=True)
    assert overlay['optimal'] and overlay['final']
    assert overlay['path'] == [[col, row] for col, row in oracle.cells]
    assert overlay['green'] == oracle.green
    assert len(overlay['all_mines']) == len(mines)


def test_resolve_mines_picks_a_case_or_falls_back_to_the_seeded_field():
    # A named case matches case(); each id round-trips.
    for case_id in mc.case_ids():
        assert mc.resolve_mines(150, 10, case_id) == mc.case(case_id)
    # No case id means the usual seeded random field.
    assert mc.resolve_mines(150, 10) == m.place_mines(150, 10)
    assert mc.resolve_mines(150, 10, '') == m.place_mines(150, 10)
    # An unknown id is refused, not silently treated as random.
    with pytest.raises(KeyError):
        mc.resolve_mines(150, 10, 'no_such_case')


def test_a_different_complete_path_is_not_called_optimal():
    mines = mc.case('three_clumps')
    oracle = _oracle(mines)
    straight = m.Plan([(0, row) for row in range(m.ROWS)], 0)
    assert not m.map_overlay(mines, set(), straight, oracle, 1.0, final=True)['optimal']


def test_east_belt_path_passes_through_a_gap_in_the_belt():
    plan = _oracle(mc.case('east_belt'))
    gaps = set(range(10, 14)) | set(range(26, 30))
    assert all(col in gaps for col, row in plan.cells if 74 <= row <= 77)


def test_west_wall_path_stays_east_of_the_wall():
    mines = mc.case('west_wall')
    plan = _oracle(mines)
    # The wall is really there, and the path keeps clear of columns 0-1 where it stands.
    assert (0, 50) in mines and (1, 50) in mines
    assert all(col >= 2 for col, row in plan.cells if 30 <= row <= 120 and row not in range(73, 78))


def test_open_south_dense_north_weaves_between_the_two_belts():
    plan = _oracle(mc.case('open_south_dense_north'))
    assert all(col >= 31 for col, row in plan.cells if row in (92, 93))  # gap in the south belt
    assert all(col <= 8 for col, row in plan.cells if row in (118, 119))  # gap in the north belt


def test_checker_choke_path_funnels_through_the_gap():
    plan = _oracle(mc.case('checker_choke'))
    assert all(17 <= col <= 22 for col, row in plan.cells if 131 <= row <= 133)


def test_three_clumps_are_three_separate_clusters():
    stats = mc.layout_stats(mc.case('three_clumps'))
    # Compact clumps: almost every mine has a neighbour.
    assert stats['touching_fraction'] > 0.8


def test_edge_mines_line_the_sides():
    mines = mc.case('edge_mines')
    assert (0, 60) in mines and (m.COLS - 1, 60) in mines
    # The planner keeps its corridor off those edge columns.
    plan = _oracle(mines)
    assert not plan.corridor() & mines
