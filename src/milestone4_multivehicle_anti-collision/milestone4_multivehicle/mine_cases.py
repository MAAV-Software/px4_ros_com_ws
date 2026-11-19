"""Hand-laid, competition-realistic minefields for testing the planner and the map overlay.

Unlike mines.place_mines(), which scatters mines pseudo-randomly from a seed, these are fixed
layouts that mimic patterns a real field might have: belts across the field, a wall with a
gate, scattered clumps, a chokepoint near the finish, a clear-then-dense field, and mines
hugging the edges. Each one still leaves a clear north-south corridor so plan_path() can find
a complete path; the tests in test/test_mine_cases.py lock that in.

No ROS here. Columns run 0..39 west to east, rows 0..149 south to north, as in mines.py.
"""
from milestone4_multivehicle.mines import COLS, ROWS, place_mines


def _add(mines, col, row):
    if 0 <= col < COLS and 0 <= row < ROWS:
        mines.add((col, row))


def _belt(mines, rows, cols):
    """Fill every (col, row) in the given row and column ranges."""
    for row in rows:
        for col in cols:
            _add(mines, col, row)


def _disk(mines, centre_col, centre_row, radius):
    """A roughly circular clump of mines."""
    for col in range(centre_col - radius, centre_col + radius + 1):
        for row in range(centre_row - radius, centre_row + radius + 1):
            if (col - centre_col) ** 2 + (row - centre_row) ** 2 <= radius * radius:
                _add(mines, col, row)


def _east_belt():
    """A dense east-west belt mid-field with two gaps: the path must dodge sideways to a gap
    and then carry on north."""
    mines = set()
    gaps = set(range(10, 14)) | set(range(26, 30))
    cols = [col for col in range(COLS) if col not in gaps]
    _belt(mines, range(74, 78), cols)
    return mines


def _west_wall():
    """A near-continuous wall two columns thick along the west edge, with one narrow gate. The
    path routes east of the wall (keeping its green zone off the mines)."""
    mines = set()
    gate = set(range(73, 78))
    rows = [row for row in range(30, 121) if row not in gate]
    _belt(mines, rows, range(0, 2))
    return mines


def _three_clumps():
    """Three compact clusters (south-west, centre, north-east), like scattered obstacles. The
    planner weaves between them and trades green-zone width for the gaps."""
    mines = set()
    for col, row in ((6, 25), (20, 75), (33, 120)):
        _disk(mines, col, row, 4)
    return mines


def _checker_choke():
    """A sparsely scattered field with one dense chokepoint near the north edge, leaving a
    narrow gap: the checker has to replan late as the path funnels through."""
    mines = set()
    for col in range(2, COLS, 7):
        for row in range(10, 130, 17):
            _add(mines, col, row)
    gap = set(range(17, 23))
    cols = [col for col in range(COLS) if col not in gap]
    _belt(mines, range(131, 134), cols)
    return mines


def _open_south_dense_north():
    """The south half is clear; the north half has two offset belts, so the path grows
    straight at first and then has to weave east-to-west between the belts."""
    mines = set()
    _belt(mines, range(92, 94), [col for col in range(COLS) if col <= 30])   # gap in cols 31-39
    _belt(mines, range(118, 120), [col for col in range(COLS) if col >= 9])  # gap in cols 0-8
    return mines


def _edge_mines():
    """Mines hugging the west and east columns, plus a few on the south and north edges. The
    path has to keep its green zone clear of the edges."""
    mines = set()
    for row in range(30, 111):
        _add(mines, 0, row)
        _add(mines, COLS - 1, row)
    for col in (5, 15, 25, 35):
        _add(mines, col, 0)
    for col in (10, 30):
        _add(mines, col, ROWS - 1)
    return mines


# id -> (one-line description, builder). Order is the order tests iterate.
_CASES = {
    'east_belt': ('Dense mid-field belt with two gaps', _east_belt),
    'west_wall': ('Two-column west wall with one gate', _west_wall),
    'three_clumps': ('Three compact clusters SW, centre, NE', _three_clumps),
    'checker_choke': ('Sparse field with a dense chokepoint near the north', _checker_choke),
    'open_south_dense_north': ('Clear south half, two offset belts in the north', _open_south_dense_north),
    'edge_mines': ('Mines hugging the west, east, south and north edges', _edge_mines),
}


def case(case_id):
    """The mines for one case, as a set of (col, row)."""
    if case_id not in _CASES:
        raise KeyError(f'no mine case {case_id!r}; have {", ".join(_CASES)}')
    return _CASES[case_id][1]()


def description(case_id):
    if case_id not in _CASES:
        raise KeyError(f'no mine case {case_id!r}; have {", ".join(_CASES)}')
    return _CASES[case_id][0]


def case_ids():
    return list(_CASES)


def all_cases():
    """{id: set of (col, row)} for every case."""
    return {case_id: builder() for case_id, (_, builder) in _CASES.items()}


def resolve_mines(count, seed, case_id=''):
    """The mines for a flight: a named case when `case_id` is given, else the seeded random
    field place_mines(count, seed). Lets mine_field pick a fixed realistic layout from a launch
    argument without any ROS in the way. Raises KeyError on an unknown case id."""
    if case_id:
        return case(case_id)
    return place_mines(count, seed)


def layout_stats(mines):
    """Count, row span and the fraction of mines with a neighbour, for sanity checks."""
    mines = set(mines)
    if not mines:
        return {'count': 0, 'min_row': None, 'max_row': None, 'touching_fraction': 0.0}
    touching = sum(
        any((col + dc, row + dr) in mines for dc in (-1, 0, 1) for dr in (-1, 0, 1) if dc or dr)
        for col, row in mines)
    rows = [row for _, row in mines]
    return {
        'count': len(mines),
        'min_row': min(rows),
        'max_row': max(rows),
        'touching_fraction': touching / len(mines),
    }
