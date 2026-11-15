"""Corridor search: all four drones fly north in formation over one band of the field.

No ROS here. Instead of scanning the whole field, the fleet scans a band just wide enough
for a path and plans inside it; scanning less finishes sooner, and the competition score
falls with every minute. Simulations of this (see the README) beat whole-field scans at
every detection probability tried.

- A formation trades band width for looks. "abreast" is one look over a 13.0 m band,
  "2x2" two looks over 6.6 m, "astern" four looks over 3.4 m. "auto" picks by how
  reliable a look is.
- The drones join up at the south edge, then fly north together, nose east. Every few
  seconds the search checks that a path with a green zone of 1 still gets through the
  scanned ground. If it stops well short of the rear drone, the band is blocked: every
  drone moves by the same vector, sideways almost one band and back south to just below
  the block, and carries on north.
- When the rear drone reaches the north end the path is chosen: the complete path with the
  best expected score, counting the chance that a square looked at k times still hides a
  mine. The drones then land where they are, north of the field.

CorridorSearch has the same shape as mines.MineSurvey: feed observe() every drone's
position, and send each drone message(drone).
"""
import math

from milestone4_multivehicle import scan_routes as s
from milestone4_multivehicle.mines import (
    COLS, MAX_GREEN, ROWS, SQUARE_M, Detector, Plan, _grow, _reach, cells_under, footprint)

ALONG_M = 4.0            # along-track gap between drones one behind another
LANE_SOUTH_M = 0.6       # where the front drones start: the first row is under them within SLOT_M of it
LANE_NORTH_M = 91.2      # where the rear drones finish: the last row is under them within SLOT_M of it
COLUMN_M = 3.2           # between drones side by side: footprints overlap 0.16 m, drones stay over 3 m apart
SLOT_M = 0.6             # a drone is in its slot when this close
REPLAN_S = 3.0
BLOCK_ROWS = 10          # the band is blocked when the path stops this far behind the rear drone
BACK_ROWS = 5            # after a sidestep, restart this far south of where the path stopped
NEAR_ROWS = 12           # found mines this far past the block decide which way to sidestep
MAX_SIDESTEPS = 8
LOOK_STEPS = (1, 2, 3, 4, 6, 8)  # the final path is tried through squares with at least this many looks
ALL_CELLS = frozenset((c, r) for c in range(COLS) for r in range(ROWS))


def formations(up=s.ALTITUDE_M):
    """name -> [(metres east of the band's centre, slots back from the front)] for drones 0-3.
    Columns are a little under a footprint apart, so no square falls between two footprints."""
    h = min(COLUMN_M, footprint(up)[0]) / 2
    return {
        'abreast': [(-3 * h, 0), (-h, 0), (h, 0), (3 * h, 0)],
        '2x2': [(-h, 1), (-h, 0), (h, 0), (h, 1)],
        'astern': [(0., 2), (0., 0), (0., 1), (0., 3)],
    }


def reach_row(blocked, green=1):
    """The furthest row a path with this green zone can get to from the south edge, or -1."""
    blocked = set(blocked)
    for _ in range(green):
        blocked = _grow(blocked)
    free = lambda col, row: green <= col <= COLS - 1 - green and 0 <= row < ROWS and (col, row) not in blocked  # noqa: E731
    stack = [(col, 0) for col in range(COLS) if free(col, 0)]
    seen, top = set(stack), -1
    while stack:
        col, row = stack.pop()
        top = max(top, row)
        for cell in ((col + 1, row), (col - 1, row), (col, row + 1), (col, row - 1)):
            if cell not in seen and free(*cell):
                seen.add(cell)
                stack.append(cell)
    return top


def pick_formation(probability):
    """More looks for a worse detector."""
    return 'abreast' if probability >= 0.97 else '2x2' if probability >= 0.85 else 'astern'


def posterior(looks, probability, prior):
    """Chance a square holds a mine after `looks` looks found nothing."""
    weight = prior * (1 - probability) ** looks
    return weight / (weight + 1 - prior)


def expected(plan, looks, probability, prior):
    """(relative expected score, chance of a mine on the path) for a plan."""
    path = set(plan.cells)
    alive = 1.0
    for cell in plan.cells:
        alive *= 1 - posterior(looks.get(cell, 0), probability, prior)
    in_green = sum(posterior(looks.get(cell, 0), probability, prior) for cell in plan.corridor() - path)
    # With B roughly Poisson, E[1 / (1 + B)] = (1 - exp(-mean)) / mean.
    spared = (1 - math.exp(-in_green)) / in_green if in_green > 1e-9 else 1.0
    return plan.width_ft / plan.steps * alive * spared, 1 - alive


def best_plan(found, looks, probability, prior, near_col=COLS // 2):
    """(Plan, chance of a mine on it): the complete path with the best expected score, or
    (None, None). Tried through squares looked at once, twice and so on, at every green width."""
    best = None
    for need in LOOK_STEPS:
        seen = {cell for cell, count in looks.items() if count >= need}
        if not seen:
            break
        blocked = set(found) | (ALL_CELLS - seen)
        for green in range(MAX_GREEN + 1):
            cells = _reach(blocked, green, set(), near_col)
            if cells is None or cells[-1][1] != ROWS - 1:
                break  # a wider zone only blocks more
            plan = Plan(cells, green)
            value, death = expected(plan, looks, probability, prior)
            if best is None or value > best[0]:
                best = (value, plan, death)
            blocked = _grow(blocked)
    return (best[1], best[2]) if best else (None, None)


class CorridorSearch:
    """One flight's corridor search. Feed it every drone's position about ten times a second."""

    def __init__(self, mines, probability=0.9, seed=10, vehicle_count=4, formation='auto', log=print):
        if vehicle_count != 4:
            raise ValueError('the corridor search flies four drones')
        self.mines = set(mines)
        self.probability = probability
        self.prior = max(len(self.mines), 1) / (COLS * ROWS)
        self.detector = Detector(mines, probability, seed)
        self.formation = pick_formation(probability) if formation == 'auto' else formation
        self.slots = formations()[self.formation]
        self.rows_back = max(back for _, back in self.slots)
        width = max(e for e, _ in self.slots) - min(e for e, _ in self.slots) + footprint(s.ALTITUDE_M)[0]
        self.half_band = width / 2
        self.shift = (int(width / SQUARE_M) - 1) * SQUARE_M
        self.centre = s.FIELD_EAST_M / 2
        self.west, self.east = self.centre, self.centre  # the outermost band centres flown so far
        self.log = log
        self.count = vehicle_count
        self.started_s = None
        self.at, self.up = {}, {}
        self.under = {drone: set() for drone in range(vehicle_count)}
        self.looks = {}
        self.phase = 'wait'              # wait, join, transit, sidestep, done
        self.routes = {drone: [] for drone in range(vehicle_count)}
        self.version, self.final = 0, False
        self.planned_s = -math.inf
        self.sidesteps, self.direction = 0, 0
        self.plan, self.death, self.minutes = None, None, None
        log(f'Corridor search: {self.formation} formation, band {width:.1f} m wide')

    # Where things are

    def target(self, drone, rear_north):
        """World (east, north) of a drone's slot when the rearmost slot is at rear_north."""
        east, back = self.slots[drone]
        return self.centre + east, rear_north + (self.rows_back - back) * ALONG_M

    @property
    def start_north(self):
        return LANE_SOUTH_M - self.rows_back * ALONG_M

    def rear_north(self):
        return min(self.at[drone][1] - (self.rows_back - back) * ALONG_M
                   for drone, (_, back) in enumerate(self.slots))

    def in_slots(self, rear_north):
        return all(math.dist(self.at[drone], self.target(drone, rear_north)) <= SLOT_M
                   for drone in range(self.count))

    def send(self, *rear_norths, final=False):
        """Give every drone the route through its slot at each of these rear positions."""
        for drone in range(self.count):
            self.routes[drone] = [(*self.target(drone, north), s.ALTITUDE_M, s.YAW) for north in rear_norths]
        self.version += 1
        self.final = final

    # The flight

    def observe(self, drone, east, north, up, heading, now_s):
        """Returns the mines this look newly found, as (column, row)."""
        if self.started_s is None and up > 1.0:
            self.started_s = now_s
        new = self.detector.observe(drone, east, north, up, heading)
        under = cells_under(east, north, up, heading)
        for cell in under - self.under[drone]:
            self.looks[cell] = self.looks.get(cell, 0) + 1
        self.under[drone] = under
        self.at[drone], self.up[drone] = (east, north), up
        if drone == self.count - 1 and not self.final:
            self._think(now_s)
        return new

    def _think(self, now_s):
        if len(self.at) < self.count:
            return
        if self.phase == 'wait':
            if min(self.up.values()) >= s.ALTITUDE_M - 1.0:
                self.phase = 'join'
                self.send(self.start_north)
        elif self.phase == 'join':
            if self.in_slots(self.start_north):
                self.log('Formation joined; flying north')
                self.phase = 'transit'
                self.send(LANE_NORTH_M)
        elif self.phase == 'sidestep':
            # Back in the lanes of the new band, at or below the restart point: checks resume.
            in_lanes = all(abs(self.at[drone][0] - self.target(drone, 0.)[0]) <= SLOT_M
                           for drone in range(self.count))
            if in_lanes and self.rear_north() <= self.restart_north + 1.0:
                self.phase = 'transit'
        elif self.phase == 'transit':
            if self.in_slots(LANE_NORTH_M):
                self._decide(now_s)
            elif now_s - self.planned_s >= REPLAN_S:
                self.planned_s = now_s
                self._check_block()

    def _check_block(self):
        if self.sidesteps >= MAX_SIDESTEPS:
            return
        blocked = set(self.detector.found) | (ALL_CELLS - set(self.looks))
        near_col = int(self.centre / SQUARE_M)
        top = reach_row(blocked)
        rear_row = min(int(self.rear_north() / SQUARE_M), ROWS - 1)
        if top >= ROWS - 1 or top >= rear_row - BLOCK_ROWS:
            return
        # Which way: away from the found mines just past the block, else toward more field.
        ahead = [col for col, row in self.detector.found if top - 2 <= row <= top + NEAR_ROWS]
        west, east = sum(col < near_col for col in ahead), sum(col > near_col for col in ahead)
        direction = self.direction or (1 if west > east else -1 if east > west
                                       else 1 if self.centre < s.FIELD_EAST_M / 2 else -1)

        def beyond(way):
            """The next band on that side of everything flown so far, kept on the field."""
            centre = (self.east if way > 0 else self.west) + way * self.shift
            return min(max(centre, self.half_band), s.FIELD_EAST_M - self.half_band)

        if abs(beyond(direction) - (self.east if direction > 0 else self.west)) < SQUARE_M:
            direction = -direction  # that side is used up
            if abs(beyond(direction) - (self.east if direction > 0 else self.west)) < SQUARE_M:
                self.sidesteps = MAX_SIDESTEPS  # the whole width is scanned: carry on in this band
                return
        self.direction = direction
        self.sidesteps += 1
        moved = beyond(direction) - self.centre
        self.centre += moved
        self.west, self.east = min(self.west, self.centre), max(self.east, self.centre)
        self.restart_north = max(self.start_north, (top - BACK_ROWS) * SQUARE_M)
        self.phase = 'sidestep'
        self.send(self.restart_north, LANE_NORTH_M)
        self.log(f"Band blocked at row {top}: sidestep {self.sidesteps} {abs(moved):.1f} m "
                 f"{'east' if direction > 0 else 'west'}, restarting from row {max(top - BACK_ROWS, 0)}")

    def _decide(self, now_s):
        self.phase = 'done'
        self.send(LANE_NORTH_M, final=True)  # land where they are, north of the field
        self.minutes = (now_s - self.started_s) / 60 if self.started_s is not None else 0.0
        found, seen = len(self.detector.found), len(self.looks)
        self.log(f'Transit complete: {seen} of {COLS * ROWS} squares scanned, {found} mines found, '
                 f'{self.sidesteps} sidesteps')
        self.plan, self.death = best_plan(self.detector.found, self.looks, self.probability, self.prior,
                                          int(self.centre / SQUARE_M))
        if self.plan is None:
            self.log('No path through the scanned band')
            return
        plan = self.plan
        on_path = len(set(plan.cells) & self.mines)
        in_green = len((plan.corridor() - set(plan.cells)) & self.mines)
        fewest = min(self.looks.get(cell, 0) for cell in plan.corridor())
        self.log(f"Final: {plan.text().strip().replace(chr(10), ' ')}")
        self.log(f'Final: green {plan.green}, width {plan.width_ft} ft, length {plan.length_ft} ft, '
                 f'{self.minutes:.2f} min, score {plan.score(self.minutes, self.mines):.1f}; '
                 f'at least {fewest} looks at every square of it, estimated chance of a mine on the path '
                 f'{100 * self.death:.1f}%; real mines on the path {on_path}, in its green zone {in_green}')

    def message(self, drone):
        """[version, final, points, then east, north, up, yaw for each] for one drone; the
        same form as mines.MineSurvey.message. Version 0 has no points."""
        flat = [value for point in self.routes[drone] for value in point]
        return [float(self.version), float(self.final), float(len(self.routes[drone])), *flat]
