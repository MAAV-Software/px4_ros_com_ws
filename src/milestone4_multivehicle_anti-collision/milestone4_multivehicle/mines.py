"""Simulated mines for milestone 4's field scan: placing them, detecting them, the best path.

No ROS here. The field is the competition's grid of 2 ft squares, 40 columns (west to
east) by 150 rows (south to north); a mine fills its square. World X=East, Y=North, metres,
south-west corner at (0, 0), as in scan_routes.py.

- place_mines() scatters mines in clumps and broken walls, repeatably from a seed.
- Detector decides what a drone's camera finds. Each time a mine comes under a drone's
  footprint it is found with a set probability, so one look leaves some unfound.
- plan_route() gives the best path through the ground the mowing drones have scanned so
  far, in the competition's path.txt form (https://fexl.com/iarc): S,column,green then
  U/D/L/R runs. While the scan runs the path stops where the scanned ground does.
- MineSurvey ties these together for a flight. While drones 0-2 mow, it keeps planning
  the path through what they have scanned and sends the last drone, the checker, along
  the parts of it the checker has not yet looked at. When the path changes, only its new
  squares are checked. The path is final once the scan is over and all of it is checked;
  a mine the checker finds on it while flying to land reopens it.
"""
import heapq
import math
import random

from milestone4_multivehicle import scan_routes as s

SQUARE_M = 2 * s.FT_M
COLS, ROWS = 40, 150
MAX_GREEN = 4            # widest green zone the planner tries, in squares each side
MIN_SCAN_ALTITUDE_M = 3.0
LEAVE_MARGIN_M = 0.3     # a mine must leave the footprint by this much before it can be found again
PASS_END_INSET_M = 1.0   # the checker's route ends, inside the field's south and north edges
PASS_MARGIN_M = 0.3      # footprint to spare beside the corridor when checking
MAX_PASS_ALTITUDE_M = 12.0
ROW_END_M = 1.3          # a mower within this of the field's end has finished a row
REPLAN_S = 3.0
REACH_STEP = 5           # a partial path ends on a multiple of this many rows, so it grows in steps
MIN_REACH = 10           # rows a partial path must reach before the checker sets off
KEEP_GREEN = 0.9         # keep the current green-zone width unless another is this much better
ROUTE_END_M = 0.6        # the checker has finished a stretch when this close to its end
GIVE_UP_S = 300.0        # after the scan, stop checking after this long
MOVES = {'U': (0, 1), 'D': (0, -1), 'L': (-1, 0), 'R': (1, 0)}


def centre(cell):
    """World (east, north) of a square's centre."""
    return (cell[0] + 0.5) * SQUARE_M, (cell[1] + 0.5) * SQUARE_M


def place_mines(count=150, seed=10):
    """A set of `count` (column, row) squares: two broken walls, six clumps, the rest scattered."""
    count = max(0, min(int(count), COLS * ROWS))
    rng = random.Random(seed)
    mines = set()

    def add(col, row):
        if 0 <= col < COLS and 0 <= row < ROWS and len(mines) < count:
            mines.add((col, row))

    for _ in range(2):
        row, start, length = rng.randrange(15, ROWS - 15), rng.randrange(0, COLS - 10), rng.randrange(12, 26)
        for col in range(start, min(COLS, start + length)):
            if rng.random() < 0.7:
                add(col, row + rng.choice((0, 0, 1)))
    for _ in range(6):
        col, row, spread = rng.uniform(0, COLS), rng.uniform(0, ROWS), rng.uniform(1.5, 4.0)
        for _ in range(count // 10):
            add(round(rng.gauss(col, spread)), round(rng.gauss(row, spread)))
    while len(mines) < count:
        add(rng.randrange(COLS), rng.randrange(ROWS))
    return mines


def footprint(up):
    """(along the nose, across the nose) camera footprint from `up` metres."""
    scale = up / s.CAMERA_ALTITUDE_M
    return s.CAMERA_NOSE_M * scale, s.CAMERA_SIDE_M * scale


def in_footprint(point, east, north, up, heading, margin=0.0):
    """Whether a world point is under the camera of a drone at (east, north, up), PX4 heading."""
    if up < MIN_SCAN_ALTITUDE_M:
        return False
    nose, side = footprint(up)
    dx, dy = point[0] - east, point[1] - north
    forward = dx * math.sin(heading) + dy * math.cos(heading)
    right = dx * math.cos(heading) - dy * math.sin(heading)
    return abs(forward) <= nose / 2 + margin and abs(right) <= side / 2 + margin


class Detector:
    """What the cameras find. A mine coming under a drone's footprint is found with
    `probability`; it gets another chance only after leaving that footprint."""

    def __init__(self, mines, probability=0.9, seed=10):
        self.mines = sorted(mines)
        self.probability = probability
        self.rng = random.Random(seed + 1)
        self.under = {}      # drone -> mines under its footprint now
        self.found = {}      # mine -> drone that found it first

    def observe(self, drone, east, north, up, heading):
        """The mines this look newly finds."""
        under = self.under.setdefault(drone, set())
        new = []
        for mine in self.mines:
            point = centre(mine)
            if mine in under:
                if not in_footprint(point, east, north, up, heading, LEAVE_MARGIN_M):
                    under.discard(mine)
            elif in_footprint(point, east, north, up, heading):
                under.add(mine)
                if self.rng.random() < self.probability and mine not in self.found:
                    self.found[mine] = drone
                    new.append(mine)
        return new


class Plan:
    """A path through the field: the squares walked, south to north, and its green zone."""

    def __init__(self, cells, green):
        self.cells, self.green = list(cells), green

    @property
    def steps(self):
        return len(self.cells)  # the S command starts one row south of row 0

    @property
    def width_ft(self):
        return 2 * (1 + 2 * self.green)

    @property
    def length_ft(self):
        return 2 * self.steps

    def corridor(self):
        """Every square of the path and its green zone."""
        g = self.green
        return {(c + dc, r + dr) for c, r in self.cells for dc in range(-g, g + 1) for dr in range(-g, g + 1)
                if 0 <= c + dc < COLS and 0 <= r + dr < ROWS}

    def runs(self):
        """[(direction, squares)] in walking order; the first U includes the step onto row 0."""
        out, previous = [], (self.cells[0][0], -1)
        for cell in self.cells:
            step = (cell[0] - previous[0], cell[1] - previous[1])
            direction = next(d for d, move in MOVES.items() if move == step)
            if out and out[-1][0] == direction:
                out[-1][1] += 1
            else:
                out.append([direction, 1])
            previous = cell
        return [tuple(run) for run in out]

    def text(self):
        """The competition's path.txt."""
        lines = [f'S,{self.cells[0][0]},{self.green}'] + [f'{d},{n}' for d, n in self.runs()]
        return '\n'.join(lines) + '\n'

    def score(self, minutes, mines=(), ounces_over=0.0):
        """The competition score; `mines` are the real ones. 0 if the path steps on one
        or stops short of the north edge."""
        if not self.complete or set(self.cells) & set(mines):
            return 0.0
        missed = len((self.corridor() - set(self.cells)) & set(mines))
        return 150000 * self.width_ft / ((1 + missed) * self.length_ft * (1 + 7 * minutes + 100 * ounces_over))

    @property
    def complete(self):
        return self.cells[-1][1] == ROWS - 1

    def __eq__(self, other):
        return isinstance(other, Plan) and (self.cells, self.green) == (other.cells, other.green)


def cells_under(east, north, up, heading):
    """The squares whose centres are under the camera."""
    if up < MIN_SCAN_ALTITUDE_M:
        return set()
    reach = math.hypot(*footprint(up)) / 2
    cols = range(max(0, int((east - reach) / SQUARE_M)), min(COLS, int((east + reach) / SQUARE_M) + 1))
    rows = range(max(0, int((north - reach) / SQUARE_M)), min(ROWS, int((north + reach) / SQUARE_M) + 1))
    return {(c, r) for c in cols for r in rows if in_footprint(centre((c, r)), east, north, up, heading)}


def _grow(cells):
    """`cells` and every square touching one, corners included."""
    return {(c + dc, r + dr) for c, r in cells for dc in (-1, 0, 1) for dr in (-1, 0, 1)
            if 0 <= c + dc < COLS and 0 <= r + dr < ROWS}


def _reach(blocked, green, checked, near_col):
    """The path from south of row 0 that gets furthest north avoiding `blocked`.

    Short of the north edge it ends on a multiple of REACH_STEP rows. Among paths there it
    takes the fewest squares, then the fewest turns, then the fewest squares not yet
    `checked`, then the start nearest `near_col`. The path keeps `green` squares from the east
    and west edges so its green zone is on the field. None if row 0 is blocked.
    """
    low, high = green, COLS - 1 - green
    best, heap, came = {}, [], {}
    for col in range(low, high + 1):
        if (col, 0) not in blocked:
            state = (col, 0, 'U')
            best[state] = (1, 0, (col, 0) not in checked, abs(col - near_col))
            heapq.heappush(heap, (*best[state], state))
    top = max((state[1] for state in best), default=None)
    settled = []
    while heap:
        *cost, state = heapq.heappop(heap)
        cost = tuple(cost)
        if best.get(state) != cost:
            continue
        settled.append((cost, state))
        col, row, direction = state
        top = max(top, row)
        for new_direction, (dc, dr) in MOVES.items():
            nxt = (col + dc, row + dr)
            if not (low <= nxt[0] <= high and 0 <= nxt[1] < ROWS) or nxt in blocked:
                continue
            new_cost = (cost[0] + 1, cost[1] + (new_direction != direction), cost[2] + (nxt not in checked), cost[3])
            new_state = (*nxt, new_direction)
            if new_cost < best.get(new_state, (math.inf,)):
                best[new_state] = new_cost
                came[new_state] = state
                heapq.heappush(heap, (*new_cost, new_state))
    if top is None:
        return None
    goal = top if top == ROWS - 1 or top < REACH_STEP else top - top % REACH_STEP
    _, state = min(entry for entry in settled if entry[1][1] == goal)
    cells = []
    while state is not None:
        cells.append(state[:2])
        state = came.get(state)
    return cells[::-1]


def plan_route(found, scanned=None, checked=(), keep_green=None, near_col=COLS // 2, max_green=MAX_GREEN):
    """The best Plan through `scanned` squares with no `found` mine on it or in its green zone.

    `scanned=None` means the whole field has been scanned. A path that reaches the north
    edge beats one that does not. Among complete paths the competition score decides:
    width over length. Among partial ones, how far north they get comes first (within
    two REACH_STEPs), then width over length. `keep_green` is the width in use, kept
    unless another is clearly better, so the checker is not sent back and forth.
    None if nothing can be planned.
    """
    blocked = set(found)
    if scanned is not None:
        blocked |= {(c, r) for c in range(COLS) for r in range(ROWS)} - set(scanned)
    checked = set(checked)
    plans = []
    for green in range(max_green + 1):
        cells = _reach(blocked, green, checked, near_col)
        if cells is None:
            break  # a wider zone only blocks more
        plans.append(Plan(cells, green))
        blocked = _grow(blocked)
    if not plans:
        return None
    furthest = max(plan.cells[-1][1] for plan in plans)
    plans = [plan for plan in plans if plan.cells[-1][1] >= (furthest if furthest == ROWS - 1
                                                              else furthest - 2 * REACH_STEP)]

    def value(plan):
        return plan.width_ft / plan.steps * (plan.cells[-1][1] + 1), plan.green

    best = max(plans, key=value)
    kept = next((plan for plan in plans if plan.green == keep_green), None)
    if kept is not None and value(kept)[0] >= KEEP_GREEN * value(best)[0]:
        return kept
    return best


def plan_path(mines, max_green=MAX_GREEN):
    """The best-scoring complete Plan through a fully scanned field, or None."""
    plan = plan_route(mines, max_green=max_green)
    return plan if plan is not None and plan.complete else None


def map_overlay(mines, found, plan, oracle, minutes, final):
    """The web map's view of a survey, as plain data (no version): every mine, the found
    ones, the current path and its green zone, and how that path scores against the oracle's
    optimal `plan_path` one. `plan`/`oracle` are Plans or None."""
    complete = plan is not None and plan.complete
    cells = [[c, r] for c, r in plan.cells] if plan is not None else []
    return {
        'all_mines': [[c, r] for c, r in sorted(mines)],
        'found': [[c, r] for c, r in sorted(found)],
        'path': cells,
        'green': plan.green if plan is not None else 0,
        'final': bool(final),
        # Optimal once a full path exists and its squares and green zone match the oracle's.
        'optimal': bool(complete and oracle is not None
                        and plan.cells == oracle.cells and plan.green == oracle.green),
        'score': plan.score(minutes, mines) if complete else None,
        'oracle_score': oracle.score(minutes, mines) if oracle is not None else None,
    }


def pass_altitude(plan):
    """Height for checking, nose east: the footprint's east-west side must span the
    corridor on north-south legs, and its north-south side on any east-west leg."""
    width = (1 + 2 * plan.green) * SQUARE_M + PASS_MARGIN_M
    needed = width / s.CAMERA_NOSE_M
    if any(direction in 'LR' for direction, _ in plan.runs()):
        needed = max(needed, width / s.CAMERA_SIDE_M)
    return round(min(max(s.ALTITUDE_M, needed * s.CAMERA_ALTITUDE_M), MAX_PASS_ALTITUDE_M), 2)


def stretch_route(cells, up):
    """World [(east, north, up, yaw)] along the centres of `cells`, corners only."""
    low, high = PASS_END_INSET_M, s.FIELD_NORTH_M - PASS_END_INSET_M
    points = []
    for cell in cells:
        east, north = centre(cell)
        point = (round(east, 3), round(min(max(north, low), high), 3))
        if points and point == points[-1]:
            continue
        if len(points) >= 2 and (points[-2][0] == points[-1][0] == point[0]
                                 or points[-2][1] == points[-1][1] == point[1]):
            points[-1] = point  # still on the same straight
        else:
            points.append(point)
    return [(east, north, up, s.YAW) for east, north in points]


class MineSurvey:
    """One flight's mine hunt. Feed it every drone's position about ten times a second."""

    def __init__(self, mines, probability=0.9, seed=10, vehicle_count=4, log=print):
        self.mines = set(mines)
        self.detector = Detector(mines, probability, seed)
        self.checker = vehicle_count - 1
        self.mowers = range(vehicle_count - 1)
        self.log = log
        self.started_s = None
        self.rows_flown = {drone: 0 for drone in self.mowers}
        self.scanned, self.checked, self.skipped = set(), set(), set()
        self.plan, self.route, self.stretch, self.version, self.final = None, [], [], 0, False
        self.checker_at = None
        self.planned_s, self.logged_s, self.scan_over_s = -math.inf, -math.inf, None
        self.paths = 0
        self.minutes = None

    @property
    def scan_over(self):
        return all(rows >= s.ROWS_PER_DRONE for rows in self.rows_flown.values())

    def observe(self, drone, east, north, up, heading, now_s):
        """Returns the mines this look newly found, as (column, row)."""
        if self.started_s is None and up > 1.0:
            self.started_s = now_s
        new = self.detector.observe(drone, east, north, up, heading)
        under = cells_under(east, north, up, heading)
        if drone == self.checker:
            self.checked |= under
            self.checker_at = (east, north)
            if self.final and self.plan and set(new) & self.plan.corridor():
                # Found on the way to land: the path is not final after all.
                self.log('A mine turned up on the final path; planning again')
                self.final, self.planned_s = False, -math.inf
            if not self.final:
                self._think(now_s)
        else:
            self.scanned |= under
            # Rows alternate: an even one ends at the north edge, an odd one at the south.
            flown = self.rows_flown[drone]
            at_end = north > s.FIELD_NORTH_M - ROW_END_M if flown % 2 == 0 else north < ROW_END_M
            if flown < s.ROWS_PER_DRONE and at_end:
                self.rows_flown[drone] += 1
                if self.scan_over:
                    self.scan_over_s = now_s
                    self.log(f'Scan complete: {len(self.detector.found)} of {len(self.mines)} mines found, '
                             f'{len(self.scanned)} of {COLS * ROWS} squares scanned')
        return new

    def _unchecked(self, plan):
        """Indexes of the path squares whose stretch of corridor still has an unchecked square."""
        done, g = self.checked | self.skipped, plan.green
        return [i for i, (c, r) in enumerate(plan.cells)
                if any((c + dc, r + dr) not in done for dc in range(-g, g + 1) for dr in range(-g, g + 1)
                       if 0 <= c + dc < COLS and 0 <= r + dr < ROWS)]

    def _think(self, now_s):
        # A stretch flown to its end is done with: squares the camera still missed (a cut
        # corner, a corridor wider than the footprint) are not tried again.
        if self.stretch and math.dist(self.checker_at, self.route[-1][:2]) <= ROUTE_END_M:
            g = self.plan.green
            self.skipped |= {(c + dc, r + dr) for c, r in self.stretch
                             for dc in range(-g, g + 1) for dr in range(-g, g + 1)
                             if 0 <= c + dc < COLS and 0 <= r + dr < ROWS} - self.checked
            self.stretch = []
        if now_s - self.planned_s < REPLAN_S:
            return
        self.planned_s = now_s
        near_col = int(self.checker_at[0] / SQUARE_M)
        plan = plan_route(self.detector.found, self.scanned, self.checked | self.skipped,
                          self.plan.green if self.plan else None, near_col)
        if plan is not None and (plan.complete or plan.cells[-1][1] >= MIN_REACH) and plan != self.plan:
            self.plan = plan
            if now_s - self.logged_s >= 20:
                self.logged_s = now_s
                self._report(now_s, 'Path' if plan.complete else 'Path so far')
        if self.plan is None:
            if self.scan_over:
                self.log('No path through the scanned ground')
                self._land(now_s)
            return
        todo = self._unchecked(self.plan)
        gave_up = self.scan_over and now_s - self.scan_over_s > GIVE_UP_S
        if todo and not gave_up:
            cells = self.plan.cells[todo[0]:todo[-1] + 1]
            # Start from whichever end is nearer.
            if math.dist(self.checker_at, centre(cells[-1])) < math.dist(self.checker_at, centre(cells[0])):
                cells = cells[::-1]
            route = stretch_route(cells, pass_altitude(self.plan))
            if route != self.route:
                self.route, self.stretch = route, cells
                self.version += 1
        elif self.scan_over:
            self._land(now_s)

    def _land(self, now_s):
        self.final = True
        slot = min(s.checker_landings(), key=lambda place: math.dist(place, self.checker_at))
        up = pass_altitude(self.plan) if self.plan else s.ALTITUDE_M
        self.route = [(slot[0], s.FIELD_NORTH_M - PASS_END_INSET_M, up, s.YAW), (*slot, up, s.YAW)]
        self.stretch = []
        self.version += 1
        if self.plan is None:
            return
        self._report(now_s, 'Final')
        corridor = self.plan.corridor()
        self.log(f'Final: the checker looked at {len(corridor & self.checked)} of the path\'s {len(corridor)} '
                 f'squares ({len(corridor - self.checked)} missed), and at {len(self.checked - corridor)} '
                 f'squares that are not on it')

    def _report(self, now_s, label):
        self.minutes = (now_s - self.started_s) / 60 if self.started_s is not None else 0.0
        plan = self.plan
        on_path = len(set(plan.cells) & self.mines)
        in_green = len((plan.corridor() - set(plan.cells)) & self.mines)
        self.paths += 1
        self.log(f"{label}: {plan.text().strip().replace(chr(10), ' ')}")
        reach = '' if plan.complete else f'reaches row {plan.cells[-1][1]}, '
        self.log(f'{label}: {reach}green {plan.green}, width {plan.width_ft} ft, length {plan.length_ft} ft, '
                 f'{self.minutes:.2f} min, score {plan.score(self.minutes, self.mines):.1f}; '
                 f'{len(self.detector.found)} of {len(self.mines)} mines found; '
                 f'real mines on the path {on_path}, in its green zone {in_green}')

    def message(self):
        """[version, final, points, then east, north, up, yaw for each]. Version 0 has no
        points: the survey is alive but has nowhere to send the checker yet. `final` means
        land at the end of this route."""
        flat = [value for point in self.route for value in point]
        return [float(self.version), float(self.final), float(len(self.route)), *flat]


def read_message(data):
    """(version, final, [(east, north, up, yaw)]) from MineSurvey.message(), or None if malformed."""
    if len(data) < 3 or not all(math.isfinite(v) for v in data):
        return None
    count = int(data[2])
    if count < 0 or len(data) != 3 + 4 * count or (count == 0) != (data[0] == 0):
        return None
    points = [tuple(data[3 + 4 * i:7 + 4 * i]) for i in range(count)]
    return int(data[0]), bool(data[1]), points
