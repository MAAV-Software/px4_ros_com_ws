"""Pass/fail scoring for Milestone 1 test flights.

Turns one flight's recorded position, heading, tilt, setpoints and vehicle
status into the measures from documentation/milestone-1-single-vehicle.md (position error,
overshoot, settle time, cross-track error, heading error, hover drift, tilt),
and checks them against the thresholds in the path YAML.

No ROS or pyulog imports: analyze_flight.py feeds this from a .ulg log, and the
unit tests feed it made-up data.
"""
import math
from dataclasses import dataclass, field

import numpy as np
import yaml

# Must match offboard_control_solution.py: the scorer treats a waypoint as
# reached the same way the node does.
ACCEPTANCE_RADIUS_M = 0.5
ACCEPTANCE_YAW_DEG = 5.0

# How close a logged setpoint must be to a path waypoint to count as that waypoint.
SETPOINT_MATCH_M = 0.1
SETPOINT_MATCH_YAW_DEG = 1.0

POSITION_ERROR_WINDOW_S = 1.0  # position error = mean over this much of the end of the leg
MIN_POST_ARRIVAL_S = 0.5       # less time than this at the waypoint: overshoot/settle can't be measured
MIN_HOVER_S = 1.0              # minimum hover time needed to measure hover drift
MIN_LEG_LENGTH_M = 0.1         # shorter legs (yaw-only) have no direction to overshoot along
S1_TIMEOUT_S = 5.0             # arming to Offboard mode must take at most this long

THRESHOLD_KEYS = (
    'position_error_m', 'overshoot_m', 'settle_time_s', 'cross_track_m',
    'heading_error_deg', 'hover_drift_m', 'tilt_max_deg',
)
STAGES = ('sitl', 'hitl')

# Which measures decide each per-waypoint test-matrix row, by row-ID prefix.
TEST_MEASURES = {
    'T': ('position_error_m', 'overshoot_m', 'settle_time_s', 'cross_track_m'),
    'Y': ('heading_error_deg',),
    'H': ('hover_drift_m',),
}
# Rows scored over the whole flight rather than per waypoint.
FLIGHT_ROWS = ('R1', 'S1', 'L1', 'F1')

# px4_msgs/msg/VehicleStatus.msg
ARMING_STATE_ARMED = 2
NAV_STATE_HOLD = 4
NAV_STATE_OFFBOARD = 14
NAV_STATE_LAND = 18
NAV_STATE_NAMES = {
    0: 'Manual', 1: 'Altitude', 2: 'Position', 3: 'Mission', 4: 'Hold', 5: 'Return (RTL)',
    6: 'Position slow', 8: 'Altitude cruise', 10: 'Acro', 12: 'Descend', 13: 'Termination',
    14: 'Offboard', 15: 'Stabilized', 17: 'Takeoff', 18: 'Land', 19: 'Follow target',
    20: 'Precision land', 21: 'Orbit', 22: 'VTOL takeoff',
}
# Offboard-loss failsafe actions that are acceptable for hardware testing.
# RTL is not: it climbs to RTL_RETURN_ALT, far above the test area.
ACCEPTABLE_FAILSAFE_MODES = (NAV_STATE_HOLD, NAV_STATE_LAND)

# Row statuses, worst first. A row with several waypoints takes its worst status.
STATUS_ORDER = ('FAIL', 'INCOMPLETE', 'NO DATA', 'PASS', 'NOT FLOWN', 'NOT TESTED', 'NOT SCORED')


# --- Path file ---------------------------------------------------------------

@dataclass
class Waypoint:
    pos: np.ndarray            # local NED, metres
    yaw_deg: float = None      # None: heading not commanded (plain [x, y, z] entry)
    hold_s: float = 0.0
    tests: list = field(default_factory=list)


@dataclass
class Config:
    waypoints: list
    thresholds: dict           # stage -> {measure: limit}


def load_config(path):
    with open(path) as f:
        return parse_config(yaml.safe_load(f), source=path)


def parse_config(data, source='path file'):
    """Validate the path YAML. Waypoints are plain [x, y, z] lists or
    {pos: [x, y, z], yaw: deg, hold_s: s, tests: [IDs]} entries."""
    if not isinstance(data, dict):
        raise ValueError(f'{source}: expected a mapping with "waypoints" and "thresholds"')

    raw_waypoints = data.get('waypoints')
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError(f'{source}: "waypoints" must be a non-empty list')
    waypoints = [_parse_waypoint(entry, f'{source}: waypoint {i}')
                 for i, entry in enumerate(raw_waypoints)]

    raw_thresholds = data.get('thresholds')
    if not isinstance(raw_thresholds, dict):
        raise ValueError(f'{source}: "thresholds" with "sitl" and "hitl" limits is required for scoring')
    thresholds = {}
    for stage in STAGES:
        limits = raw_thresholds.get(stage)
        if not isinstance(limits, dict):
            raise ValueError(f'{source}: thresholds.{stage} is missing')
        missing = [key for key in THRESHOLD_KEYS if key not in limits]
        if missing:
            raise ValueError(f'{source}: thresholds.{stage} is missing {", ".join(missing)}')
        thresholds[stage] = {key: float(limits[key]) for key in THRESHOLD_KEYS}

    return Config(waypoints, thresholds)


def _parse_waypoint(entry, where):
    if isinstance(entry, (list, tuple)):
        return Waypoint(_parse_position(entry, where))
    if not isinstance(entry, dict):
        raise ValueError(f'{where}: must be [x, y, z] or a mapping with "pos"')

    unknown = set(entry) - {'pos', 'yaw', 'hold_s', 'tests'}
    if unknown:
        raise ValueError(f'{where}: unknown key(s) {", ".join(sorted(unknown))}')
    if 'pos' not in entry:
        raise ValueError(f'{where}: "pos" is required')

    hold_s = float(entry.get('hold_s', 0.0))
    if hold_s < 0:
        raise ValueError(f'{where}: "hold_s" must not be negative')
    tests = entry.get('tests', [])
    if not isinstance(tests, list) or not all(isinstance(t, str) for t in tests):
        raise ValueError(f'{where}: "tests" must be a list of test IDs, e.g. [T1]')
    yaw = entry.get('yaw')

    return Waypoint(_parse_position(entry['pos'], where),
                    None if yaw is None else float(yaw), hold_s, list(tests))


def _parse_position(value, where):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{where}: position must be [x, y, z]')
    return np.array([float(v) for v in value])


# --- Flight data -------------------------------------------------------------

@dataclass
class FlightData:
    """Time series from one flight. Times in seconds, positions in metres (local NED)."""
    pos_t: np.ndarray
    pos: np.ndarray            # (N, 3) vehicle_local_position x, y, z
    heading: np.ndarray        # (N,) radians
    att_t: np.ndarray
    tilt_deg: np.ndarray       # (M,) angle between body z-axis and vertical
    sp_t: np.ndarray
    sp_pos: np.ndarray         # (K, 3) trajectory_setpoint position
    sp_yaw: np.ndarray         # (K,) radians
    status_t: np.ndarray
    arming_state: np.ndarray
    nav_state: np.ndarray
    failsafe: np.ndarray       # bool


def tilt_from_quaternion(q):
    """Tilt in degrees from (N, 4) PX4 attitude quaternions [w, x, y, z]."""
    q = np.asarray(q, dtype=float)
    cos_tilt = 1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2)
    return np.degrees(np.arccos(np.clip(cos_tilt, -1.0, 1.0)))


def _wrap(angle_rad):
    return (np.asarray(angle_rad) + math.pi) % (2 * math.pi) - math.pi


# --- Splitting the flight into legs -----------------------------------------

@dataclass
class Segment:
    """A stretch of time with one constant setpoint, while in Offboard mode."""
    start: float
    end: float
    pos: np.ndarray
    yaw: float
    ended_by: str              # next, land, failsafe, mode_change, disarmed, log_end


def _offboard_intervals(flight):
    in_offboard = ((flight.nav_state == NAV_STATE_OFFBOARD)
                   & (flight.arming_state == ARMING_STATE_ARMED))
    intervals, start = [], None
    for t, on in zip(flight.status_t, in_offboard):
        if on and start is None:
            start = t
        elif not on and start is not None:
            intervals.append((start, t))
            start = None
    if start is not None:
        ends = [series[-1] for series in (flight.status_t, flight.pos_t, flight.sp_t) if len(series)]
        intervals.append((start, max(ends)))
    return intervals


def _how_offboard_ended(flight, end):
    i = np.searchsorted(flight.status_t, end)
    if i >= len(flight.status_t):
        return 'log_end'
    if flight.failsafe[i]:
        return 'failsafe'
    if flight.nav_state[i] == NAV_STATE_LAND:
        return 'land'
    if flight.arming_state[i] != ARMING_STATE_ARMED:
        return 'disarmed'
    return 'mode_change'


def setpoint_segments(flight):
    """Split the Offboard-mode setpoints into constant-setpoint segments.

    Only setpoints logged while armed and in Offboard mode are used: PX4's own
    Hold and Land modes also publish trajectory_setpoint, and theirs drift.
    """
    segments = []
    for start, end in _offboard_intervals(flight):
        mask = ((flight.sp_t >= start) & (flight.sp_t < end)
                & np.all(np.isfinite(flight.sp_pos), axis=1))
        t, pos, yaw = flight.sp_t[mask], flight.sp_pos[mask], flight.sp_yaw[mask]
        if not len(t):
            continue

        yaw_step = np.nan_to_num(np.abs(_wrap(np.diff(yaw))))
        changed = np.ones(len(t), dtype=bool)
        changed[1:] = (np.linalg.norm(np.diff(pos, axis=0), axis=1) > 1e-3) | (yaw_step > 1e-3)
        starts = np.flatnonzero(changed)

        for k, i in enumerate(starts):
            last = k == len(starts) - 1
            segments.append(Segment(
                start=float(t[i]),
                end=float(end if last else t[starts[k + 1]]),
                pos=pos[i],
                yaw=float(yaw[i]),
                ended_by=_how_offboard_ended(flight, end) if last else 'next'))
    return segments


def _matches(segment, waypoint):
    if np.linalg.norm(segment.pos - waypoint.pos) > SETPOINT_MATCH_M:
        return False
    if waypoint.yaw_deg is None:
        return True
    if not math.isfinite(segment.yaw):
        return False
    error = abs(math.degrees(_wrap(segment.yaw - math.radians(waypoint.yaw_deg))))
    return error <= SETPOINT_MATCH_YAW_DEG


def match_segments(segments, waypoints):
    """Pair segments with path waypoints, in path order. Returns
    ([(waypoint_index, segment)], [unmatched segments])."""
    matched, unmatched, next_index = [], [], 0
    for segment in segments:
        index = next((j for j in range(next_index, len(waypoints))
                      if _matches(segment, waypoints[j])), None)
        if index is None:
            unmatched.append(segment)
            continue
        matched.append((index, segment))
        next_index = index + 1
    return matched, unmatched


# --- Scoring one leg ---------------------------------------------------------

@dataclass
class LegResult:
    waypoint: int
    tests: list
    complete: bool             # the node moved on or landed (not cut short)
    ended_by: str
    reached: bool
    measures: dict
    checks: dict               # test ID -> status
    failed_measures: dict      # test ID -> [measure names over their limit]


def _distance_to_segment(points, a, b):
    ab = b - a
    length_sq = float(ab @ ab)
    if length_sq < 1e-9:
        return np.linalg.norm(points - a, axis=1)
    s = np.clip((points - a) @ ab / length_sq, 0.0, 1.0)
    return np.linalg.norm(points - (a + s[:, None] * ab), axis=1)


def leg_measures(t, pos, heading, tilt_deg, target, target_yaw_deg, start_pos):
    """Measures for one leg from its samples. Returns (reached, measures).

    Arrival is the first sample within the acceptance radius (and heading
    tolerance, if a yaw is commanded), the same test the node uses to move on.
    - overshoot_m: furthest the drone went past the target, along the leg's
      direction of travel (distance from the target for yaw-only legs)
    - settle_time_s: from arrival until the error stays within the acceptance
      radius; inf if it never does
    - hover_drift_m: furthest from the mean position over the second half of
      the time after arrival, when the drone should be hovering
    A measure is None when the leg doesn't have the data for it."""
    measures = dict.fromkeys(THRESHOLD_KEYS)
    measures['arrival_s'] = None
    if len(tilt_deg):
        measures['tilt_max_deg'] = float(np.max(tilt_deg))
    if len(t) < 2:
        return False, measures

    error = np.linalg.norm(pos - target, axis=1)
    heading_error = None
    at_target = error <= ACCEPTANCE_RADIUS_M
    if target_yaw_deg is not None:
        heading_error = np.abs(np.degrees(_wrap(heading - math.radians(target_yaw_deg))))
        at_target &= heading_error <= ACCEPTANCE_YAW_DEG

    if not at_target.any():
        measures['position_error_m'] = float(error[-1])
        measures['cross_track_m'] = float(np.max(_distance_to_segment(pos, start_pos, target)))
        return False, measures

    arrival = int(np.argmax(at_target))
    measures['arrival_s'] = float(t[arrival] - t[0])
    before = pos[:arrival]
    measures['cross_track_m'] = (float(np.max(_distance_to_segment(before, start_pos, target)))
                                 if len(before) else 0.0)

    if t[-1] - t[arrival] < MIN_POST_ARRIVAL_S:
        return True, measures  # moved on immediately (no hold): nothing more to measure

    after = slice(arrival, None)
    leg_vector = target - start_pos
    leg_length = float(np.linalg.norm(leg_vector))
    if leg_length >= MIN_LEG_LENGTH_M:
        past_target = (pos[after] - target) @ (leg_vector / leg_length)
        measures['overshoot_m'] = max(0.0, float(np.max(past_target)))
    else:
        measures['overshoot_m'] = float(np.max(error[after]))

    outside = np.flatnonzero(error[after] > ACCEPTANCE_RADIUS_M)
    if not len(outside):
        settled = arrival
    elif arrival + outside[-1] == len(t) - 1:
        settled = None
    else:
        settled = arrival + int(outside[-1]) + 1
    measures['settle_time_s'] = math.inf if settled is None else float(t[settled] - t[arrival])

    window = (t >= t[-1] - POSITION_ERROR_WINDOW_S) & (np.arange(len(t)) >= arrival)
    measures['position_error_m'] = float(np.mean(error[window]))

    hover = pos[t >= (t[arrival] + t[-1]) / 2]
    if (t[-1] - t[arrival]) / 2 >= MIN_HOVER_S and len(hover) >= 2:
        measures['hover_drift_m'] = float(np.max(np.linalg.norm(hover - hover.mean(axis=0), axis=1)))

    if settled is not None and heading_error is not None:
        measures['heading_error_deg'] = float(np.max(heading_error[settled:]))

    return True, measures


def _check(test, complete, reached, measures, limits):
    """Status of one test ID on one leg, plus the measures over their limit."""
    names = TEST_MEASURES.get(test[:1])
    if names is None:
        return 'NOT SCORED', []
    if not complete:
        return 'INCOMPLETE', []
    if not reached:
        return 'FAIL', ['not reached']
    values = {name: measures[name] for name in names}
    failed = [name for name, value in values.items() if value is not None and value > limits[name]]
    if failed:
        return 'FAIL', failed
    if any(value is None for value in values.values()):
        return 'NO DATA', [name for name, value in values.items() if value is None]
    return 'PASS', []


def score_leg(index, waypoint, start_pos, segment, flight, limits):
    in_leg = (flight.pos_t >= segment.start) & (flight.pos_t <= segment.end)
    in_leg_att = (flight.att_t >= segment.start) & (flight.att_t <= segment.end)
    reached, measures = leg_measures(
        flight.pos_t[in_leg], flight.pos[in_leg], flight.heading[in_leg],
        flight.tilt_deg[in_leg_att], waypoint.pos, waypoint.yaw_deg, start_pos)

    complete = segment.ended_by in ('next', 'land')
    checks, failed = {}, {}
    for test in waypoint.tests:
        if test in FLIGHT_ROWS:
            continue
        checks[test], failed[test] = _check(test, complete, reached, measures, limits)
    return LegResult(index, list(waypoint.tests), complete, segment.ended_by,
                     reached, measures, checks, failed)


# --- Scoring the whole flight ------------------------------------------------

@dataclass
class RowResult:
    status: str
    waypoints: list
    detail: str = ''


@dataclass
class FlightReport:
    stage: str
    legs: list
    rows: dict                 # row ID -> RowResult
    overall: str
    notes: list
    failsafe_mode: str = ''    # mode PX4 switched to on offboard loss, if it happened


def _worst(statuses):
    return min(statuses, key=STATUS_ORDER.index)


def _mode_name(nav_state):
    return NAV_STATE_NAMES.get(int(nav_state), f'nav_state {int(nav_state)}')


def score_flight(flight, config, stage):
    if stage not in config.thresholds:
        raise ValueError(f'unknown stage "{stage}" (expected one of {", ".join(STAGES)})')
    limits = config.thresholds[stage]
    notes = []

    segments = setpoint_segments(flight)
    matched, unmatched = match_segments(segments, config.waypoints)
    for segment in unmatched:
        position = [round(float(v), 2) for v in segment.pos]
        notes.append(f'Setpoint {position} at {segment.start:.1f} s '
                     'is not in the path file. Was the right --path used?')

    # Each leg starts from the previous commanded setpoint (the first from
    # wherever the drone was when Offboard mode started).
    waypoint_of = {id(segment): index for index, segment in matched}
    legs, previous = [], None
    for segment in segments:
        index = waypoint_of.get(id(segment))
        if index is not None:
            if previous is None:
                first = np.searchsorted(flight.pos_t, segment.start)
                start_pos = flight.pos[min(first, len(flight.pos) - 1)]
            else:
                start_pos = previous.pos
            legs.append(score_leg(index, config.waypoints[index], start_pos, segment, flight, limits))
        previous = segment
        if segment.ended_by == 'mode_change':
            mode = _mode_name(flight.nav_state[np.searchsorted(flight.status_t, segment.end)])
            notes.append(f'Left Offboard mode for {mode} at {segment.end:.1f} s without a '
                         'failsafe: a safety-pilot takeover or a mode switch from the ground station?')

    rows = _waypoint_rows(config, legs)
    rows.update(_flight_rows(flight, segments, limits))
    failsafe_mode = rows['F1'].detail if rows['F1'].status in ('PASS', 'FAIL') else ''

    overall = _overall(rows, unmatched, legs, config)
    return FlightReport(stage, legs, rows, overall, notes, failsafe_mode)


def _waypoint_rows(config, legs):
    by_waypoint = {leg.waypoint: leg for leg in legs}
    rows = {}
    for index, waypoint in enumerate(config.waypoints):
        for test in waypoint.tests:
            if test in FLIGHT_ROWS:
                continue
            leg = by_waypoint.get(index)
            status = leg.checks[test] if leg else 'NOT FLOWN'
            row = rows.setdefault(test, RowResult(status, []))
            row.waypoints.append(index)
            row.status = _worst([row.status, status])

    for test, row in rows.items():
        failed = sorted({name for index in row.waypoints if index in by_waypoint
                         for name in by_waypoint[index].failed_measures.get(test, [])})
        if failed:
            row.detail = ('failed: ' if row.status == 'FAIL' else 'missing: ') + ', '.join(failed)
    return rows


def _flight_rows(flight, segments, limits):
    rows = {}

    # Tilt over all Offboard time, whether or not the setpoints matched the path.
    in_offboard = np.zeros(len(flight.att_t), dtype=bool)
    for segment in segments:
        in_offboard |= (flight.att_t >= segment.start) & (flight.att_t <= segment.end)
    if in_offboard.any():
        tilt = float(np.max(flight.tilt_deg[in_offboard]))
        rows['R1'] = RowResult('PASS' if tilt <= limits['tilt_max_deg'] else 'FAIL', [],
                               f'max tilt {tilt:.1f}°')
    else:
        rows['R1'] = RowResult('NO DATA', [], 'no attitude data in Offboard mode')

    armed = np.flatnonzero(flight.arming_state == ARMING_STATE_ARMED)
    offboard = np.flatnonzero((flight.arming_state == ARMING_STATE_ARMED)
                              & (flight.nav_state == NAV_STATE_OFFBOARD))
    if not len(offboard):
        rows['S1'] = RowResult('FAIL', [], 'never armed and in Offboard mode')
    else:
        delay = flight.status_t[offboard[0]] - flight.status_t[armed[0]]
        rows['S1'] = RowResult('PASS' if delay <= S1_TIMEOUT_S else 'FAIL', [],
                               f'armed to Offboard in {delay:.1f} s')

    landing = next((s for s in segments if s.ended_by == 'land'), None)
    if landing is None:
        rows['L1'] = RowResult('NOT TESTED', [], 'no landing commanded in Offboard mode')
    else:
        disarmed = np.flatnonzero((flight.status_t >= landing.end)
                                  & (flight.arming_state != ARMING_STATE_ARMED))
        if len(disarmed):
            rows['L1'] = RowResult('PASS', [],
                                   f'landed and disarmed {flight.status_t[disarmed[0]] - landing.end:.1f} s '
                                   'after Land mode started')
        else:
            rows['L1'] = RowResult('FAIL', [], 'Land mode started but no disarm in the log')

    lost = next((s for s in segments if s.ended_by == 'failsafe'), None)
    if lost is None:
        rows['F1'] = RowResult('NOT TESTED', [], 'no offboard-loss failsafe in this flight')
    else:
        nav_state = flight.nav_state[np.searchsorted(flight.status_t, lost.end)]
        rows['F1'] = RowResult('PASS' if nav_state in ACCEPTABLE_FAILSAFE_MODES else 'FAIL', [],
                               _mode_name(nav_state))
    return rows


def _overall(rows, unmatched, legs, config):
    if rows['F1'].status != 'NOT TESTED':
        # Failsafe run: the path is cut short on purpose, so only F1 and the
        # legs that were actually flown to completion count.
        return 'FAIL' if any(row.status == 'FAIL' for row in rows.values()) else 'PASS'

    # F1 is tested in its own run, so NOT TESTED doesn't make a normal run incomplete.
    statuses = [row.status for row in rows.values()]
    if 'FAIL' in statuses:
        return 'FAIL'
    flown = {leg.waypoint for leg in legs if leg.complete}
    if (unmatched or len(flown) < len(config.waypoints)
            or any(s in ('INCOMPLETE', 'NO DATA', 'NOT FLOWN') for s in statuses)):
        return 'INCOMPLETE'
    return 'PASS'


# --- Report ------------------------------------------------------------------

def _fmt(value, digits=2):
    if value is None:
        return '—'
    if math.isinf(value):
        return 'not settled'
    return f'{value:.{digits}f}'


def _max_measure(legs, name, prefix):
    values = [leg.measures[name] for leg in legs
              if leg.measures[name] is not None and any(t.startswith(prefix) for t in leg.tests)]
    return max(values) if values else None


def report_to_dict(report, meta):
    def clean(value):
        if isinstance(value, float) and math.isinf(value):
            return 'inf'
        return value

    return {
        **meta,
        'stage': report.stage,
        'overall': report.overall,
        'rows': {test: {'status': row.status, 'waypoints': row.waypoints, 'detail': row.detail}
                 for test, row in sorted(report.rows.items())},
        'legs': [{'waypoint': leg.waypoint, 'tests': leg.tests, 'complete': leg.complete,
                  'ended_by': leg.ended_by, 'reached': leg.reached, 'checks': leg.checks,
                  'measures': {k: clean(v) for k, v in leg.measures.items()}}
                 for leg in report.legs],
        'notes': report.notes,
    }


def format_markdown(report, meta):
    """meta: log, date, px4, path (strings shown in the report)."""
    lines = [
        f'# Milestone 1 flight score: {meta["log"]}',
        '',
        '| | |',
        '|---|---|',
        f'| Overall | **{report.overall}** |',
        f'| Stage | {report.stage.upper()} |',
        f'| Flight log | `{meta["log"]}` ({meta["date"]}) |',
        f'| PX4 | {meta["px4"]} |',
        f'| Path file | `{meta["path"]}` |',
        '',
        '## Matrix rows',
        '',
        '| Row | Result | Waypoints | Detail |',
        '|---|---|---|---|',
    ]
    for test in sorted(report.rows, key=lambda t: (t[0], int(t[1:]) if t[1:].isdigit() else 0, t)):
        row = report.rows[test]
        waypoints = ', '.join(str(i) for i in row.waypoints) or 'whole flight'
        lines.append(f'| {test} | {row.status} | {waypoints} | {row.detail} |')

    lines += [
        '',
        '## Legs',
        '',
        '| WP | Tests | Reached | Pos err (m) | Overshoot (m) | Settle (s) | Cross-track (m) '
        '| Heading err (°) | Hover drift (m) | Max tilt (°) | Result |',
        '|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    for leg in report.legs:
        m = leg.measures
        result = _worst(list(leg.checks.values())) if leg.checks else ('—' if leg.complete else 'INCOMPLETE')
        lines.append(
            f'| {leg.waypoint} | {", ".join(leg.tests) or "—"} | {"yes" if leg.reached else "no"} '
            f'| {_fmt(m["position_error_m"])} | {_fmt(m["overshoot_m"])} | {_fmt(m["settle_time_s"], 1)} '
            f'| {_fmt(m["cross_track_m"])} | {_fmt(m["heading_error_deg"], 1)} | {_fmt(m["hover_drift_m"])} '
            f'| {_fmt(m["tilt_max_deg"], 1)} | {result} |')

    if report.notes:
        lines += ['', '## Notes', ''] + [f'- {note}' for note in report.notes]

    rows_covered = ', '.join(test for test, row in sorted(report.rows.items())
                             if row.status not in ('NO DATA', 'NOT FLOWN', 'NOT TESTED', 'NOT SCORED'))
    log_row = [
        meta['date'], report.stage.upper(), meta['px4'], meta['path'], rows_covered,
        _fmt(_max_measure(report.legs, 'position_error_m', 'T')),
        _fmt(_max_measure(report.legs, 'overshoot_m', 'T')),
        _fmt(_max_measure(report.legs, 'settle_time_s', 'T'), 1),
        _fmt(_max_measure(report.legs, 'heading_error_deg', 'Y'), 1),
        _fmt(_max_measure(report.legs, 'hover_drift_m', 'H')),
        report.failsafe_mode or '—', report.overall, meta['log'], '',
    ]
    lines += [
        '',
        '## Results log row',
        '',
        'Paste into the results log in documentation/milestone-1-single-vehicle.md:',
        '',
        '```',
        '| ' + ' | '.join(log_row) + ' |',
        '```',
        '',
        'Oscillation is not scored automatically: judge it from the plots and add it to the notes.',
    ]
    return '\n'.join(lines) + '\n'
