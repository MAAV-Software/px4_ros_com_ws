"""Alternative four-drone scan patterns for milestone 4, beside the classic 3-mowers scan.

Each strategy gives every drone a world route (east, north metres, field SW corner at (0, 0))
and names which drone checks the planned path after coverage. All four drones cover the field;
when coverage is done one of them (the checker) re-flies the best path as a second look. The
routes reuse the geometry in scan_routes.py (rows, altitude, spawns).

Strategies:
- mow4: all four lawnmower; the nine rows are split 3/2/2/2 across the drones.
- swarm2x2: a 2x2 block (two east columns, two north ranks) sweeps the field in bands.
- quadrant: the field is quartered; each drone scans one quadrant (SW, SE, NW, NE).

No ROS here. Regenerate the route YAMLs with:
  python3 -m milestone4_multivehicle.strategies resource/
"""
import os
import sys

from milestone4_multivehicle import scan_routes as s

# swarm2x2 block geometry: two camera columns this far east of the block centre, two ranks
# this far apart north-south, so each square the block passes gets looked at twice.
BLOCK_HALF_EAST_M = 1.6
BLOCK_RANK_M = 4.0
# quadrant drones in the south half come back to land here, south of the field, near the spawns.
SOUTH_LAND_M = -3.0
VEHICLES = len(s.SPAWNS)


def _mow_rows(eastings, lo, hi, land):
    """A lawnmower over `eastings` between northings lo and hi, ending at world point `land`.
    Each row alternates direction; the first starts at lo."""
    points = []
    for i, east in enumerate(eastings):
        ends = [(east, lo), (east, hi)]
        points += ends if i % 2 == 0 else ends[::-1]
    if points:
        points.append(land if land is not None else (points[-1][0], s.LANDING_NORTH_M))
    return points


# --- mow4: all four lawnmower -------------------------------------------------------------

def _mow4_groups():
    """The row eastings each drone mows: 3 for the first, then 2, 2, 2."""
    eastings = s.row_eastings()
    return [eastings[0:3], eastings[3:5], eastings[5:7], eastings[7:9]]


def _mow4_route(drone):
    south, north = s.row_ends()
    return _mow_rows(_mow4_groups()[drone], south, north, None)


# --- swarm2x2: a 2x2 block sweeps the field in bands --------------------------------------

_SWARM_OFFSETS = [(-BLOCK_HALF_EAST_M, 0.0), (BLOCK_HALF_EAST_M, 0.0),
                  (-BLOCK_HALF_EAST_M, BLOCK_RANK_M), (BLOCK_HALF_EAST_M, BLOCK_RANK_M)]


def _swarm_band_centres():
    """East of each band the block sweeps: its two columns, BLOCK_HALF_EAST_M either side,
    tile the field west to east."""
    step = 2 * BLOCK_HALF_EAST_M
    columns, east = [], BLOCK_HALF_EAST_M
    while east <= s.FIELD_EAST_M + 1e-6:
        columns.append(round(east, 3))
        east += step
    if len(columns) % 2:
        columns.append(round(columns[-1] + step, 3))  # pair up the last column
    return [round((columns[i] + columns[i + 1]) / 2, 3) for i in range(0, len(columns), 2)]


def _swarm_route(drone):
    south, north = s.row_ends()
    off_east, off_north = _SWARM_OFFSETS[drone]
    points = []
    for i, centre in enumerate(_swarm_band_centres()):
        ends = [(centre, south), (centre, north)]
        points += ends if i % 2 == 0 else ends[::-1]
    points = [(round(east + off_east, 3), round(north_ + off_north, 3)) for east, north_ in points]
    points.append((points[-1][0], s.LANDING_NORTH_M))
    return points


# --- quadrant: each drone scans one quarter -----------------------------------------------

def _quadrant_halves():
    """(west eastings, east eastings): the field's width split in two."""
    eastings = s.row_eastings()
    return eastings[0:5], eastings[5:9]


def _mid_north():
    return round(s.FIELD_NORTH_M / 2, 3)


def _quadrant_route(drone):
    south, north = s.row_ends()
    mid = _mid_north()
    west, east = _quadrant_halves()
    if drone == 0:    # south-west: mow the west rows over the south half, land south
        return _mow_rows(west, south, mid, (west[0], SOUTH_LAND_M))
    if drone == 1:    # south-east
        return _mow_rows(east, south, mid, (east[0], SOUTH_LAND_M))
    if drone == 2:    # north-west: mow the west rows over the north half, land north
        return _mow_rows(west, mid, north, (west[-1], s.LANDING_NORTH_M))
    return _mow_rows(east, mid, north, (east[-1], s.LANDING_NORTH_M))  # north-east


class ScanStrategy:
    def __init__(self, strategy_id, label, description, route_fn, checker):
        self.id = strategy_id
        self.label = label
        self.description = description
        self._route_fn = route_fn
        self.checker = checker

    def world_route(self, drone):
        """[(east, north)] after takeoff, ending at the drone's land or (for the checker) hold
        point. The checker holds there and then flies the survey's best path."""
        return self._route_fn(drone)


STRATEGIES = {
    'mow4': ScanStrategy('mow4', 'All four lawnmower',
                         'Four drones each mow a share of the nine rows', _mow4_route, 3),
    'swarm2x2': ScanStrategy('swarm2x2', '2x2 swarm',
                             'A 2x2 block sweeps the field in bands', _swarm_route, 3),
    'quadrant': ScanStrategy('quadrant', 'Quadrants',
                             'Each drone scans one quadrant (SW, SE, NW, NE)', _quadrant_route, 0),
}


def strategy_ids():
    return list(STRATEGIES)


def survey_config(strategy_id):
    """Keyword arguments for mines.MineSurvey so all four drones cover the field and the named
    checker verifies the path once coverage is done."""
    strategy = STRATEGIES[strategy_id]
    ends = {drone: strategy.world_route(drone)[-1] for drone in range(VEHICLES)}
    return {'mowers': list(range(VEHICLES)), 'checker': strategy.checker,
            'coverage_ends': ends, 'check_after_coverage': True}


def waypoints(strategy_id, drone):
    """Local North/East/Down waypoints from the drone's spawn, like scan_routes.waypoints."""
    spawn = s.SPAWNS[drone]
    route = [spawn] + STRATEGIES[strategy_id].world_route(drone)
    out = []
    for i, point in enumerate(route):
        hold = s.TAKEOFF_HOLD_S if i == 0 else s.FINAL_HOLD_S if i == len(route) - 1 else 0.0
        out.append({'pos': [round(point[1] - spawn[1], 3), round(point[0] - spawn[0], 3), -s.ALTITUDE_M],
                    'yaw': round(s.YAW, 4), 'hold_s': hold})
    return out


def yaml_text(strategy_id, drone):
    strategy = STRATEGIES[strategy_id]
    spawn = s.SPAWNS[drone]
    role = ('Checker: holds after covering, then flies the survey\'s best path'
            if drone == strategy.checker else 'Mows its share of the field and lands')
    lines = [
        f'# Milestone 4 {strategy.label} scan, drone {drone} (system_id {drone + 1}). Generated by',
        '# milestone4_multivehicle/strategies.py; edit that and regenerate, not this file.',
        f'# {strategy.description}. {role}.',
        f'# Spawn: world east {spawn[0]:g}, north {spawn[1]:g} m; field SW corner at (0, 0).',
        '# Local North/East/Down from the spawn, yaw radians.',
        'waypoints:',
    ]
    for way in waypoints(strategy_id, drone):
        lines.append(f"  - {{pos: [{', '.join(f'{v:g}' for v in way['pos'])}], "
                     f"yaw: {way['yaw']:g}, hold_s: {way['hold_s']:g}}}")
    return '\n'.join(lines) + '\n'


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    folder = argv[0] if argv else 'resource'
    for strategy_id in STRATEGIES:
        for drone in range(VEHICLES):
            path = os.path.join(folder, f'milestone4_{strategy_id}_{drone}.yaml')
            with open(path, 'w') as stream:
                stream.write(yaml_text(strategy_id, drone))
            print(f'wrote {path} ({len(waypoints(strategy_id, drone))} waypoints)')


if __name__ == '__main__':
    main()
