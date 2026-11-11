"""The competition grid, and where each drone is on it.

The field is 80 ft by 300 ft in 2 ft squares: 40 columns (x, west to east) by 150 rows
(y, south to north), with (0, 0) at the south-west corner. It is only roughly north-south,
so four measured WGS84 corners set the grid, and every drone's position goes through the
one transform here, so all of them agree.

Drone positions come from PX4: local NED metres from the drone's own EKF origin, which
PX4 publishes as ref_lat/ref_lon. The projection below is PX4's own (MapProjection, an
azimuthal equidistant projection), so the latitude and longitude match what PX4 would say.

Pure Python, no ROS.
"""
import math

FOOT_M = 0.3048
SQUARE_M = 2 * FOOT_M
COLUMNS = 40
ROWS = 150
WIDTH_M = COLUMNS * SQUARE_M
LENGTH_M = ROWS * SQUARE_M
EARTH_RADIUS_M = 6371000.0  # PX4's CONSTANTS_RADIUS_OF_EARTH
CORNERS = ('south_west', 'south_east', 'north_east', 'north_west')
CORNER_LABELS = {'south_west': 'South-west', 'south_east': 'South-east',
                 'north_east': 'North-east', 'north_west': 'North-west'}
# A side more than one square off its length means a corner was typed or measured wrong.
FIT_TOLERANCE_M = SQUARE_M


class AreaError(ValueError):
    pass


def local_to_global(ref_lat, ref_lon, north, east):
    """PX4's MapProjection::reproject: NED metres from (ref_lat, ref_lon) to degrees."""
    lat0, lon0 = math.radians(ref_lat), math.radians(ref_lon)
    x_rad, y_rad = north / EARTH_RADIUS_M, east / EARTH_RADIUS_M
    c = math.hypot(x_rad, y_rad)
    if c == 0:
        return ref_lat, ref_lon
    sin_c, cos_c = math.sin(c), math.cos(c)
    lat = math.asin(cos_c * math.sin(lat0) + x_rad * sin_c * math.cos(lat0) / c)
    lon = lon0 + math.atan2(y_rad * sin_c, c * math.cos(lat0) * cos_c - x_rad * math.sin(lat0) * sin_c)
    return math.degrees(lat), math.degrees(lon)


def global_to_local(ref_lat, ref_lon, lat, lon):
    """PX4's MapProjection::project: degrees to NED metres (north, east) from (ref_lat, ref_lon)."""
    lat0, lon0 = math.radians(ref_lat), math.radians(ref_lon)
    lat1, lon1 = math.radians(lat), math.radians(lon)
    cos_d_lon = math.cos(lon1 - lon0)
    arg = max(-1.0, min(1.0, math.sin(lat0) * math.sin(lat1) + math.cos(lat0) * math.cos(lat1) * cos_d_lon))
    c = math.acos(arg)
    k = 1.0 if abs(c) < 1e-12 else c / math.sin(c)
    north = k * (math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * cos_d_lon) * EARTH_RADIUS_M
    east = k * math.cos(lat1) * math.sin(lon1 - lon0) * EARTH_RADIUS_M
    return north, east


def parse_corners(data):
    """Four corners as {'south_west': {'lat': .., 'lon': ..}, ...}, checked, as (lat, lon) in CORNERS order."""
    if not isinstance(data, dict):
        raise AreaError('corners must be a mapping of south_west, south_east, north_east and north_west')
    corners = []
    for name in CORNERS:
        point = data.get(name)
        if not isinstance(point, dict):
            raise AreaError(f'{CORNER_LABELS[name]} corner is missing')
        try:
            lat, lon = float(point['lat']), float(point['lon'])
        except (KeyError, TypeError, ValueError):
            raise AreaError(f'{CORNER_LABELS[name]} corner needs a latitude and longitude in degrees') from None
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise AreaError(f'{CORNER_LABELS[name]} corner is not a real latitude and longitude')
        corners.append((lat, lon))
    return corners


class Field:
    """The grid set by four corners, in CORNERS order. Columns run along the south side,
    rows along the west side, both from the south-west corner.
    """

    def __init__(self, corners):
        self.corners = [tuple(corner) for corner in corners]
        self.origin = self.corners[0]
        # (east, north) metres of each corner from the south-west one.
        points = [tuple(reversed(global_to_local(*self.origin, *corner))) for corner in self.corners]
        self.points = points
        sw, se, ne, nw = points
        self.across = (se[0] - sw[0], se[1] - sw[1])  # South side: one step per column.
        self.along = (nw[0] - sw[0], nw[1] - sw[1])   # West side: one step per row.
        self.determinant = self.across[0] * self.along[1] - self.across[1] * self.along[0]
        if abs(self.determinant) < 1e-6:
            raise AreaError('the corners are all on one line')
        self.sides_m = {
            'south': _distance(sw, se), 'north': _distance(nw, ne),
            'west': _distance(sw, nw), 'east': _distance(se, ne),
        }
        self.flags = self._fit_flags()

    def cell(self, lat, lon):
        """Where a position falls: fractional column and row, and whether that is on the field."""
        north, east = global_to_local(*self.origin, lat, lon)
        # Solve east,north = a * across + b * along for (a, b).
        a = (east * self.along[1] - north * self.along[0]) / self.determinant
        b = (self.across[0] * north - self.across[1] * east) / self.determinant
        column, row = a * COLUMNS, b * ROWS
        return {'col': round(column, 2), 'row': round(row, 2),
                'on_field': 0 <= column < COLUMNS and 0 <= row < ROWS}

    def view(self):
        return {
            'corners': {name: {'lat': lat, 'lon': lon} for name, (lat, lon) in zip(CORNERS, self.corners)},
            'sides_m': {side: round(length, 2) for side, length in self.sides_m.items()},
            'flags': list(self.flags),
        }

    def _fit_flags(self):
        """Ways the corners don't fit an 80 by 300 ft field. Shown, not refused: the team
        may have nothing better on the day.
        """
        flags = []
        if self.determinant < 0:
            flags.append('the corners are in the wrong order: going south-west, south-east, '
                         'north-east, north-west should go anticlockwise seen from above')
        for side, expected, feet in (('south', WIDTH_M, 80), ('north', WIDTH_M, 80),
                                     ('west', LENGTH_M, 300), ('east', LENGTH_M, 300)):
            length = self.sides_m[side]
            if abs(length - expected) > FIT_TOLERANCE_M:
                flags.append(f'the {side} side is {length:.1f} m, not {expected:.1f} m ({feet} ft)')
        diagonal = math.hypot(WIDTH_M, LENGTH_M)
        sw, se, ne, nw = self.points
        for name, length in (('south-west to north-east', _distance(sw, ne)),
                             ('south-east to north-west', _distance(se, nw))):
            if abs(length - diagonal) > FIT_TOLERANCE_M:
                flags.append(f'the {name} diagonal is {length:.1f} m, not {diagonal:.1f} m: '
                             'the corners are not square')
        return flags


def fit_around(lat, lon, east_m=2.0, north_m=2.0):
    """Corners of a true north-south field with (lat, lon) east_m and north_m in from its
    south-west corner. For trying the map in SITL, where there are no measured corners.
    """
    south_west = local_to_global(lat, lon, -north_m, -east_m)
    offsets = {'south_west': (0, 0), 'south_east': (0, WIDTH_M),
               'north_east': (LENGTH_M, WIDTH_M), 'north_west': (LENGTH_M, 0)}
    corners = {}
    for name in CORNERS:
        north, east = offsets[name]
        corner_lat, corner_lon = local_to_global(*south_west, north, east)
        corners[name] = {'lat': round(corner_lat, 8), 'lon': round(corner_lon, 8)}
    return corners


def _distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])
