"""The competition grid: PX4's projection, the corner checks and the column/row transform."""
import math

import pytest

from swarm_launcher.grid import (
    COLUMNS, LENGTH_M, ROWS, SQUARE_M, WIDTH_M, AreaError, Field, fit_around, global_to_local,
    local_to_global, parse_corners)

# PX4's Gazebo world origin, where SITL drones start.
HOME = (47.397971057728974, 8.546163739800146)


def test_projection_round_trips():
    for north, east in [(0, 0), (10, 0), (0, 10), (-91.44, 24.38), (250, -130)]:
        lat, lon = local_to_global(*HOME, north, east)
        back = global_to_local(*HOME, lat, lon)
        assert back == pytest.approx((north, east), abs=1e-6)


def test_projection_matches_px4_at_small_distances():
    # 1e-5 degrees of latitude is about 1.11 m everywhere.
    north, east = global_to_local(*HOME, HOME[0] + 1e-5, HOME[1])
    assert north == pytest.approx(1.112, abs=0.001) and east == pytest.approx(0, abs=1e-9)
    # A degree of longitude shrinks by cos(latitude).
    north, east = global_to_local(*HOME, HOME[0], HOME[1] + 1e-5)
    assert east == pytest.approx(1.112 * math.cos(math.radians(HOME[0])), abs=0.001)


def square_field():
    return Field(parse_corners(fit_around(*HOME, east_m=0, north_m=0)))


def test_a_true_field_has_no_flags_and_the_right_sides():
    field = square_field()
    assert field.flags == []
    assert field.sides_m['south'] == pytest.approx(WIDTH_M, abs=0.01)
    assert field.sides_m['west'] == pytest.approx(LENGTH_M, abs=0.01)


def test_cells_count_from_the_south_west_corner():
    field = square_field()
    sw = field.corners[0]
    # The middle of square (0, 0), and of the north-east square.
    assert field.cell(*local_to_global(*sw, SQUARE_M / 2, SQUARE_M / 2)) == {'col': 0.5, 'row': 0.5, 'on_field': True}
    ne = field.cell(*local_to_global(*sw, LENGTH_M - SQUARE_M / 2, WIDTH_M - SQUARE_M / 2))
    assert ne == {'col': COLUMNS - 0.5, 'row': ROWS - 0.5, 'on_field': True}
    outside = field.cell(*local_to_global(*sw, -1, 5))
    assert outside['on_field'] is False and outside['row'] < 0


def test_a_rotated_field_still_maps_its_own_corners():
    """The field is only roughly north-south: turn it 20 degrees and the grid turns with it."""
    angle = math.radians(20)

    def at(across, along):
        east = across * math.cos(angle) - along * math.sin(angle)
        north = across * math.sin(angle) + along * math.cos(angle)
        lat, lon = local_to_global(*HOME, north, east)
        return {'lat': lat, 'lon': lon}

    field = Field(parse_corners({'south_west': at(0, 0), 'south_east': at(WIDTH_M, 0),
                                 'north_east': at(WIDTH_M, LENGTH_M), 'north_west': at(0, LENGTH_M)}))
    assert field.flags == []
    middle = at(WIDTH_M / 2, LENGTH_M / 2)
    assert field.cell(middle['lat'], middle['lon']) == {'col': 20.0, 'row': 75.0, 'on_field': True}


def test_corners_that_do_not_fit_are_flagged():
    corners = fit_around(*HOME, east_m=0, north_m=0)
    lat, lon = corners['north_east']['lat'], corners['north_east']['lon']
    north, east = global_to_local(*HOME, lat, lon)
    moved_lat, moved_lon = local_to_global(*HOME, north + 3, east)
    corners['north_east'] = {'lat': moved_lat, 'lon': moved_lon}
    flags = Field(parse_corners(corners)).flags
    assert any(flag.startswith('the east side is 94.4 m') for flag in flags)
    assert any('not square' in flag for flag in flags)


def test_corners_in_the_wrong_order_are_flagged():
    corners = fit_around(*HOME)
    corners['south_east'], corners['north_west'] = corners['north_west'], corners['south_east']
    assert 'wrong order' in Field(parse_corners(corners)).flags[0]


@pytest.mark.parametrize('data, message', [
    ([], 'must be a mapping'),
    ({}, 'South-west corner is missing'),
    ({'south_west': {'lat': 'x', 'lon': 1}}, 'needs a latitude and longitude'),
    ({'south_west': {'lat': 91, 'lon': 1}}, 'not a real latitude'),
    ({'south_west': {'lat': float('nan'), 'lon': 1}}, 'not a real latitude'),
])
def test_bad_corners_are_refused(data, message):
    with pytest.raises(AreaError, match=message):
        parse_corners(data)


def test_corners_on_one_line_are_refused():
    point = {'lat': HOME[0], 'lon': HOME[1]}
    with pytest.raises(AreaError, match='one line'):
        Field(parse_corners({name: point for name in ('south_west', 'south_east', 'north_east', 'north_west')}))


def test_fit_around_puts_the_point_just_inside_the_south_west_corner():
    field = Field(parse_corners(fit_around(*HOME)))
    cell = field.cell(*HOME)
    assert cell['col'] == pytest.approx(2 / SQUARE_M, abs=0.01)
    assert cell['row'] == pytest.approx(2 / SQUARE_M, abs=0.01)
    # A second SITL drone 20 m east is still on the field.
    east_lat, east_lon = local_to_global(*HOME, 0, 20)
    assert field.cell(east_lat, east_lon)['on_field']
