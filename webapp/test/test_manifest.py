"""Unit tests for mission manifests."""
import copy
from pathlib import Path

import pytest
import yaml

from swarm_launcher.manifest import ManifestError, load_missions, parse_manifest, public_mission

MISSIONS = Path(__file__).resolve().parent.parent / 'missions'
GOOD = {
    'id': 'm1_square',
    'name': 'Square',
    'drones': {'drone1': {'role': 'search'}},
    'launch': {'package': 'pkg', 'file': 'a.launch.py', 'args': {'record': 'true'}},
}


def changed(path, value):
    data = copy.deepcopy(GOOD)
    target = data
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    return data


def test_good_manifest_fills_in_defaults():
    mission = parse_manifest(GOOD)
    assert mission['inputs'] == []
    assert mission['spoken_name'] == 'Square'
    assert mission['drones'] == {'drone1': {'role': 'search'}}


@pytest.mark.parametrize('path, value', [
    (['id'], 'has space'),
    (['id'], None),
    (['name'], '  '),
    (['inputs'], ['telepathy']),
    (['drones'], {}),
    (['drones'], {'bad id!': {}}),
    (['launch', 'package'], 'rm -rf'),
    (['launch', 'file'], '../../evil.py'),
    (['launch', 'file'], 'dir/a.launch.py'),
    (['launch', 'args'], {'bad key': 'x'}),
    (['launch', 'args'], {'record': True}),
    (['launch', 'waypoints'], {'package': 'pkg', 'file': '../secret.yaml'}),
])
def test_bad_manifest_is_refused(path, value):
    with pytest.raises(ManifestError):
        parse_manifest(changed(path, value))


def test_one_bad_file_keeps_the_other_missions(tmp_path):
    (tmp_path / 'good.yaml').write_text(yaml.safe_dump(GOOD))
    (tmp_path / 'bad.yaml').write_text('id: [unclosed')
    missions, errors = load_missions(tmp_path)
    assert list(missions) == ['m1_square']
    assert len(errors) == 1 and errors[0].startswith('bad.yaml')


def test_duplicate_id_is_reported(tmp_path):
    (tmp_path / 'a.yaml').write_text(yaml.safe_dump(GOOD))
    (tmp_path / 'b.yaml').write_text(yaml.safe_dump(GOOD))
    missions, errors = load_missions(tmp_path)
    assert len(missions) == 1
    assert 'duplicate' in errors[0]


def test_phone_never_sees_launch_details():
    assert 'launch' not in public_mission(parse_manifest(GOOD))


def test_shipped_missions_are_valid_and_never_own_sitl():
    missions, errors = load_missions(MISSIONS)
    assert errors == []
    assert missions
    for mission in missions.values():
        # Stopping a launch that owns SITL also stops PX4 and Gazebo.
        assert mission['launch']['args']['launch_sitl'] == 'false'
