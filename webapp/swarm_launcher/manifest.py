"""Mission manifests: one YAML file per mission, so adding a mission needs no web code.

Pure Python (no ROS imports) so it can be unit tested without a ROS install. The
hub only ever sends a drone what a manifest says, which is why these checks are
strict: nothing typed on a phone reaches a launch command.
"""
import re
from pathlib import Path

import yaml

SLUG = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
LAUNCH_FILE = re.compile(r'^[A-Za-z0-9_.-]+$')
RELATIVE_FILE = re.compile(r'^[A-Za-z0-9_-][A-Za-z0-9_./-]*$')
KNOWN_INPUTS = {'search_area'}


class ManifestError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise ManifestError(message)


def parse_manifest(data):
    """Check one loaded manifest and return it with every optional field filled in."""
    _require(isinstance(data, dict), 'manifest is not a mapping')
    mission_id = data.get('id')
    _require(isinstance(mission_id, str) and SLUG.match(mission_id), f'bad id: {mission_id!r}')
    name = data.get('name')
    _require(isinstance(name, str) and name.strip(), 'name is required')

    inputs = data.get('inputs') or []
    _require(isinstance(inputs, list) and set(inputs) <= KNOWN_INPUTS, f'unknown inputs: {inputs!r}')

    drones = data.get('drones')
    _require(isinstance(drones, dict) and drones, 'at least one drone is required')
    roster = {}
    for drone_id, fields in drones.items():
        _require(isinstance(drone_id, str) and SLUG.match(drone_id), f'bad drone id: {drone_id!r}')
        fields = fields or {}
        _require(isinstance(fields, dict), f'{drone_id}: fields must be a mapping')
        roster[drone_id] = {'role': str(fields.get('role', 'search'))}

    launch = data.get('launch')
    _require(isinstance(launch, dict), 'launch is required')
    package = launch.get('package')
    _require(isinstance(package, str) and SLUG.match(package), f'bad launch package: {package!r}')
    launch_file = launch.get('file')
    _require(isinstance(launch_file, str) and LAUNCH_FILE.match(launch_file),
             f'launch file must be a bare file name: {launch_file!r}')

    args = launch.get('args') or {}
    _require(isinstance(args, dict), 'launch args must be a mapping')
    for key, value in args.items():
        _require(isinstance(key, str) and SLUG.match(key), f'bad launch arg name: {key!r}')
        # YAML reads true and 10 as non-strings; quoting them keeps the launch
        # argument exactly as written ("true", not Python's "True").
        _require(isinstance(value, str), f'launch arg {key} must be a quoted string')

    waypoints = launch.get('waypoints')
    if waypoints is not None:
        _require(isinstance(waypoints, dict), 'waypoints must be a mapping')
        _require(isinstance(waypoints.get('package'), str) and SLUG.match(waypoints['package']),
                 'bad waypoints package')
        wp_file = waypoints.get('file')
        _require(isinstance(wp_file, str) and RELATIVE_FILE.match(wp_file) and '..' not in wp_file,
                 f'bad waypoints file: {wp_file!r}')

    return {
        'id': mission_id,
        'name': name.strip(),
        'spoken_name': str(data.get('spoken_name') or name).strip(),
        'inputs': inputs,
        'drones': roster,
        'launch': {'package': package, 'file': launch_file, 'args': args, 'waypoints': waypoints},
    }


def load_missions(directory):
    """Every valid manifest in `directory`, plus an error line for each bad file.

    One bad file must not take the other missions off the phone.
    """
    missions = {}
    errors = []
    for path in sorted(Path(directory).glob('*.yaml')):
        try:
            mission = parse_manifest(yaml.safe_load(path.read_text()))
            if mission['id'] in missions:
                raise ManifestError(f"duplicate id {mission['id']}")
            missions[mission['id']] = mission
        except (OSError, yaml.YAMLError, ManifestError) as error:
            errors.append(f'{path.name}: {error}')
    return missions, errors


def public_mission(mission):
    """The fields a phone needs. Launch details stay between the hub and the agents."""
    return {
        'id': mission['id'],
        'name': mission['name'],
        'inputs': mission['inputs'],
        'drones': mission['drones'],
    }
