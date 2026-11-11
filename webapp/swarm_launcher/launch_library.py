"""The launch files a drone can run: every package folder in its workspace's src, the
launch files in each, and the arguments each file declares.

Pure Python with no ROS imports. Finding a built file and reading its arguments are
passed in, so the real versions (ament index, a `launch_args` subprocess) and the
pretend ones (for --fake and the tests) share everything else.

Every run starts from this list. Nothing the phone sends names a path, only a package
and a file that this drone found in its own src, so a phone can't launch anything else.
"""
import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

PACKAGE_NAME = re.compile(r'<name>\s*([A-Za-z0-9_]+)\s*</name>')
LAUNCH_SUFFIXES = ('_launch.py', '.launch.py')
# Starting the simulator from a mission would make Abort take the simulator down too.
HIDDEN_PREFIXES = ('px4_sitl',)
# Set on every launch by the launcher itself, so not editable from the phone. topic_prefix
# and system_id say which PX4 this drone is; launch_sitl is always false.
SET_BY_LAUNCHER = ('launch_sitl', 'topic_prefix', 'system_id')
ARG_NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')
MAX_VALUE_LENGTH = 1000
DESCRIBE_TIMEOUT_S = 10.0
DESCRIBE_RETRY_S = 10.0

PENDING = 'pending'
READY = 'ready'
ERROR = 'error'


def is_listed(name):
    return name.endswith(LAUNCH_SUFFIXES) and not name.startswith(HIDDEN_PREFIXES)


def package_name(folder):
    """The name in package.xml, which need not match the folder's name."""
    try:
        match = PACKAGE_NAME.search((folder / 'package.xml').read_text())
    except OSError:
        return None
    return match.group(1) if match else None


def scan(src_dir, find_installed):
    """[{'folder', 'package', 'files': [{'file', 'built'}]}], one per package folder in
    src that has launch files to list. `find_installed(package, file)` returns the path
    `ros2 launch` would run, or None when the package or file isn't built.
    """
    folders = []
    try:
        candidates = sorted(Path(src_dir).iterdir())
    except OSError:
        return folders
    for folder in candidates:
        launch_dir = folder / 'launch'
        if not launch_dir.is_dir():
            continue
        package = package_name(folder)
        if package is None:
            continue
        files = [{'file': path.name, 'built': find_installed(package, path.name) is not None}
                 for path in sorted(launch_dir.iterdir()) if path.is_file() and is_listed(path.name)]
        if files:
            folders.append({'folder': folder.name, 'package': package, 'files': files})
    return folders


def ros_find_installed(package, file):
    """Where `ros2 launch package file` finds the file: the package's share directory."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        share = Path(get_package_share_directory(package))
    except (PackageNotFoundError, ValueError):
        return None
    for candidate in (share / file, share / 'launch' / file):
        if candidate.is_file():
            return str(candidate)
    return None


def subprocess_describer(path):
    """The arguments `path` declares, read by launch_args.py in its own process."""
    result = subprocess.run([sys.executable, '-m', 'swarm_launcher.launch_args', path],
                            capture_output=True, text=True, timeout=DESCRIBE_TIMEOUT_S)
    if result.returncode != 0:
        lines = [line for line in result.stderr.strip().splitlines() if line.strip()]
        raise RuntimeError(lines[-1] if lines else f'exit code {result.returncode}')
    return json.loads(result.stdout)


def _in_background(work):
    threading.Thread(target=work, daemon=True).start()


class LaunchLibrary:
    """`list_folders()` gives the catalog (see scan), `find_installed(package, file)` the path
    `ros2 launch` would run, and `describer(path)` that file's arguments.
    """

    def __init__(self, list_folders, find_installed, describer, run=_in_background, now=time.monotonic):
        self.list_folders = list_folders
        self.find_installed = find_installed
        self.describer = describer
        self.run = run
        self.now = now
        self.lock = threading.Lock()
        self.described = {}

    def catalog(self):
        return self.list_folders()

    def find(self, package, file):
        """This drone's entry for one file, or None if its src has no such file."""
        for folder in self.catalog():
            if folder['package'] == package:
                for entry in folder['files']:
                    if entry['file'] == file:
                        return {'folder': folder['folder'], 'package': package, **entry}
        return None

    def describe(self, package, file):
        """(PENDING, None), (READY, arguments) or (ERROR, reason). Never waits: reading a
        file's arguments runs in the background, and the answer is kept until the
        installed file changes.
        """
        path = self.find_installed(package, file)
        if path is None:
            return ERROR, f'{package} has no built {file}'
        try:
            changed = os.stat(path).st_mtime
        except OSError:
            changed = None
        key = (package, file)
        with self.lock:
            entry = self.described.get(key)
            stale = (entry is None or entry['path'] != path or entry['changed'] != changed
                     or (entry['state'] == ERROR and self.now() - entry['at'] > DESCRIBE_RETRY_S))
            if not stale:
                return entry['state'], entry['result']
            self.described[key] = {'path': path, 'changed': changed, 'state': PENDING,
                                   'result': None, 'at': self.now()}

        def work():
            try:
                state, result = READY, self.describer(path)
            except Exception as error:  # Any failure in someone's launch file is just a reason to show.
                state, result = ERROR, f'could not read its arguments: {error}'
            with self.lock:
                current = self.described.get(key)
                if current and current['path'] == path and current['changed'] == changed:
                    current.update(state=state, result=result, at=self.now())

        self.run(work)
        with self.lock:
            entry = self.described[key]
            return entry['state'], entry['result']


def ros_library(src_dir):
    """The real thing: this drone's src, built files from the ament index."""
    src_dir = Path(src_dir).expanduser()
    return LaunchLibrary(lambda: scan(src_dir, ros_find_installed), ros_find_installed, subprocess_describer)


def override_problems(args):
    """What is wrong with argument values typed on a phone. Values go to `ros2 launch` as
    one argv entry each, never through a shell, but are still kept to plain text.
    """
    if not isinstance(args, dict):
        return ['launch arguments must be a mapping']
    problems = []
    for name, value in args.items():
        if not isinstance(name, str) or not ARG_NAME.match(name):
            problems.append(f'bad argument name: {name!r}')
        elif name in SET_BY_LAUNCHER:
            problems.append(f'{name} is set by the launcher')
        elif (not isinstance(value, str) or not value or len(value) > MAX_VALUE_LENGTH
              or any(ord(character) < 32 for character in value)):
            problems.append(f'bad value for {name}')
    return problems


# --- A pretend library for --fake and the tests ----------------------------------

FAKE_CATALOG = [
    {'folder': 'milestone1_singlevehicle', 'package': 'milestone1_singlevehicle', 'files': [
        {'file': 'offboard_solution.launch.py', 'built': True},
        {'file': 'square_launch.py', 'built': True},
        {'file': 'updown_launch.py', 'built': True},
    ]},
    {'folder': 'milestone3_multivehicle_anti-collision', 'package': 'milestone3_multivehicle', 'files': [
        {'file': 'collision_launch.py', 'built': False},
    ]},
]

FAKE_ARGUMENTS = [
    {'name': 'path', 'default': '/fake/share/resource/path.yaml', 'description': 'Waypoint YAML file',
     'choices': [], 'included': False},
    {'name': 'launch_sitl', 'default': 'true', 'description': '', 'choices': [], 'included': False},
    {'name': 'topic_prefix', 'default': '', 'description': '', 'choices': [], 'included': False},
    {'name': 'relative_start', 'default': 'false', 'description': '', 'choices': [], 'included': False},
    {'name': 'record', 'default': 'false', 'description': '', 'choices': [], 'included': False},
    {'name': 'arm_delay_s', 'default': '0.0', 'description': 'Extra seconds before arming',
     'choices': [], 'included': True},
]


def fake_library(catalog=FAKE_CATALOG, arguments=FAKE_ARGUMENTS, run=_in_background):
    """A fixed catalog where every file declares the same made-up arguments."""

    def find_installed(package, file):
        for folder in catalog:
            for entry in folder['files']:
                if folder['package'] == package and entry['file'] == file and entry['built']:
                    return f"/fake/share/{package}/{file}"
        return None

    return LaunchLibrary(lambda: [dict(f, files=[dict(e) for e in f['files']]) for f in catalog],
                         find_installed, lambda path: [dict(a) for a in arguments], run=run)
