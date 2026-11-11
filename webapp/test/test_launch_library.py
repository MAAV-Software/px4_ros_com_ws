"""The launch files a drone offers: scanning src, reading arguments, checking typed values."""
import os
import textwrap

import pytest

from swarm_launcher.launch_library import (
    ERROR, PENDING, READY, LaunchLibrary, fake_library, override_problems, scan)


def make_package(src, folder, name, files):
    (src / folder / 'launch').mkdir(parents=True)
    (src / folder / 'package.xml').write_text(f'<package format="3">\n  <name>{name}</name>\n</package>\n')
    for file in files:
        (src / folder / 'launch' / file).write_text('# launch\n')


def test_scan_lists_launch_files_by_folder_with_the_package_name(tmp_path):
    make_package(tmp_path, 'm3_anti-collision', 'm3', ['collision_launch.py', 'offboard.launch.py',
                                                       'px4_sitl.launch.py', 'helper.py', 'notes.md'])
    make_package(tmp_path, 'blank', 'blank', ['px4_sitl.launch.py'])
    (tmp_path / 'px4_msgs').mkdir()
    (tmp_path / 'px4_msgs' / 'package.xml').write_text('<name>px4_msgs</name>')
    built = {('m3', 'collision_launch.py')}
    folders = scan(tmp_path, lambda package, file: '/x' if (package, file) in built else None)
    assert folders == [{'folder': 'm3_anti-collision', 'package': 'm3', 'files': [
        {'file': 'collision_launch.py', 'built': True},
        {'file': 'offboard.launch.py', 'built': False},
    ]}]


def test_scan_skips_a_folder_without_a_readable_package_name(tmp_path):
    (tmp_path / 'odd' / 'launch').mkdir(parents=True)
    (tmp_path / 'odd' / 'launch' / 'a_launch.py').write_text('')
    assert scan(tmp_path, lambda package, file: None) == []
    assert scan(tmp_path / 'missing', lambda package, file: None) == []


def test_the_workspace_src_scans_cleanly():
    """Every real package folder should come through, whatever its folder name."""
    src = os.path.join(os.path.dirname(__file__), '..', '..', 'src')
    if not os.path.isdir(src):
        pytest.skip('not inside the workspace')
    packages = {folder['package'] for folder in scan(src, lambda package, file: None)}
    assert 'milestone1_singlevehicle' in packages
    assert 'px4_msgs' not in packages


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def library_for(tmp_path, describer, run=lambda work: work(), now=None):
    path = tmp_path / 'f.launch.py'
    path.write_text('')
    catalog = [{'folder': 'p', 'package': 'p', 'files': [{'file': 'f.launch.py', 'built': True}]}]
    return path, LaunchLibrary(lambda: catalog, lambda package, file: str(path) if file == 'f.launch.py' else None,
                               describer, run=run, now=now or Clock())


def test_describe_is_kept_until_the_installed_file_changes(tmp_path):
    calls = []
    path, library = library_for(tmp_path, lambda p: calls.append(p) or [{'name': f'a{len(calls)}'}])
    assert library.describe('p', 'f.launch.py') == (READY, [{'name': 'a1'}])
    assert library.describe('p', 'f.launch.py') == (READY, [{'name': 'a1'}])
    stat = path.stat()
    os.utime(path, (stat.st_atime, stat.st_mtime + 5))
    assert library.describe('p', 'f.launch.py') == (READY, [{'name': 'a2'}])


def test_describe_runs_in_the_background(tmp_path):
    queued = []
    _, library = library_for(tmp_path, lambda p: [], run=queued.append)
    assert library.describe('p', 'f.launch.py') == (PENDING, None)
    assert library.describe('p', 'f.launch.py') == (PENDING, None)
    assert len(queued) == 1
    queued[0]()
    assert library.describe('p', 'f.launch.py') == (READY, [])


def test_a_failed_read_is_retried_after_a_while(tmp_path):
    clock = Clock()
    attempts = []

    def flaky(path):
        attempts.append(path)
        if len(attempts) == 1:
            raise RuntimeError('ModuleNotFoundError: No module named launch_ros')
        return []

    _, library = library_for(tmp_path, flaky, now=clock)
    state, reason = library.describe('p', 'f.launch.py')
    assert state == ERROR and 'launch_ros' in reason
    assert library.describe('p', 'f.launch.py')[0] == ERROR
    clock.t += 11
    assert library.describe('p', 'f.launch.py') == (READY, [])


def test_describe_an_unbuilt_file(tmp_path):
    _, library = library_for(tmp_path, lambda p: [])
    assert library.describe('p', 'other.py') == (ERROR, 'p has no built other.py')


def test_find_returns_the_folder_too():
    library = fake_library()
    assert library.find('milestone3_multivehicle', 'collision_launch.py') == {
        'folder': 'milestone3_multivehicle_anti-collision', 'package': 'milestone3_multivehicle',
        'file': 'collision_launch.py', 'built': False}
    assert library.find('milestone3_multivehicle', 'nope.py') is None


@pytest.mark.parametrize('args, problems', [
    ({}, []),
    ({'record': 'true', 'path': '/home/nav/a b.yaml'}, []),
    ({'launch_sitl': 'true'}, ['launch_sitl is set by the launcher']),
    ({'topic_prefix': '/px4_2'}, ['topic_prefix is set by the launcher']),
    ({'bad-name': 'x'}, ["bad argument name: 'bad-name'"]),
    ({'record': ''}, ['bad value for record']),
    ({'record': 'x' * 1001}, ['bad value for record']),
    ({'record': 'a\nb'}, ['bad value for record']),
    ({'record': 3}, ['bad value for record']),
    (['record'], ['launch arguments must be a mapping']),
])
def test_override_problems(args, problems):
    assert override_problems(args) == problems


def test_launch_args_reads_a_real_launch_file(tmp_path):
    """Needs a sourced ROS 2 environment, so this runs on the VM or a drone, not a laptop."""
    # Not just 'launch': without ROS, milestone1's launch/ folder imports as an empty namespace package.
    pytest.importorskip('launch.actions')
    from swarm_launcher.launch_args import describe

    (tmp_path / 'inner.launch.py').write_text(textwrap.dedent('''
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('arm_delay_s', default_value='0.0', description='Wait before arming'),
                DeclareLaunchArgument('record', default_value='false'),
            ])
    '''))
    (tmp_path / 'outer_launch.py').write_text(textwrap.dedent(f'''
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('folder', default_value='/data'),
                DeclareLaunchArgument('path', default_value=PathJoinSubstitution(
                    [LaunchConfiguration('folder'), 'square.yaml'])),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource('{tmp_path}/inner.launch.py'),
                    launch_arguments={{'record': 'true'}}.items()),
            ])
    '''))
    arguments = describe(str(tmp_path / 'outer_launch.py'))
    by_name = {argument['name']: argument for argument in arguments}
    # record is set by the include itself, so a value from the phone would never reach it.
    assert list(by_name) == ['folder', 'path', 'arm_delay_s']
    assert by_name['path']['default'] == '/data/square.yaml'
    assert by_name['arm_delay_s'] == {'name': 'arm_delay_s', 'default': '0.0', 'description': 'Wait before arming',
                                      'choices': [], 'included': True}
