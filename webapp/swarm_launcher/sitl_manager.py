"""SITL drones started from the Fleet tab, all on the hub's machine.

Each SITL drone is one PX4 instance and one agent started with --sim. They share one
Micro XRCE-DDS agent and one Gazebo world, the same layout as milestone 2's
px4_sitl.launch.py: instance 0 starts the world, and instance N joins it 20 m further
east, with topics under /px4_N and MAV_SYS_ID N+1.

Instance 0 owns the world, so drones are added in order, each once the one before it
is up (the hub checks that), and removed newest first. The real launch files never
start a simulator, which is what keeps them the same in SITL and HITL.

Pure Python apart from the backend. ProcessBackend starts real processes, and
FakeBackend stands in for --fake-sitl and the tests.
"""
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from swarm_launcher.launch_process import LaunchProcess

MAX_DRONES = 6
SPACING_M = 20
XRCE_PORT = '8888'
# The same limits milestone 2's SITL sets, and the HITL runbook asks of real drones. No GCS
# link failsafe, since nothing needs QGroundControl open to fly a simulated drone.
SITL_PARAMS = {
    'MPC_XY_VEL_MAX': '2',
    'MPC_Z_VEL_MAX_UP': '1',
    'MPC_Z_VEL_MAX_DN': '1',
    'MPC_YAWRAUTO_MAX': '45',
    'NAV_DLL_ACT': '0',
}
STRAY_KILL_AFTER_S = 5.0
# Gazebo outlives the PX4 instance that started it and ignores SIGINT and SIGTERM, so
# whatever is left in a stopped drone's process groups gets SIGKILL after this long.
LEFTOVER_KILL_AFTER_S = 5.0
ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')
GAZEBO_PATTERN = r'(^|/| )gz sim( |$)|gz-sim.*server'


class SitlError(Exception):
    pass


def drone_id_for(instance):
    return f'sim{instance + 1}'


def topic_prefix_for(instance):
    return '' if instance == 0 else f'/px4_{instance}'


def line_level(line):
    """The level worth passing on to the hub's log, or None for routine output."""
    text = ANSI_ESCAPE.sub('', line).strip()
    if text.startswith(('ERROR', 'PANIC')) or 'Traceback' in text or 'Error:' in text:
        return 'ERROR'
    if text.startswith('WARN'):
        return 'WARN'
    return None


class SitlManager:
    def __init__(self, backend, now=time.monotonic):
        self.backend = backend
        self.now = now
        self.entries = []
        self.xrce = None

    def add(self):
        """Start the next SITL drone and return its id."""
        if any(entry['stopping'] for entry in self.entries):
            raise SitlError('wait for the last SITL drone to stop')
        if len(self.entries) >= MAX_DRONES:
            raise SitlError(f'at most {MAX_DRONES} SITL drones')
        instance = len(self.entries)
        problems = self.backend.problems(first=instance == 0)
        if problems:
            raise SitlError('; '.join(problems))
        if self.xrce is None or not self.xrce.running():
            self.xrce = self.backend.start_xrce()
        drone_id = drone_id_for(instance)
        self.entries.append({
            'instance': instance,
            'drone_id': drone_id,
            'px4': self.backend.start_px4(instance),
            'agent': self.backend.start_agent(instance, drone_id),
            'started_at': self.now(),
            'stopping': False,
            'exit': '',
        })
        return drone_id

    def remove(self, drone_id):
        live = [entry for entry in self.entries if not entry['stopping']]
        if not any(entry['drone_id'] == drone_id for entry in live):
            raise SitlError(f'no SITL drone {drone_id}')
        if live[-1]['drone_id'] != drone_id:
            raise SitlError(f"remove {live[-1]['drone_id']} first. Each SITL drone joins the world "
                            'the first one started')
        self._stop(live[-1])

    def stop_all(self):
        """Every SITL drone, plus anything a previous run of this hub left behind."""
        for entry in reversed(self.entries):
            if not entry['stopping']:
                self._stop(entry)
        self.backend.stop_strays()

    def poll(self):
        """Escalate stops, notice crashes and pass on errors. Returns (source, level, text) lines."""
        lines = []
        for entry in self.entries:
            for name in ('px4', 'agent'):
                handle = entry[name]
                handle.poll()
                for line in handle.drain_lines():
                    level = line_level(line)
                    if level:
                        lines.append((entry['drone_id'], level, ANSI_ESCAPE.sub('', line).strip()))
            if not entry['stopping'] and not entry['exit']:
                for name, label in (('px4', 'PX4'), ('agent', 'its agent')):
                    if not entry[name].running():
                        entry['exit'] = f'{label} exited with code {entry[name].returncode}'
                        lines.append((entry['drone_id'], 'ERROR', entry['exit']))
                        break
        for entry in self.entries:
            if entry['stopping'] and self.now() - entry['stopped_at'] > LEFTOVER_KILL_AFTER_S:
                for name in ('px4', 'agent'):
                    handle = entry[name]
                    if not handle.running() and handle.group_alive():
                        handle.kill_group()
        # A removed drone is forgotten once everything it started has gone.
        self.entries = [entry for entry in self.entries
                        if not (entry['stopping'] and not any(
                            entry[name].running() or entry[name].group_alive() for name in ('px4', 'agent')))]
        if self.xrce is not None:
            self.xrce.poll()
            for line in self.xrce.drain_lines():
                level = line_level(line)
                if level:
                    lines.append(('xrce', level, ANSI_ESCAPE.sub('', line).strip()))
            if not self.entries:
                if self.xrce.running():
                    self.xrce.stop()
                else:
                    self.xrce = None
        self.backend.poll()
        return lines

    def view(self):
        return [{
            'drone_id': entry['drone_id'],
            'instance': entry['instance'],
            'started_at': entry['started_at'],
            'state': 'stopping' if entry['stopping'] else 'exited' if entry['exit'] else 'running',
            'detail': entry['exit'],
        } for entry in self.entries]

    def _stop(self, entry):
        entry['stopping'] = True
        entry['stopped_at'] = self.now()
        # The agent first, so it stops any launch before its PX4 goes.
        entry['agent'].stop()
        entry['px4'].stop()


class ProcessBackend:
    """Real PX4, Micro XRCE-DDS and agent processes. Each runs in its own process group, as
    milestone 2's sitl_process.py does, so stopping PX4 instance 0 also stops the Gazebo it started.
    """

    def __init__(self, px4_dir, hub_url, workspace, state_file, now=time.monotonic):
        self.px4_dir = Path(px4_dir).expanduser().resolve()
        self.binary = self.px4_dir / 'build/px4_sitl_default/bin/px4'
        self.hub_url = hub_url
        self.workspace = str(Path(workspace).expanduser())
        self.state_file = Path(state_file).expanduser()
        self.now = now
        self.strays = []

    def problems(self, first):
        problems = []
        if not os.access(self.binary, os.X_OK):
            problems.append(f'no PX4 SITL build at {self.binary}. Run make px4_sitl in {self.px4_dir}')
        if shutil.which('MicroXRCEAgent') is None:
            problems.append('MicroXRCEAgent is not installed or not on PATH')
        if first and not problems:
            running = [name for name, command in (
                ('PX4', ['pgrep', '-x', 'px4']),
                ('Gazebo', ['pgrep', '-f', GAZEBO_PATTERN]),
                ('MicroXRCEAgent', ['pgrep', '-x', 'MicroXRCEAgent']),
            ) if subprocess.run(command, stdout=subprocess.DEVNULL).returncode == 0]
            if running:
                problems.append(f"{' and '.join(running)} already running outside these SITL drones. "
                                'Stop that run first')
        return problems

    def start_xrce(self):
        return self._start(['MicroXRCEAgent', 'udp4', '-p', XRCE_PORT])

    def start_px4(self, instance):
        run_dir = self.px4_dir / 'build/px4_sitl_default/rootfs' / str(instance)
        run_dir.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ)
        env.update({
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500',
            'PX4_GZ_WORLD': 'default',
            'PX4_GZ_MODEL_POSE': f'{instance * SPACING_M},0,0',
            # Instance 0 starts the world; the rest join it.
            'PX4_GZ_STANDALONE': '1' if instance else '',
            'PX4_UXRCE_DDS_NS': topic_prefix_for(instance).lstrip('/'),
            'PX4_UXRCE_DDS_PORT': XRCE_PORT,
            'HEADLESS': '1',
        })
        env.update({f'PX4_PARAM_{name}': value for name, value in SITL_PARAMS.items()})
        # -w keeps each instance's parameters and logs in its own folder. The build's etc folder
        # has to be named too: PX4 links it into that folder, and finds no startup script without
        # it. PX4's own gz script then finds gz_env.sh two folders up, in the build.
        etc = self.px4_dir / 'build/px4_sitl_default/etc'
        return self._start([str(self.binary), str(etc), '-i', str(instance), '-d', '-w', str(run_dir)],
                           env=env, cwd=str(self.px4_dir))

    def start_agent(self, instance, drone_id):
        return self._start([
            sys.executable, '-m', 'swarm_launcher.agent', '--hub', self.hub_url, '--drone-id', drone_id,
            '--topic-prefix', topic_prefix_for(instance), '--system-id', str(instance + 1),
            '--workspace', self.workspace, '--sim'])

    def stop_strays(self):
        """Processes a previous run of this hub started and left running, found from the
        record it kept. Only those: never a MicroXRCEAgent or PX4 this hub didn't start.
        """
        for record in self._records():
            if self._is_ours(record):
                self._signal(record['pid'], signal.SIGTERM)
                self.strays.append((record['pid'], self.now() + STRAY_KILL_AFTER_S))

    def poll(self):
        still = []
        for pid, deadline in self.strays:
            if self.now() >= deadline:
                self._signal(pid, signal.SIGKILL)
            else:
                still.append((pid, deadline))
        self.strays = still

    def _start(self, argv, env=None, cwd=None):
        process = LaunchProcess(argv, now=self.now, env=env, cwd=cwd)
        process.start()
        records = [r for r in self._records() if self._is_ours(r)]
        records.append({'pid': process.process.pid, 'argv0': argv[0], 'arg': argv[-1]})
        try:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            self.state_file.write_text(json.dumps(records))
        except OSError:
            pass  # Only Stop all after a hub restart needs it.
        return process

    def _records(self):
        try:
            return json.loads(self.state_file.read_text())
        except (OSError, ValueError):
            return []

    @staticmethod
    def _is_ours(record):
        """Still alive and still the same program, not a new process given an old pid."""
        try:
            command = Path(f"/proc/{record['pid']}/cmdline").read_bytes().split(b'\0')
        except (OSError, KeyError, TypeError):
            return False
        return record.get('argv0', '').encode() in command and record.get('arg', '').encode() in command

    @staticmethod
    def _signal(pid, number):
        try:
            os.killpg(pid, number)
        except (ProcessLookupError, PermissionError):
            pass


class FakeHandle:
    def __init__(self):
        self.alive = True
        self.code = None
        self.lines = []

    def running(self):
        return self.alive

    @property
    def returncode(self):
        return self.code

    def stop(self):
        if self.alive:
            self.alive = False
            self.code = 0

    def poll(self):
        pass

    def group_alive(self):
        return self.alive

    def kill_group(self):
        self.alive = False

    def drain_lines(self):
        lines, self.lines = self.lines, []
        return lines


class FakeBackend:
    """No processes. `start_agent(instance, drone_id)` can return a handle for a pretend
    drone, as `hub --fake-sitl` does; by default it returns an inert one.
    """

    def __init__(self, start_agent=None, problems=()):
        self.agent_starter = start_agent
        self.problem_list = list(problems)
        self.started = []
        self.strays_stopped = 0

    def problems(self, first):
        return list(self.problem_list)

    def start_xrce(self):
        self.started.append('xrce')
        return FakeHandle()

    def start_px4(self, instance):
        self.started.append(f'px4 {instance}')
        return FakeHandle()

    def start_agent(self, instance, drone_id):
        self.started.append(f'agent {drone_id}')
        return self.agent_starter(instance, drone_id) if self.agent_starter else FakeHandle()

    def stop_strays(self):
        self.strays_stopped += 1

    def poll(self):
        pass
