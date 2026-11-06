"""The real hub and real agent code over real sockets, with a pretend drone.

Covers the wiring between them: REST commands, the phone WebSocket, the agent
WebSocket and reconnecting. Run faster than real time.
"""
import asyncio
import json
import socket
import threading
import time
from pathlib import Path

import pytest
import requests
import uvicorn
import websockets

from swarm_launcher.agent import run_agent
from swarm_launcher.flight_fake import FakeFlight
from swarm_launcher.hub import create_app
from swarm_launcher.hub_core import HubCore
from swarm_launcher.manifest import load_missions

MISSIONS = Path(__file__).resolve().parent.parent / 'missions'
SPEED = 40.0


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class AgentThread:
    """The agent on its own event loop, so it outlives a hub that goes away."""

    def __init__(self, flight, url, runs_dir):
        self.loop = asyncio.new_event_loop()
        self.main = self.loop.create_task(run_agent('drone1', flight, url, runs_dir))
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.main)
        except asyncio.CancelledError:
            pass

    def start(self):
        self.thread.start()

    def stop(self):
        self.loop.call_soon_threadsafe(self.main.cancel)
        self.thread.join(timeout=5)


class Stack:
    def __init__(self, tmp_path, flight=None):
        self.port = free_port()
        self.base = f'http://127.0.0.1:{self.port}'
        self.flight = flight or FakeFlight(speed=SPEED)
        missions, _ = load_missions(MISSIONS)
        self.core = HubCore(missions)
        self.agent = AgentThread(self.flight, f'ws://127.0.0.1:{self.port}/agent', tmp_path / 'runs')
        self.server = None
        self.server_thread = None

    def start_hub(self):
        config = uvicorn.Config(create_app(self.core), host='127.0.0.1', port=self.port, log_level='error')
        self.server = uvicorn.Server(config)
        self.server_thread = threading.Thread(target=self.server.run, daemon=True)
        self.server_thread.start()

    def stop_hub(self):
        self.server.should_exit = True
        self.server_thread.join(timeout=5)
        assert not self.server_thread.is_alive()

    def __enter__(self):
        self.start_hub()
        self.agent.start()
        self.wait_for(lambda s: any(d['connected'] for d in s['drones']), 'agent to connect')
        return self

    def __exit__(self, *exc):
        self.agent.stop()
        self.stop_hub()

    def state(self):
        return requests.get(f'{self.base}/api/state', timeout=2).json()

    def post(self, path, **body):
        return requests.post(f'{self.base}/api/mission/{path}', json=body, timeout=2)

    def wait_for(self, condition, what, timeout=15.0):
        deadline = time.monotonic() + timeout
        state = None
        while time.monotonic() < deadline:
            try:
                state = self.state()
                if condition(state):
                    return state
            except requests.ConnectionError:
                pass
            time.sleep(0.02)
        raise AssertionError(f'timed out waiting for {what}; last state: {state}')

    def wait_for_state(self, name):
        return self.wait_for(lambda s: s['state'] == name, f'state {name}')

    def to_ready(self, mission='m1_updown'):
        assert self.post('select', mission_id=mission).status_code == 200
        return self.wait_for_state('ready')


def test_missions_are_listed_without_launch_details(tmp_path):
    with Stack(tmp_path) as stack:
        missions = requests.get(f'{stack.base}/api/missions', timeout=2).json()
        assert {m['id'] for m in missions} == {'m1_updown', 'm1_square', 'm1_full_path'}
        assert all('launch' not in m for m in missions)


def test_full_mission_start_to_complete(tmp_path):
    with Stack(tmp_path) as stack:
        stack.to_ready()
        assert stack.post('start', press_age_ms=1500).status_code == 200
        state = stack.wait_for_state('searching')
        assert state['committed']
        assert state['clock']['running'] and state['clock']['elapsed_ms'] >= 1500
        drone = state['drones'][0]
        assert drone['role'] == 'search' and drone['airborne'] is True
        state = stack.wait_for_state('complete')
        assert state['clock']['running'] is False
        assert state['drones'][0]['run_state'] == 'done'
        assert not stack.flight.launch_running
        saved = list((tmp_path / 'runs').glob('*/package_drone1.json'))
        assert len(saved) == 1
        assert stack.post('reset').status_code == 200
        stack.wait_for_state('idle')


def test_abort_hold_then_operator_chooses_land(tmp_path):
    with Stack(tmp_path) as stack:
        stack.to_ready()
        stack.post('start')
        stack.wait_for_state('searching')
        assert stack.post('abort').status_code == 200
        state = stack.wait_for_state('aborted')
        assert state['abort_results'] == {'drone1': 'holding'}
        assert not stack.flight.launch_running
        assert stack.state()['drones'][0]['airborne'] is True
        assert stack.post('reset').status_code == 409  # Still in the air.
        assert stack.post('recover', action='land').status_code == 200
        stack.wait_for(lambda s: s['drones'][0]['airborne'] is False, 'landing')
        assert stack.post('reset').status_code == 200
        stack.wait_for_state('idle')


def test_abort_during_countdown_keeps_the_drone_on_the_ground(tmp_path):
    flight = FakeFlight(speed=0.01)
    with Stack(tmp_path, flight) as stack:
        stack.to_ready()
        stack.post('start')
        stack.wait_for(lambda s: s['committed'] and s['drones'][0]['run_state'] == 'running', 'Go')
        stack.post('abort')
        state = stack.wait_for_state('aborted')
        assert state['abort_results'] == {'drone1': 'not_armed'}
        assert flight.commands_sent == []


def test_a_retried_start_does_not_fire_twice(tmp_path):
    with Stack(tmp_path) as stack:
        stack.to_ready()
        first = stack.post('start', command_id='same')
        second = stack.post('start', command_id='same')
        assert first.status_code == 200 and second.status_code == 200
        assert first.json() == second.json()
        assert stack.post('start', command_id='different').status_code == 409


def test_failed_check_is_reported(tmp_path):
    flight = FakeFlight(speed=SPEED, problems=['no valid local position estimate yet'])
    with Stack(tmp_path, flight) as stack:
        stack.post('select', mission_id='m1_updown')
        state = stack.wait_for(lambda s: s['state'] == 'idle' and s['reason'], 'check failure')
        assert state['checks']['drone1']['problems'] == ['no valid local position estimate yet']


def test_launch_failure_returns_to_idle_with_the_reason(tmp_path):
    flight = FakeFlight(speed=SPEED, launch_fails=True)
    with Stack(tmp_path, flight) as stack:
        stack.to_ready()
        stack.post('start')
        state = stack.wait_for(lambda s: s['state'] == 'idle' and s['reason'], 'failure')
        assert 'never became ready' in state['reason']


def test_phone_socket_sends_state_and_catches_up_on_logs(tmp_path):
    with Stack(tmp_path) as stack:
        stack.to_ready()
        missed_after = stack.state()['log_seq'] - 1

        async def read_phone():
            url = f'ws://127.0.0.1:{stack.port}/ws?after={missed_after}'
            async with websockets.connect(url) as phone:
                kinds = {}
                while not {'state', 'log'} <= set(kinds):
                    message = json.loads(await asyncio.wait_for(phone.recv(), timeout=5))
                    kinds[message['type']] = message
                return kinds

        kinds = asyncio.run(read_phone())
        assert kinds['state']['state'] == 'ready'
        assert kinds['log']['lines'][0]['seq'] > missed_after


def test_abort_still_works_after_the_hub_restarts_mid_flight(tmp_path):
    """The hub loses everything it knew. It must rebuild from the drone, so abort still acts."""
    with Stack(tmp_path, FakeFlight(speed=2.0)) as stack:
        stack.to_ready()
        stack.post('start')
        stack.wait_for_state('searching')
        stack.stop_hub()
        missions, _ = load_missions(MISSIONS)
        stack.core = HubCore(missions)
        stack.start_hub()
        state = stack.wait_for_state('searching')
        assert state['run_id'] and state['clock']['running']
        assert stack.post('abort').status_code == 200
        state = stack.wait_for_state('aborted')
        assert state['abort_results'] == {'drone1': 'holding'}
        assert not stack.flight.launch_running


def test_phone_gets_the_whole_log_from_a_restarted_hub(tmp_path):
    with Stack(tmp_path) as stack:
        async def read_phone():
            # A number past the end: this phone last saw a hub that has since restarted.
            async with websockets.connect(f'ws://127.0.0.1:{stack.port}/ws?after=999999') as phone:
                while True:
                    message = json.loads(await asyncio.wait_for(phone.recv(), timeout=5))
                    if message['type'] == 'log':
                        return message

        assert asyncio.run(read_phone())['lines'][0]['seq'] == 1


def test_drone_flies_on_without_the_hub_and_the_hub_catches_up(tmp_path):
    """The hub goes away mid-flight. The drone finishes, then reports in when the hub returns."""
    with Stack(tmp_path) as stack:
        stack.to_ready()
        stack.post('start')
        stack.wait_for_state('searching')
        assert stack.flight.tracker.airborne

        stack.stop_hub()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not stack.flight.tracker.landed_after_flight:
            time.sleep(0.02)
        assert stack.flight.tracker.landed_after_flight
        assert stack.core.state in ('searching', 'landing')  # The hub has seen none of it.

        stack.start_hub()
        state = stack.wait_for_state('complete')
        assert state['drones'][0]['run_state'] == 'done'
