#!/usr/bin/env python3
"""The hub process: serves the phone client and connects phones and agents to hub_core.

Phones send commands over REST and get everything live over /ws. Agents hold their
own WebSocket on /agent/<drone_id>. Everything runs on one asyncio loop, so hub_core
needs no locks.

  ros2 run swarm_launcher hub --port 8080
  # No ROS or PX4 on this machine: SITL drones on the Fleet tab are pretend ones.
  python3 -m swarm_launcher.hub --mode sitl --fake-sitl
"""
import argparse
import asyncio
import contextlib
import json
from pathlib import Path
from typing import Dict, List, Optional

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from swarm_launcher.hub_core import MODES, HubCore
from swarm_launcher.mdns import DEFAULT_NAME, MdnsName
from swarm_launcher.run_store import RunStore
from swarm_launcher.sitl_manager import FakeBackend, ProcessBackend, SitlManager
from swarm_launcher.voice import DEFAULT_MODEL, FakeRecognizer, VoiceRecognizer, read_wav

TICK_S = 0.25
MDNS_REFRESH_S = 30.0
# A 10 s clip of 16 kHz 16-bit audio is about 320 kB.
MAX_VOICE_BYTES = 2_000_000
SETTINGS_FILE = 'settings.json'
PHONE_UPDATE_S = 0.5
NO_CLIENT_PAGE = ('<!doctype html><title>Swarm launcher</title><p>The web client is not built. '
                  'Run <code>npm ci &amp;&amp; npm run build</code> in <code>webapp/web</code>, '
                  'then rebuild this package.</p>')


class Command(BaseModel):
    command_id: Optional[str] = None


class SelectCommand(Command):
    package: str
    file: str
    drones: List[str]
    args: Dict[str, str] = {}


class StartCommand(Command):
    press_age_ms: float = 0.0


class RecoverCommand(Command):
    action: str


class ModeCommand(Command):
    mode: str


class SitlCommand(Command):
    drone_id: str = ''


class TextCommand(Command):
    text: str


class AreaCommand(Command):
    corners: Optional[Dict[str, Dict[str, float]]] = None


class WebFiles(StaticFiles):
    """The web client. The page itself is always rechecked, so a phone picks up a new client
    as soon as the hub has one; the scripts it names carry a hash, so those can be cached.
    """

    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        if response.media_type == 'text/html':
            response.headers['Cache-Control'] = 'no-cache'
        return response


def _respond(result):
    return JSONResponse(result, status_code=200 if result['ok'] else 409)


def create_app(core, web_dir=None, background=(), ca_path=None):
    """`background` holds extra coroutine functions to run for the app's lifetime."""

    @contextlib.asynccontextmanager
    async def lifespan(app):
        async def tick_forever():
            while True:
                core.tick()
                await asyncio.sleep(TICK_S)

        tasks = [asyncio.ensure_future(work()) for work in (tick_forever, *background)]
        try:
            yield
        finally:
            for task in tasks:
                task.cancel()

    app = FastAPI(title='Swarm launcher', lifespan=lifespan)

    @app.get('/api/state')
    async def get_state():
        return core.snapshot()

    @app.post('/api/mission/select')
    async def select(body: SelectCommand):
        return _respond(core.run_command(body.command_id, lambda: core.select(body.package, body.file, body.drones, body.args)))

    @app.post('/api/mission/start')
    async def start(body: StartCommand):
        return _respond(core.run_command(body.command_id, lambda: core.start(body.press_age_ms)))

    @app.post('/api/mission/abort')
    async def abort(body: Command):
        return _respond(core.run_command(body.command_id, core.abort))

    @app.post('/api/mission/recover')
    async def recover(body: RecoverCommand):
        return _respond(core.run_command(body.command_id, lambda: core.recover(body.action)))

    @app.post('/api/mission/reset')
    async def reset(body: Command):
        return _respond(core.run_command(body.command_id, core.reset))

    @app.post('/api/voice')
    async def voice(request: Request, command_id: str = ''):
        """A hold-to-talk clip as 16-bit mono WAV. Returns what was heard and the command."""
        if command_id and command_id in core.commands:
            return _respond(core.commands[command_id])
        refusal = core.voice_refusal()
        if refusal:
            return _respond(refusal)
        body = await request.body()
        if len(body) > MAX_VOICE_BYTES:
            return _respond({'ok': False, 'state': core.state, 'reason': 'voice clip too long'})
        try:
            pcm, rate = read_wav(body)
            # Decoding takes a moment, and the first clip loads the model: keep the loop free for aborts.
            text = await asyncio.to_thread(core.voice.recognize, pcm, rate)
        except Exception as error:  # A bad clip or a Vosk failure is a reason to show, not a crash.
            return _respond({'ok': False, 'state': core.state, 'reason': f'could not hear that: {error}'})
        return _respond(core.run_command(command_id, lambda: core.spoken(text)))

    @app.post('/api/command')
    async def typed_command(body: TextCommand):
        """Typed or keyboard-dictated text, read the same way as a voice clip."""
        return _respond(core.run_command(body.command_id, lambda: core.spoken(body.text[:200])))

    @app.post('/api/area')
    async def set_area(body: AreaCommand):
        return _respond(core.run_command(body.command_id, lambda: core.set_area(body.corners)))

    @app.post('/api/area/fit')
    async def fit_area(body: SitlCommand):
        return _respond(core.run_command(body.command_id, lambda: core.fit_area(body.drone_id)))

    @app.post('/api/mode')
    async def set_mode(body: ModeCommand):
        return _respond(core.run_command(body.command_id, lambda: core.set_mode(body.mode)))

    @app.post('/api/sitl/add')
    async def sitl_add(body: SitlCommand):
        return _respond(core.run_command(body.command_id, core.sitl_add))

    @app.post('/api/sitl/remove')
    async def sitl_remove(body: SitlCommand):
        return _respond(core.run_command(body.command_id, lambda: core.sitl_remove(body.drone_id)))

    @app.post('/api/sitl/stop')
    async def sitl_stop(body: SitlCommand):
        return _respond(core.run_command(body.command_id, core.sitl_stop_all))

    @app.websocket('/ws')
    async def phone_socket(socket: WebSocket, after: int = 0):
        """State twice a second, plus any log lines and trail points the phone hasn't had."""
        await socket.accept()
        # A number past the end means this hub restarted: send its whole log.
        cursor = after if after <= core.log_seq else 0
        # Each phone gets the drones' trails whole once, then only new points.
        trail_version, trail_counts = None, {}
        try:
            while True:
                await socket.send_text(json.dumps(core.snapshot()))
                lines = core.logs_after(cursor)
                if lines:
                    cursor = lines[-1]['seq']
                    await socket.send_text(json.dumps({'type': 'log', 'lines': lines}))
                trail, trail_version, trail_counts = core.trail_update(trail_version, trail_counts)
                if trail:
                    await socket.send_text(json.dumps(trail))
                await asyncio.sleep(PHONE_UPDATE_S)
        except (WebSocketDisconnect, RuntimeError):
            pass

    @app.websocket('/agent/{drone_id}')
    async def agent_socket(socket: WebSocket, drone_id: str):
        await socket.accept()
        outgoing = asyncio.Queue()

        async def pump():
            while True:
                await socket.send_text(json.dumps(await outgoing.get()))

        send = outgoing.put_nowait
        core.agent_connected(drone_id, send)
        pump_task = asyncio.ensure_future(pump())
        try:
            while True:
                core.agent_message(drone_id, json.loads(await socket.receive_text()), send)
        except WebSocketDisconnect:
            pass
        finally:
            pump_task.cancel()
            core.agent_disconnected(drone_id, send)

    if ca_path:
        @app.get('/ca.crt')
        async def certificate_authority():
            """The hub's own CA, for each phone to install once so HTTPS works offline."""
            return FileResponse(str(ca_path), media_type='application/x-x509-ca-cert',
                                filename='swarm-launcher-ca.crt')

    if web_dir and Path(web_dir).is_dir():
        app.mount('/', WebFiles(directory=str(web_dir), html=True), name='web')
    else:
        @app.get('/')
        async def no_client():
            return HTMLResponse(NO_CLIENT_PAGE)

    return app


def share_dir():
    """The installed share directory, or the source tree when run without ROS."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('swarm_launcher'))
    except (ImportError, KeyError):
        return Path(__file__).resolve().parent.parent


def load_settings(runs_dir):
    try:
        settings = json.loads((Path(runs_dir).expanduser() / SETTINGS_FILE).read_text())
    except (OSError, ValueError):
        return {}
    return settings if isinstance(settings, dict) else {}


def save_setting(runs_dir, key, value):
    """Kept, so a hub that restarts mid-run comes back with the same mode and search area."""
    settings = load_settings(runs_dir)
    settings[key] = value
    try:
        folder = Path(runs_dir).expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        (folder / SETTINGS_FILE).write_text(json.dumps(settings))
    except OSError:
        pass


class TaskHandle:
    """A pretend SITL drone's agent, running as a task in the hub's own event loop."""

    def __init__(self, task):
        self.task = task

    def running(self):
        return not self.task.done()

    @property
    def returncode(self):
        return None if self.running() else 0

    def stop(self):
        self.task.cancel()

    def poll(self):
        pass

    def group_alive(self):
        return self.running()

    def kill_group(self):
        self.task.cancel()

    def drain_lines(self):
        return []


def main():
    parser = argparse.ArgumentParser(description='Hub for the swarm launcher')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--web-dir', default=None, help='default: the installed web client')
    parser.add_argument('--runs-dir', default='~/launcher_runs')
    parser.add_argument('--mode', choices=MODES, default=None,
                        help='start in SITL or HITL (default: the mode last chosen on the phone, else HITL)')
    parser.add_argument('--px4-dir', default='~/PX4-Autopilot', help='built PX4, for SITL drones')
    parser.add_argument('--workspace', default='~/px4_ros_com_ws',
                        help="workspace whose src the SITL drones' agents offer")
    parser.add_argument('--fake-drone', action='append', default=[], metavar='ID',
                        help='also run a pretend SITL drone in this process (repeatable)')
    parser.add_argument('--fake-sitl', action='store_true',
                        help='Add SITL drone on the Fleet tab makes pretend drones, not PX4 instances')
    parser.add_argument('--fake-speed', type=float, default=1.0)
    parser.add_argument('--vosk-model', default=DEFAULT_MODEL, help='Vosk model folder, for voice commands')
    parser.add_argument('--name', default=DEFAULT_NAME,
                        help='announce this hub as NAME.local, so phones can type that (empty to turn off)')
    parser.add_argument('--https-port', type=int, default=8443,
                        help='also serve the site over HTTPS here, for the phone microphone (0 to turn off)')
    parser.add_argument('--https-host', action='append', default=[], metavar='NAME',
                        help='another name or address phones use for this hub, for its certificate (repeatable)')
    parser.add_argument('--fake-voice', default=None, metavar='PHRASE',
                        help='no Vosk: every voice clip is heard as PHRASE (for trying the phone side)')
    args = parser.parse_args()

    from swarm_launcher.agent import run_agent
    from swarm_launcher.flight_fake import SITL_HOME, FakeFlight
    from swarm_launcher.grid import local_to_global
    from swarm_launcher.launch_library import fake_library

    hub_url = f'ws://127.0.0.1:{args.port}/agent'

    def start_fake_agent(instance, drone_id):
        # Spaced like real SITL drones: 20 m east of the one before.
        flight = FakeFlight(speed=args.fake_speed, origin=local_to_global(*SITL_HOME, 0, 20 * instance))
        return TaskHandle(asyncio.ensure_future(
            run_agent(drone_id, flight, fake_library(), hub_url, args.runs_dir, sim=True)))

    if args.fake_sitl:
        backend = FakeBackend(start_agent=start_fake_agent)
    else:
        backend = ProcessBackend(args.px4_dir, hub_url, args.workspace,
                                 Path(args.runs_dir).expanduser() / 'sitl_processes.json')
    settings = load_settings(args.runs_dir)
    mode = args.mode or settings.get('mode') or 'hitl'
    core = HubCore(store=RunStore(args.runs_dir), mode=mode, sitl=SitlManager(backend),
                   save_mode=lambda chosen: save_setting(args.runs_dir, 'mode', chosen),
                   area=settings.get('area'),
                   save_area=lambda corners: save_setting(args.runs_dir, 'area', corners),
                   voice=FakeRecognizer(args.fake_voice) if args.fake_voice else VoiceRecognizer(args.vosk_model))

    background = []
    for drone_id in args.fake_drone:
        async def fake_drone(drone_id=drone_id):
            flight = FakeFlight(speed=args.fake_speed)
            await run_agent(drone_id, flight, fake_library(), hub_url, args.runs_dir, sim=True)

        background.append(fake_drone)

    configs = []
    ca_path = None
    if args.https_port:
        from swarm_launcher.tls import ensure_certificates, local_hosts
        hosts = local_hosts(args.https_host + ([f'{args.name}.local'] if args.name else []))
        cert, key, ca_path = ensure_certificates(Path(args.runs_dir).expanduser() / 'tls', hosts)
        core.https_port = args.https_port

    mdns = MdnsName(args.name) if args.name else None
    if mdns:
        async def announce_name():
            while True:
                line = await asyncio.to_thread(mdns.refresh)
                if line:
                    core.log('hub', 'WARN' if mdns.problem else 'INFO', line)
                    print(line)
                core.local_name = None if mdns.problem or not mdns.published else mdns.name
                await asyncio.sleep(MDNS_REFRESH_S)

        background.append(announce_name)

    share = share_dir()
    app = create_app(core, args.web_dir or share / 'web' / 'dist', background, ca_path)
    # Agents and plain-HTTP browsers on one port, phones over HTTPS on another. The app's
    # startup (the hub's tick) runs once, with the first.
    configs.append(uvicorn.Config(app, host=args.host, port=args.port, log_level='warning'))
    print(f'Swarm launcher hub on http://{args.host}:{args.port}, in {mode.upper()} mode')
    if args.https_port:
        configs.append(uvicorn.Config(app, host=args.host, port=args.https_port, log_level='warning',
                                      lifespan='off', ssl_certfile=str(cert), ssl_keyfile=str(key)))
        print(f'  and https://<this machine>:{args.https_port} for phones. Certificate covers: {", ".join(hosts)}')
        print(f'  Phones install its CA once from http://<this machine>:{args.port}/ca.crt')
    if args.name:
        print(f'  Phones on the same network can use http://{args.name}.local:{args.port}'
              + (f' or https://{args.name}.local:{args.https_port}' if args.https_port else '')
              + ', or no port at all after tools/serve_on_80_and_443.sh')
    try:
        asyncio.run(serve_all(configs))
    except KeyboardInterrupt:
        pass
    finally:
        if mdns:
            mdns.close()


async def serve_all(configs):
    """Run several servers on one loop, and stop them all when any stops (Ctrl-C included)."""
    servers = [uvicorn.Server(config) for config in configs]
    tasks = [asyncio.ensure_future(server.serve()) for server in servers]
    await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    for server in servers:
        server.should_exit = True
    await asyncio.gather(*tasks, return_exceptions=True)


if __name__ == '__main__':
    main()
