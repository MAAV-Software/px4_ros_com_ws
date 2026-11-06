#!/usr/bin/env python3
"""The hub process: serves the phone client and connects phones and agents to hub_core.

Phones send commands over REST and get everything live over /ws. Agents hold their
own WebSocket on /agent/<drone_id>. Everything runs on one asyncio loop, so hub_core
needs no locks.

  ros2 run swarm_launcher hub --port 8080
  # No ROS or PX4 on this machine: hub plus a pretend drone in one process.
  python3 -m swarm_launcher.hub --fake-drone drone1
"""
import argparse
import asyncio
import contextlib
import json
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from swarm_launcher.hub_core import HubCore
from swarm_launcher.manifest import load_missions
from swarm_launcher.run_store import RunStore

TICK_S = 0.25
PHONE_UPDATE_S = 0.5
NO_CLIENT_PAGE = ('<!doctype html><title>Swarm launcher</title><p>The web client is not built. '
                  'Run <code>npm ci &amp;&amp; npm run build</code> in <code>webapp/web</code>, '
                  'then rebuild this package.</p>')


class Command(BaseModel):
    command_id: Optional[str] = None


class SelectCommand(Command):
    mission_id: str


class StartCommand(Command):
    press_age_ms: float = 0.0


class RecoverCommand(Command):
    action: str


def _respond(result):
    return JSONResponse(result, status_code=200 if result['ok'] else 409)


def create_app(core, web_dir=None, background=()):
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

    @app.get('/api/missions')
    async def get_missions():
        return core.mission_list()

    @app.post('/api/mission/select')
    async def select(body: SelectCommand):
        return _respond(core.run_command(body.command_id, lambda: core.select(body.mission_id)))

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

    @app.websocket('/ws')
    async def phone_socket(socket: WebSocket, after: int = 0):
        """State twice a second, plus any log lines after the last one the phone saw."""
        await socket.accept()
        # A number past the end means this hub restarted: send its whole log.
        cursor = after if after <= core.log_seq else 0
        try:
            while True:
                await socket.send_text(json.dumps(core.snapshot()))
                lines = core.logs_after(cursor)
                if lines:
                    cursor = lines[-1]['seq']
                    await socket.send_text(json.dumps({'type': 'log', 'lines': lines}))
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
                core.agent_message(drone_id, json.loads(await socket.receive_text()))
        except WebSocketDisconnect:
            pass
        finally:
            pump_task.cancel()
            core.agent_disconnected(drone_id, send)

    if web_dir and Path(web_dir).is_dir():
        app.mount('/', StaticFiles(directory=str(web_dir), html=True), name='web')
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


def main():
    parser = argparse.ArgumentParser(description='Hub for the swarm launcher')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--missions-dir', default=None, help='default: the installed missions')
    parser.add_argument('--web-dir', default=None, help='default: the installed web client')
    parser.add_argument('--runs-dir', default='~/launcher_runs')
    parser.add_argument('--fake-drone', action='append', default=[], metavar='ID',
                        help='also run a pretend drone in this process (repeatable)')
    parser.add_argument('--fake-speed', type=float, default=1.0)
    args = parser.parse_args()

    share = share_dir()
    missions, errors = load_missions(args.missions_dir or share / 'missions')
    core = HubCore(missions, store=RunStore(args.runs_dir), load_errors=errors)

    background = []
    for drone_id in args.fake_drone:
        from swarm_launcher.agent import run_agent
        from swarm_launcher.flight_fake import FakeFlight

        async def fake_drone(drone_id=drone_id):
            flight = FakeFlight(speed=args.fake_speed)
            await run_agent(drone_id, flight, f'ws://127.0.0.1:{args.port}/agent', args.runs_dir)

        background.append(fake_drone)

    app = create_app(core, args.web_dir or share / 'web' / 'dist', background)
    print(f'Swarm launcher hub on http://{args.host}:{args.port} with {len(missions)} missions')
    uvicorn.run(app, host=args.host, port=args.port, log_level='warning')


if __name__ == '__main__':
    main()
