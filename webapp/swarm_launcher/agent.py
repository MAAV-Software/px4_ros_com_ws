#!/usr/bin/env python3
"""The agent process on each drone: runs agent_core and keeps a link to the hub.

The tick keeps running while the link is down, and anything worth keeping waits in
a queue until the link returns. Only the newest status is kept, because an old one
is useless.

  ros2 run swarm_launcher agent --hub ws://192.168.1.10:8080/agent --drone-id drone1
  # Real fleet drone (PX4 namespace px4_1, MAV_SYS_ID 2):
  ros2 run swarm_launcher agent --hub ws://hub:8080/agent --drone-id drone2 --topic-prefix /px4_1 --system-id 2
  # A SITL instance (the hub's Fleet tab starts these itself):
  ros2 run swarm_launcher agent --drone-id sim2 --topic-prefix /px4_1 --system-id 2 --sim
  # No ROS or PX4 on this machine:
  python3 -m swarm_launcher.agent --hub ws://localhost:8080/agent --drone-id drone1 --fake

The phone lists the launch files in --workspace's src (default ~/px4_ros_com_ws). Packages
built after the agent started need it restarting, from a shell that has sourced them.
"""
import argparse
import asyncio
import collections
import json
from pathlib import Path

import websockets
from websockets.exceptions import WebSocketException

from swarm_launcher.agent_core import Agent
from swarm_launcher.launch_library import fake_library, ros_library

TICK_S = 0.5
RECONNECT_S = 1.0
QUEUED_MESSAGES = 5000


class HubLink:
    """Outgoing messages for the hub, kept while the link is down."""

    def __init__(self):
        self.queue = collections.deque(maxlen=QUEUED_MESSAGES)
        self.latest_status = None
        self.wake = asyncio.Event()

    def send(self, message):
        if message.get('type') == 'status':
            self.latest_status = message
        else:
            self.queue.append(message)
        self.wake.set()

    def next_message(self):
        if self.queue:
            return self.queue.popleft()
        message, self.latest_status = self.latest_status, None
        return message


async def _pump_outgoing(socket, link):
    while True:
        message = link.next_message()
        if message is None:
            link.wake.clear()
            await link.wake.wait()
            continue
        await socket.send(json.dumps(message))


async def _stay_connected(agent, link, url):
    while True:
        try:
            async with websockets.connect(f'{url}/{agent.drone_id}') as socket:
                pump = asyncio.ensure_future(_pump_outgoing(socket, link))
                try:
                    async for text in socket:
                        agent.handle(json.loads(text))
                finally:
                    pump.cancel()
        except (OSError, WebSocketException, asyncio.TimeoutError):
            pass
        await asyncio.sleep(RECONNECT_S)


async def _tick_forever(agent):
    while True:
        agent.tick()
        await asyncio.sleep(TICK_S)


async def run_agent(drone_id, flight, library, url, runs_dir, sim=False):
    link = HubLink()
    agent = Agent(drone_id, flight, link.send, library, runs_dir=runs_dir, sim=sim)
    await asyncio.gather(_stay_connected(agent, link, url), _tick_forever(agent))


def main():
    parser = argparse.ArgumentParser(description='Drone agent for the swarm launcher')
    parser.add_argument('--hub', default='ws://localhost:8080/agent', help='hub agent URL')
    parser.add_argument('--drone-id', default='drone1')
    parser.add_argument('--topic-prefix', default='',
                        help='PX4 topic prefix, e.g. /px4_1 for /px4_1/fmu/... (default: none)')
    parser.add_argument('--system-id', type=int, default=1, help="this PX4's MAV_SYS_ID")
    parser.add_argument('--workspace', default='~/px4_ros_com_ws',
                        help='ROS 2 workspace whose src holds the launch files to offer')
    parser.add_argument('--runs-dir', default='~/launcher_runs')
    parser.add_argument('--sim', action='store_true',
                        help='this drone is a SITL instance, so it only flies with the hub in SITL mode')
    parser.add_argument('--fake', action='store_true', help='pretend drone: no ROS or PX4 needed')
    parser.add_argument('--fake-speed', type=float, default=1.0)
    args = parser.parse_args()

    if args.fake:
        from swarm_launcher.flight_fake import FakeFlight
        flight = FakeFlight(speed=args.fake_speed)
        library = fake_library()
    else:
        from swarm_launcher.flight_ros import RosFlight
        flight = RosFlight(args.drone_id, args.topic_prefix.rstrip('/'), args.system_id)
        library = ros_library(Path(args.workspace).expanduser() / 'src')
    try:
        # A pretend drone is a simulation too.
        sim = args.sim or args.fake
        asyncio.run(run_agent(args.drone_id, flight, library, args.hub, args.runs_dir, sim=sim))
    except KeyboardInterrupt:
        pass
    finally:
        flight.close()


if __name__ == '__main__':
    main()
