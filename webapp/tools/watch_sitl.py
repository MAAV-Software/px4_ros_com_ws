#!/usr/bin/env python3
"""Log the hub's view beside PX4's own, one row each time either changes.

PX4's side comes from its `listener` command, so this works even though the workspace's
px4_msgs can't read vehicle_status. It's how to check the launcher isn't fooling itself.

  python3 tools/watch_sitl.py complete,aborted 120   # until a state, or for 120 s
"""
import os
import re
import subprocess
import sys
import time

import requests

HUB = 'http://localhost:8080'
PX4_BIN = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin')
NAV_STATES = {0: 'MANUAL', 2: 'POSCTL', 4: 'HOLD', 5: 'RTL', 14: 'OFFBOARD', 18: 'LAND'}


def listener(topic, *fields):
    output = subprocess.run([f'{PX4_BIN}/px4-listener', topic, '-n', '1'],
                            capture_output=True, text=True, timeout=5).stdout
    found = {}
    for field in fields:
        match = re.search(rf'^\s*{field}: (\S+)', output, re.MULTILINE)
        found[field] = match.group(1) if match else None
    return found


def px4_view():
    status = listener('vehicle_status', 'arming_state', 'nav_state')
    landed = listener('vehicle_land_detected', 'landed')['landed']
    nav = int(status['nav_state'] or -1)
    return f"armed={status['arming_state'] == '2'} nav={NAV_STATES.get(nav, nav)} landed={landed}"


def hub_view():
    try:
        state = requests.get(f'{HUB}/api/state', timeout=2).json()
    except requests.RequestException:
        return None, ('HUB DOWN',)
    drone = (state['drones'] or [{}])[0]
    return state, (state['state'], drone.get('run_state'), drone.get('activity'), drone.get('armed'),
                   drone.get('airborne'), drone.get('waypoint'), state['abort_results'].get(drone.get('id')))


def main():
    targets = sys.argv[1].split(',')
    deadline = time.monotonic() + float(sys.argv[2])
    started = time.monotonic()
    last = None
    while time.monotonic() < deadline:
        state, hub = hub_view()
        px4 = px4_view()
        if (hub, px4) != last:
            z = listener('vehicle_local_position', 'z')['z']
            print(f'{time.monotonic() - started:5.1f}s z={float(z or 0):6.2f} | hub {hub} | px4 {px4}', flush=True)
            last = (hub, px4)
        if state and state['state'] in targets:
            print('reached', state['state'], state['reason'], state['clock'], flush=True)
            return
        time.sleep(0.5)
    print('stopped watching', flush=True)


if __name__ == '__main__':
    main()
