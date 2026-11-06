"""A pretend drone with the same interface as RosFlight, for tests and for running
the whole launcher on a machine with no ROS or PX4.

Its log lines copy the real offboard node's, so they go through the same parsing.
It never touches real hardware, and passing against it says nothing about PX4.
"""
import collections
import time

from swarm_launcher.flight_state import (
    ARMING_STATE_ARMED, NAVIGATION_STATE_AUTO_LAND, NAVIGATION_STATE_AUTO_LOITER,
    NAVIGATION_STATE_AUTO_RTL, NAVIGATION_STATE_OFFBOARD, FlightTracker, log_level)

CLIMB_S = 3.0
WAYPOINT_S = 3.0
DESCENT_S = 4.0
DISARM_AFTER_LANDING_S = 2.0
WAYPOINTS = 6
NAV_WHEN_COMMANDED = {
    'hold': NAVIGATION_STATE_AUTO_LOITER,
    'return': NAVIGATION_STATE_AUTO_RTL,
    'land': NAVIGATION_STATE_AUTO_LAND,
}


class FakeFlight:
    """reports_status=False stands in for firmware whose vehicle_status can't be read."""

    def __init__(self, speed=1.0, now=time.monotonic, problems=(), launch_fails=False,
                 gives_up=False, ignores_hold=False, crashes_after_s=None, reports_status=True):
        self.speed = speed
        self.now = now
        self.preflight_problems = list(problems)
        self.launch_fails = launch_fails
        self.gives_up = gives_up
        self.ignores_hold = ignores_hold
        self.crashes_after_s = crashes_after_s
        self.reports_status = reports_status
        self.tracker = FlightTracker(now)
        self.tracker.on_land_detected(True)
        self.logs = collections.deque()
        self.launch_running = False
        self.exit_code = None
        self.commanded = None
        self.commands_sent = []
        self.accepted_commands = set()
        self.started_at = None
        self.arm_delay_s = 0.0
        self.progress_s = 0.0
        self.last_poll = None
        self.descent_left_s = None
        self.z = 0.0
        self.px4_armed = False
        self.disarm_left_s = None
        self.crashed = False
        self.forced_nav_state = None

    def pilot_takes_over(self, nav_state):
        """What flipping the RC mode switch does: PX4 changes mode, and the node can't tell."""
        self.forced_nav_state = nav_state

    # --- Flight interface -------------------------------------------------------

    def problems(self, launch):
        return list(self.preflight_problems)

    def start(self, launch):
        self.tracker.begin_run()
        self.tracker.on_land_detected(True)
        self.commanded = None
        self.commands_sent = []
        self.accepted_commands = set()
        self.started_at = self.now()
        self.last_poll = self.started_at
        self.progress_s = 0.0
        self.descent_left_s = None
        self.disarm_left_s = None
        self.crashed = False
        self.forced_nav_state = None
        self.exit_code = None
        self.arm_delay_s = float(launch.get('args', {}).get('arm_delay_s', 0.0))
        self.launch_running = True
        self._node_log('INFO', f'loaded {WAYPOINTS} waypoints from fake.yaml')

    def stop(self):
        if self.launch_running:
            self.launch_running = False
            self.exit_code = 0
            self._node_log('INFO', 'shutting down')

    def launch_exit_code(self):
        return None if self.launch_running else self.exit_code

    def command(self, name):
        self.commands_sent.append(name)
        if name == 'disarm':
            self.px4_armed = False
            return
        if name == 'hold' and self.ignores_hold:
            return
        self.commanded = name
        self.accepted_commands.add(name)
        if name in ('return', 'land') and self.tracker.airborne and self.descent_left_s is None:
            self.descent_left_s = DESCENT_S

    def accepted(self, name):
        return name in self.accepted_commands

    def poll(self):
        if self.started_at is None:
            self._report_status()
            return
        now = self.now()
        step = (now - self.last_poll) * self.speed
        self.last_poll = now
        self._run_failures(now)
        if self.descent_left_s is not None:
            self._descend(step)
        elif self.launch_running and self.commanded is None and self.forced_nav_state is None:
            self._fly(step)
        self._count_down_disarm(step)
        self._report_status()

    def snapshot(self):
        tracker = self.tracker
        return {
            'position_ok': not self.preflight_problems,
            'position': {'x': 0.0, 'y': 0.0, 'z': round(self.z, 1)},
            'battery_pct': 100 if not tracker.was_airborne else 80,
            'global_position_ok': True,
            'gps_fix': 6,
            'rc_lost': False,
            **tracker.summary(self.launch_running, self.commanded),
        }

    def drain_logs(self):
        lines = []
        while self.logs:
            lines.append(self.logs.popleft())
        return lines

    def close(self):
        self.stop()

    # --- The pretend flight -----------------------------------------------------

    def _report_status(self):
        if self.reports_status:
            armed = ARMING_STATE_ARMED if self.px4_armed else 1
            self.tracker.on_status(armed == ARMING_STATE_ARMED, self._nav_state())

    def _nav_state(self):
        if self.forced_nav_state is not None:
            return self.forced_nav_state
        if self.commanded:
            return NAV_WHEN_COMMANDED[self.commanded]
        if self.crashed:
            return NAVIGATION_STATE_AUTO_RTL
        if self.tracker.landing_logged and self.px4_armed:
            return NAVIGATION_STATE_AUTO_LAND
        # Like real PX4, it goes back to Offboard after disarming until told otherwise.
        if (self.px4_armed and self.launch_running) or self.tracker.entered_offboard:
            return NAVIGATION_STATE_OFFBOARD
        return NAVIGATION_STATE_AUTO_LOITER

    def _node_log(self, level, text):
        line = f'[offboard_solution-1] [{level}] [offboard_control_solution]: {text}'
        self.tracker.on_log(line)
        self.logs.append((log_level(line), line))

    def _launch_log(self, level, text):
        line = f'[{level}] {text}'
        self.tracker.on_log(line)
        self.logs.append((log_level(line), line))

    def _run_failures(self, now):
        if not self.launch_running:
            return
        if self.launch_fails:
            self.launch_running = False
            self.exit_code = 1
            self._node_log('ERROR', 'PX4 never became ready; not starting the offboard node.')
        elif self.gives_up and not self.tracker.gave_up:
            self._node_log('ERROR', 'not flying: no arm + offboard within 30 s; giving up (fake)')
        elif (self.crashes_after_s is not None
              and (now - self.started_at) * self.speed > self.crashes_after_s):
            self.launch_running = False
            self.exit_code = -11
            self.crashed = True
            self.descent_left_s = DESCENT_S
            self._launch_log('ERROR', '[offboard_solution-2]: process has died [pid 1, exit code -11, cmd x]')

    def _fly(self, step):
        before = self.progress_s
        self.progress_s += step
        now_s = self.progress_s
        tracker = self.tracker
        if tracker.gave_up:
            return
        arm_at = self.arm_delay_s
        if before < arm_at <= now_s or (arm_at == 0.0 and not tracker.arm_accepted):
            tracker.on_arm_ack()
            self.px4_armed = True
            self._node_log('INFO', 'arm command sent')
            self._node_log('INFO', 'arm and offboard mode accepted by PX4')
        if now_s >= arm_at + CLIMB_S and not tracker.airborne and not tracker.was_airborne:
            tracker.on_land_detected(False)
            self.z = -5.0
        if tracker.airborne:
            done = int((now_s - arm_at - CLIMB_S) / WAYPOINT_S)
            while (tracker.waypoint or 0) < min(done, WAYPOINTS - 1):
                self._node_log('INFO', f'advancing to waypoint {(tracker.waypoint or 0) + 1}')
            if done >= WAYPOINTS and not tracker.landing_logged:
                self._node_log('INFO', 'Final hold complete; landing')
                self.descent_left_s = DESCENT_S

    def _descend(self, step):
        self.descent_left_s -= step
        if self.descent_left_s <= 0 and self.tracker.airborne:
            self.tracker.on_land_detected(True)
            self.z = 0.0
            self.disarm_left_s = DISARM_AFTER_LANDING_S

    def _count_down_disarm(self, step):
        if self.disarm_left_s is not None:
            self.disarm_left_s -= step
            if self.disarm_left_s <= 0:
                self.disarm_left_s = None
                self.px4_armed = False
