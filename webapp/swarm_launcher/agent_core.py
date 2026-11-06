"""What runs on each drone: starts and stops the mission, aborts, and reports back.

Pure Python with no network or ROS imports. It talks to a flight backend
(RosFlight or FakeFlight) and hands outgoing messages to `send`, so it can be
tested with a fake clock. agent.py wires it to the hub's WebSocket.

Once Go has arrived nothing here depends on the hub. The tick keeps running and
the flight keeps going, whether or not any message ever arrives again.
"""
import json
import time
from pathlib import Path

from swarm_launcher.flight_state import NAVIGATION_STATE_AUTO_LOITER, NAVIGATION_STATE_OFFBOARD

IDLE = 'idle'
PREPARED = 'prepared'
RUNNING = 'running'
LANDING = 'landing'
DONE = 'done'
FAILED = 'failed'
ABORTING = 'aborting'
ABORTED = 'aborted'

HOLD_CONFIRM_S = 5.0
# With no choice from the operator, a held drone returns. A dropped link must
# not leave it hovering until the battery failsafe.
HOLD_LIMIT_S = 60.0
RECOVERY_RESEND_S = 10.0


class Agent:
    def __init__(self, drone_id, flight, send, runs_dir=None, now=time.monotonic):
        self.drone_id = drone_id
        self.flight = flight
        self.send = send
        self.runs_dir = Path(runs_dir).expanduser() if runs_dir else None
        self.now = now
        self.run_state = IDLE
        self.run_id = None
        self.mission_id = None
        self.run_started = None
        self.run_ended = None
        self.launch = None
        self.failure = ''
        self.last_launch_line = ''
        self.abort_started = None
        self.abort_result = None
        self.hold_deadline = None
        self.recovery = None
        self.recovery_until = None
        self.handlers = {
            'preflight': self._on_preflight,
            'package': self._on_package,
            'go': self._on_go,
            'cancel': self._on_cancel,
            'abort': self._on_abort,
            'recover': self._on_recover,
            'reset': self._on_reset,
        }

    # --- Messages from the hub --------------------------------------------------

    def handle(self, message):
        handler = self.handlers.get(message.get('type'))
        if handler:
            handler(message)

    def _on_preflight(self, message):
        self.send({'type': 'preflight_result', 'mission_id': message.get('mission_id'),
                   'problems': self._start_problems(message.get('launch') or {})})

    def _on_package(self, message):
        """Mission package: saved to disk and acknowledged before anything can fly."""
        run_id = message.get('run_id')
        if self.run_id == run_id and self.run_state != IDLE:
            self.send({'type': 'package_ack', 'run_id': run_id, 'problems': []})
            return
        launch = message.get('launch') or {}
        problems = self._start_problems(launch)
        if not problems:
            problems = self._save_package(run_id, message)
        if not problems:
            self.run_id = run_id
            self.mission_id = message.get('mission_id')
            self.launch = launch
            self.failure = ''
            self.run_state = PREPARED
        self.send({'type': 'package_ack', 'run_id': run_id, 'problems': problems})

    def _on_go(self, message):
        run_id = message.get('run_id')
        if run_id != self.run_id:
            return
        if self.run_state == PREPARED:
            try:
                self.flight.start(self.launch)
                self.run_started = self.now()
                self.run_state = RUNNING
            except Exception as error:  # A launch that can't start must report, not crash the agent.
                self._fail(f'could not start the launch: {error}')
        self.send({'type': 'go_ack', 'run_id': run_id})

    def _on_cancel(self, message):
        if self.run_state == PREPARED:
            self._clear_run()

    def _on_abort(self, message):
        if self.abort_result:
            self.send({'type': 'abort_ack', 'result': self.abort_result})
        elif self.abort_started is None:
            self.abort_started = self.now()
            self._end_clock()
            self.run_state = ABORTING
            self._step_abort()

    def _on_recover(self, message):
        if self.abort_result in ('holding', 'hold_unconfirmed'):
            if not self.flight.snapshot()['pilot_control']:
                self._recover(message.get('action'))

    def _on_reset(self, message):
        self._clear_finished_run()

    # --- Time-driven work -------------------------------------------------------

    def tick(self):
        self.flight.poll()
        lines = self.flight.drain_logs()
        if lines:
            self.last_launch_line = lines[-1][1]
            self.send({'type': 'log', 'lines': [{'level': level, 'text': text} for level, text in lines]})
        if self.run_state in (RUNNING, LANDING):
            self._step_run()
        elif self.run_state == ABORTING:
            self._step_abort()
        self._step_recovery()
        self._step_armed_after_abort()
        self._step_leave_offboard()
        self.send(self.status())

    def _step_run(self):
        snapshot = self.flight.snapshot()
        if snapshot['landed_after_flight']:
            # The offboard node keeps spinning after it lands, so the launch never ends by itself.
            self.flight.stop()
            self._end_clock()
            # Not done until PX4 disarms: until then the props may still spin.
            self.run_state = LANDING if snapshot['armed'] else DONE
            return
        code = self.flight.launch_exit_code()
        reason = snapshot['gave_up'] or snapshot['node_died']
        if reason or code is not None:
            self.flight.stop()
            self._fail(reason or f'launch exited with code {code}: {self.last_launch_line}')
        elif snapshot['left_offboard']:
            # The node can't see this. Left running it would finish its route and send LAND,
            # overriding whoever now has the drone.
            self.flight.stop()
            self._fail(f"PX4 left Offboard for {snapshot['left_offboard']}. Launch stopped.")
        elif snapshot['landing']:
            self._end_clock()
            self.run_state = LANDING

    def _step_abort(self):
        """Order matters. Unarmed: stop the launch first, because the node would keep
        trying to arm. Armed on the ground: disarm too, in the same step, because with no
        setpoints PX4 would fly a failsafe Return about a second later. In the air: Hold first,
        because stopping the launch ends the setpoint stream and leaves the drone to whatever
        COM_OBL_RC_ACT is set to.

        A drone a pilot is flying is left alone. So is one already coming down.
        """
        snapshot = self.flight.snapshot()
        if not (snapshot['armed'] or snapshot['airborne']):
            result = 'not_armed'
        elif snapshot['pilot_control']:
            result = 'pilot_control'
        elif not snapshot['airborne'] and snapshot['nav_state'] is not None:
            # PX4 refuses a disarm if it has actually left the ground, so this is safe to try,
            # and the tick after finds it airborne and takes the Hold path below.
            self.flight.stop()
            self.flight.command('disarm')
            if self.now() - self.abort_started > HOLD_CONFIRM_S:
                result = 'hold_unconfirmed'
            else:
                return
        elif snapshot['coming_down'] and not self.flight.accepted('hold'):
            result = 'already_landing'
        elif self._hold_confirmed(snapshot):
            result = 'holding'
        elif self.now() - self.abort_started > HOLD_CONFIRM_S:
            # The launch still stops, so PX4's offboard-loss failsafe is the backstop.
            result = 'hold_unconfirmed'
        else:
            self.flight.command('hold')
            return
        self.flight.stop()
        self.abort_result = result
        self.run_state = ABORTED
        if result in ('holding', 'hold_unconfirmed'):
            self.hold_deadline = self.now() + HOLD_LIMIT_S
        self.send({'type': 'abort_ack', 'result': result})

    def _step_recovery(self):
        if self.hold_deadline is not None and self.recovery is None:
            snapshot = self.flight.snapshot()
            if snapshot['pilot_control']:
                self.hold_deadline = None
            elif snapshot['airborne'] and self.now() >= self.hold_deadline:
                # Waits while on the ground instead of cancelling: the land detector can flip
                # back, and a drone left holding with no timer stays armed until someone notices.
                self._recover('return')
        # Commands travel best-effort, so keep sending until PX4 acknowledges one.
        if self.recovery and self.recovery_until is not None:
            if self.flight.accepted(self.recovery) or self.now() > self.recovery_until:
                self.recovery_until = None
            else:
                self.flight.command(self.recovery)

    def _step_armed_after_abort(self):
        """An arm command already on its way can land just after the launch stops. With no
        setpoints coming, PX4 would treat that as a failsafe and fly a Return from the ground.
        """
        if self.abort_result != 'not_armed':
            return
        snapshot = self.flight.snapshot()
        if snapshot['nav_state'] is None or not snapshot['armed']:
            return
        if not snapshot['airborne']:
            self.flight.command('disarm')
            return
        self.flight.command('hold')
        self.abort_result = 'holding'
        self.hold_deadline = self.now() + HOLD_LIMIT_S
        self.send({'type': 'abort_ack', 'result': self.abort_result})

    def _step_leave_offboard(self):
        """PX4 goes back to Offboard after disarming, and its preflight result always fails
        there with no setpoints arriving. Hold makes that result mean something again.
        """
        snapshot = self.flight.snapshot()
        if (snapshot['nav_state'] == NAVIGATION_STATE_OFFBOARD
                and not snapshot['armed'] and not snapshot['launch_running']):
            self.flight.command('hold')

    def status(self):
        snapshot = self.flight.snapshot()
        hold_left = None
        if self.hold_deadline is not None and self.recovery is None and snapshot['airborne']:
            hold_left = max(0, round(self.hold_deadline - self.now()))
        return {
            'type': 'status',
            'run_id': self.run_id,
            'mission_id': self.mission_id,
            'run_state': self.run_state,
            'run_age_s': self._run_age(),
            'abort_result': self.abort_result,
            'landed_after_flight': snapshot['landed_after_flight'],
            'failure': self.failure,
            'recovery': self.recovery,
            'hold_left_s': hold_left,
            'activity': snapshot['activity'],
            'armed': snapshot['armed'],
            'airborne': snapshot['airborne'],
            'position_ok': snapshot['position_ok'],
            'position': snapshot['position'],
            'battery_pct': snapshot['battery_pct'],
            'global_position_ok': snapshot['global_position_ok'],
            'gps_fix': snapshot['gps_fix'],
            'rc_lost': snapshot['rc_lost'],
            'waypoint': snapshot['waypoint'],
            'waypoint_total': snapshot['waypoint_total'],
        }

    # --- Internals --------------------------------------------------------------

    def _hold_confirmed(self, snapshot):
        # PX4's own mode is the better evidence. The ack is for when vehicle_status isn't arriving.
        if snapshot['nav_state'] is not None:
            return snapshot['nav_state'] == NAVIGATION_STATE_AUTO_LOITER
        return self.flight.accepted('hold')

    def _recover(self, action):
        if action not in ('return', 'land'):
            return
        self.recovery = action
        self.recovery_until = self.now() + RECOVERY_RESEND_S
        self.flight.command(action)

    def _end_clock(self):
        if self.run_ended is None:
            self.run_ended = self.now()

    def _run_age(self):
        if self.run_started is None:
            return None
        return (self.run_ended if self.run_ended is not None else self.now()) - self.run_started

    def _fail(self, reason):
        self._end_clock()
        self.failure = reason
        self.run_state = FAILED

    def _start_problems(self, launch):
        self._clear_finished_run()
        snapshot = self.flight.snapshot()
        if snapshot['airborne']:
            return ['drone is still airborne']
        # Only trust this from PX4 itself: without vehicle_status, armed is a cautious guess.
        if snapshot['nav_state'] is not None and snapshot['armed']:
            return ['PX4 is already armed']
        if self.run_state not in (IDLE, PREPARED):
            return [f'a run is still active ({self.run_state})']
        return self.flight.problems(launch)

    def _save_package(self, run_id, message):
        if self.runs_dir is None:
            return []
        try:
            folder = self.runs_dir / run_id
            folder.mkdir(parents=True, exist_ok=True)
            (folder / f'package_{self.drone_id}.json').write_text(json.dumps(message))
        except OSError as error:
            return [f'could not save the mission package: {error}']
        return []

    def _clear_finished_run(self):
        """Forget a run that is over, or one that was prepared but never started."""
        if self.run_state in (PREPARED, DONE, FAILED, ABORTED, IDLE):
            if not self.flight.snapshot()['airborne']:
                self._clear_run()

    def _clear_run(self):
        self.run_state = IDLE
        self.run_id = None
        self.mission_id = None
        self.run_started = None
        self.run_ended = None
        self.launch = None
        self.failure = ''
        self.abort_started = None
        self.abort_result = None
        self.hold_deadline = None
        self.recovery = None
        self.recovery_until = None
