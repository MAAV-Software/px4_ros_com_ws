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
from swarm_launcher.launch_library import ERROR, PENDING, override_problems

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
# How often src is looked at again, so a new launch file or a build shows up on the phone.
CATALOG_EVERY_S = 5.0
MODES = ('sitl', 'hitl')
# A landed drone keeps its launch running until the whole run is over, as it would inside one
# launch that flies every drone: its controller may be what the others are waiting on. Milestone
# 3's yield rule holds a lower-priority drone wherever it is while a peer's position goes stale.
# The hub says when the run is over (finish); without a hub, the drone stops it this long after
# landing.
LINGER_AFTER_LANDING_S = 600.0


class Agent:
    def __init__(self, drone_id, flight, send, library, runs_dir=None, now=time.monotonic, sim=False):
        self.drone_id = drone_id
        self.flight = flight
        self.library = library
        # A SITL instance flies only in SITL mode, and a real drone only in HITL mode.
        self.sim = sim
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
        self.catalog_sent = None
        self.catalog_checked = None
        self.pending_preflight = None
        self.landed_at = None
        self.handlers = {
            'catalog_request': self._on_catalog_request,
            'preflight': self._on_preflight,
            'package': self._on_package,
            'go': self._on_go,
            'cancel': self._on_cancel,
            'abort': self._on_abort,
            'recover': self._on_recover,
            'reset': self._on_reset,
            'finish': self._on_finish,
        }

    # --- Messages from the hub --------------------------------------------------

    def handle(self, message):
        handler = self.handlers.get(message.get('type'))
        if handler:
            handler(message)

    def _on_catalog_request(self, message):
        self._step_catalog(force=True)

    def _on_preflight(self, message):
        # Answered once this drone has read the file's arguments, which runs in the background.
        self.pending_preflight = message
        self._step_preflight()

    def _on_package(self, message):
        """Mission package: saved to disk and acknowledged before anything can fly."""
        run_id = message.get('run_id')
        if self.run_id == run_id and self.run_state != IDLE:
            self.send({'type': 'package_ack', 'run_id': run_id, 'problems': []})
            return
        launch, problems, _ = self._resolve(message.get('launch') or {})
        if launch is None and not problems:
            problems = ['still reading the launch arguments. Check again']
        if not problems:
            problems = self._start_problems(launch, message.get('mode'))
        if not problems:
            problems = self._save_package(run_id, {**message, 'resolved': launch})
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

    def _on_finish(self, message):
        """Every drone in the run has landed, so this one's launch can stop."""
        if message.get('run_id') == self.run_id and self.run_state in (DONE, FAILED):
            self.flight.stop()

    # --- Time-driven work -------------------------------------------------------

    def tick(self):
        self.flight.poll()
        self._step_catalog()
        self._step_preflight()
        lines = self.flight.drain_logs()
        if lines:
            self.last_launch_line = lines[-1][1]
            self.send({'type': 'log', 'lines': [{'level': level, 'text': text} for level, text in lines]})
        if self.run_state in (RUNNING, LANDING):
            self._step_run()
        elif self.run_state == ABORTING:
            self._step_abort()
        self._step_linger()
        self._step_recovery()
        self._step_armed_after_abort()
        self._step_leave_offboard()
        self.send(self.status())

    def _step_catalog(self, force=False):
        now = self.now()
        if not force and self.catalog_checked is not None and now - self.catalog_checked < CATALOG_EVERY_S:
            return
        self.catalog_checked = now
        folders = self.library.catalog()
        if force or folders != self.catalog_sent:
            self.catalog_sent = folders
            self.send({'type': 'catalog', 'folders': folders, 'sim': self.sim})

    def _step_preflight(self):
        message = self.pending_preflight
        if message is None:
            return
        launch, problems, arguments = self._resolve(message.get('launch') or {})
        if launch is None and not problems:
            return
        self.pending_preflight = None
        if not problems:
            problems = self._start_problems(launch, message.get('mode'))
        self.send({'type': 'preflight_result', 'check_id': message.get('check_id'),
                   'mission_id': message.get('mission_id'), 'problems': problems,
                   'launch_args': arguments})

    def _step_run(self):
        snapshot = self.flight.snapshot()
        if snapshot['landed_after_flight']:
            # The offboard node keeps spinning after it lands, so the launch never ends by itself.
            # It is stopped once the whole run is over: see LINGER_AFTER_LANDING_S.
            if self.landed_at is None:
                self.landed_at = self.now()
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

    def _step_linger(self):
        if (self.run_state == DONE and self.landed_at is not None
                and self.now() - self.landed_at > LINGER_AFTER_LANDING_S):
            self.flight.stop()

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
            'sim': self.sim,
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
            'global': snapshot.get('global'),
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

    def _resolve(self, launch):
        """Check a launch the hub sent against this drone's own src and the file's arguments.

        Returns (launch, problems, arguments). The launch it returns adds what this drone
        works out for itself: every argument the file declares, and the value each will
        have, with launch_sitl always false. (None, [], None) means the arguments are
        still being read.
        """
        package, file = launch.get('package'), launch.get('file')
        overrides = launch.get('args') or {}
        problems = override_problems(overrides)
        if problems:
            return None, problems, None
        entry = self.library.find(package, file) if isinstance(package, str) and isinstance(file, str) else None
        if entry is None:
            return None, [f"no {file} for {package} in this drone's src"], None
        if not entry['built']:
            return None, [f'{package} is not built here, or was built after the agent started'], None
        state, arguments = self.library.describe(package, file)
        if state == PENDING:
            return None, [], None
        if state == ERROR:
            return None, [arguments], None
        declared = {argument['name'] for argument in arguments}
        unknown = sorted(set(overrides) - declared)
        if unknown:
            return None, [f"{file} has no argument {', '.join(unknown)}"], arguments
        effective = {a['name']: a['default'] for a in arguments if a['default'] is not None}
        effective.update(overrides)
        if 'launch_sitl' in declared:
            effective['launch_sitl'] = 'false'
        resolved = {'package': package, 'file': file, 'args': dict(overrides),
                    'declared': sorted(declared), 'effective': effective}
        return resolved, [], arguments

    def _mode_problems(self, mode):
        if mode not in MODES:
            return ['the hub did not say whether this is SITL or HITL']
        if mode == 'sitl' and not self.sim:
            return ['this is a real drone and the hub is in SITL mode. Switch to HITL to fly it']
        if mode == 'hitl' and self.sim:
            return ['this is a SITL drone and the hub is in HITL mode. Switch to SITL to fly it']
        return []

    def _link_problems(self, snapshot):
        """HITL only: what has to be connected before a real drone may fly. SITL has no RC,
        and its PX4 link is the simulator's own.
        """
        if snapshot['nav_state'] is None:
            return ['no vehicle_status from PX4. Check the XRCE-DDS agent and its link to the flight controller']
        if snapshot['rc_lost'] is None:
            return ['PX4 has not reported the RC link yet (no failsafe_flags)']
        if snapshot['rc_lost']:
            return ["no RC link. The safety pilot's transmitter must be on and bound"]
        return []

    def _start_problems(self, launch, mode):
        self._clear_finished_run()
        problems = self._mode_problems(mode)
        if problems:
            return problems
        snapshot = self.flight.snapshot()
        if mode == 'hitl':
            problems = self._link_problems(snapshot)
            if problems:
                return problems
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
        # A finished run's launch may still be running for the others; the run is over now.
        if self.flight.snapshot()['launch_running']:
            self.flight.stop()
        self.landed_at = None
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
