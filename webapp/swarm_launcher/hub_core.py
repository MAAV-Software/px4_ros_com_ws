"""Mission logic for the hub: one state machine, the fleet table and the log.

Pure Python with no network or ROS imports, so every rule can be tested with a
fake clock. hub.py wires this to FastAPI and the WebSockets.

The hub starts a mission and collects what comes back, but does not run it.
After Go each drone flies on its own, and this class only mirrors its reports.
"""
import collections
import time

from swarm_launcher.manifest import public_mission

IDLE = 'idle'
PREFLIGHT = 'preflight'
READY = 'ready'
STARTING = 'starting'
SEARCHING = 'searching'
LANDING = 'landing'
COMPLETE = 'complete'
ABORTING = 'aborting'
ABORTED = 'aborted'

PREFLIGHT_TIMEOUT_S = 5.0
PACKAGE_TIMEOUT_S = 5.0
GO_TIMEOUT_S = 5.0
RESEND_S = 0.5
ABORT_NO_RESPONSE_S = 2.0
# A phone's held abort can reach a restarted hub before the drones have reconnected.
ABORT_CARRY_S = 5.0
LOG_LINES = 2000
COMMANDS_REMEMBERED = 200
MAX_PRESS_AGE_S = 60.0
RECOVERY_ACTIONS = ('return', 'land')


class HubCore:
    def __init__(self, missions, now=time.monotonic, store=None, load_errors=()):
        self.missions = missions
        self.now = now
        self.store = store
        self.state = IDLE
        self.mission = None
        self.run_id = None
        self.reason = ''
        self.committed = False
        self.clock_started = None
        self.clock_stopped = None
        self.links = {}
        self.drones = {}
        self.checks = {}
        self.package_acks = {}
        self.go_acks = {}
        self.abort_results = {}
        self.recovery = None
        self.closed_runs = set()
        self.idle_abort_at = None
        self.step_started = self.now()
        self.last_send = self.now()
        self.logs = collections.deque(maxlen=LOG_LINES)
        self.log_seq = 0
        self.commands = collections.OrderedDict()
        for error in load_errors:
            self.log('hub', 'ERROR', f'mission file skipped: {error}')

    # --- Agent links -----------------------------------------------------------

    def agent_connected(self, drone_id, send):
        self.links[drone_id] = send
        self._drone(drone_id)['last_seen'] = self.now()
        self.log('hub', 'INFO', f'{drone_id} connected')

    def agent_disconnected(self, drone_id, send):
        # A reconnect can arrive before the old socket's close does.
        if self.links.get(drone_id) is send:
            del self.links[drone_id]
            self.log('hub', 'WARN', f'{drone_id} disconnected')

    def agent_message(self, drone_id, message):
        record = self._drone(drone_id)
        record['last_seen'] = self.now()
        kind = message.get('type')
        if kind == 'status':
            self._on_status(drone_id, record, message)
        elif kind == 'log':
            for line in message.get('lines', []):
                self.log(drone_id, line.get('level', 'INFO'), line.get('text', ''))
        elif kind == 'preflight_result':
            self._on_preflight_result(drone_id, message)
        elif kind == 'package_ack':
            self._on_package_ack(drone_id, message)
        elif kind == 'go_ack':
            if message.get('run_id') == self.run_id:
                self.go_acks[drone_id] = True
        elif kind == 'abort_ack':
            self._on_abort_ack(drone_id, message)

    # --- Operator commands: each returns {'ok', 'state', 'reason'} -------------

    def run_command(self, command_id, command):
        """Run `command` once per command_id, so a retry over a bad link cannot fire twice."""
        if command_id and command_id in self.commands:
            return self.commands[command_id]
        result = command()
        if command_id:
            self.commands[command_id] = result
            while len(self.commands) > COMMANDS_REMEMBERED:
                self.commands.popitem(last=False)
        return result

    def select(self, mission_id):
        if self.state not in (IDLE, READY, COMPLETE):
            return self._refused(f'cannot pick a mission while {self.state}')
        mission = self.missions.get(mission_id)
        if mission is None:
            return self._refused(f'unknown mission: {mission_id}')
        self._clear_run()
        self.mission = mission
        self.checks = {drone_id: None for drone_id in mission['drones']}
        missing = [drone_id for drone_id in mission['drones'] if drone_id not in self.links]
        if missing:
            self._set_state(IDLE, f"{', '.join(missing)} not connected")
            return self._result(False)
        self._set_state(PREFLIGHT)
        self._send_all({'type': 'preflight', 'mission_id': mission['id'], 'launch': mission['launch']})
        return self._result(True)

    def start(self, press_age_ms=0):
        """Begin the handoff. The clock starts at the press, not at this request."""
        if self.state != READY:
            return self._refused(f'cannot start while {self.state}')
        press_age_s = min(max(float(press_age_ms), 0.0) / 1000.0, MAX_PRESS_AGE_S)
        self.clock_started = self.now() - press_age_s
        self.clock_stopped = None
        self.run_id = time.strftime('%Y%m%d_%H%M%S_') + self.mission['id']
        self.committed = False
        self.package_acks = {drone_id: None for drone_id in self.mission['drones']}
        self.go_acks = {}
        if self.store:
            self.store.begin(self.run_id, self.mission['id'])
        self._set_state(STARTING)
        self._send_package()
        return self._result(True)

    def abort(self):
        """Always accepted: stopping must never be refused."""
        if self.state in (PREFLIGHT, READY) or (self.state == STARTING and not self.committed):
            self._send_all({'type': 'cancel', 'run_id': self.run_id})
            self._clear_run()
            self._set_state(IDLE, 'Cancelled before anything flew')
        elif self.state in (STARTING, SEARCHING, LANDING):
            self._begin_abort('Aborted by the operator')
        elif self.state == IDLE:
            self.idle_abort_at = self.now()
        return self._result(True)

    def recover(self, action):
        """The operator's choice after Hold: 'return' or 'land'."""
        if self.state != ABORTED:
            return self._refused('nothing is holding')
        if action not in RECOVERY_ACTIONS:
            return self._refused(f'unknown action: {action}')
        self.recovery = action
        self._send_all({'type': 'recover', 'action': action}, list(self.abort_results))
        self.log('hub', 'INFO', f'operator chose {action}')
        return self._result(True)

    def reset(self):
        if self.state not in (COMPLETE, ABORTED, ABORTING):
            return self._result(self.state == IDLE)
        pending = [d for d, result in self.abort_results.items() if result is None and d in self.links]
        if self.state == ABORTING and pending:
            return self._refused(f"still waiting on {', '.join(pending)}")
        flying = [d for d in self._mission_drones()
                  if d in self.links and self._report(d).get('airborne')]
        if flying:
            return self._refused(f"{', '.join(flying)} still airborne")
        self._send_all({'type': 'reset'})
        self._clear_run()
        self.mission = None
        self.checks = {}
        self._set_state(IDLE)
        return self._result(True)

    # --- Time-driven work -------------------------------------------------------

    def tick(self):
        now = self.now()
        if self.state == PREFLIGHT and now - self.step_started > PREFLIGHT_TIMEOUT_S:
            silent = [d for d, check in self.checks.items() if check is None]
            self._set_state(IDLE, f"No preflight answer from {', '.join(silent)}")
        elif self.state == READY:
            lost = [d for d in self._mission_drones() if d not in self.links]
            if lost:
                self._set_state(IDLE, f"{', '.join(lost)} disconnected")
        elif self.state == STARTING and not self.committed:
            silent = [d for d, ack in self.package_acks.items() if ack is None]
            if now - self.step_started > PACKAGE_TIMEOUT_S:
                self._fail_start(f"{', '.join(silent)} did not confirm the mission package. Nothing flew")
            elif now - self.last_send > RESEND_S:
                self._send_package(silent)
        elif self.state == STARTING:
            silent = [d for d in self._mission_drones() if not self.go_acks.get(d)]
            if silent and now - self.step_started > GO_TIMEOUT_S:
                self._begin_abort(f"{', '.join(silent)} did not confirm Go")
            elif silent and now - self.last_send > RESEND_S:
                self._send_all({'type': 'go', 'run_id': self.run_id}, silent)
                self.last_send = now
        elif self.state == ABORTING and now - self.last_send > RESEND_S:
            silent = [d for d, result in self.abort_results.items() if result is None]
            self._send_all({'type': 'abort'}, silent)
            self.last_send = now

    # --- What the phone sees ----------------------------------------------------

    def snapshot(self):
        now = self.now()
        return {
            'type': 'state',
            'state': self.state,
            'reason': self.reason,
            'mission': public_mission(self.mission) if self.mission else None,
            'run_id': self.run_id,
            'committed': self.committed,
            'clock': self._clock(now),
            'checks': self.checks,
            'abort_results': self._abort_view(now),
            'recovery': self.recovery,
            'drones': [self._drone_view(d, now) for d in sorted(self.drones)],
            'log_seq': self.log_seq,
        }

    def mission_list(self):
        return [public_mission(m) for m in self.missions.values()]

    def log(self, source, level, text):
        self.log_seq += 1
        line = {'seq': self.log_seq, 'source': source, 'level': level, 'text': text}
        self.logs.append(line)
        if self.store:
            self.store.log(line)

    def logs_after(self, seq):
        return [line for line in self.logs if line['seq'] > seq]

    # --- Internals --------------------------------------------------------------

    def _drone(self, drone_id):
        return self.drones.setdefault(drone_id, {'last_seen': None, 'status': {}})

    def _report(self, drone_id):
        return self._drone(drone_id)['status']

    def _mission_drones(self):
        return list(self.mission['drones']) if self.mission else []

    def _clear_run(self):
        if self.run_id:
            self.closed_runs.add(self.run_id)
        self.run_id = None
        self.committed = False
        self.clock_started = None
        self.clock_stopped = None
        self.package_acks = {}
        self.go_acks = {}
        self.abort_results = {}
        self.recovery = None
        if self.store:
            self.store.folder = None

    def _set_state(self, state, reason=''):
        self.state = state
        self.reason = reason
        self.step_started = self.now()
        self.last_send = self.now()
        self.log('hub', 'WARN' if reason else 'INFO',
                 f'state: {state}' + (f' ({reason})' if reason else ''))
        if self.store:
            self.store.event(kind='state', state=state, reason=reason)

    def _send_all(self, message, drone_ids=None):
        for drone_id in (self._mission_drones() if drone_ids is None else drone_ids):
            send = self.links.get(drone_id)
            if send:
                send(message)

    def _send_package(self, drone_ids=None):
        self._send_all({'type': 'package', 'run_id': self.run_id, 'mission_id': self.mission['id'],
                        'launch': self.mission['launch']}, drone_ids)
        self.last_send = self.now()

    def _stop_clock(self):
        if self.clock_started is not None and self.clock_stopped is None:
            self.clock_stopped = self.now()

    def _fail_start(self, reason):
        """Nothing has been told to fly yet, so this returns straight to Idle."""
        self._send_all({'type': 'cancel', 'run_id': self.run_id})
        self._clear_run()
        self._set_state(IDLE, reason)

    def _begin_abort(self, reason):
        self._stop_clock()
        self.abort_results = {drone_id: None for drone_id in self._mission_drones()}
        self._set_state(ABORTING, reason)
        self._send_all({'type': 'abort'})

    def _on_preflight_result(self, drone_id, message):
        if self.state != PREFLIGHT or not self.mission or message.get('mission_id') != self.mission['id']:
            return
        if drone_id not in self.checks:
            return
        problems = list(message.get('problems', []))
        self.checks[drone_id] = {'ok': not problems, 'problems': problems}
        if any(check is None for check in self.checks.values()):
            return
        failed = [d for d, check in self.checks.items() if not check['ok']]
        if failed:
            self._set_state(IDLE, f"Start checks failed: {', '.join(failed)}")
        else:
            self._set_state(READY)

    def _on_package_ack(self, drone_id, message):
        if self.state != STARTING or self.committed or message.get('run_id') != self.run_id:
            return
        if drone_id not in self.package_acks:
            return
        problems = list(message.get('problems', []))
        if problems:
            self._fail_start(f"{drone_id} refused the mission: {'; '.join(problems)}")
            return
        self.package_acks[drone_id] = True
        if all(self.package_acks.values()):
            self.committed = True
            self.step_started = self.now()
            self.last_send = self.now()
            self._send_all({'type': 'go', 'run_id': self.run_id})
            self.log('hub', 'INFO', 'committed: every drone holds its mission package')

    def _on_abort_ack(self, drone_id, message):
        if self.state not in (ABORTING, ABORTED) or drone_id not in self.abort_results:
            return
        self.abort_results[drone_id] = message.get('result', 'unknown')
        if self.state == ABORTING and all(result is not None for result in self.abort_results.values()):
            self._set_state(ABORTED, self.reason)

    def _on_status(self, drone_id, record, message):
        previous = record['status'].get('run_state')
        record['status'] = message
        if message.get('run_state') == 'failed' and previous != 'failed':
            self.log(drone_id, 'ERROR', message.get('failure') or 'launch failed')
        if self.state == IDLE:
            self._adopt_run(drone_id, message)
        self._advance()

    def _adopt_run(self, drone_id, message):
        """A hub that restarts comes back Idle while drones may still be flying. Rebuild
        the run from their reports, or the abort button would have nothing to act on.
        """
        run_id = message.get('run_id')
        run_state = message.get('run_state')
        if not run_id or run_id in self.closed_runs:
            return
        airborne = message.get('airborne')
        # A failed run can still be in the air: PX4 flying a failsafe, or a pilot flying it.
        flying = run_state in ('running', 'landing') or (run_state == 'failed' and airborne)
        holding = run_state in ('aborting', 'aborted') and airborne
        # Finished while the hub was down: without this the results would have nowhere to go.
        finished = run_state == 'done' or (run_state == 'failed' and message.get('landed_after_flight'))
        if not (flying or holding or finished):
            return
        mission_id = message.get('mission_id')
        self.mission = self.missions.get(mission_id) or {
            'id': mission_id, 'name': mission_id or 'Unknown mission', 'inputs': [],
            'drones': {drone_id: {'role': 'search'}}, 'launch': None}
        self.run_id = run_id
        self.committed = True
        self.clock_started = self.now() - float(message.get('run_age_s') or 0.0)
        self.go_acks = {d: True for d in self._mission_drones()}
        self.log('hub', 'WARN', f'picked up run {run_id} from {drone_id}')
        if flying:
            self._set_state(SEARCHING)
            if self.idle_abort_at is not None and self.now() - self.idle_abort_at < ABORT_CARRY_S:
                self._begin_abort('Aborted by the operator')
        elif finished:
            self._stop_clock()
            self._set_state(LANDING)
        else:
            self._stop_clock()
            self.abort_results = {
                d: self._report(d).get('abort_result') or 'unknown' for d in self._mission_drones()}
            self._set_state(ABORTED, 'Aborted before the hub restarted')

    def _finished(self, drone_id):
        report = self._report(drone_id)
        return (report.get('run_state') in ('done', 'failed')
                and not report.get('airborne') and not report.get('armed'))

    def _advance(self):
        """Mirror what the drones report. None of this tells a drone what to do."""
        drones = self._mission_drones()
        if not drones or not self.committed:
            return
        if self.state == STARTING:
            failed = [d for d in drones if self._report(d).get('run_state') == 'failed']
            if failed:
                reason = '; '.join(self._report(d).get('failure') or f'{d} failed' for d in failed)
                if any(self._report(d).get('armed') or self._report(d).get('airborne') for d in drones):
                    self._begin_abort(reason)
                else:
                    # Nothing armed: stop any other launches and go straight back to Idle.
                    self._send_all({'type': 'abort'})
                    self._clear_run()
                    self._set_state(IDLE, reason)
            elif all(self._report(d).get('airborne') or self._finished(d) for d in drones):
                self._set_state(SEARCHING)
        elif self.state == SEARCHING:
            if all(self._report(d).get('run_state') == 'landing' or self._finished(d) for d in drones):
                self._stop_clock()
                self._set_state(LANDING)
        elif self.state == LANDING:
            if all(self._finished(d) for d in drones):
                self._stop_clock()
                self._set_state(COMPLETE)

    def _refused(self, reason):
        return {'ok': False, 'state': self.state, 'reason': reason}

    def _result(self, ok):
        return {'ok': ok, 'state': self.state, 'reason': self.reason}

    def _clock(self, now):
        if self.clock_started is None:
            return {'running': False, 'elapsed_ms': 0}
        end = self.clock_stopped if self.clock_stopped is not None else now
        return {'running': self.clock_stopped is None,
                'elapsed_ms': int((end - self.clock_started) * 1000)}

    def _abort_view(self, now):
        if not self.abort_results:
            return {}
        unanswered = 'sending' if now - self.step_started < ABORT_NO_RESPONSE_S else 'no_response'
        return {d: result if result is not None else unanswered
                for d, result in self.abort_results.items()}

    def _drone_view(self, drone_id, now):
        record = self.drones[drone_id]
        status = record['status']
        role = None
        if self.mission and drone_id in self.mission['drones']:
            role = self.mission['drones'][drone_id]['role']
        return {
            'id': drone_id,
            'role': role,
            'connected': drone_id in self.links,
            'link_age_s': None if record['last_seen'] is None else round(now - record['last_seen'], 1),
            'run_state': status.get('run_state'),
            'activity': status.get('activity'),
            'armed': status.get('armed'),
            'airborne': status.get('airborne'),
            'position_ok': status.get('position_ok'),
            'position': status.get('position'),
            'battery_pct': status.get('battery_pct'),
            'global_position_ok': status.get('global_position_ok'),
            'gps_fix': status.get('gps_fix'),
            'rc_lost': status.get('rc_lost'),
            'waypoint': status.get('waypoint'),
            'waypoint_total': status.get('waypoint_total'),
            'recovery': status.get('recovery'),
            'hold_left_s': status.get('hold_left_s'),
            'failure': status.get('failure'),
        }
