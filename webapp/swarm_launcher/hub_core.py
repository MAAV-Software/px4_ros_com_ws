"""Mission logic for the hub: one state machine, the fleet table and the log.

Pure Python with no network or ROS imports, so every rule can be tested with a
fake clock. hub.py wires this to FastAPI and the WebSockets.

The hub starts a mission and collects what comes back, but does not run it.
After Go each drone flies on its own, and this class only mirrors its reports.
"""
import collections
import math
import re
import time

from swarm_launcher.grid import AreaError, Field, fit_around, global_to_local, parse_corners
from swarm_launcher.launch_library import override_problems
from swarm_launcher.sitl_manager import SitlError
from swarm_launcher.voice import parse_command

IDLE = 'idle'
PREFLIGHT = 'preflight'
READY = 'ready'
STARTING = 'starting'
SEARCHING = 'searching'
LANDING = 'landing'
COMPLETE = 'complete'
ABORTING = 'aborting'
ABORTED = 'aborted'

# Long enough for a drone to read a launch file's arguments the first time it's picked.
PREFLIGHT_TIMEOUT_S = 15.0
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
# SITL: drones the hub starts itself, from the Fleet tab. HITL: real drones, connected by hand.
MODES = ('sitl', 'hitl')
MODE_NAMES = {'sitl': 'SITL', 'hitl': 'HITL'}
RUN_STATES = (STARTING, SEARCHING, LANDING, ABORTING, ABORTED)
SITL_START_TIMEOUT_S = 180.0
# Each drone keeps its own colour slot while the hub knows it. The phone holds the colours;
# past this many, a drone is drawn in ink.
COLOR_SLOTS = 6
# A trail point at most every this many metres, so hovering doesn't pile points up.
TRAIL_STEP_M = 0.3
TRAIL_MAX_POINTS = 20000
PACKAGE_NAME = re.compile(r'^[A-Za-z0-9_]+$')
LAUNCH_FILE = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_.-]*\.py$')


class HubCore:
    def __init__(self, now=time.monotonic, store=None, mode='hitl', save_mode=None, sitl=None,
                 area=None, save_area=None, voice=None):
        self.now = now
        self.store = store
        self.voice = voice
        self.https_port = None  # Set by hub.py when it serves HTTPS, so the phone can offer the link.
        self.voice_reason = voice.unavailable_reason() if voice else 'voice is not set up on this hub'
        self.save_area = save_area
        self.area = None
        if area:
            try:
                self.area = Field(parse_corners(area))
            except AreaError:
                pass  # A saved area that no longer parses is simply not loaded.
        self.mode = mode if mode in MODES else 'hitl'
        self.save_mode = save_mode
        self.sitl = sitl
        self.sitl_gone = set()
        self.colors = {}
        # Where each picked drone has been this mission, as (lat, lon). The version changes
        # whenever the trails start again or their grid does, so phones know to redraw.
        self.trails = {}
        self.trail_version = 0
        self.state = IDLE
        self.mission = None
        self.run_id = None
        self.reason = ''
        self.committed = False
        self.clock_started = None
        self.clock_stopped = None
        self.links = {}
        self.drones = {}
        self.catalogs = {}
        self.check_id = 0
        self.described = {}
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

    # --- Agent links -----------------------------------------------------------

    def agent_connected(self, drone_id, send):
        self.links[drone_id] = send
        self._drone(drone_id)['last_seen'] = self.now()
        send({'type': 'catalog_request'})
        self.log('hub', 'INFO', f'{drone_id} connected')

    def agent_disconnected(self, drone_id, send):
        # A reconnect can arrive before the old socket's close does.
        if self.links.get(drone_id) is send:
            del self.links[drone_id]
            self.catalogs.pop(drone_id, None)
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
        elif kind == 'catalog':
            self.catalogs[drone_id] = message.get('folders') or []
            record['sim'] = message.get('sim')
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

    def set_mode(self, mode):
        """SITL or HITL. Not during a run, and it clears the pick, since its drones belong
        to the other mode.
        """
        if mode not in MODES:
            return self._refused(f'unknown mode: {mode}')
        if mode == self.mode:
            return self._result(True)
        if self.state not in (IDLE, PREFLIGHT, READY, COMPLETE):
            return self._refused(f'cannot switch to {MODE_NAMES[mode]} during a run')
        self.mode = mode
        if self.save_mode:
            self.save_mode(mode)
        self._clear_run()
        self.mission = None
        self.checks = {}
        self.described = {}
        self.log('hub', 'INFO', f'switched to {MODE_NAMES[mode]}')
        self._set_state(IDLE)
        return self._result(True)

    def voice_refusal(self):
        """Why a voice clip can't be heard now, as a refused result, or None."""
        if self.voice:
            self.voice_reason = self.voice.unavailable_reason()
        reason = self.voice_reason or self._voice_flying()
        return self._refused(reason) if reason else None

    def spoken(self, text):
        """What a voice clip or typed command said. Abort happens here and now; start,
        confirm and cancel go back to the phone, which holds the confirm and its timing.
        """
        command = parse_command(text)
        self.log('hub', 'INFO', f'heard "{text}"' + (f': {command}' if command else ', not a command'))
        result = self.abort() if command == 'abort' else self._result(True)
        return {**result, 'heard': text, 'command': command}

    def set_area(self, corners):
        """The search area's four corners, or None to clear it. It sets the grid every drone's
        position is drawn on, so it can't change during a run.
        """
        if self.state not in (IDLE, PREFLIGHT, READY, COMPLETE):
            return self._refused('cannot change the search area during a run')
        if corners is None:
            self.area = None
            self.log('hub', 'INFO', 'search area cleared')
        else:
            try:
                self.area = Field(parse_corners(corners))
            except AreaError as error:
                return self._refused(str(error))
            flags = '; '.join(self.area.flags)
            self.log('hub', 'WARN' if flags else 'INFO',
                     'search area set' + (f', but {flags}' if flags else ''))
        self.trail_version += 1  # The same trails, on a new grid or none.
        if self.save_area:
            self.save_area(self.area.view()['corners'] if self.area else None)
        return self._result(True)

    def fit_area(self, drone_id):
        """SITL only: a north-south field with this drone just inside its south-west corner,
        since simulated drones have no measured corners.
        """
        if self.mode != 'sitl' or not self._is_sim(drone_id):
            return self._refused('only for SITL drones. Type the measured corners for a real field')
        place = self._status(drone_id).get('global')
        if not place:
            return self._refused(f'{drone_id} has no global position yet')
        return self.set_area(fit_around(place['lat'], place['lon']))

    def sitl_add(self):
        if self.mode != 'sitl':
            return self._refused('switch to SITL to add SITL drones')
        if self.sitl is None:
            return self._refused('this hub cannot start SITL drones')
        starting = [d['drone_id'] for d in self._sitl_view() if d['state'] == 'starting']
        if starting:
            # Each new instance joins the world the first one starts.
            return self._refused(f"wait for {', '.join(starting)} to finish starting")
        try:
            drone_id = self.sitl.add()
        except SitlError as error:
            return self._refused(str(error))
        self.log('hub', 'INFO', f'starting SITL drone {drone_id}')
        return self._result(True)

    def sitl_remove(self, drone_id):
        if self.sitl is None:
            return self._refused('this hub cannot start SITL drones')
        busy = self._sitl_busy([drone_id])
        if busy:
            return self._refused(busy)
        try:
            self.sitl.remove(drone_id)
        except SitlError as error:
            return self._refused(str(error))
        self.log('hub', 'INFO', f'stopping SITL drone {drone_id}')
        return self._result(True)

    def sitl_stop_all(self):
        if self.sitl is None:
            return self._refused('this hub cannot start SITL drones')
        if self.mode != 'sitl':
            return self._refused('switch to SITL to stop SITL drones')
        busy = self._sitl_busy([d['drone_id'] for d in self.sitl.view()])
        if busy:
            return self._refused(busy)
        self.sitl.stop_all()
        self.log('hub', 'INFO', 'stopping every SITL drone')
        return self._result(True)

    def select(self, package, file, drones, args=None):
        """Pick a launch file, the drones to run it and any arguments changed from the
        file's defaults, then run the start checks. Each drone checks the file against its
        own src, so this only checks the request is well formed.
        """
        # Allowed while checking: an older check's answers are told apart by check_id.
        if self.state not in (IDLE, PREFLIGHT, READY, COMPLETE):
            return self._refused(f'cannot pick a mission while {self.state}')
        if not (isinstance(package, str) and PACKAGE_NAME.match(package)
                and isinstance(file, str) and LAUNCH_FILE.match(file)):
            return self._refused('pick a folder and a launch file')
        if not (isinstance(drones, list) and drones and all(isinstance(d, str) for d in drones)):
            return self._refused('pick at least one drone')
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return self._refused('launch arguments must be a mapping')
        # An emptied field means the file's default.
        args = {name: value for name, value in args.items() if value != ''}
        problems = override_problems(args)
        if problems:
            return self._refused('; '.join(problems))
        wrong_kind = [d for d in drones if self._is_sim(d) is not None and self._is_sim(d) != (self.mode == 'sitl')]
        if wrong_kind:
            kind = 'real' if self.mode == 'sitl' else 'SITL'
            other = 'HITL' if self.mode == 'sitl' else 'SITL'
            return self._refused(f"{', '.join(wrong_kind)}: {kind} drone. Switch to {other} to fly it")
        same_file = bool(self.mission) and (self.mission['package'], self.mission['file']) == (package, file)
        self._clear_run()
        self.mission = self._new_mission(package, file, list(dict.fromkeys(drones)), args)
        if not same_file:
            self.described = {}
        self.checks = {drone_id: None for drone_id in self.mission['drones']}
        missing = [drone_id for drone_id in self.mission['drones'] if drone_id not in self.links]
        if missing:
            self._set_state(IDLE, f"{', '.join(missing)} not connected")
            return self._result(False)
        self.check_id += 1
        self._set_state(PREFLIGHT)
        self._send_all({'type': 'preflight', 'check_id': self.check_id, 'mission_id': self.mission['id'],
                        'mode': self.mode, 'launch': self.mission['launch']})
        return self._result(True)

    def start(self, press_age_ms=0):
        """Begin the handoff. The clock starts at the press, not at this request."""
        if self.state != READY:
            return self._refused(f'cannot start while {self.state}')
        press_age_s = min(max(float(press_age_ms), 0.0) / 1000.0, MAX_PRESS_AGE_S)
        self.clock_started = self.now() - press_age_s
        self.clock_stopped = None
        self.run_id = time.strftime('%Y%m%d_%H%M%S_') + re.sub(r'[^A-Za-z0-9_]', '_', self.mission['name'][:-3])
        self.committed = False
        self.package_acks = {drone_id: None for drone_id in self.mission['drones']}
        self.go_acks = {}
        if self.store:
            self.store.begin(self.run_id, self.mission['id'])
        self._clear_trails()
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
        self._clear_trails()
        # The pick stays, so flying it again is Check again, then Start. One picked up from a
        # drone after a restart doesn't: the hub never knew its arguments.
        if self.mission and self.mission.get('adopted'):
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
        self._tick_sitl()
        self._tick_voice()

    def _tick_voice(self):
        # About 300 MB that a flying computer can use better. In flight, abort is the button.
        if self.voice and self.voice.loaded and self._voice_flying():
            self.voice.unload()
            self.log('hub', 'INFO', 'voice recognizer unloaded while drones fly')

    def _voice_flying(self):
        if any(self._status(d).get('airborne') for d in self.links):
            return 'voice is off while drones fly. Use the abort button'
        return ''

    def _tick_sitl(self):
        if self.sitl is None:
            return
        for source, level, text in self.sitl.poll():
            self.log(source, level, text)
        # A removed SITL drone leaves the fleet once its agent has gone, not as a lost drone.
        current = {d['drone_id'] for d in self.sitl.view()}
        for drone_id in list(self.sitl_gone - current):
            if drone_id not in self.links:
                self._forget_drone(drone_id)
                self.sitl_gone.discard(drone_id)
        self.sitl_gone |= current

    # --- What the phone sees ----------------------------------------------------

    def snapshot(self):
        now = self.now()
        return {
            'type': 'state',
            'state': self.state,
            'reason': self.reason,
            'mode': self.mode,
            'sitl': {'available': self.sitl is not None, 'drones': self._sitl_view()},
            'area': self.area.view() if self.area else None,
            'https': {'port': self.https_port} if self.https_port else None,
            'voice': {'available': not (self.voice_reason or self._voice_flying()),
                      'reason': self.voice_reason or self._voice_flying()},
            'mission': self._mission_view(),
            'launch_args': self._launch_args_view(),
            'catalogs': {d: folders for d, folders in sorted(self.catalogs.items()) if d in self.links},
            'run_id': self.run_id,
            'committed': self.committed,
            'clock': self._clock(now),
            'checks': self.checks,
            'abort_results': self._abort_view(now),
            'recovery': self.recovery,
            'drones': [self._drone_view(d, now) for d in sorted(self.drones)],
            'log_seq': self.log_seq,
        }

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
        if drone_id not in self.drones:
            free = [slot for slot in range(COLOR_SLOTS) if slot not in self.colors.values()]
            self.colors[drone_id] = free[0] if free else None
        return self.drones.setdefault(drone_id, {'last_seen': None, 'status': {}})

    def _forget_drone(self, drone_id):
        self.drones.pop(drone_id, None)
        self.colors.pop(drone_id, None)
        self.catalogs.pop(drone_id, None)

    def _record_trail(self, drone_id, message):
        """Add to a picked drone's trail while its run is on."""
        if self.state not in RUN_STATES or drone_id not in self._mission_drones():
            return
        place = message.get('global')
        if not place:
            return
        point = (place['lat'], place['lon'])
        trail = self.trails.setdefault(drone_id, [])
        if trail and math.hypot(*global_to_local(*trail[-1], *point)) < TRAIL_STEP_M:
            return
        if len(trail) < TRAIL_MAX_POINTS:
            trail.append(point)

    def _clear_trails(self):
        self.trails = {}
        self.trail_version += 1

    def trail_update(self, known_version, known_counts):
        """What a phone hasn't been sent yet: (message or None, version, counts). A phone that
        has nothing, or an older version, gets everything; otherwise only the new points. Points
        go out as grid columns and rows, so without a search area there is nothing to send.
        """
        full = known_version != self.trail_version
        counts = {} if full else dict(known_counts)
        points = {}
        for drone_id, trail in self.trails.items():
            start = counts.get(drone_id, 0)
            if self.area and len(trail) > start:
                points[drone_id] = [[cell['col'], cell['row']]
                                    for cell in (self.area.cell(*point) for point in trail[start:])]
            counts[drone_id] = len(trail) if self.area else 0
        if not full and not points:
            return None, known_version, known_counts
        return ({'type': 'trail', 'version': self.trail_version, 'full': full, 'points': points},
                self.trail_version, counts)

    def _report(self, drone_id):
        return self._drone(drone_id)['status']

    def _mission_drones(self):
        return list(self.mission['drones']) if self.mission else []

    def _grid_cell(self, place):
        if not self.area or not place:
            return None
        return self.area.cell(place['lat'], place['lon'])

    def _status(self, drone_id):
        """A drone's last report, without adding an unknown drone to the fleet."""
        return self.drones.get(drone_id, {}).get('status', {})

    def _is_sim(self, drone_id):
        """True for a SITL drone, False for a real one, None until it has said."""
        record = self.drones.get(drone_id, {})
        sim = record.get('status', {}).get('sim', record.get('sim'))
        return None if sim is None else bool(sim)

    def _sitl_busy(self, drone_ids):
        flying = [d for d in drone_ids if self._status(d).get('airborne')]
        if flying:
            return f"{', '.join(flying)} still airborne"
        if self.state in RUN_STATES and set(drone_ids) & set(self._mission_drones()):
            return 'a run is using it. Finish or reset the run first'
        return ''

    def _sitl_view(self):
        """The SITL manager's list, with each running drone's progress from its own reports."""
        if self.sitl is None:
            return []
        now = self.now()
        view = []
        for entry in self.sitl.view():
            state, detail = entry['state'], entry['detail']
            if state == 'running':
                drone_id = entry['drone_id']
                if drone_id in self.links and self._status(drone_id).get('position_ok'):
                    state = 'ready'
                elif now - entry['started_at'] > SITL_START_TIMEOUT_S:
                    state, detail = 'stuck', f'no position after {SITL_START_TIMEOUT_S:.0f} s. See the Log tab'
                else:
                    state = 'starting'
                    detail = 'waiting for a position' if drone_id in self.links else 'starting PX4'
            view.append({'drone_id': entry['drone_id'], 'instance': entry['instance'],
                         'state': state, 'detail': detail})
        return view

    def _new_mission(self, package, file, drones, args):
        folder = next((f['folder'] for folders in self.catalogs.values() for f in folders
                       if f['package'] == package), package)
        return {'id': f'{package}/{file}', 'name': file, 'folder': folder, 'package': package,
                'file': file, 'drones': drones, 'args': args,
                'launch': {'package': package, 'file': file, 'args': args}}

    def _mission_view(self):
        if not self.mission:
            return None
        return {key: self.mission[key] for key in ('id', 'name', 'folder', 'package', 'file', 'drones', 'args')}

    def _launch_args_view(self):
        """The chosen file's arguments as the first drone that read them sees them. Defaults
        that are paths can differ from drone to drone, and each drone uses its own.
        """
        for drone_id in self._mission_drones() + sorted(self.described):
            if self.described.get(drone_id) is not None:
                return {'source': drone_id, 'args': self.described[drone_id]}
        return None

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
                        'mode': self.mode, 'launch': self.mission['launch']}, drone_ids)
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
        if self.state != PREFLIGHT or not self.mission or message.get('check_id') != self.check_id:
            return
        if drone_id not in self.checks:
            return
        if message.get('launch_args') is not None:
            self.described[drone_id] = message['launch_args']
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
        self._record_trail(drone_id, message)
        if message.get('run_state') == 'failed' and previous != 'failed':
            self.log(drone_id, 'ERROR', message.get('failure') or 'launch failed')
        if self.state == IDLE:
            self._adopt_run(drone_id, message)
        elif self.run_id and message.get('run_id') == self.run_id:
            self._join_run(drone_id)
        self._advance()

    def _join_run(self, drone_id):
        """A drone reporting this run that the hub didn't know about, which happens after the
        hub restarts and picks up a run from whichever drone reported first. Abort must reach it.
        """
        if drone_id in self.mission['drones']:
            return
        self.mission['drones'].append(drone_id)
        self.go_acks[drone_id] = True
        if self.state in (ABORTING, ABORTED):
            self.abort_results[drone_id] = self._report(drone_id).get('abort_result')
            if self.abort_results[drone_id] is None:
                self._send_all({'type': 'abort'}, [drone_id])
                if self.state == ABORTED:
                    self._set_state(ABORTING, self.reason)
        self.log('hub', 'WARN', f'{drone_id} is also flying run {self.run_id}')

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
        package, _, file = (message.get('mission_id') or '').partition('/')
        drones = [d for d in sorted(self.drones) if self._report(d).get('run_id') == run_id]
        self.mission = dict(self._new_mission(package or 'unknown', file or 'Unknown mission', drones, {}),
                            adopted=True)
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
        return {
            'id': drone_id,
            'in_mission': bool(self.mission) and drone_id in self.mission['drones'],
            'sim': self._is_sim(drone_id),
            'color': self.colors.get(drone_id),
            'connected': drone_id in self.links,
            'link_age_s': None if record['last_seen'] is None else round(now - record['last_seen'], 1),
            'run_state': status.get('run_state'),
            'activity': status.get('activity'),
            'armed': status.get('armed'),
            'airborne': status.get('airborne'),
            'position_ok': status.get('position_ok'),
            'position': status.get('position'),
            'global': status.get('global'),
            'grid': self._grid_cell(status.get('global')),
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
