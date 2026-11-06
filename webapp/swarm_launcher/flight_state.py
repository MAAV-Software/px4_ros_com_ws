"""What the agent can work out about a flight, shared by the real and fake backends.

Pure Python (no ROS imports) so it can be unit tested without a ROS install.

vehicle_status gives arming state and nav_state directly when px4_msgs matches the
firmware. The inferences here remain for a drone whose firmware doesn't match, where
that topic delivers nothing: armed comes from command acks and being in the air, and
progress from the node's log lines. Armed is deliberately conservative then: when
unsure it says armed, because the abort path for an armed drone is the safe one.
"""
import re

# VehicleCommand values, checked against px4_msgs by test_flight_state.py.
VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_RESULT_ACCEPTED = 0

PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3.0

# Identifies the agent's commands, so an ack can be told apart from one meant
# for the offboard node (which uses component 1). 191 is MAVLink's onboard computer.
AGENT_COMPONENT = 191

# name -> (command, param1, param2, param3)
COMMANDS = {
    'hold': (VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    'return': (VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0, 0.0),
    'land': (VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0),
    'disarm': (VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, 0.0),
}

# VehicleStatus values, checked against px4_msgs by test_flight_state.py.
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_MANUAL = 0
NAVIGATION_STATE_ALTCTL = 1
NAVIGATION_STATE_POSCTL = 2
NAVIGATION_STATE_AUTO_LOITER = 4
NAVIGATION_STATE_AUTO_RTL = 5
NAVIGATION_STATE_POSITION_SLOW = 6
NAVIGATION_STATE_ALTITUDE_CRUISE = 8
NAVIGATION_STATE_ACRO = 10
NAVIGATION_STATE_DESCEND = 12
NAVIGATION_STATE_OFFBOARD = 14
NAVIGATION_STATE_STAB = 15
NAVIGATION_STATE_AUTO_LAND = 18
NAVIGATION_STATE_AUTO_PRECLAND = 20

# Modes only a pilot flies. Nothing here may command a drone out of one of these.
PILOT_NAV_STATES = {
    NAVIGATION_STATE_MANUAL, NAVIGATION_STATE_ALTCTL, NAVIGATION_STATE_POSCTL,
    NAVIGATION_STATE_POSITION_SLOW, NAVIGATION_STATE_ALTITUDE_CRUISE,
    NAVIGATION_STATE_ACRO, NAVIGATION_STATE_STAB,
}
COMING_DOWN_NAV_STATES = {
    NAVIGATION_STATE_AUTO_RTL, NAVIGATION_STATE_AUTO_LAND,
    NAVIGATION_STATE_AUTO_PRECLAND, NAVIGATION_STATE_DESCEND,
}
NAV_STATE_NAMES = {
    NAVIGATION_STATE_MANUAL: 'Manual',
    NAVIGATION_STATE_ALTCTL: 'Altitude',
    NAVIGATION_STATE_POSCTL: 'Position',
    NAVIGATION_STATE_AUTO_LOITER: 'Hold',
    NAVIGATION_STATE_AUTO_RTL: 'Return',
    NAVIGATION_STATE_POSITION_SLOW: 'Position slow',
    NAVIGATION_STATE_ALTITUDE_CRUISE: 'Altitude cruise',
    NAVIGATION_STATE_ACRO: 'Acro',
    NAVIGATION_STATE_DESCEND: 'Descend',
    NAVIGATION_STATE_OFFBOARD: 'Offboard',
    NAVIGATION_STATE_STAB: 'Stabilized',
    NAVIGATION_STATE_AUTO_LAND: 'Land',
    NAVIGATION_STATE_AUTO_PRECLAND: 'Precision land',
}

ACTIVITY_WHEN_COMMANDED = {'hold': 'Hold', 'return': 'Returning', 'land': 'Landing'}

STATUS_MAX_AGE_S = 3.0
# Used only when vehicle_land_detected is not arriving.
LAND_DETECTED_MAX_AGE_S = 2.0
AIRBORNE_ABOVE_START_M = 1.0
LANDED_WITHIN_START_M = 0.5
LANDED_MAX_SPEED_M_S = 0.2
LANDED_SETTLE_S = 3.0

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')
LOADED = re.compile(r'loaded (\d+) waypoints')
ADVANCING = re.compile(r'advancing to waypoint (\d+)')
GAVE_UP = re.compile(r'not flying: (.*)')
NEVER_READY = 'PX4 never became ready'
# Launch only exits once every process has, so a recorder still running would hide this.
NODE_DIED = re.compile(r'\[(offboard\w*)-\d+\]: process has died \[pid \d+, exit code (-?\d+)')


def nav_state_name(nav_state):
    return NAV_STATE_NAMES.get(nav_state, f'mode {nav_state}')


def log_level(line):
    for level in ('FATAL', 'ERROR', 'WARN', 'INFO', 'DEBUG'):
        if f'[{level}]' in line:
            return 'ERROR' if level == 'FATAL' else level
    return 'INFO'


def clean_log_line(line):
    return ANSI_ESCAPE.sub('', line).rstrip()


def versioned_topic(prefix, name, message_type):
    """PX4 adds _v<N> to a versioned message's topic name, so a topic follows px4_msgs.

    Against firmware with a different version, this names a topic nobody publishes: quiet,
    where subscribing to a type that doesn't fit floods the output with RTPS errors.
    """
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    return f'{prefix}/fmu/out/{name}{suffix}'


def launch_argv(launch, topic_prefix, share_directory):
    """The `ros2 launch` command for a manifest's launch section.

    share_directory is get_package_share_directory, passed in so this stays ROS-free.
    """
    argv = ['ros2', 'launch', launch['package'], launch['file']]
    argv += [f'{key}:={value}' for key, value in launch.get('args', {}).items()]
    # ros2 launch refuses an argument with an empty value, and the launch default is no prefix.
    if topic_prefix:
        argv.append(f'topic_prefix:={topic_prefix}')
    waypoints = launch.get('waypoints')
    if waypoints:
        argv.append(f"path:={share_directory(waypoints['package'])}/{waypoints['file']}")
    return argv


class FlightTracker:
    """Tracks one run from PX4 data and the offboard node's log lines."""

    def __init__(self, now):
        self.now = now
        self.land_detected = None
        self.land_detected_at = None
        self.px4_armed = None
        self.reported_nav_state = None
        self.status_at = None
        self.begin_run()

    def begin_run(self):
        self.arm_accepted = False
        self.airborne = False
        self.was_airborne = False
        self.start_z = None
        self.settled_since = None
        self.landing_logged = False
        self.flying_route = False
        self.entered_offboard = False
        self.gave_up = ''
        self.node_died = ''
        self.waypoint = None
        self.waypoint_total = None

    def on_arm_ack(self):
        self.arm_accepted = True

    def on_status(self, armed, nav_state):
        self.px4_armed = armed
        self.reported_nav_state = nav_state
        self.status_at = self.now()
        if nav_state == NAVIGATION_STATE_OFFBOARD:
            self.entered_offboard = True

    def on_land_detected(self, landed):
        self.land_detected = landed
        self.land_detected_at = self.now()
        self._set_airborne(not landed)

    def on_position(self, z, vz, z_valid):
        if not z_valid:
            return
        if self.start_z is None and not self.airborne:
            self.start_z = z
        if self.start_z is None or self._land_detected_fresh():
            return
        # NED: up is negative z.
        height = self.start_z - z
        if height > AIRBORNE_ABOVE_START_M:
            self.settled_since = None
            self._set_airborne(True)
        elif self.airborne and height < LANDED_WITHIN_START_M and abs(vz) < LANDED_MAX_SPEED_M_S:
            if self.settled_since is None:
                self.settled_since = self.now()
            if self.now() - self.settled_since >= LANDED_SETTLE_S:
                self._set_airborne(False)
        else:
            self.settled_since = None

    def on_log(self, line):
        loaded = LOADED.search(line)
        if loaded:
            self.waypoint_total = int(loaded.group(1))
            self.waypoint = 0
        advancing = ADVANCING.search(line)
        if advancing:
            self.waypoint = int(advancing.group(1))
        if 'arm and offboard mode accepted by PX4' in line:
            self.flying_route = True
        if 'Final hold complete; landing' in line:
            self.landing_logged = True
        gave_up = GAVE_UP.search(line)
        if gave_up:
            self.gave_up = f'Not flying: {gave_up.group(1).strip()}'
        if NEVER_READY in line:
            self.gave_up = NEVER_READY
        died = NODE_DIED.search(line)
        if died:
            self.node_died = f'{died.group(1)} died with exit code {died.group(2)}'

    @property
    def nav_state(self):
        """PX4's current mode, or None when vehicle_status isn't arriving."""
        return self.reported_nav_state if self.status_fresh else None

    @property
    def status_fresh(self):
        return self.status_at is not None and self.now() - self.status_at < STATUS_MAX_AGE_S

    @property
    def landed_after_flight(self):
        return self.was_airborne and not self.airborne

    @property
    def armed(self):
        if self.status_fresh:
            return self.px4_armed
        if self.gave_up or self.landed_after_flight:
            return False
        return self.arm_accepted or self.airborne

    @property
    def landing(self):
        """The route is over and the node asked to land. A failsafe's Return is not this."""
        return self.landing_logged or self.nav_state == NAVIGATION_STATE_AUTO_LAND

    @property
    def pilot_control(self):
        return self.nav_state in PILOT_NAV_STATES

    @property
    def coming_down(self):
        return self.nav_state in COMING_DOWN_NAV_STATES

    @property
    def left_offboard(self):
        """Why the node should stop: someone else changed the mode while it was flying.

        Left running it would finish its route and send LAND, overriding the pilot.
        Returns the mode it went to, or '' if nothing has changed.
        """
        nav_state = self.nav_state
        if (not self.entered_offboard or nav_state is None or self.landing
                or nav_state == NAVIGATION_STATE_OFFBOARD):
            return ''
        return nav_state_name(nav_state)

    def activity(self, launch_running, commanded):
        nav_state = self.nav_state
        # Airborne comes first: this must never read On ground for a drone in the air.
        if self.airborne:
            if nav_state is None:
                return self._activity_in_the_air_without_status(launch_running, commanded)
            if nav_state == NAVIGATION_STATE_OFFBOARD:
                return 'Flying route'
            if nav_state == NAVIGATION_STATE_AUTO_LOITER:
                return 'Hold'
            if nav_state == NAVIGATION_STATE_AUTO_RTL:
                return 'Returning'
            if nav_state in COMING_DOWN_NAV_STATES:
                return 'Landing'
            if nav_state in PILOT_NAV_STATES:
                return f'Pilot control, {nav_state_name(nav_state)}'
            return nav_state_name(nav_state)
        if self.armed:
            if self.landed_after_flight:
                return 'Landed, still armed'
            return ACTIVITY_WHEN_COMMANDED.get(commanded, 'Armed')
        if self.landed_after_flight or not launch_running:
            return 'On ground'
        return 'Starting'

    def summary(self, launch_running, commanded):
        """The fields both flight backends report in their snapshot."""
        return {
            'launch_running': launch_running,
            'armed': self.armed,
            'airborne': self.airborne,
            'landing': self.landing,
            'landed_after_flight': self.landed_after_flight,
            'gave_up': self.gave_up,
            'node_died': self.node_died,
            'left_offboard': self.left_offboard,
            'pilot_control': self.pilot_control,
            'coming_down': self.coming_down,
            'nav_state': self.nav_state,
            'waypoint': self.waypoint,
            'waypoint_total': self.waypoint_total,
            'activity': self.activity(launch_running, commanded),
        }

    def _activity_in_the_air_without_status(self, launch_running, commanded):
        if commanded:
            return ACTIVITY_WHEN_COMMANDED[commanded]
        if not launch_running:
            return 'In the air, PX4 failsafe'
        return 'Landing' if self.landing else 'Flying route'

    def _land_detected_fresh(self):
        return (self.land_detected_at is not None
                and self.now() - self.land_detected_at < LAND_DETECTED_MAX_AGE_S)

    def _set_airborne(self, airborne):
        self.airborne = airborne
        if airborne:
            self.was_airborne = True
