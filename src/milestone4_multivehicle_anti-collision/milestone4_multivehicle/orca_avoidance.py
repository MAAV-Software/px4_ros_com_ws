"""ORCA anti-collision with the same shape as avoidance.Avoidance; metres, local NED in, local NED out.

Every drone shares [instance, global_x, global_y, global_vx, global_vy] (world
East/North) and every drone avoids: there is no priority. Each tick the drone
picks the horizontal velocity closest to its preferred one that ORCA allows, and
flies it as a velocity setpoint at the waypoint's altitude. It never stops to
climb, so waypoint progress continues while it sidesteps. The preferred velocity
follows the line from the previous waypoint to this one, so after a sidestep the
drone returns to the line (a scan row) rather than cutting across to the waypoint.

A peer whose messages stop is kept as a stationary obstacle at its last
position, which this drone avoids on its own (full responsibility).
"""
import math

from milestone4_multivehicle import orca

POSITION_TOPIC = '/milestone4/global_positions'
PEER_TIMEOUT_S = 0.5      # ORCA needs fresh velocity; peers publish at 10 Hz
NEIGHBOR_DIST_M = 10.0    # peers farther than this are ignored
RADIUS_M = 1.5            # per drone: airframe plus tracking and latency margin
STALE_RADIUS_M = 2.25     # a stale peer may have moved since its last message
TIME_HORIZON_S = 3.0      # how far ahead collisions are avoided
TIME_STEP_S = 0.1         # the controller's tick, used once drones overlap
APPROACH_GAIN = 0.8       # 1/s: preferred speed slows within max_speed/gain of the waypoint
LOOKAHEAD_M = 2.0         # aim this far along the leg from the closest point on it
# While ORCA is changing this drone's velocity, rotate the preferred velocity this
# far to the right, so exactly head-on pairs break symmetry the same way and pass
# each other instead of both slowing to a stop. Only then: a constant bias near a
# peer that isn't in the way bows the drone off its line (run 4's scan rows).
RIGHT_BIAS_RAD = math.radians(10.0)
# Symmetric groups (four drones meeting at one point) can still all stop, each
# blocked by the others. A drone held below STUCK_SPEED_M_S for STUCK_S while it
# wants to move turns its bias to ROUNDABOUT_BIAS_RAD: everyone then circles the
# same way, as at a roundabout, until no peer is near or ROUNDABOUT_S has passed.
STUCK_SPEED_M_S = 0.2
STUCK_S = 1.5
ROUNDABOUT_BIAS_RAD = math.radians(70.0)
ROUNDABOUT_S = 4.0
DEVIATION_M_S = 0.05      # ORCA counts as "avoiding" when it moves the velocity this much
AVOIDING_LATCH_S = 1.0    # stay AVOIDING this long after the last deviation


class OrcaAvoidance:
    def __init__(self, instance, origin_x, origin_y, vehicle_count=4, max_speed=1.0):
        if type(vehicle_count) is not int or not 2 <= vehicle_count <= 4:
            raise ValueError('vehicle_count must be 2, 3 or 4')
        if type(instance) is not int or not 0 <= instance < vehicle_count:
            raise ValueError('vehicle_instance must be within the configured fleet')
        if not all(math.isfinite(v) for v in (origin_x, origin_y)):
            raise ValueError('Initial global XY must be finite')
        if not (math.isfinite(max_speed) and max_speed > 0):
            raise ValueError('max_speed must be positive')
        self.vehicle_count = vehicle_count
        self.instance = instance
        self.origin = (origin_x, origin_y)
        self.max_speed = float(max_speed)
        self.peers = {}           # instance -> (x, y, vx, vy, received_s), world E/N
        self.state = 'CLEAR'
        self.target = None
        self.last_deviation_s = None
        self.stuck_since = None
        self.roundabout_since = None

    def global_xy(self, position):
        # PX4 local x=N, y=E; the shared world frame is X=E, Y=N.
        return self.origin[0] + position.y, self.origin[1] + position.x

    @staticmethod
    def global_v(position):
        return position.vy, position.vx

    def payload(self, position):
        vx, vy = self.global_v(position) if position.v_xy_valid else (math.nan, math.nan)
        return [float(self.instance), *self.global_xy(position), float(vx), float(vy)]

    def receive(self, data, now_s):
        """Wire payload is exactly [instance, global_x, global_y, global_vx, global_vy]."""
        if len(data) != 5 or not all(math.isfinite(v) for v in data[:3]):
            return
        instance, x, y, vx, vy = data
        if instance != int(instance) or not 0 <= instance < self.vehicle_count or instance == self.instance:
            return
        # A peer without a valid velocity is still a position; assume it is stopped.
        if not (math.isfinite(vx) and math.isfinite(vy)):
            vx = vy = 0.0
        self.peers[int(instance)] = (x, y, vx, vy, now_s)

    def peers_ready(self, now_s):
        # Symmetric: every other drone must be heard from before arming.
        return all(i in self.peers and 0 <= now_s - self.peers[i][4] <= PEER_TIMEOUT_S
                   for i in range(self.vehicle_count) if i != self.instance)

    @staticmethod
    def hold(position):
        return {'pos': (position.x, position.y, position.z),
                'yaw': position.heading, 'hold_s': 0.0}

    def neighbors(self, own_xy, now_s):
        found = []
        for i, (px, py, pvx, pvy, received_s) in sorted(self.peers.items()):
            if math.hypot(px - own_xy[0], py - own_xy[1]) > NEIGHBOR_DIST_M:
                continue
            if 0 <= now_s - received_s <= PEER_TIMEOUT_S:
                found.append(orca.Neighbor(px, py, pvx, pvy, RADIUS_M, 0.5))
            else:
                found.append(orca.Neighbor(px, py, 0.0, 0.0, STALE_RADIUS_M, 1.0))
        return found

    def world(self, waypoint):
        north, east, _ = waypoint['pos']
        return self.origin[0] + east, self.origin[1] + north

    def preferred_velocity(self, own_xy, waypoint, previous=None):
        """Toward the waypoint along the leg from the previous one, at up to max_speed.

        Aims LOOKAHEAD_M along the leg from the closest point on it (never past the
        waypoint), so a drone off the leg steers back onto it. Without a previous
        waypoint, or within LOOKAHEAD_M of this one, it aims at the waypoint itself.
        """
        target = self.world(waypoint)
        distance = math.hypot(target[0] - own_xy[0], target[1] - own_xy[1])
        if distance < 1e-6:
            return 0.0, 0.0
        aim = target
        if previous is not None and distance > LOOKAHEAD_M:
            start = self.world(previous)
            leg = (target[0] - start[0], target[1] - start[1])
            length = math.hypot(*leg)
            if length > LOOKAHEAD_M:
                u = (leg[0] / length, leg[1] / length)
                along = (own_xy[0] - start[0]) * u[0] + (own_xy[1] - start[1]) * u[1]
                along = min(max(along, 0.0) + LOOKAHEAD_M, length)
                aim = (start[0] + u[0] * along, start[1] + u[1] * along)
        dx, dy = aim[0] - own_xy[0], aim[1] - own_xy[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return 0.0, 0.0
        speed = min(self.max_speed, APPROACH_GAIN * distance)
        return dx / norm * speed, dy / norm * speed

    def bias(self, neighbors, own_v, preferred, now_s):
        if not neighbors:
            self.stuck_since = self.roundabout_since = None
            return RIGHT_BIAS_RAD
        if self.roundabout_since is not None:
            if 0 <= now_s - self.roundabout_since <= ROUNDABOUT_S:
                return ROUNDABOUT_BIAS_RAD
            self.roundabout_since = None
        if math.hypot(*own_v) < STUCK_SPEED_M_S and math.hypot(*preferred) > 2 * STUCK_SPEED_M_S:
            if self.stuck_since is None or now_s < self.stuck_since:
                self.stuck_since = now_s
            if now_s - self.stuck_since >= STUCK_S:
                self.stuck_since = None
                self.roundabout_since = now_s
                return ROUNDABOUT_BIAS_RAD
        else:
            self.stuck_since = None
        return RIGHT_BIAS_RAD

    def update(self, position, now_s, waypoint, previous=None):
        """Return the setpoint for this tick. Called only with fresh own position.

        'vel' (local north, east) replaces the waypoint's XY; 'pos' carries the
        waypoint's altitude. keep_progress says the waypoint can still be reached
        and held while this target is flown. previous is the waypoint before this
        one, whose leg the drone follows; None aims straight at the waypoint.
        """
        if not position.v_xy_valid or not all(math.isfinite(v) for v in (position.vx, position.vy)):
            # A wrong own velocity gives wrong half-planes: hold instead.
            if self.state != 'DATA_HOLD' or self.target is None:
                self.target = self.hold(position)
            self.state = 'DATA_HOLD'
            return self.target

        own_xy = self.global_xy(position)
        own_v = self.global_v(position)
        neighbors = self.neighbors(own_xy, now_s)
        preferred = self.preferred_velocity(own_xy, waypoint, previous)
        bias = self.bias(neighbors, own_v, preferred, now_s)
        chosen = preferred
        if neighbors:
            def solve(v):
                return orca.solve(own_xy, own_v, RADIUS_M, neighbors, v, self.max_speed,
                                  TIME_HORIZON_S, TIME_STEP_S)

            def deviates(v, wanted):
                return math.hypot(v[0] - wanted[0], v[1] - wanted[1]) > DEVIATION_M_S

            chosen = solve(preferred)
            if deviates(chosen, preferred) or self.roundabout_since is not None:
                # In someone's way: pass them on the right.
                c, s = math.cos(bias), math.sin(bias)
                biased = (preferred[0] * c + preferred[1] * s, -preferred[0] * s + preferred[1] * c)
                chosen = solve(biased)
                self.last_deviation_s = now_s

        if self.roundabout_since is not None:
            self.last_deviation_s = now_s
        avoiding = (self.last_deviation_s is not None
                    and 0 <= now_s - self.last_deviation_s <= AVOIDING_LATCH_S)
        self.state = 'AVOIDING' if avoiding else 'CLEAR'
        ve, vn = chosen
        self.target = {'pos': (math.nan, math.nan, waypoint['pos'][2]),
                       'vel': (vn, ve), 'yaw': waypoint['yaw'], 'hold_s': 0.0,
                       'keep_progress': True}
        return self.target
