"""Small, ROS-independent priority/yield state machine; metres and local NED.

Shared XY is East/North, aligned with the default Gazebo world. No Z, velocity,
mission state or clock is transmitted. Peer age is measured at the receiver.
"""
import math

POSITION_TOPIC = '/milestone3/global_positions'
COLLISION_RADIUS_M = 5.0
CLIMB_M = 5.0
PEER_TIMEOUT_S = 1.0
STOP_SPEED_M_S = 0.2
STOP_HOLD_S = 0.5
ARRIVAL_M = 0.5


class Avoidance:
    def __init__(self, instance, origin_x, origin_y):
        if instance not in (0, 1):
            raise ValueError('Milestone 3 supports vehicle_instance 0 or 1')
        if not all(math.isfinite(v) for v in (origin_x, origin_y)):
            raise ValueError('Initial global XY must be finite')
        self.instance = instance
        # Immutable origin: never recomputed from the moving aircraft.
        self.origin = (origin_x, origin_y)
        self.peers = {}
        self.state = 'NORMAL'
        self.target = None
        self.stopped_since = None
        self.climb_z = None

    def global_xy(self, position):
        # PX4 local x=N, y=E; Gazebo/default shared world X=E, Y=N.
        return self.origin[0] + position.y, self.origin[1] + position.x

    def receive(self, data, now_s):
        """Wire payload is exactly [instance, global_x, global_y]."""
        if len(data) != 3 or not all(math.isfinite(v) for v in data):
            return
        instance, x, y = data
        if instance not in (0, 1) or instance == self.instance:
            return
        self.peers[int(instance)] = (x, y, now_s)

    def peers_ready(self, now_s):
        # Only higher-priority peers can interrupt this vehicle's mission.
        return all(i in self.peers and 0 <= now_s - self.peers[i][2] <= PEER_TIMEOUT_S
                   for i in range(self.instance + 1, 2))

    def conflicts(self, position):
        x, y = self.global_xy(position)
        # Keep a last-known conflict even when its messages stop arriving.
        return any(i > self.instance and math.hypot(px - x, py - y) <= COLLISION_RADIUS_M
                   for i, (px, py, _) in self.peers.items())

    @staticmethod
    def hold(position):
        return {'pos': (position.x, position.y, position.z),
                'yaw': position.heading, 'hold_s': 0.0}

    def update(self, position, now_s):
        """Return an overriding target, or None to fly the original waypoint.

        Called only with fresh, valid own position. Never advance the waypoint
        or its hold timer while an override is active.
        """
        conflict = self.conflicts(position)
        ready = self.peers_ready(now_s)
        if self.state in ('NORMAL', 'DATA_HOLD') and conflict:
            self.target = self.hold(position)
            self.climb_z = position.z - CLIMB_M  # NED: up means more negative
            self.stopped_since = None
            self.state = 'BRAKE'

        if self.state == 'BRAKE':
            speed = math.hypot(position.vx, position.vy)
            if position.v_xy_valid and math.isfinite(speed) and speed <= STOP_SPEED_M_S:
                if self.stopped_since is None or now_s < self.stopped_since:
                    self.stopped_since = now_s
                if now_s - self.stopped_since >= STOP_HOLD_S:
                    x, y, _ = self.target['pos']
                    self.target = {**self.target, 'pos': (x, y, self.climb_z)}
                    self.state = 'CLIMB'
            else:
                self.stopped_since = None
        elif self.state == 'CLIMB':
            if math.dist((position.x, position.y, position.z), self.target['pos']) <= ARRIVAL_M:
                self.state = 'WAIT'
        elif self.state == 'WAIT':
            if ready and not conflict:
                self.state, self.target = 'NORMAL', None

        if self.state in ('NORMAL', 'DATA_HOLD'):
            if not ready:
                if self.target is None:
                    self.target = self.hold(position)
                self.state = 'DATA_HOLD'
            else:
                self.state, self.target = 'NORMAL', None
        return self.target
