# Slotting ORCA into the `Avoidance` interface

I'd keep the class and its method names (`receive`, `peers_ready`, `update`) so the controller wiring barely changes. The sketch is untested and meant to show the shape. Three things in the current interface have to change, covered after the code.

## Structure

```text
avoidance.py       # OrcaAvoidance: same public interface as Avoidance
orca.py            # pure functions, no ROS, no PX4 types (easy to unit-test)
  orca_line(...)
  linear_program_1d / 2d / 3d
  solve(agent, neighbors, v_pref) -> (vx, vy)
```

## `OrcaAvoidance` (drop-in shape)

```python
import math
from orca import solve, Agent   # pure-python port of RVO2

POSITION_TOPIC = '/milestone3/global_positions'
N_VEHICLES = 4
PEER_TIMEOUT_S = 0.5          # tighter than before; ORCA needs fresh velocity
NEIGHBOR_DIST_M = 10.0
RADIUS_M = 1.0                # per drone, inflated for tracking lag (tune in SITL)
TAU_S = 3.0                   # time horizon (tune)
V_MAX = 2.0                   # matches MPC_XY_VEL_MAX
LOOKAHEAD_S = 0.5             # how far ahead the position setpoint is placed
DEVIATION_EPS = 0.05          # m/s; below this, ORCA is not "active"


class OrcaAvoidance:
    def __init__(self, instance, origin_x, origin_y):
        if instance not in range(N_VEHICLES):
            raise ValueError(...)
        self.instance = instance
        self.origin = (origin_x, origin_y)
        self.peers = {}                # i -> (x, y, vx, vy, t_rx)
        self.waypoint = None           # set by controller each tick (NEW)

    # ---- frame helpers (same swap as before, now for velocity too) ----
    def global_xy(self, p):  return self.origin[0] + p.y,  self.origin[1] + p.x
    def global_v(self, p):   return p.vy, p.vx           # (East, North)

    # ---- wire: [instance, gx, gy, gvx, gvy] ----
    def receive(self, data, now_s):
        if len(data) != 5 or not all(math.isfinite(v) for v in data):
            return
        i, x, y, vx, vy = data
        i = int(i)
        if i not in range(N_VEHICLES) or i == self.instance:
            return
        self.peers[i] = (x, y, vx, vy, now_s)

    def peers_ready(self, now_s):
        # Pre-arm gate: every expected peer seen recently.
        # (Unlike before, this is symmetric: nobody has priority.)
        return all(i in self.peers and 0 <= now_s - self.peers[i][4] <= PEER_TIMEOUT_S
                   for i in range(N_VEHICLES) if i != self.instance)

    # ---- main entry ----
    def update(self, position, now_s, waypoint):
        """Return override target dict, or None to fly the waypoint directly."""
        self.waypoint = waypoint
        x, y = self.global_xy(position)
        vx, vy = self.global_v(position)

        # 1. preferred velocity toward waypoint XY (global frame)
        wx = self.origin[0] + waypoint['pos'][1]
        wy = self.origin[1] + waypoint['pos'][0]
        v_pref = self._preferred_velocity((x, y), (wx, wy))

        # 2. build neighbors; stale peers become cautious static obstacles
        neighbors = []
        for i, (px, py, pvx, pvy, t) in self.peers.items():
            if math.hypot(px - x, py - y) > NEIGHBOR_DIST_M:
                continue
            if now_s - t <= PEER_TIMEOUT_S:
                neighbors.append(Agent(px, py, pvx, pvy, RADIUS_M, responsibility=0.5))
            else:
                neighbors.append(Agent(px, py, 0.0, 0.0, RADIUS_M * 1.5,
                                       responsibility=1.0))   # we dodge fully

        me = Agent(x, y, vx, vy, RADIUS_M, v_max=V_MAX)
        v_new = solve(me, neighbors, v_pref, TAU_S, dt=0.1)

        # 3. no active constraint -> let the existing controller fly the waypoint
        if math.hypot(v_new[0] - v_pref[0], v_new[1] - v_pref[1]) < DEVIATION_EPS:
            return None

        # 4. convert velocity back to a local-NED position setpoint
        north = position.x + v_new[1] * LOOKAHEAD_S
        east  = position.y + v_new[0] * LOOKAHEAD_S
        return {'pos': (north, east, waypoint['pos'][2]),   # z held at waypoint z
                'yaw': waypoint['yaw'], 'hold_s': 0.0,
                'keep_progress': True}                      # NEW flag, see below

    def _preferred_velocity(self, p, target):
        dx, dy = target[0] - p[0], target[1] - p[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return (0.0, 0.0)
        speed = min(V_MAX, 0.8 * d)   # P-controller: slows into the waypoint and gives a gentle hold
        return (dx / d * speed, dy / d * speed)
```

## What changes versus the current file

| Current | ORCA version | Why |
|---|---|---|
| `update(position, now_s)` | `update(position, now_s, waypoint)` | ORCA needs the waypoint to compute `v_pref`. The old class never needed it because it returned `None` to mean "fly the waypoint". |
| Wire payload `[inst, x, y]` | `[inst, x, y, vx, vy]` | ORCA needs neighbor velocity. The publisher side of the controller must send the new fields, with velocity in the same East/North swap as position. |
| States NORMAL/BRAKE/CLIMB/WAIT/DATA_HOLD | None; the "state" is whether ORCA is constraining you | No stop-climb-wait sequence. |
| Priority via `i > self.instance` | Symmetric | Reciprocity replaces priority. |
| Stale peer: hold position | Stale peer: static obstacle, full responsibility | Holding still while others fly on defeats your no-stopping goal. If you want a hard fallback, add a `DATA_HOLD` only for the case where *every* peer is stale. |
| `instance in (0, 1)` | `range(N_VEHICLES)` | Four drones. |

## Changes needed in the controller (the real integration cost)

1. **Don't freeze the waypoint during avoidance.** Right now the controller resets the hold timer and freezes the index whenever `update` returns a target. With ORCA, the drone is still progressing toward its waypoint while sidestepping, so the controller should honor `keep_progress` and keep evaluating arrival against the *real* waypoint using actual position. Otherwise a drone that is slightly deflected never advances.
2. **Land blocking.** The current rule blocks landing while an override is active. Keep that, but note that ORCA overrides can flicker on and off, so latch it for a short time (for example, don't request land while `update` returned a target in the last second).
3. **Hold dwell.** A drone holding at a waypoint can be pushed out of the 0.5 m tolerance by a passing neighbor, which resets its continuous hold. Either accept that (the drone returns and restarts the hold) or loosen the tolerance for hold only.
4. **Publisher.** Add velocity to the published message, and keep the "no stale own XY rebroadcast" rule: if own position is invalid, publish nothing.

## Pitfalls specific to this adapter

- **Position setpoint as a velocity proxy.** Placing the target at `p + v_new * 0.5 s` makes PX4's position controller chase a moving carrot. It works, but the realized velocity won't equal `v_new` exactly. If tracking is poor, a velocity setpoint (with a fixed-z position hold) is cleaner, but it changes your offboard control mode.
- **Radius is no longer 5 m.** The old 5 m was a trigger distance for a climb. Here `RADIUS_M` is each drone's physical size plus tracking error and latency margin, so sum of radii ≈ 2 m. Use `NEIGHBOR_DIST_M` and `TAU_S` to control how early drones react.
- **Velocity validity.** Gate on `position.v_xy_valid` as the old code did. If invalid, skip ORCA and fall back to a hold, since a wrong `v_self` produces wrong half-planes.
- **Symmetry deadlocks.** Add a tiny perturbation to `v_pref` inside `_preferred_velocity` for exact head-on cases.

## Next steps

- Write out `orca.py`: the pure-Python port of `compute_orca_line`, `linear_program_1d/2d/3d`, plus a small four-agent crossing test.
- Sketch the controller-side changes in more detail.
