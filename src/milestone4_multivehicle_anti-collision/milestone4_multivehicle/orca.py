"""Optimal Reciprocal Collision Avoidance (ORCA), 2D, agents only.

A pure-Python port of RVO2's Agent::computeNewVelocity and its three linear
programs (van den Berg, Guy, Lin and Manocha, "Reciprocal n-body collision
avoidance", 2011; RVO2 library, Apache 2.0). No ROS, no PX4 types: vectors are
(x, y) tuples in metres and metres per second, in any one fixed frame.

Each neighbour contributes a half-plane of velocities that keep the pair apart
for `time_horizon` seconds, assuming the neighbour takes `responsibility` of
the avoidance itself (0.5 when both run ORCA, 0.0 when it won't move, so we
take all of it). solve() returns the velocity closest to the preferred one that
satisfies every half-plane within max_speed, or, when they conflict, the one
that violates them least.
"""
import math
from collections import namedtuple

EPSILON = 1e-5

# A neighbour as seen by the solver. `responsibility` is this agent's share.
Neighbor = namedtuple('Neighbor', 'x y vx vy radius responsibility')
# A half-plane: velocities v with det(direction, v - point) <= 0 are allowed
# (the allowed side is to the left of direction).
Line = namedtuple('Line', 'point direction')


def _sub(a, b): return a[0] - b[0], a[1] - b[1]
def _add(a, b): return a[0] + b[0], a[1] + b[1]
def _mul(a, s): return a[0] * s, a[1] * s
def _dot(a, b): return a[0] * b[0] + a[1] * b[1]
def _det(a, b): return a[0] * b[1] - a[1] * b[0]
def _abs_sq(a): return a[0] * a[0] + a[1] * a[1]


def _normalize(a):
    length = math.sqrt(_abs_sq(a))
    return (a[0] / length, a[1] / length) if length > 0 else (0.0, 0.0)


def orca_line(position, velocity, radius, neighbor, time_horizon, time_step):
    """The half-plane of permitted velocities for one neighbour."""
    rel_pos = (neighbor.x - position[0], neighbor.y - position[1])
    rel_vel = (velocity[0] - neighbor.vx, velocity[1] - neighbor.vy)
    dist_sq = _abs_sq(rel_pos)
    combined = radius + neighbor.radius
    combined_sq = combined * combined

    if dist_sq > combined_sq:
        inv_tau = 1.0 / time_horizon
        # Vector from the cutoff circle's centre to the relative velocity.
        w = _sub(rel_vel, _mul(rel_pos, inv_tau))
        w_len_sq = _abs_sq(w)
        dot1 = _dot(w, rel_pos)
        if dot1 < 0 and dot1 * dot1 > combined_sq * w_len_sq:
            # Closest point is on the cutoff circle.
            w_len = math.sqrt(w_len_sq)
            unit_w = _mul(w, 1.0 / w_len)
            direction = (unit_w[1], -unit_w[0])
            u = _mul(unit_w, combined * inv_tau - w_len)
        else:
            # Closest point is on one of the cone's legs.
            leg = math.sqrt(dist_sq - combined_sq)
            px, py = rel_pos
            if _det(rel_pos, w) > 0:
                direction = ((px * leg - py * combined) / dist_sq,
                             (px * combined + py * leg) / dist_sq)
            else:
                direction = (-(px * leg + py * combined) / dist_sq,
                             -(-px * combined + py * leg) / dist_sq)
            dot2 = _dot(rel_vel, direction)
            u = _sub(_mul(direction, dot2), rel_vel)
    else:
        # Already overlapping: separate within one time step.
        inv_dt = 1.0 / time_step
        w = _sub(rel_vel, _mul(rel_pos, inv_dt))
        w_len = math.sqrt(_abs_sq(w))
        if w_len < EPSILON:
            # Exactly on top of each other with matched velocity: pick a side.
            w, w_len = (0.0, -1.0), 1.0
        unit_w = _mul(w, 1.0 / w_len)
        direction = (unit_w[1], -unit_w[0])
        u = _mul(unit_w, combined * inv_dt - w_len)

    return Line(_add(velocity, _mul(u, neighbor.responsibility)), direction)


def _linear_program1(lines, line_no, radius, opt, direction_opt):
    """Optimise along one line inside the speed circle; None if infeasible."""
    line = lines[line_no]
    dot = _dot(line.point, line.direction)
    discriminant = dot * dot + radius * radius - _abs_sq(line.point)
    if discriminant < 0:
        return None  # the speed circle misses this line entirely
    root = math.sqrt(discriminant)
    t_left, t_right = -dot - root, -dot + root

    for i in range(line_no):
        denominator = _det(line.direction, lines[i].direction)
        numerator = _det(lines[i].direction, _sub(line.point, lines[i].point))
        if abs(denominator) <= EPSILON:
            if numerator < 0:
                return None  # parallel and on the wrong side
            continue
        t = numerator / denominator
        if denominator >= 0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None

    if direction_opt:
        t = t_right if _dot(opt, line.direction) > 0 else t_left
    else:
        t = min(max(_dot(line.direction, _sub(opt, line.point)), t_left), t_right)
    return _add(line.point, _mul(line.direction, t))


def _linear_program2(lines, radius, opt, direction_opt):
    """Return (index of first line that failed or len(lines), velocity)."""
    if direction_opt:
        result = _mul(opt, radius)
    elif _abs_sq(opt) > radius * radius:
        result = _mul(_normalize(opt), radius)
    else:
        result = opt
    for i, line in enumerate(lines):
        if _det(line.direction, _sub(line.point, result)) > 0:
            candidate = _linear_program1(lines, i, radius, opt, direction_opt)
            if candidate is None:
                return i, result
            result = candidate
    return len(lines), result


def _linear_program3(lines, begin, radius, result):
    """Infeasible case: minimise the largest violation, from line `begin` on."""
    distance = 0.0
    for i in range(begin, len(lines)):
        if _det(lines[i].direction, _sub(lines[i].point, result)) <= distance:
            continue
        projected = []
        for j in range(i):
            determinant = _det(lines[i].direction, lines[j].direction)
            if abs(determinant) <= EPSILON:
                if _dot(lines[i].direction, lines[j].direction) > 0:
                    continue  # same direction: line j is redundant here
                point = _mul(_add(lines[i].point, lines[j].point), 0.5)
            else:
                t = _det(lines[j].direction, _sub(lines[i].point, lines[j].point)) / determinant
                point = _add(lines[i].point, _mul(lines[i].direction, t))
            projected.append(Line(point, _normalize(_sub(lines[j].direction, lines[i].direction))))
        failed, candidate = _linear_program2(
            projected, radius, (-lines[i].direction[1], lines[i].direction[0]), True)
        if failed >= len(projected):
            result = candidate
        distance = _det(lines[i].direction, _sub(lines[i].point, result))
    return result


def solve(position, velocity, radius, neighbors, preferred, max_speed,
          time_horizon, time_step):
    """The new velocity for one agent. All arguments in one fixed frame."""
    lines = [orca_line(position, velocity, radius, n, time_horizon, time_step)
             for n in neighbors]
    failed, result = _linear_program2(lines, max_speed, preferred, False)
    if failed < len(lines):
        result = _linear_program3(lines, failed, max_speed, result)
    return result
