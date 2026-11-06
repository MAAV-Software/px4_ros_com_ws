"""Pre-arm checks shared by the offboard node and the readiness wait.

Pure Python (no ROS imports) so it can be unit tested without a ROS install.
`position` is anything with the VehicleLocalPosition fields x, y, xy_valid and
z_valid.
"""
import math

DEFAULT_MAX_START_OFFSET_M = 2.0


def position_problem(position):
    """Why `position` can't be flown on yet, or '' when it is usable."""
    if position is None:
        return 'no vehicle_local_position yet (is PX4 up and the XRCE agent connected?)'
    if not (position.xy_valid and position.z_valid):
        return 'no valid local position estimate yet'
    if not all(math.isfinite(v) for v in (position.x, position.y)):
        return 'local position estimate is not finite'
    return ''


def arm_blocker(position, first_waypoint_pos, max_start_offset_m=DEFAULT_MAX_START_OFFSET_M):
    """Why the node must not arm yet, or '' when it is safe to.

    Waypoints are absolute local NED metres from the EKF origin (where the
    estimator started, normally where the drone was powered on). If the drone
    isn't close to the first waypoint's x/y, the first leg would be a long,
    unplanned flight, so refuse to arm. A negative max_start_offset_m disables
    the distance check (the validity check always applies).
    """
    problem = position_problem(position)
    if problem:
        return problem
    if max_start_offset_m < 0:
        return ''
    offset = math.hypot(position.x - first_waypoint_pos[0], position.y - first_waypoint_pos[1])
    if offset > max_start_offset_m:
        return (f'drone is {offset:.1f} m (x/y) from the first waypoint; limit is '
                f'{max_start_offset_m:.1f} m. Waypoints are relative to the EKF origin '
                '(where PX4 was powered on): move the drone there, or reboot PX4 where it stands')
    return ''


def target_position(waypoint_pos, relative, start_offset, current_position):
    """The local-NED position to command for `waypoint_pos`.

    Absolute mode returns the waypoint unchanged. In relative mode the waypoint
    is an offset from where the drone was when it armed (`start_offset`, an
    (x, y, z) tuple), so a "-5 m" climb is 5 m above the start no matter what
    offset the estimator has from its own origin. Until the start is captured
    the drone is told to hold its current position (so engaging offboard can't
    cause a jump), or the raw waypoint if there is no usable position yet.
    """
    if not relative:
        return tuple(waypoint_pos)
    if start_offset is not None:
        return tuple(o + w for o, w in zip(start_offset, waypoint_pos))
    if current_position is not None and all(
            math.isfinite(v) for v in (current_position.x, current_position.y, current_position.z)):
        return (current_position.x, current_position.y, current_position.z)
    return tuple(waypoint_pos)
