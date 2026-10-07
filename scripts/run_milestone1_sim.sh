#!/usr/bin/env bash
# Start one PX4 SITL vehicle and fly the 22-waypoint mission (yaw in radians).
# Each child gets its own process group so Ctrl+C can clean up this run.
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PATH_FILE="$WORKSPACE/src/week4_multivehicle/resource/milestone1_path_t1_t7.yaml"
SKIP_BUILD=false
HEADLESS_MODE=false

usage() {
    cat <<'HELP'
Usage: ./scripts/run_milestone1_sim.sh [options]

Build, start the DDS agent and PX4/Gazebo, then automatically arm and fly
one simulated drone through the waypoint YAML. Press Ctrl+C to stop this run.

  --headless        Run Gazebo without its GUI
  --skip-build      Use packages already built in this workspace
  --path FILE       Use a different waypoint YAML (yaw must be radians)
  --px4-dir DIR     PX4 source directory (default: ~/PX4-Autopilot)
  -h, --help        Show this help

Logs are saved under log/sim_runs/ in the workspace.
HELP
}

while (($#)); do
    case "$1" in
        --headless) HEADLESS_MODE=true; shift ;;
        --skip-build) SKIP_BUILD=true; shift ;;
        --path|--px4-dir)
            if (($# < 2)); then echo "Missing value for $1" >&2; exit 2; fi
            if [[ "$1" == --path ]]; then PATH_FILE="$2"; else PX4_DIR="$2"; fi
            shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

# Resolve user-supplied paths before changing the working directory.
PATH_FILE="$(realpath -e -- "$PATH_FILE")"
PX4_DIR="$(realpath -e -- "$PX4_DIR")"
[[ -f "$PATH_FILE" && -f "$PX4_DIR/Makefile" ]] || {
    echo 'Check the waypoint file and PX4 source directory.' >&2; exit 1;
}
[[ -f /opt/ros/humble/setup.bash ]] || {
    echo 'ROS 2 Humble was not found at /opt/ros/humble.' >&2; exit 1;
}
source /opt/ros/humble/setup.bash
for command in colcon MicroXRCEAgent make setsid timeout python3; do
    command -v "$command" >/dev/null || {
        echo "Required command not found: $command" >&2; exit 1;
    }
done

# Do not attach to, or terminate, another simulation already running.
if pgrep -x px4 >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
    echo 'PX4 or MicroXRCEAgent is already running. Stop that session first.' >&2
    exit 1
fi

cd "$WORKSPACE"
mkdir -p "$WORKSPACE/log/sim_runs"
RUN_LOG_DIR="$(mktemp -d "$WORKSPACE/log/sim_runs/run-XXXXXXXX")"
export ROS_LOG_DIR="$RUN_LOG_DIR/ros"
mkdir -p "$ROS_LOG_DIR"
echo "Logs: $RUN_LOG_DIR"

# Fail before starting the simulator if the mission file is malformed.
python3 - "$PATH_FILE" <<'PY'
import math
import sys
import yaml
with open(sys.argv[1]) as stream:
    data = yaml.safe_load(stream)
points = data['waypoints']
if not points:
    raise ValueError('The waypoint list is empty')
for index, point in enumerate(points):
    pos = [float(v) for v in point['pos']]
    yaw = float(point['yaw'])
    hold = float(point['hold_s'])
    if len(pos) != 3 or not all(math.isfinite(v) for v in pos + [yaw, hold]) or hold < 0:
        raise ValueError(f'Invalid waypoint {index}')
print(f'Loaded {len(points)} waypoints; yaw is in radians.')
PY

if ! $SKIP_BUILD; then
    echo 'Building ROS packages (timeout: 10 minutes)...'
    if ! timeout --foreground 600 colcon build --symlink-install \
        --packages-up-to week4_multivehicle --executor sequential \
        2>&1 | tee "$RUN_LOG_DIR/build.log"; then
        echo "Build failed or timed out. See $RUN_LOG_DIR/build.log" >&2
        exit 1
    fi
fi
[[ -f install/setup.bash ]] || {
    echo 'No workspace installation found. Run without --skip-build.' >&2; exit 1;
}
source install/setup.bash
ros2 pkg prefix week4_multivehicle >/dev/null

CHILD_GROUPS=()
cleanup() {
    trap - EXIT INT TERM
    echo 'Stopping controller, simulator, and DDS agent...'
    for pid in "${CHILD_GROUPS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    sleep 2
    for pid in "${CHILD_GROUPS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid MicroXRCEAgent udp4 -p 8888 >"$RUN_LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
CHILD_GROUPS+=("$AGENT_PID")

# PX4's SITL startup reads PX4_PARAM_* variables and applies these limits.
# This starts instance 0, matching the controller's vehicle_instance:=0.
SIM_ENV=(PX4_SYS_AUTOSTART=4001 PX4_PARAM_MPC_XY_VEL_MAX=2
         PX4_PARAM_MPC_Z_VEL_MAX_UP=1 PX4_PARAM_MPC_Z_VEL_MAX_DN=1
         PX4_PARAM_MPC_YAWRAUTO_MAX=45)
if $HEADLESS_MODE; then SIM_ENV+=(HEADLESS=1); fi
setsid env "${SIM_ENV[@]}" make -C "$PX4_DIR" px4_sitl gz_x500 \
    >"$RUN_LOG_DIR/px4.log" 2>&1 &
SIM_PID=$!
CHILD_GROUPS+=("$SIM_PID")

echo 'Waiting for PX4 position and status messages (up to 120 seconds each)...'
# An explicit type lets echo subscribe before discovery finds a publisher.
# Without it, echo exits immediately during startup instead of waiting.
for topic in vehicle_local_position_v1 vehicle_status_v1; do
    case "$topic" in
        vehicle_local_position_v1) message_type=px4_msgs/msg/VehicleLocalPosition ;;
        vehicle_status_v1) message_type=px4_msgs/msg/VehicleStatus ;;
    esac
    if ! timeout --foreground 120 ros2 topic echo "/fmu/out/$topic" "$message_type" --once \
        --qos-reliability best_effort >"$RUN_LOG_DIR/$topic.log" 2>&1; then
        echo "No messages on /fmu/out/$topic. Check the logs in $RUN_LOG_DIR" >&2
        exit 1
    fi
done
kill -0 "$AGENT_PID" "$SIM_PID" 2>/dev/null || {
    echo "Agent or simulator exited. Check $RUN_LOG_DIR" >&2; exit 1;
}

echo 'Starting mission: the simulated drone will arm, fly the path, and land.'
echo 'Press Ctrl+C to stop all processes started by this script.'
setsid ros2 run week4_multivehicle offboard_solution --ros-args \
    -p vehicle_instance:=0 -p "waypoints_file:=$PATH_FILE" \
    >"$RUN_LOG_DIR/controller.log" 2>&1 &
CONTROLLER_PID=$!
CHILD_GROUPS+=("$CONTROLLER_PID")
# Follow the controller output while retaining a log of the mission.
tail --pid="$CONTROLLER_PID" -n +1 -f "$RUN_LOG_DIR/controller.log" &
TAIL_PID=$!
wait "$CONTROLLER_PID"
wait "$TAIL_PID" || true
