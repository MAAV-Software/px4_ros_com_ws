# Sourced by the other scripts. Override any of these before calling them.
# shellcheck shell=bash

export MAAV_IMAGE="${MAAV_IMAGE:-maav-nav-humble:latest}"
export MAAV_FALLBACK_IMAGE="${MAAV_FALLBACK_IMAGE:-px4io/px4-dev-ros2:main-humble}"
export MAAV_CONTAINER="${MAAV_CONTAINER:-maav-nav}"
export MAAV_HOME="${MAAV_HOME:-$HOME}"
export MAAV_WS="${MAAV_WS:-$MAAV_HOME/px4_ros_com_ws}"
export MAAV_PX4="${MAAV_PX4:-$MAAV_HOME/PX4-Autopilot}"
export MAAV_UID="${MAAV_UID:-$(id -u)}"
export MAAV_GID="${MAAV_GID:-$(id -g)}"
export MAAV_DISPLAY="${DISPLAY:-:0}"

# Directory of this kit (personal/fedora-docker)
MAAV_KIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MAAV_KIT_DIR
