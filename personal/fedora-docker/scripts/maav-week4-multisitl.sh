#!/usr/bin/env bash
# Usage: maav-week4-multisitl.sh [--gui]
# One Gazebo world; instance 1 joins with PX4_GZ_STANDALONE=1.
set -euo pipefail
KIT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$KIT/env.sh"
GUI=0
[[ "${1:-}" == "--gui" ]] && GUI=1

"$KIT/scripts/maav-nav-up.sh" >/dev/null
xhost +local:docker >/dev/null 2>&1 || true
"$KIT/scripts/maav-week2-stop.sh" >/dev/null

docker exec -d -u "${MAAV_UID}:${MAAV_GID}" "$MAAV_CONTAINER" bash -lc \
  'MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1'

docker exec -d -u "${MAAV_UID}:${MAAV_GID}" -w "$MAAV_PX4" "$MAAV_CONTAINER" bash -lc '
  mkdir -p build/px4_sitl_default/instance_0
  cd build/px4_sitl_default/instance_0
  export PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 HEADLESS=1
  ../bin/px4 -i 0 > /tmp/px4_i0.log 2>&1
'
echo "Waiting for first vehicle / Gazebo..."
sleep 12

docker exec -d -u "${MAAV_UID}:${MAAV_GID}" -w "$MAAV_PX4" "$MAAV_CONTAINER" bash -lc '
  mkdir -p build/px4_sitl_default/instance_1
  cd build/px4_sitl_default/instance_1
  export PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500
  export PX4_GZ_MODEL_POSE="0,2" HEADLESS=1
  ../bin/px4 -i 1 > /tmp/px4_i1.log 2>&1
'
sleep 8

if [[ "$GUI" == "1" ]]; then
  docker exec -d -u "${MAAV_UID}:${MAAV_GID}" \
    -e DISPLAY="$MAAV_DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    "$MAAV_CONTAINER" bash -lc 'gz sim -g > /tmp/gz_gui.log 2>&1'
  echo "Gazebo GUI started — expect x500_0 and x500_1."
fi

echo "2-instance SITL up. Topics: /fmu/... and /uav_1/fmu/..."
echo "Stop: $KIT/scripts/maav-week2-stop.sh"
