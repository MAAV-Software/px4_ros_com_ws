#!/usr/bin/env bash
# Usage: maav-week2-sitl.sh [--gui]
set -euo pipefail
KIT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$KIT/env.sh"
HEADLESS=1
[[ "${1:-}" == "--gui" ]] && HEADLESS=0

"$KIT/scripts/maav-nav-up.sh" >/dev/null

docker exec -u 0 "$MAAV_CONTAINER" bash -lc \
  'pkill -9 -f "[b]in/px4" || true; pkill -9 -f "[g]z sim" || true; pkill -9 -f "[M]icroXRCEAgent" || true; sleep 1'

docker exec -d -u "${MAAV_UID}:${MAAV_GID}" "$MAAV_CONTAINER" bash -lc \
  'MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1'

docker exec -d -u "${MAAV_UID}:${MAAV_GID}" -w "$MAAV_PX4" "$MAAV_CONTAINER" bash -lc "
export HEADLESS=$HEADLESS
export PX4_SYS_AUTOSTART=4001
if [[ -x build/px4_sitl_default/bin/px4 ]]; then
  make px4_sitl gz_x500 > /tmp/px4_sitl.log 2>&1
else
  echo 'PX4 not built yet — run personal/fedora-docker/bootstrap.sh' >&2
  exit 1
fi
"

if [[ "$HEADLESS" == "0" ]]; then
  sleep 8
  docker exec -d -u "${MAAV_UID}:${MAAV_GID}" \
    -e DISPLAY="$MAAV_DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    "$MAAV_CONTAINER" bash -lc 'gz sim -g > /tmp/gz_gui.log 2>&1'
  echo "Started Gazebo GUI (gz sim -g)."
fi

echo "Started MicroXRCEAgent + PX4 SITL (HEADLESS=$HEADLESS)."
echo "Stop: $KIT/scripts/maav-week2-stop.sh"
