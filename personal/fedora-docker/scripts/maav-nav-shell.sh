#!/usr/bin/env bash
set -euo pipefail
KIT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$KIT/env.sh"
"$KIT/scripts/maav-nav-up.sh" >/dev/null

exec docker exec -it -u "${MAAV_UID}:${MAAV_GID}" \
  -e HOME="$MAAV_HOME" \
  -e DISPLAY="$MAAV_DISPLAY" \
  -w "$MAAV_WS" \
  "$MAAV_CONTAINER" bash -lc \
  "source /opt/ros/humble/setup.bash; \
   [ -f \"$MAAV_WS/install/setup.bash\" ] && source \"$MAAV_WS/install/setup.bash\"; \
   exec bash"
