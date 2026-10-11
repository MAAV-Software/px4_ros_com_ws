#!/usr/bin/env bash
set -euo pipefail
KIT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$KIT/env.sh"
docker exec -u 0 "$MAAV_CONTAINER" bash -lc \
  'pkill -9 -f "[b]in/px4" || true; pkill -9 -f "[g]z sim" || true; pkill -9 -f "[M]icroXRCEAgent" || true; pkill -9 -x px4 || true' \
  || true
echo "Stopped PX4 / Gazebo / MicroXRCEAgent."
