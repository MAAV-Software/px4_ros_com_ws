#!/usr/bin/env bash
# One-shot setup on a Fedora machine (desktop or laptop).
# Builds the Docker image, clones/builds PX4 if needed, colcon-builds the workspace.
set -euo pipefail
KIT="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=/dev/null
source "$KIT/env.sh"

echo "==> Checking Docker"
if ! docker info >/dev/null 2>&1; then
  echo "Start Docker first (e.g. sudo systemctl start docker), then re-run." >&2
  exit 1
fi

echo "==> Building image $MAAV_IMAGE (first time can take a while)"
docker build -t "$MAAV_IMAGE" "$KIT"

echo "==> Ensuring container is up"
"$KIT/scripts/maav-nav-up.sh"

if [[ ! -d "$MAAV_PX4/.git" ]]; then
  echo "==> Cloning PX4-Autopilot into $MAAV_PX4"
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$MAAV_PX4"
fi

# Optional documented pitfall fix for sitl_multiple_run.sh (week 4 docs)
MULTI="$MAAV_PX4/Tools/simulation/sitl_multiple_run.sh"
if [[ -f "$MULTI" ]] && grep -q 'px4 -i \$n -d "\$build_path/etc"' "$MULTI"; then
  echo "==> Patching sitl_multiple_run.sh (remove -d etc)"
  sed -i 's|\$build_path/bin/px4 -i \$n -d "\$build_path/etc" >out.log 2>err.log \&|\$build_path/bin/px4 -i \$n >out.log 2>err.log \&|' "$MULTI"
fi

echo "==> Building PX4 SITL inside container (long first time)"
docker exec -u "${MAAV_UID}:${MAAV_GID}" -w "$MAAV_PX4" "$MAAV_CONTAINER" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  make px4_sitl -j"$(nproc)"
'

echo "==> Building ROS workspace packages"
docker exec -u "${MAAV_UID}:${MAAV_GID}" -w "$MAAV_WS" "$MAAV_CONTAINER" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
'

cat <<EOF

Bootstrap done.

Daily use:
  $KIT/scripts/maav-nav-shell.sh
  $KIT/scripts/maav-week2-sitl.sh --gui
  $KIT/scripts/maav-week4-multisitl.sh --gui
  $KIT/scripts/maav-week2-stop.sh

QGroundControl (host Flatpak, optional):
  flatpak install -y flathub org.mavlink.qgroundcontrol
  flatpak run org.mavlink.qgroundcontrol

Workspace expected at: $MAAV_WS
PX4 expected at:       $MAAV_PX4
EOF
