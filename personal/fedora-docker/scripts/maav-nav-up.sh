#!/usr/bin/env bash
# Start (or create) the personal maav-nav container.
set -euo pipefail
KIT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "$KIT/env.sh"

if ! docker info >/dev/null 2>&1; then
  cat >&2 <<'EOF'
Docker daemon isn't reachable.

  docker info
  # if needed: sudo systemctl start docker
  # then open a NEW terminal if you just joined the docker group
EOF
  exit 1
fi

xhost +local:docker >/dev/null 2>&1 || true

if ! docker image inspect "$MAAV_IMAGE" >/dev/null 2>&1; then
  echo "Image $MAAV_IMAGE missing. Build it first:"
  echo "  $KIT/bootstrap.sh"
  exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$MAAV_CONTAINER"; then
  docker start "$MAAV_CONTAINER" >/dev/null
else
  docker run -d --name "$MAAV_CONTAINER" \
    --network host \
    --ipc=host \
    --security-opt label=disable \
    -e DISPLAY="$MAAV_DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e HOME="$MAAV_HOME" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$MAAV_HOME:$MAAV_HOME:z" \
    -w "$MAAV_WS" \
    "$MAAV_IMAGE" \
    sleep infinity
fi

docker exec -u 0 "$MAAV_CONTAINER" bash -lc \
  'printf "nameserver 8.8.8.8\nnameserver 1.1.1.1\n" > /etc/resolv.conf' || true

echo "Container $MAAV_CONTAINER is up."
echo "  $KIT/scripts/maav-nav-shell.sh"
