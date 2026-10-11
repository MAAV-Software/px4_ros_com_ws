# Personal Fedora Docker kit (Quinn)

**Private convenience for Fedora 44 machines. Not team infrastructure.**

- Do **not** open a PR of this into `main` / `dev-test`.
- Keep it on branch `qhague/fedora-docker-setup` (or another `qhague/*` branch).
- Safe for the group: only adds files under `personal/fedora-docker/`; does not change onboarding packages.

## Why this exists

ROS 2 Humble + PX4 SITL + Gazebo Harmonic are Ubuntu-oriented. On Fedora they run in Docker (`maav-nav-humble`), with your `$HOME` bind-mounted so `~/px4_ros_com_ws` and `~/PX4-Autopilot` stay on the host.

| Native Fedora | Inside `maav-nav` container |
|---|---|
| Docker, desktop/X11, QGC Flatpak | ROS 2 Humble, PX4 SITL, Gazebo, MicroXRCEAgent |

## Laptop / second machine

```bash
# 1) Get this branch (from a machine that already has the repo)
cd ~/px4_ros_com_ws
git fetch origin
git checkout qhague/fedora-docker-setup

# 2) One-time bootstrap (build image, PX4, colcon)
./personal/fedora-docker/bootstrap.sh
```

Requires: Docker running, user in `docker` group, network for image/PX4 clones.

Optional env overrides (see `env.sh`):

```bash
export MAAV_WS=$HOME/px4_ros_com_ws
export MAAV_PX4=$HOME/PX4-Autopilot
```

## Daily commands

```bash
KIT=~/px4_ros_com_ws/personal/fedora-docker

$KIT/scripts/maav-nav-shell.sh                 # ROS shell in container
$KIT/scripts/maav-week2-sitl.sh --gui          # 1 drone + Gazebo GUI
$KIT/scripts/maav-week4-multisitl.sh --gui     # 2 drones, one Gazebo world
$KIT/scripts/maav-week2-stop.sh                # stop sim processes
```

Use `ros2 run …` (not stock `gnome-terminal` launch files) inside the container.

## Syncing desktop ↔ laptop

Push this branch from the machine you update scripts on:

```bash
git checkout qhague/fedora-docker-setup
git add personal/fedora-docker
git commit -m "Update personal Fedora Docker kit"
git push -u origin qhague/fedora-docker-setup
```

On the other machine: `git fetch && git checkout qhague/fedora-docker-setup && git pull`.

Rebuild the image only when `Dockerfile` changes:

```bash
docker build -t maav-nav-humble:latest ~/px4_ros_com_ws/personal/fedora-docker
```

## What is NOT in git

- The Docker image layers (rebuild with `bootstrap.sh` / `docker build`)
- `~/PX4-Autopilot` (cloned by bootstrap)
- `build/` / `install/` / `log/` under the workspace
