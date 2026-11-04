"""Run one PX4 instance and clean up only its own process group on shutdown.

PX4 can start Gazebo as a child. A separate session lets us stop that child
as well, without the milestone 1 launch's machine-wide pkill commands.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', required=True)
    parser.add_argument('--instance', required=True, type=int, choices=(0, 1))
    args, _ = parser.parse_known_args()  # launch_ros appends ROS arguments
    px4_dir = Path(args.px4_dir).expanduser().resolve()
    binary = px4_dir / 'build/px4_sitl_default/bin/px4'
    # -w makes logs and saved parameters unambiguously separate per instance.
    run_dir = px4_dir / 'build/px4_sitl_default/rootfs' / str(args.instance)
    run_dir.mkdir(parents=True, exist_ok=True)
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    child = subprocess.Popen(
        [str(binary), '-i', str(args.instance), '-d', '-w', str(run_dir)],
        cwd=px4_dir, start_new_session=True)
    try:
        while not stopped:
            try:
                return child.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                pass
        return 0
    finally:
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        # The parent may have exited while Gazebo still belongs to its group.
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()


if __name__ == '__main__':
    raise SystemExit(main())
