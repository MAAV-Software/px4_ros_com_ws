"""Score a Milestone 1 test flight from its PX4 flight log (.ulg).

Usage:
  ros2 run milestone1_singlevehicle m1_analyze <flight.ulg> --stage sitl
  ros2 run milestone1_singlevehicle m1_analyze <flight.ulg> --stage hitl --path my_path.yaml

Prints a PASS/FAIL report for each test-matrix row and writes it as Markdown
and JSON to --out (default ~/px4_ros_com_ws/log/m1_reports/). The path file
must be the one the flight used: its waypoints, test IDs and thresholds decide
what is scored.

Exit code: 0 if the flight passed, 1 if it failed or was incomplete, 2 if the
log or path file couldn't be read.

Needs pyulog (pip install pyulog). SITL logs are in
~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/<date>/.
"""
import argparse
import datetime
import json
import os
import sys

import numpy as np

from milestone1_singlevehicle import scoring

DEFAULT_OUT_DIR = '~/px4_ros_com_ws/log/m1_reports'
TOPICS = ['trajectory_setpoint', 'vehicle_local_position', 'vehicle_attitude', 'vehicle_status']


def _topic(ulog, name):
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == 0:
            data = dataset.data
            # Drop samples with no timestamp (PX4 can log one before the topic is published).
            valid = data['timestamp'] > 0
            times = (data['timestamp'][valid].astype(np.int64) - int(ulog.start_timestamp)) / 1e6
            return times, {key: values[valid] for key, values in data.items()}
    raise ValueError(f'topic "{name}" is not in the log')


def read_flight(path):
    """Load the topics the scorer needs from a .ulg file. Returns (FlightData, meta)."""
    from pyulog import ULog  # imported here so the scorer itself doesn't need pyulog

    ulog = ULog(path, TOPICS)
    sp_t, sp = _topic(ulog, 'trajectory_setpoint')
    pos_t, lp = _topic(ulog, 'vehicle_local_position')
    att_t, att = _topic(ulog, 'vehicle_attitude')
    status_t, status = _topic(ulog, 'vehicle_status')

    flight = scoring.FlightData(
        pos_t=pos_t,
        pos=np.column_stack([lp['x'], lp['y'], lp['z']]),
        heading=lp['heading'],
        att_t=att_t,
        tilt_deg=scoring.tilt_from_quaternion(np.column_stack([att[f'q[{i}]'] for i in range(4)])),
        sp_t=sp_t,
        sp_pos=np.column_stack([sp[f'position[{i}]'] for i in range(3)]),
        sp_yaw=sp['yaw'],
        status_t=status_t,
        arming_state=status['arming_state'],
        nav_state=status['nav_state'],
        failsafe=status['failsafe'].astype(bool),
    )

    info = ulog.msg_info_dict
    meta = {
        'log': os.path.basename(path),
        # File date: for logs downloaded from a drone this is the download date, so check it.
        'date': datetime.date.fromtimestamp(os.path.getmtime(path)).isoformat(),
        'px4': f'{info.get("ver_hw", "?")} {str(info.get("ver_sw", ""))[:8]}'.strip(),
    }
    return flight, meta


def _default_path_file():
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        share = get_package_share_directory('milestone1_singlevehicle')
    except PackageNotFoundError:
        raise ValueError('milestone1_singlevehicle is not installed: source the workspace or pass --path')
    return os.path.join(share, 'resource', 'milestone1_path.yaml')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Score a Milestone 1 test flight from its .ulg log.')
    parser.add_argument('ulog', help='PX4 flight log (.ulg)')
    parser.add_argument('--stage', choices=scoring.STAGES, default='sitl',
                        help='which thresholds to use (default: sitl)')
    parser.add_argument('--path',
                        help='path YAML the flight used (default: the installed milestone1_path.yaml)')
    parser.add_argument('--out', default=DEFAULT_OUT_DIR, help=f'report folder (default: {DEFAULT_OUT_DIR})')
    args = parser.parse_args(argv)

    try:
        path_file = args.path or _default_path_file()
        config = scoring.load_config(path_file)
        flight, meta = read_flight(args.ulog)
    except ImportError as e:
        hint = ': pip install pyulog' if e.name == 'pyulog' else ''
        print(f'error: Python module "{e.name}" is not installed{hint}', file=sys.stderr)
        return 2
    except (OSError, ValueError, KeyError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2

    meta['path'] = os.path.basename(path_file)
    report = scoring.score_flight(flight, config, args.stage)
    markdown = scoring.format_markdown(report, meta)
    print(markdown)

    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)
    name = f'{os.path.splitext(meta["log"])[0]}_{args.stage}'
    with open(os.path.join(out_dir, name + '.md'), 'w') as f:
        f.write(markdown)
    with open(os.path.join(out_dir, name + '.json'), 'w') as f:
        json.dump(scoring.report_to_dict(report, meta), f, indent=2)
    print(f'Report written to {os.path.join(out_dir, name)}.md and .json')

    return 0 if report.overall == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
