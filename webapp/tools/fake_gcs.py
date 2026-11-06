#!/usr/bin/env python3
"""Send PX4 SITL a ground-station heartbeat, for machines with no QGroundControl.

With NAV_DLL_ACT set, PX4 refuses to arm with no ground station. This avoids changing
that saved parameter. It only says a ground station exists, and offers no way to stop
a drone, so it is for SITL only.

  python3 tools/fake_gcs.py
  # Also send centred sticks, so PX4 sees a pilot and allows manual modes. This is how to
  # test a takeover in SITL: fly, then run px4-commander mode posctl.
  python3 tools/fake_gcs.py --sticks
"""
import argparse
import socket
import struct
import time

PX4_GCS_PORT = 18570
HEARTBEAT_ID, HEARTBEAT_CRC_EXTRA = 0, 50
MANUAL_CONTROL_ID, MANUAL_CONTROL_CRC_EXTRA = 69, 243
MANUAL_CONTROL_PER_HEARTBEAT = 10


def frame(sequence, message_id, crc_extra, payload):
    """A MAVLink v1 frame from system 255, component 190: the usual ground-station IDs."""
    body = struct.pack('<BBBBB', len(payload), sequence & 0xFF, 255, 190, message_id) + payload
    value = 0xFFFF
    for byte in body + bytes([crc_extra]):
        tmp = byte ^ (value & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        value = ((value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return b'\xfe' + body + struct.pack('<H', value)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--sticks', action='store_true', help='also send centred RC sticks')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = ('127.0.0.1', PX4_GCS_PORT)
    # custom_mode, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode, MAV_STATE_ACTIVE, version
    heartbeat = struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3)
    # x, y, z (throttle mid-stick), r, buttons, target system
    sticks = struct.pack('<hhhhHB', 0, 0, 500, 0, 0, 1)
    sequence = 0
    while True:
        sock.sendto(frame(sequence, HEARTBEAT_ID, HEARTBEAT_CRC_EXTRA, heartbeat), address)
        sequence += 1
        for _ in range(MANUAL_CONTROL_PER_HEARTBEAT):
            if args.sticks:
                sock.sendto(frame(sequence, MANUAL_CONTROL_ID, MANUAL_CONTROL_CRC_EXTRA, sticks), address)
                sequence += 1
            time.sleep(1 / MANUAL_CONTROL_PER_HEARTBEAT)


if __name__ == '__main__':
    main()
