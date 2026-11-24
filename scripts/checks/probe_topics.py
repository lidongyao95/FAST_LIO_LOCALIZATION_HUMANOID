#!/usr/bin/env python3
"""Measure one Livox stream per process without deserializing point arrays."""
import argparse
import math
import struct
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stream', choices=('lidar', 'imu'))
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')

    topic = f'/livox/{args.stream}'
    lidar = args.stream == 'lidar'
    msg_type = CustomMsg if lidar else Imu
    first = last = previous_stamp = None
    count = nonmonotonic = 0
    max_gap = 0.0
    min_points = max_points = None

    def on_message(raw):
        nonlocal first, last, previous_stamp, count, nonmonotonic, max_gap, min_points, max_points
        now = time.monotonic()
        # ROS 2 CDR encapsulation, then Header (stamp, frame_id). Alignment
        # for Livox timebase is relative to the end of the 4-byte prefix.
        if raw[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('Expected ROS 2 CDR encoding')
        endian = '<' if raw[1] else '>'
        sec, nanos, frame_length = struct.unpack_from(endian + 'iII', raw, 4)
        stamp = sec * 10**9 + nanos
        if previous_stamp is not None:
            nonmonotonic += stamp <= previous_stamp
            max_gap = max(max_gap, now - last)
        else:
            first = now
        if lidar:
            timebase_offset = 4 + ((12 + frame_length + 7) // 8) * 8
            points, = struct.unpack_from(endian + 'I', raw, timebase_offset + 8)
            min_points = points if min_points is None else min(min_points, points)
            max_points = points if max_points is None else max(max_points, points)
        previous_stamp, last = stamp, now
        count += 1

    rclpy.init()
    node = rclpy.create_node(f'probe_livox_{args.stream}')
    try:
        node.create_subscription(msg_type, topic, on_message, qos_profile_sensor_data, raw=True)
        print(f'topic={topic} type={msg_type.__module__.split(".")[0]}/msg/{msg_type.__name__}', flush=True)
        start = time.monotonic()
        deadline = start + args.seconds
        next_report = start + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            now = time.monotonic()
            if now >= next_report:
                print(f't={now - start:.1f}s count={count}', flush=True)
                next_report += 1
        if count < 2:
            print(f'count={count} insufficient messages')
            return 1
        print(f'count={count} hz={(count - 1) / (last - first):.2f} '
              f'max_gap_s={max_gap:.3f} nonmonotonic={nonmonotonic}')
        if lidar:
            print(f'points_per_frame={min_points}..{max_points}')
        return int(nonmonotonic != 0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
