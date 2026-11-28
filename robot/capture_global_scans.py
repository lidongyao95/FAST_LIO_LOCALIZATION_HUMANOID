#!/usr/bin/env python3
"""Capture disjoint fresh camera_init scan batches; never publish pose or control."""
import argparse
import json
from pathlib import Path
import sys
import time

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'shared/map'))
from record_dufomap_scans import cloud_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('g1_global_scan_capture')
    clouds = message_filters.Subscriber(node, PointCloud2, '/cloud_registered_1')
    odometry = message_filters.Subscriber(node, Odometry, '/Odometry_loc')
    sync = message_filters.TimeSynchronizer([clouds, odometry], 30)
    batches, points, stamps = [], [], []
    last_stamp, next_stamp = -1, -1
    first_pose = None

    def receive(cloud, odom):
        nonlocal last_stamp, next_stamp, first_pose
        if len(batches) == 3:
            return
        if cloud.header.frame_id != 'camera_init' or odom.header.frame_id != 'camera_init' or odom.child_frame_id != 'body':
            raise ValueError('需要 camera_init 点云与 camera_init→body 里程计')
        stamp = cloud.header.stamp.sec * 10**9 + cloud.header.stamp.nanosec
        if stamp <= last_stamp:
            return
        if abs(node.get_clock().now().nanoseconds - stamp) > 2e9:
            raise ValueError('实时点云过期或主机/雷达时钟不同步')
        last_stamp = stamp
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(np.r_[position, quaternion]).all() or np.linalg.norm(quaternion) < .9:
            raise ValueError('里程计位姿无效')
        quaternion /= np.linalg.norm(quaternion)
        if first_pose is None:
            first_pose = position, quaternion
        if (np.linalg.norm(position - first_pose[0]) > .12 or
                abs(np.dot(quaternion, first_pose[1])) < np.cos(np.deg2rad(2))):
            raise ValueError('自动重定位采集期间请保持机器人静止')
        if stamp < next_stamp:
            return
        xyz = cloud_xyz(cloud)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[np.linalg.norm(xyz - position, axis=1) < 30]
        if len(xyz) < 100:
            return
        points.append(xyz)
        stamps.append(stamp)
        if len(points) == 6:
            batch = np.vstack(points).astype('<f4')
            path = args.output / f'scan{len(batches)+1}.ply'
            with path.open('xb') as stream:
                stream.write(('ply\nformat binary_little_endian 1.0\nelement vertex '
                              f'{len(batch)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode())
                stream.write(batch.tobytes())
            batches.append({'file': path.name, 'stamps': stamps.copy(), 'points': len(batch)})
            print(f'已采集验证组 {len(batches)}/3：{len(batch)} 点', flush=True)
            points.clear()
            stamps.clear()
            next_stamp = stamp + 400_000_000

    sync.registerCallback(receive)
    try:
        deadline = time.monotonic() + args.timeout
        while len(batches) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(batches) < 3:
            raise RuntimeError('没有收到三组新鲜同步点云，自动重定位未开始')
        (args.output / 'capture.json').write_text(json.dumps({'frame': 'camera_init', 'batches': batches}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError) as error:
        sys.exit(str(error))
