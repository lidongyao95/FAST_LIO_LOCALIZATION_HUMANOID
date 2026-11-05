#!/usr/bin/env python3
"""Capture exact-stamp FAST-LIO scan/odometry pairs for offline DUFOMap."""
import argparse
from pathlib import Path
import time

import message_filters
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy


def rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--lidar-in-body", type=float, nargs=3, required=True)
    parser.add_argument("--frames", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    offset = np.asarray(args.lidar_in_body)
    if args.frames < 2 or not np.isfinite(offset).all() or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("Require at least two frames, a finite offset, and positive timeout")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("record_dufomap_scans")
    last_stamp = -1
    count = 0

    def receive(cloud, odom):
        nonlocal last_stamp, count
        if cloud.header.frame_id != "body" or odom.child_frame_id != "body" or odom.header.frame_id != "camera_init":
            raise ValueError("Expected FAST-LIO camera_init → body and body-frame cloud")
        stamp = cloud.header.stamp.sec * 10**9 + cloud.header.stamp.nanosec
        if stamp <= last_stamp:
            raise ValueError("Paired timestamps must be strictly increasing")
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        translation = np.array([p.x, p.y, p.z])
        quaternion = np.array([q.w, q.x, q.y, q.z])
        if not np.isfinite(translation).all() or not np.isclose(np.linalg.norm(quaternion), 1.0):
            raise ValueError("Invalid FAST-LIO pose")
        rotation = rotation_matrix(quaternion / np.linalg.norm(quaternion))
        xyz = read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            return
        world = xyz @ rotation.T + translation
        origin = translation + rotation @ offset
        pose = np.r_[origin, 1., 0., 0., 0.]  # world points require only the ray origin
        final = args.output / f"{stamp}.npz"
        temporary = final.with_suffix(".tmp")
        with temporary.open("xb") as stream:
            np.savez(stream, points=world.astype(np.float32), pose=pose)
        temporary.rename(final)
        last_stamp = stamp
        count += 1

    try:
        cloud = message_filters.Subscriber(node, PointCloud2, "/cloud_registered_body_1")
        odom = message_filters.Subscriber(node, Odometry, "/Odometry_loc")
        synchronizer = message_filters.TimeSynchronizer([cloud, odom], 30)
        synchronizer.registerCallback(receive)
        deadline = time.monotonic() + args.timeout
        while count < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if count < args.frames:
            raise RuntimeError(f"Timed out after capturing {count}/{args.frames} exact pairs")
        print(f"Captured {count} exact scan/odometry pairs")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
