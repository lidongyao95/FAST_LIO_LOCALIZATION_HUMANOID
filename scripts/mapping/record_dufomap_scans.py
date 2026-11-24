#!/usr/bin/env python3
"""Capture exact-stamp FAST-LIO scan/odometry pairs for offline DUFOMap."""
import argparse
from pathlib import Path
import time

import message_filters
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import PointCloud2, PointField


def rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def cloud_xyz(cloud):
    fields = {field.name: field for field in cloud.fields}
    float_types = {PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}
    endian = ">" if cloud.is_bigendian else "<"
    xyz = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in float_types:
            raise ValueError(f"PointCloud2 needs a scalar floating-point {name} field")
        dtype = np.dtype(endian + float_types[field.datatype])
        if field.offset + dtype.itemsize > cloud.point_step:
            raise ValueError(f"PointCloud2 field {name} extends beyond point_step")
        xyz.append(np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                              offset=field.offset, strides=(cloud.row_step, cloud.point_step)))
    return np.column_stack([axis.ravel() for axis in xyz])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--lidar-in-body", type=float, nargs=3, required=True)
    end = parser.add_mutually_exclusive_group(required=True)
    end.add_argument("--frames", type=int)
    end.add_argument("--idle-timeout", type=float, metavar="SECONDS",
                     help="Finish after this long without a pair once capture has started")
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    offset = np.asarray(args.lidar_in_body)
    limits = [args.timeout]
    if args.idle_timeout is not None:
        limits.append(args.idle_timeout)
    if (args.frames is not None and args.frames < 2 or not np.isfinite(offset).all()
            or not all(np.isfinite(value) and value > 0 for value in limits)):
        parser.error("Require at least two frames, a finite offset, and positive time limits")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("record_dufomap_scans")
    last_stamp = -1
    count = 0
    last_pair = None

    def receive(cloud, odom):
        nonlocal last_stamp, count, last_pair
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
        if not cloud.width or not cloud.height:
            return
        xyz = cloud_xyz(cloud)
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
        last_pair = time.monotonic()

    try:
        cloud = message_filters.Subscriber(node, PointCloud2, "/cloud_registered_body_1")
        odom = message_filters.Subscriber(node, Odometry, "/Odometry_loc")
        synchronizer = message_filters.TimeSynchronizer([cloud, odom], 30)
        synchronizer.registerCallback(receive)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.frames is not None and count >= args.frames:
                break
            if (args.idle_timeout is not None and last_pair is not None
                    and time.monotonic() - last_pair >= args.idle_timeout):
                break
        else:
            raise RuntimeError(f"Capture timed out after {count} exact pairs")
        if count < 2:
            raise RuntimeError(f"Captured only {count} exact pairs")
        print(f"Captured {count} exact scan/odometry pairs")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
