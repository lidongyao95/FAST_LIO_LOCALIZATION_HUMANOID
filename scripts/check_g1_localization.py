#!/usr/bin/env python3
"""Measure the running demo; nearest timestamp truth, never input to LIO."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('check_g1_localization')
    counts, latest, truth, poses = {}, {}, deque(maxlen=1000), []

    def callback(msg, key):
        counts[key] = counts.get(key, 0) + 1
        if isinstance(msg, PoseStamped):
            stamp = msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec
            p = msg.pose.position
            xyz = [p.x, p.y, p.z]
            latest[key] = xyz
            if key == 'truth':
                truth.append((stamp, xyz))
            else:
                poses.append((stamp, xyz))
        elif key == 'confidence':
            latest[key] = msg.data
        elif key == 'cloud':
            latest['cloud_points'] = msg.width

    for typ, key, topic in [(PoseStamped, 'loc', '/localization_3d'),
                            (PoseStamped, 'truth', '/g1_sim/ground_truth'),
                            (Imu, 'imu', '/livox/imu'),
                            (PointCloud2, 'cloud', '/livox/lidar'),
                            (Float32, 'confidence', '/localization_3d_confidence')]:
        node.create_subscription(typ, topic, lambda m, k=key: callback(m, k), 50)
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.1)
    elapsed = time.monotonic()-start
    errors, time_errors = [], []
    for stamp, xyz in poses:
        if not truth:
            break
        gt_stamp, gt_xyz = min(truth, key=lambda pair: abs(pair[0]-stamp))
        if abs(gt_stamp-stamp) <= .06:
            errors.append(math.dist(xyz, gt_xyz))
            time_errors.append(abs(gt_stamp-stamp))
    result = dict(duration_s=elapsed, counts=counts,
                  rates_hz={k: v/elapsed for k, v in counts.items()}, latest=latest,
                  matched_pose_samples=len(errors),
                  position_rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
                  max_pair_timestamp_delta_s=max(time_errors) if time_errors else None)
    output = Path(__file__).resolve().parents[1] / 'sim/generated/validation.json'
    output.parent.mkdir(exist_ok=True, parents=True)
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if not all(counts.get(k, 0) for k in ['imu', 'cloud', 'loc', 'confidence']):
        raise SystemExit('Missing sensor or localization output')


if __name__ == '__main__':
    main()
