#!/usr/bin/env python3
"""Drive the simulated G1 fast and compare localization (map -> base_footprint) with ground truth."""
import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

SEGMENTS = [
    ('forward 1.0', (1.0, 0., 0.), 4.5),
    ('turn 0.8', (0., 0., 0.8), 4.0),
    ('forward 0.8', (0.8, 0., 0.), 6.0),
    ('sidestep 0.4', (0., 0.4, 0.), 4.0),
    ('arc 0.6/0.3/0.6', (0.6, 0.3, 0.6), 6.0),
    ('slow 0.2', (0.2, 0., 0.), 4.0),
]


class Check(Node):
    def __init__(self):
        super().__init__('g1_localization_speed_check',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(PoseStamped, '/g1_sim/ground_truth', self.on_truth, 50)
        self.create_subscription(Float32, '/localization_3d_confidence', self.on_confidence, 10)
        self.pending = deque()
        self.latest = None
        self.confidence = float('nan')
        self.label = 'idle'
        self.rows = []

    def on_confidence(self, msg):
        self.confidence = msg.data

    def on_truth(self, msg):
        self.latest = msg
        self.pending.append((self.label, msg, self.confidence))

    def compare(self):
        if self.latest is None:
            return
        newest = Time.from_msg(self.latest.header.stamp).nanoseconds
        while self.pending and newest - Time.from_msg(self.pending[0][1].header.stamp).nanoseconds > 4e8:
            label, truth, confidence = self.pending.popleft()
            try:
                tf = self.buffer.lookup_transform('map', 'base_footprint', Time.from_msg(truth.header.stamp))
            except TransformException:
                self.rows.append((label, None, confidence))
                continue
            error = math.hypot(tf.transform.translation.x - truth.pose.position.x,
                               tf.transform.translation.y - truth.pose.position.y)
            self.rows.append((label, error, confidence))

    def drive(self, label, command, seconds):
        self.label = label
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = command
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=.04)
            self.compare()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='sim/generated/localization_speed.json')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Check()
    node.drive('idle', (0., 0., 0.), 3.)
    for label, command, seconds in SEGMENTS:
        node.drive(label, command, seconds)
        node.drive(label + ' (stop)', (0., 0., 0.), 2.)
    node.drive('idle', (0., 0., 0.), 1.5)
    report = []
    for label in dict.fromkeys(row[0] for row in node.rows):
        errors = [row[1] for row in node.rows if row[0] == label and row[1] is not None]
        missing = sum(1 for row in node.rows if row[0] == label and row[1] is None)
        confidences = [row[2] for row in node.rows if row[0] == label and not math.isnan(row[2])]
        item = dict(segment=label, samples=len(errors), tf_missing=missing,
                    mean_error=sum(errors) / len(errors) if errors else None,
                    max_error=max(errors) if errors else None,
                    min_confidence=min(confidences) if confidences else None)
        report.append(item)
        print(json.dumps(item), flush=True)
    truth = node.latest.pose.position
    print('final ground truth', round(truth.x, 2), round(truth.y, 2), round(truth.z, 2))
    Path(args.output).write_text(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
