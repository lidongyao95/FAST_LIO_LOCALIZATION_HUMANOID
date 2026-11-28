#!/usr/bin/env python3
"""Measure how well the simulated G1 tracks velocity commands across the policy's range."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

CASES = [
    (0.2, 0, 0), (0.4, 0, 0), (0.6, 0, 0), (0.8, 0, 0), (1.0, 0, 0),
    (-0.3, 0, 0), (-0.5, 0, 0),
    (0, 0.12, 0), (0, 0.2, 0), (0, 0.3, 0), (0, 0.4, 0), (0, 0.5, 0), (0, -0.4, 0),
    (0, 0, 0.3), (0, 0, 0.6), (0, 0, 0.8), (0, 0, 1.0), (0, 0, -0.8),
    (0.6, 0, 0.6), (0.8, 0, 0.8), (0.6, 0.3, 0), (0.5, -0.3, 0.5), (0.3, 0.4, 0),
    (0.05, 0, 0), (0, 0.05, 0), (0.07, 0.07, 0),
]


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_of(q):
    w, x, y, z = q
    return math.degrees(math.acos(max(-1., min(1., 1 - 2 * (x * x + y * y)))))


class Sweep(Node):
    def __init__(self):
        super().__init__('g1_walking_sweep')
        self.pose = None
        self.samples = []
        self.mode_pub = self.create_publisher(String, '/g1/controller_mode', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/g1_sim/ground_truth', self.on_pose, 10)
        self.create_subscription(Float32MultiArray, '/g1_sim/body_extent', self.on_extent, 10)
        self.extent = None
        self.reset_srv = self.create_client(Trigger, '/g1/reset')
        self.release_srv = self.create_client(Trigger, '/g1/release')

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose = (stamp, p.x, p.y, p.z, (q.w, q.x, q.y, q.z))

    def on_extent(self, msg):
        # Keep the widest reach seen on each side: xmin, xmax, ymin, ymax.
        low_high = [min, max, min, max]
        self.extent = list(msg.data) if self.extent is None else [
            pick(old, new) for pick, old, new in zip(low_high, self.extent, msg.data)]

    def run_for(self, seconds, mode, command=(0., 0., 0.), record=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # The bridge zeroes its velocity on every mode message, so only send the
            # mode while standing still.
            if not record:
                self.mode_pub.publish(String(data=mode))
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = map(float, command)
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=.02)
            if record and self.pose and (not self.samples or self.samples[-1][0] != self.pose[0]):
                self.samples.append(self.pose)

    def call(self, client):
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('simulation service unavailable')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
        if not future.done() or not future.result().success:
            raise RuntimeError('service call failed')

    def case(self, command, seconds):
        self.run_for(.5, 'passive')
        self.call(self.reset_srv)
        self.run_for(3., 'stand')
        self.run_for(1., 'velocity')
        self.call(self.release_srv)
        self.run_for(2.5, 'velocity')
        self.samples = []
        self.extent = None
        self.run_for(seconds, 'velocity', command, record=True)
        moving = self.samples
        self.samples = []
        self.run_for(2.5, 'velocity', record=True)
        stopping = self.samples
        result = summarize(command, moving, stopping)
        result['body_extent'] = [round(v, 3) for v in self.extent] if self.extent else None
        return result


def summarize(command, moving, stopping):
    result = dict(cmd=list(command), samples=len(moving))
    if len(moving) < 20:
        return dict(result, ok=False, reason='no ground truth')
    cut = [s for s in moving if s[0] - moving[0][0] >= 1.5]  # skip acceleration
    first, last = cut[0], cut[-1]
    dt = last[0] - first[0]
    yaws = [yaw_of(s[4]) for s in cut]
    unwrapped = [yaws[0]]
    for value in yaws[1:]:
        step = (value - unwrapped[-1] + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + step)
    # Body-frame velocity: rotate each world displacement by the heading at that moment.
    vx = vy = 0.
    for a, b, yaw in zip(cut, cut[1:], unwrapped):
        dx, dy = b[1] - a[1], b[2] - a[2]
        vx += math.cos(yaw) * dx + math.sin(yaw) * dy
        vy += -math.sin(yaw) * dx + math.cos(yaw) * dy
    everything = moving + stopping
    min_z = min(s[3] for s in everything)
    max_tilt = max(tilt_of(s[4]) for s in everything)
    stop_distance = math.hypot(stopping[-1][1] - stopping[0][1], stopping[-1][2] - stopping[0][2]) if stopping else None
    fell = min_z < 0.55 or max_tilt > 35
    profile, mark = [], moving[0]
    for sample in moving:
        if sample[0] - mark[0] >= 1.:
            profile.append(round(math.hypot(sample[1] - mark[1], sample[2] - mark[2]) / (sample[0] - mark[0]), 2))
            mark = sample
    result['speed_per_second'] = profile
    return dict(result, vx=vx / dt, vy=vy / dt, wz=(unwrapped[-1] - unwrapped[0]) / dt,
                min_z=min_z, max_tilt_deg=max_tilt, stop_distance=stop_distance, fell=fell, ok=not fell)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='sim/generated/walking_sweep.json')
    parser.add_argument('--seconds', type=float, default=4.5)
    parser.add_argument('--case', type=float, nargs=3, action='append', metavar=('VX', 'VY', 'WZ'),
                        help='Run only this command; repeatable')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Sweep()
    results = []
    try:
        for command in (args.case or CASES):
            try:
                result = node.case(command, args.seconds)
            except RuntimeError as error:
                result = dict(cmd=list(command), ok=False, reason=str(error))
            results.append(result)
            print(json.dumps(result), flush=True)
            Path(args.output).write_text(json.dumps(results, indent=1))
    finally:
        node.run_for(.3, 'passive')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
