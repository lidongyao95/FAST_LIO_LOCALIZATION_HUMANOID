#!/usr/bin/env python3
"""Publish /clock that follows bag message timestamps during offline replay.

Foxy's `ros2 bag play` has no --clock option, so nodes started with
use_sim_time:=true would freeze at t=0 on replay. This node republishes the
header stamp of an incoming topic (default /livox/imu) as /clock, offset by
lead_ms so consumers see a clock slightly ahead of the data they process.

FAST-LIO stamps its messages and TF from the lidar stamp regardless of
use_sim_time, so it can keep use_sim_time:=false while consumers use this clock.
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


class ReplayClock(Node):
    def __init__(self, topic, lead_ms):
        super().__init__('replay_clock')
        self.lead = lead_ms / 1000.0
        self.last = None
        self.published = 0
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            return
        target = stamp + self.lead
        if self.last is not None and target <= self.last:
            return
        self.last = target
        out = Clock()
        out.clock.sec = int(target)
        out.clock.nanosec = int(round((target - int(target)) * 1e9))
        if out.clock.nanosec >= 1_000_000_000:
            out.clock.sec += 1
            out.clock.nanosec -= 1_000_000_000
        self.pub.publish(out)
        self.published += 1
        if self.published == 1:
            self.get_logger().info(f'first /clock at {target:.6f} (lead {self.lead * 1000:.0f} ms)')
        elif self.published % 2000 == 0:
            self.get_logger().info(f'{self.published} /clock msgs, now {target:.6f}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/livox/imu')
    parser.add_argument('--lead-ms', type=float, default=100.0)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = ReplayClock(args.topic, args.lead_ms)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
