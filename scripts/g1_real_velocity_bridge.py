#!/usr/bin/env python3
"""Offline G1 velocity bridge. Output is for a fake receiver only."""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class VelocityBridge(Node):
    LIMITS = (0.30, 0.0, 0.25)
    ACCEL = (0.30, 0.0, 0.50)
    TIMEOUT = 0.30
    ZERO = (0.0, 0.0, 0.0)

    def __init__(self):
        super().__init__('g1_real_velocity_bridge')
        self.enabled = False
        self.health = {'localization': None, 'control': None}
        self.command_time = 0.0
        self.target = self.output = self.ZERO
        self.last_tick = time.monotonic()
        self.output_pub = self.create_publisher(Twist, '~/dry_run_velocity', 1)
        self.state_pub = self.create_publisher(String, '~/state', 1)
        self.create_subscription(Twist, '~/candidate_velocity', self.command, 1)
        self.declare_parameter('use_registration_health', False)
        if self.get_parameter('use_registration_health').value:
            self.health = dict(control=None, registration=None, scan=None, odometry=None)
            self.create_subscription(Bool, '/localization_3d_registration_ok',
                                     self.registration, 1)
            self.create_subscription(PointCloud2, '/cloud_registered_1', self.scan, 1)
            self.create_subscription(Odometry, '/Odometry_loc', self.odometry, 1)
        else:
            self.create_subscription(Bool, '~/localization_ok', self.localization, 1)
        self.create_subscription(Bool, '~/control_ok', self.control, 1)
        self.create_service(SetBool, '~/enable', self.enable)
        self.declare_parameter('zero_session_executable', '')
        self.declare_parameter('robot_interface', '')
        self.zero_executable = self.get_parameter('zero_session_executable').value
        self.robot_interface = self.get_parameter('robot_interface').value
        if bool(self.zero_executable) != bool(self.robot_interface):
            raise ValueError('Set both zero_session_executable and robot_interface')
        if self.zero_executable:
            self.create_service(Trigger, '~/request_sdk_zero', self.request_sdk_zero)
        self.create_timer(0.05, self.tick)

    def request_sdk_zero(self, request, response):
        self.stop('SDK zero requested')
        try:
            result = subprocess.run(
                [self.zero_executable, '--zero-only', self.robot_interface],
                input='stop\nquit\n', capture_output=True, text=True, timeout=2.0,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            response.message = str(error)
            return response
        response.message = (result.stdout or result.stderr).strip()
        response.success = result.returncode == 0 and response.message == 'stop_result=0'
        return response

    def localization(self, msg):
        self.update_health('localization', msg.data)

    def registration(self, msg):
        self.update_health('registration', msg.data)

    def scan(self, msg):
        self.update_health('scan', bool(msg.width and msg.height) and self.fresh(msg.header.stamp))

    def odometry(self, msg):
        self.update_health('odometry', self.fresh(msg.header.stamp))

    def fresh(self, stamp):
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_ns = self.get_clock().now().nanoseconds - source_ns
        return source_ns > 0 and 0 <= age_ns < self.TIMEOUT * 1_000_000_000

    def control(self, msg):
        self.update_health('control', msg.data)

    def update_health(self, name, healthy):
        self.health[name] = time.monotonic() if healthy else None
        if self.enabled and not healthy:
            self.stop(name + ' unhealthy')

    def healthy(self, now):
        return all(stamp is not None and now - stamp < (2.0 if name == 'registration' else self.TIMEOUT)
                   for name, stamp in self.health.items())

    def stop(self, reason):
        self.enabled = False
        self.target = self.output = self.ZERO
        self.publish(reason)

    def enable(self, request, response):
        now = time.monotonic()
        if not request.data:
            self.stop('disabled by operator')
        elif not self.healthy(now):
            self.stop('health missing or stale')
            response.message = 'Both health heartbeats must be fresh'
            return response
        elif not self.enabled:
            self.target = self.output = self.ZERO
            self.command_time = self.last_tick = now
            self.enabled = True
        response.success = True
        response.message = 'enabled for dry run' if self.enabled else 'disabled'
        return response

    def command(self, msg):
        if not self.enabled:
            return
        now = time.monotonic()
        if not self.healthy(now) or now - self.command_time >= self.TIMEOUT:
            self.stop('health or command timed out')
            return
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.stop('non-finite command')
            return
        self.target = tuple(max(-limit, min(limit, value))
                            for value, limit in zip(values, self.LIMITS))
        self.command_time = now

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.05)
        self.last_tick = now
        if not self.enabled:
            return
        if not self.healthy(now) or now - self.command_time >= self.TIMEOUT:
            self.stop('health or command timed out')
            return
        self.output = tuple(current + max(-rate * dt, min(rate * dt, target - current))
                            for current, target, rate in zip(self.output, self.target, self.ACCEL))
        self.publish('enabled for dry run')

    def publish(self, reason):
        velocity = Twist()
        velocity.linear.x, velocity.linear.y, velocity.angular.z = self.output
        self.output_pub.publish(velocity)
        state = String()
        state.data = reason
        self.state_pub.publish(state)


def main():
    rclpy.init()
    node = VelocityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop('shutdown')
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
