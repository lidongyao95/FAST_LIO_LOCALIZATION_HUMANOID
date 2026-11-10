#!/usr/bin/env python3
"""G1 velocity bridge with dry-run output by default.

SDK session I/O runs on a worker thread. Blocking on the session from the timer
callback starved this node's own subscriptions, which then reported stale scan
and odometry even though both inputs were healthy.
"""

import math
import select
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class VelocityBridge(Node):
    LIMITS = (0.30, 0.0, 0.25)
    ACCEL = (0.30, 0.0, 0.50)
    TIMEOUT = 0.30
    ACK_TIMEOUT = 0.22
    FIRST_ACK_TIMEOUT = 1.0
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
        self.declare_parameter('preflight_robot_frame', '')
        self.preflight_frame = self.get_parameter('preflight_robot_frame').value
        self.preflight_since = None
        if self.preflight_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('control_permission_topic', '')
        permit = self.get_parameter('control_permission_topic').value
        self.create_service(SetBool, '~/enable', self.enable)
        self.declare_parameter('zero_session_executable', '')
        self.declare_parameter('robot_interface', '')
        self.declare_parameter('hardware_output', False)
        self.hardware_output = self.get_parameter('hardware_output').value
        self.session = None
        self.zero_executable = self.get_parameter('zero_session_executable').value
        self.robot_interface = self.get_parameter('robot_interface').value
        if bool(self.zero_executable) != bool(self.robot_interface):
            raise ValueError('Set both zero_session_executable and robot_interface')
        if self.hardware_output and not self.zero_executable:
            raise ValueError('Hardware output requires the SDK executable and interface')
        if self.hardware_output and (
            not self.preflight_frame or 'registration' not in self.health or
            not permit.startswith('/') or
            permit == self.get_namespace().rstrip('/') + '/' + self.get_name() + '/control_ok'
        ):
            raise ValueError('Hardware output requires localization preflight and a separate control permission topic')
        self.create_subscription(Bool, permit if self.hardware_output else '~/control_ok',
                                 self.control, 1)
        if self.zero_executable:
            self.create_service(Trigger, '~/request_sdk_zero', self.request_sdk_zero)
        self.io_lock = threading.Lock()
        self.pending_command = None
        self.pending_stop = False
        self.io_error = None
        self.first_exchange = False
        self.stop_ok = True
        self.stop_done = threading.Event()
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()
        self.create_timer(0.05, self.tick)

    def io_loop(self):
        while rclpy.ok():
            with self.io_lock:
                session = self.session
                stop_requested = self.pending_stop
                command = self.pending_command
                self.pending_command = None
            if session is None:
                time.sleep(0.01)
                continue
            if stop_requested:
                self.stop_ok = False
                try:
                    self.stop_ok = self.exchange('stop', self.ACK_TIMEOUT) == b'stop_result=0'
                except (OSError, TimeoutError):
                    pass
                self.close_session()
                with self.io_lock:
                    self.pending_stop = False
                self.stop_done.set()
                continue
            if command is None:
                time.sleep(0.005)
                continue
            timeout = self.FIRST_ACK_TIMEOUT if self.first_exchange else self.ACK_TIMEOUT
            try:
                result = self.exchange(command, timeout)
            except (OSError, TimeoutError):
                self.io_error = 'SDK velocity unconfirmed'
                self.close_session()
                continue
            self.first_exchange = False
            if result != b'send_result=0':
                self.io_error = ('robot control revoked' if result == b'control_revoked'
                                 else 'SDK velocity rejected')
                self.close_session()

    def close_session(self):
        with self.io_lock:
            session, self.session = self.session, None
        if session is None:
            return
        for stream in (session.stdin, session.stdout):
            try:
                stream.close()
            except OSError:
                pass
        try:
            session.wait(timeout=0.25)
        except subprocess.TimeoutExpired:
            session.kill()
            session.wait()

    def exchange(self, command, timeout):
        self.session.stdin.write((command + '\n').encode('ascii'))
        if not select.select([self.session.stdout], [], [], timeout)[0]:
            raise TimeoutError('SDK session acknowledgement timed out')
        return self.session.stdout.read(256).strip()

    def request_sdk_zero(self, request, response):
        if self.session is not None:
            response.success = self.stop('SDK zero requested')
            response.message = 'stop_result=0' if response.success else 'SDK stop unconfirmed'
            return response
        if self.hardware_output:
            response.message = 'No active SDK velocity session'
            return response
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
        if not healthy:
            if name != 'control':
                self.preflight_since = None
            if self.enabled:
                self.stop(name + ' unhealthy')

    def stale_health(self, now):
        stale = []
        for name, stamp in self.health.items():
            if name == 'control' and not self.hardware_output:
                continue
            if stamp is None or now - stamp >= (2.0 if name == 'registration' else self.TIMEOUT):
                stale.append(name)
        return stale

    def preflight_issues(self):
        if not self.preflight_frame:
            return []
        issues = []
        try:
            transform = self.tf_buffer.lookup_transform('map', self.preflight_frame, Time())
            if not self.fresh(transform.header.stamp):
                issues.append('map TF stale')
        except TransformException:
            issues.append('map TF missing')
        if self.hardware_output:
            permit = self.get_parameter('control_permission_topic').value
            if self.count_publishers(permit) != 1:
                issues.append('control permission requires exactly one publisher')
        publishers = self.count_publishers('/global_costmap/costmap_raw')
        if publishers != 1:
            issues.append(f'global costmap publishers={publishers}, expected 1')
        return issues

    def timeout_reason(self, now):
        stale = self.stale_health(now)
        if now - self.command_time >= self.TIMEOUT:
            stale.append('command')
        return 'timed out: ' + ', '.join(stale) if stale else None

    def stop(self, reason):
        self.enabled = False
        self.target = self.output = self.ZERO
        if rclpy.ok():
            self.publish(reason)
        with self.io_lock:
            pending = self.session is not None
            self.pending_command = None
            self.pending_stop = pending
        if not pending:
            return True
        self.stop_done.clear()
        if not self.stop_done.wait(0.8):
            self.close_session()
            if rclpy.ok():
                self.get_logger().error('SDK stop unconfirmed: ' + reason)
            return False
        if not self.stop_ok and rclpy.ok():
            self.get_logger().error('SDK stop unconfirmed: ' + reason)
        return self.stop_ok

    def enable(self, request, response):
        now = time.monotonic()
        if not request.data:
            stopped = self.stop('disabled by operator')
        else:
            issues = self.stale_health(now) + self.preflight_issues()
            if not issues and self.preflight_frame and (
                self.preflight_since is None or now - self.preflight_since < 2.0
            ):
                issues.append('waiting for 2 seconds of continuous valid localization and graph')
            if issues:
                response.message = 'Not ready: ' + ', '.join(issues)
                self.stop(response.message)
                return response
        if request.data and not self.enabled:
            self.target = self.output = self.ZERO
            self.command_time = self.last_tick = now
            if self.hardware_output:
                try:
                    session = subprocess.Popen(
                        [self.zero_executable, '--velocity-session', self.robot_interface],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0,
                    )
                except OSError as error:
                    response.message = str(error)
                    return response
                with self.io_lock:
                    self.session = session
                self.first_exchange = True
                self.io_error = None
            self.enabled = True
        response.success = stopped if not request.data else True
        response.message = (
            ('enabled for hardware' if self.hardware_output else 'enabled for dry run')
            if self.enabled else ('disabled' if response.success else 'SDK stop unconfirmed')
        )
        return response

    def command(self, msg):
        if not self.enabled:
            return
        now = time.monotonic()
        reason = self.timeout_reason(now)
        if reason:
            self.stop(reason)
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
        if self.preflight_frame:
            issues = self.preflight_issues()
            localization_stale = set(self.stale_health(now)) - {'control'}
            if issues or localization_stale:
                self.preflight_since = None
                if self.enabled and issues:
                    self.stop(', '.join(issues))
                    return
            elif self.preflight_since is None:
                self.preflight_since = now
        if not self.enabled:
            return
        reason = self.timeout_reason(now)
        if reason:
            self.stop(reason)
            return
        self.output = tuple(current + max(-rate * dt, min(rate * dt, target - current))
                            for current, target, rate in zip(self.output, self.target, self.ACCEL))
        if self.session is not None:
            with self.io_lock:
                self.pending_command = 'velocity {:.6f} {:.6f} {:.6f}'.format(*self.output)
        if self.io_error:
            reason, self.io_error = self.io_error, None
            self.stop(reason)
            return
        self.publish('enabled for hardware' if self.hardware_output else 'enabled for dry run')

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
        node.stop('shutdown')
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
