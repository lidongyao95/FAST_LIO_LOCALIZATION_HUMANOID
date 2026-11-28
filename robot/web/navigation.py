"""Web-owned real navigation: explicit start, expiring ownership, bounded SDK transport."""
import math
import os
from pathlib import Path
import secrets
import selectors
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from std_msgs.msg import Bool


class SdkTransport:
    def __init__(self, executable, interface):
        env = os.environ.copy()
        # The SDK ships its own DDS; do not load ROS Foxy's incompatible DDS.
        env.pop('LD_LIBRARY_PATH', None)
        self.process = subprocess.Popen([str(executable), interface], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        bufsize=0, env=env)
        self.lock = threading.Lock()
        self.closed = False
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.buffer = b''
        try:
            self._reply('READY', 3.)
            self.send('ARM')
        except Exception:
            self.close()
            raise

    def _reply(self, expected='OK', timeout=1.):
        deadline = time.monotonic() + timeout
        while b'\n' not in self.buffer:
            if not self.selector.select(max(0., deadline-time.monotonic())):
                raise RuntimeError('SDK 会话响应超时')
            chunk = os.read(self.process.stdout.fileno(), 256)
            if not chunk:
                raise RuntimeError('SDK 会话已退出，请检查运控状态')
            self.buffer += chunk
            if len(self.buffer) > 1024:
                raise RuntimeError('SDK 会话输出无效')
        line, self.buffer = self.buffer.split(b'\n', 1)
        if line.decode().strip() != expected:
            raise RuntimeError('SDK 拒绝导航：'+line.decode(errors='replace'))

    def send(self, line):
        with self.lock:
            if self.closed:
                raise RuntimeError('SDK 会话已关闭')
            self.process.stdin.write((line+'\n').encode())
            self.process.stdin.flush()
            self._reply()

    def close(self):
        with self.lock:
            if self.closed:
                return True
            self.closed = True
            ok = True
            try:
                if self.process.poll() is None:
                    self.process.stdin.write(b'STOP\n'); self.process.stdin.flush()
                    self._reply(timeout=1.)
                elif self.process.returncode != 0:
                    ok = False
            except (OSError, RuntimeError):
                ok = False
            finally:
                try:
                    self.process.stdin.close()
                except OSError:
                    ok = False
                try:
                    self.process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait()
                self.process.stdout.close(); self.selector.close()
            return ok


class NavigationSession:
    def __init__(self, node, enabled=False, executable='', interface='eth0'):
        self.node, self.enabled = node, enabled
        self.executable, self.interface = Path(executable), interface
        self.action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.lock = threading.RLock()
        self.phase, self.message = 'idle', '等待开始实机导航'
        self.token = None
        self.handle = None
        self.transport = None
        self.velocity = None
        self.health_time = 0.
        self.health_ok = False
        self.deadline = 0.
        group = ReentrantCallbackGroup()
        node.create_subscription(Twist, '/g1/navigation/cmd_vel', self.on_velocity, 1, callback_group=group)
        node.create_subscription(Bool, '/localization_3d_registration_ok', self.on_health, 1, callback_group=group)

    def on_health(self, message):
        self.health_ok, self.health_time = message.data, time.monotonic()

    def on_velocity(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.cancel('控制器输出非有限速度')
            return
        with self.lock:
            self.velocity = values, time.monotonic()

    def healthy(self):
        if not self.health_ok or time.monotonic()-self.health_time > 5.:
            return False
        try:
            transform = self.node.tf.buffer.lookup_transform('map', self.node.robot_frame, Time())
            stamp = transform.header.stamp
            age = self.node.get_clock().now().nanoseconds - (stamp.sec*10**9+stamp.nanosec)
            return abs(age) < 1e9
        except Exception:
            return False

    def state(self):
        reason = ''
        if not self.enabled: reason = '当前使用 --plan-only 启动；重新运行 05 默认入口可启用实机导航'
        elif not self.executable.is_file() or not os.access(self.executable, os.X_OK): reason = '未构建本机导航 SDK：运行 bash robot/sdk/build.sh'
        elif not self.action.server_is_ready(): reason = '等待 Nav2 导航服务'
        elif not self.healthy(): reason = '等待持续更新的定位与配准健康'
        with self.lock:
            return {'mode': 'real' if self.enabled else 'plan', 'available': not reason, 'reason': reason, 'phase': self.phase,
                    'active': self.token is not None, 'message': self.message}

    def heartbeat(self, token):
        with self.lock:
            if not self.token or not secrets.compare_digest(str(token), self.token):
                raise ValueError('导航会话已失效')
            self.deadline = time.monotonic()+2.

    def start(self, goal, wait_for):
        ready = self.state()
        if not ready['available']: raise RuntimeError(ready['reason'])
        if self.node.count_publishers('/g1/navigation/cmd_vel') != 1:
            raise RuntimeError('导航速度发布者缺失或重复')
        with self.lock:
            if self.token: raise RuntimeError('已有实机导航任务，请先取消')
            token = secrets.token_urlsafe(24)
            self.token, self.phase, self.message = token, 'starting', '正在检查 SDK 并接受目标'
            self.velocity = None
        future = self.action.send_goal_async(goal)
        def cancel_late(done):
            if self.token != token:
                try:
                    handle = done.result()
                    if handle.accepted: handle.cancel_goal_async()
                except Exception:
                    pass
        future.add_done_callback(cancel_late)
        try:
            handle = wait_for(future, 4.)
            if not handle.accepted: raise RuntimeError('Nav2 拒绝导航目标')
            with self.lock:
                if self.token != token:
                    handle.cancel_goal_async()
                    raise RuntimeError('启动已取消')
                self.handle = handle
            transport = SdkTransport(self.executable, self.interface)
            with self.lock:
                if self.token != token:
                    transport.close(); raise RuntimeError('启动已取消')
                self.transport = transport
                self.deadline = time.monotonic()+2.
                self.phase, self.message = 'running', '实机导航执行中'
            handle.get_result_async().add_done_callback(lambda result: self.finished(token, result))
            threading.Thread(target=self.worker, args=(token,), daemon=True).start()
            return token
        except Exception:
            self.cancel('启动实机导航失败')
            raise

    def finished(self, token, future):
        if self.token != token: return
        try:
            status = future.result().status
        except Exception as error:
            self.cancel(f'导航结果异常：{error}', phase='failed')
            return
        self.cancel('已到达目标' if status == GoalStatus.STATUS_SUCCEEDED else f'导航结束（状态 {status}）',
                    phase='succeeded' if status == GoalStatus.STATUS_SUCCEEDED else 'stopped')

    def worker(self, token):
        first_command_deadline = time.monotonic()+2.
        try:
            while self.token == token:
                if time.monotonic() > self.deadline: raise RuntimeError('网页连接超时，已停止导航')
                if not self.healthy(): raise RuntimeError('定位失效，已停止导航')
                with self.lock: velocity, transport = self.velocity, self.transport
                if velocity is None:
                    if time.monotonic() > first_command_deadline: raise RuntimeError('控制器未输出速度')
                    values = (0., 0., 0.)
                else:
                    values, stamp = velocity
                    if time.monotonic()-stamp > .35: raise RuntimeError('导航速度超时')
                transport.send('V '+' '.join(f'{v:.6f}' for v in values))
                time.sleep(.05)
        except Exception as error:
            if self.token == token: self.cancel(str(error), phase='failed')

    def cancel(self, message='已取消导航', phase='stopped'):
        with self.lock:
            self.token = None
            transport, handle = self.transport, self.handle
            self.transport, self.handle = None, None
            self.phase, self.message = phase, message
        stopped = not transport or transport.close()
        if handle:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        if not stopped:
            with self.lock: self.phase, self.message = 'failed', '取消已请求，但 SDK 零速度确认失败，请人工接管'
        return self.state()
