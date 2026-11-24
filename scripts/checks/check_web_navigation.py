#!/usr/bin/env python3
"""Navigation ownership tests with fake ROS and a fake SDK; never contacts a robot."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[2]
modules = {name: ModuleType(name) for name in (
    'action_msgs', 'action_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
    'nav2_msgs', 'nav2_msgs.action', 'rclpy', 'rclpy.action',
    'rclpy.callback_groups', 'rclpy.time', 'std_msgs', 'std_msgs.msg')}
modules['action_msgs.msg'].GoalStatus = SimpleNamespace(STATUS_SUCCEEDED=4)
modules['geometry_msgs.msg'].Twist = object
modules['std_msgs.msg'].Bool = object
modules['nav2_msgs.action'].NavigateToPose = object
modules['rclpy.action'].ActionClient = lambda *a: Mock(server_is_ready=lambda: True)
modules['rclpy.callback_groups'].ReentrantCallbackGroup = object
modules['rclpy.time'].Time = object
with patch.dict(sys.modules, modules):
    spec = importlib.util.spec_from_file_location('navigation', ROOT/'scripts/real_robot/web/navigation.py')
    navigation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(navigation)


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.fake = Path(self.tmp.name)/'sdk'
        self.fake.write_text('''#!/usr/bin/env python3
import sys
print('READY', flush=True)
for line in sys.stdin:
    print('OK', flush=True)
    if line.strip() == 'STOP': break
''')
        self.fake.chmod(0o755)
        self.node = Mock(count_publishers=lambda _: 1)
        self.session = navigation.NavigationSession(self.node, True, self.fake)
        self.session.healthy = lambda: True

    def test_sdk_handshake_and_stop_reaps_process(self):
        sdk = navigation.SdkTransport(self.fake, 'fake')
        sdk.send('V 0 0 0')
        self.assertTrue(sdk.close())
        self.assertIsNotNone(sdk.process.poll())
        self.assertTrue(sdk.close())

    def test_sdk_crash_is_reported_and_reaped(self):
        sdk = navigation.SdkTransport(self.fake, 'fake')
        sdk.process.kill(); sdk.process.wait()
        self.assertFalse(sdk.close())
        self.assertTrue(sdk.closed)

    def test_heartbeat_requires_private_owner_token(self):
        self.session.token = 'secret'
        with self.assertRaises(ValueError): self.session.heartbeat('another-tab')
        self.session.heartbeat('secret')
        self.assertGreater(self.session.deadline, time.monotonic())
        self.assertNotIn('token', self.session.state())

    def run_worker_failure(self, *, expired=False, stale=False, unhealthy=False):
        sdk = Mock()
        self.session.transport = sdk
        self.session.token = 'owner'
        self.session.deadline = time.monotonic()+(-1 if expired else 3)
        self.session.velocity = ((.1,0,0), time.monotonic()-(1 if stale else 0))
        self.session.healthy = lambda: not unhealthy
        self.session.worker('owner')
        sdk.close.assert_called_once()
        sdk.send.assert_not_called()
        self.assertFalse(self.session.state()['active'])
        self.assertEqual(self.session.phase, 'failed')

    def test_expired_web_lease_stops_before_velocity(self): self.run_worker_failure(expired=True)
    def test_stale_controller_stops_before_velocity(self): self.run_worker_failure(stale=True)
    def test_lost_localization_stops_before_velocity(self): self.run_worker_failure(unhealthy=True)

    def test_cancel_stops_sdk_before_nav2_and_invalidates_heartbeat(self):
        order=[]
        self.session.token='owner'
        self.session.transport=Mock(close=lambda: order.append('SDK') or True)
        self.session.handle=Mock(cancel_goal_async=lambda: order.append('Nav2'))
        self.session.cancel()
        self.assertEqual(order,['SDK','Nav2'])
        with self.assertRaises(ValueError): self.session.heartbeat('owner')

    def test_late_goal_acceptance_after_cancel_never_opens_sdk(self):
        class Future:
            def add_done_callback(self, callback): self.callback=callback
            def result(self): return handle
        future=Future();handle=Mock(accepted=True)
        self.session.action.send_goal_async=Mock(return_value=future)
        def wait_for(*_):
            self.session.cancel()
            future.callback(future)
            return handle
        with patch.object(navigation,'SdkTransport') as sdk:
            with self.assertRaisesRegex(RuntimeError,'启动已取消'):
                self.session.start(object(),wait_for)
            sdk.assert_not_called()
        handle.cancel_goal_async.assert_called()

    def test_no_sdk_start_when_navigation_disabled_or_health_missing(self):
        for disabled in (True,False):
            self.session.enabled=not disabled
            self.session.healthy=lambda: False
            with patch.object(navigation,'SdkTransport') as sdk:
                with self.assertRaises(RuntimeError): self.session.start(object(),lambda *_:None)
                sdk.assert_not_called()


if __name__ == '__main__': unittest.main()
