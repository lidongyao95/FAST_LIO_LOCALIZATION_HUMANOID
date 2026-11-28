#!/usr/bin/env python3
"""Offline regression for the Nav2 obstacle service and Web client (no ROS)."""
import sys
import threading
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'shared/nav'))
from planning_obstacles import OBSTACLE_TOPIC, CLEARING_TOPIC, DEFAULT_THRESHOLDS, ObstacleGate, RosPlanningObstacles


class GateTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.cleared = []
        self.validation_error = None
        self.clear_error = None
        self.gate = ObstacleGate(self.validate, self.clear, self.sent.append,
                                 lambda cloud: ('empty', cloud), timeout=.3)

    def validate(self):
        if self.validation_error:
            raise RuntimeError(self.validation_error)

    def clear(self):
        # New empty observations must replace the previous real observation
        # before we ask Nav2 to clear accumulated obstacle cells.
        self.assertEqual(self.sent[-2:], [('empty', 2), ('empty', 3)])
        self.cleared.append(True)
        if self.clear_error:
            raise RuntimeError(self.clear_error)

    def ready(self):
        self.gate.receive(1)
        self.gate.refresh()
        self.assertTrue(self.gate.state()['enabled'])

    def switch(self, enabled, clouds=(2, 3)):
        errors = []
        def change():
            try:
                self.gate.set_enabled(enabled)
            except Exception as error:
                errors.append(error)
        worker = threading.Thread(target=change)
        worker.start()
        deadline = time.monotonic() + 1
        while not self.gate.state()['busy'] and worker.is_alive():
            if time.monotonic() > deadline:
                self.fail('Switch never entered pending state')
            time.sleep(.001)
        for cloud in clouds:
            self.gate.receive(cloud)
        worker.join(1)
        self.assertFalse(worker.is_alive())
        return errors

    def test_no_assumed_state_before_graph_and_input_are_ready(self):
        self.assertIsNone(self.gate.state()['enabled'])
        self.gate.refresh()
        self.assertFalse(self.gate.state()['available'])
        self.gate.receive(1)
        self.assertTrue(self.gate.state()['enabled'])

    def test_disable_replaces_input_then_clears_global_obstacles(self):
        self.ready()
        self.assertEqual(self.switch(False), [])
        self.assertEqual(self.cleared, [True])
        self.assertFalse(self.gate.state()['enabled'])
        self.gate.receive(4)
        self.assertEqual(self.sent[-1], ('empty', 4))

    def test_enable_waits_for_new_real_input(self):
        self.ready()
        self.assertEqual(self.switch(False), [])
        self.assertEqual(self.switch(True, (4,)), [])
        self.assertEqual(self.sent[-1], 4)
        self.assertTrue(self.gate.state()['enabled'])
        self.assertEqual(len(self.cleared), 1)

    def test_revision_changes_when_other_browser_switches(self):
        self.ready()
        before = self.gate.state()['revision']
        self.assertEqual(self.switch(False), [])
        self.assertEqual(self.gate.state()['revision'], before + 1)
        self.gate.set_enabled(False)
        self.assertEqual(self.gate.state()['revision'], before + 1)

    def test_sensor_callbacks_keep_running_while_costmaps_acknowledge_clear(self):
        self.ready()
        def clear():
            incoming = threading.Thread(target=lambda: self.gate.receive(4))
            incoming.start()
            incoming.join(.1)
            self.assertFalse(incoming.is_alive(), 'Costmap RPC blocked the sensor executor')
        self.gate.clear = clear
        self.assertEqual(self.switch(False), [])

    def test_misconfigured_costmap_cannot_be_reported_as_switched(self):
        self.ready()
        self.validation_error = 'wrong obstacle topic'
        with self.assertRaisesRegex(RuntimeError, 'wrong obstacle topic'):
            self.gate.set_enabled(False)
        self.gate.receive(2)
        self.assertEqual(self.sent[-1], 2)
        self.assertIsNone(self.gate.state()['enabled'])
        self.assertEqual(self.cleared, [])

    def test_clear_failure_restores_previous_input_and_reports_unknown(self):
        self.ready()
        self.clear_error = 'global costmap unavailable'
        errors = self.switch(False)
        self.assertEqual(len(errors), 1)
        self.assertEqual(self.gate.state()['revision'], 2)
        self.assertIsNone(self.gate.state()['enabled'])
        self.gate.receive(4)
        self.assertEqual(self.sent[-1], 4)
        self.gate.refresh()
        self.assertTrue(self.gate.state()['enabled'])

    def test_stopped_sensor_times_out_and_restores_input(self):
        self.ready()
        with self.assertRaises(TimeoutError):
            self.gate.set_enabled(False)
        self.assertIsNone(self.gate.state()['enabled'])
        self.gate.receive(4)
        self.assertEqual(self.sent[-1], 4)
        self.assertEqual(self.cleared, [])

    def test_sensor_becomes_stale(self):
        self.ready()
        with self.gate.condition:
            self.gate.last_input -= 5
        self.assertIsNone(self.gate.state()['enabled'])
        self.assertFalse(self.gate.state()['available'])

    def test_unconfirmed_costmap_update_rolls_back_and_invalidates_paths(self):
        self.ready()
        def timeout():
            raise TimeoutError('costmap update missing')
        self.gate.updated = timeout
        errors = self.switch(False)
        self.assertIsInstance(errors[0], TimeoutError)
        self.assertEqual(self.gate.state()['revision'], 2)
        self.assertIsNone(self.gate.state()['enabled'])
        self.gate.receive(4)
        self.assertEqual(self.sent[-1], 4)

    def test_invalid_boolean_is_rejected_without_mutating_input(self):
        self.ready()
        for value in ['false', 0, 1, None, [], {}]:
            with self.assertRaises(ValueError):
                self.gate.set_enabled(value)
        self.assertTrue(self.gate.state()['enabled'])
        self.assertEqual(self.sent, [1])


class RosAccessTests(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(sent=[], clears=[], calls=[])
        self.node.count_publishers = lambda topic: 1
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10))
        publisher = SimpleNamespace(publish=self.node.sent.append, get_subscription_count=lambda: 1)
        self.node.create_publisher = lambda *args, **kw: publisher
        self.node.create_subscription = lambda *args, **kw: None
        self.node.create_service = lambda *args, **kw: None
        self.node.create_timer = lambda *args, **kw: None
        self.tf = SimpleNamespace(lookup_transform=lambda *args: object())
        types = SimpleNamespace(PARAMETER_NOT_SET=0, PARAMETER_STRING_ARRAY=9,
                                PARAMETER_BOOL=1, PARAMETER_STRING=4, PARAMETER_DOUBLE=3)
        values = [(9, 'string_array_value', ['static_layer', 'obstacle_layer', 'inflation_layer']),
                  (1, 'bool_value', True), (4, 'string_value', 'lidar clearing'),
                  (4, 'string_value', OBSTACLE_TOPIC), (3, 'double_value', 0.0),
                  (9, 'string_array_value', ['obstacle_layer']),
                  (4, 'string_value', 'nav2_costmap_2d::StaticLayer'),
                  (4, 'string_value', 'nav2_costmap_2d::ObstacleLayer'),
                  (4, 'string_value', 'nav2_costmap_2d::InflationLayer'),
                  (4, 'string_value', 'map'), (4, 'string_value', 'body'),
                  (3, 'double_value', .6), (3, 'double_value', .15), (3, 'double_value', 1.8),
                  (3, 'double_value', 8.), (3, 'double_value', 8.),
                  (1, 'bool_value', True), (1, 'bool_value', False),
                  (4, 'string_value', CLEARING_TOPIC), (1, 'bool_value', False),
                  (1, 'bool_value', True), (3, 'double_value', 0.)]
        self.values = [SimpleNamespace(type=t, **{key: value}) for t, key, value in values]
        self.node.create_client = self.client
        modules = {name: ModuleType(name) for name in [
            'nav2_msgs.msg', 'nav2_msgs.srv', 'rcl_interfaces.msg', 'rcl_interfaces.srv',
            'rclpy.callback_groups', 'sensor_msgs.msg', 'std_srvs.srv', 'rclpy.time', 'tf2_ros']}
        modules['nav2_msgs.msg'].Costmap = object
        modules['nav2_msgs.srv'].ClearCostmapExceptRegion = SimpleNamespace(Request=SimpleNamespace)
        modules['rcl_interfaces.msg'].ParameterType = types
        modules['rcl_interfaces.srv'].GetParameters = SimpleNamespace(Request=SimpleNamespace)
        modules['rcl_interfaces.srv'].SetParametersAtomically = SimpleNamespace(Request=SimpleNamespace)
        modules['rclpy.callback_groups'].ReentrantCallbackGroup = object
        modules['rclpy.callback_groups'].MutuallyExclusiveCallbackGroup = object
        modules['sensor_msgs.msg'].PointCloud2 = object
        modules['std_srvs.srv'].SetBool = modules['std_srvs.srv'].Trigger = SimpleNamespace(Request=SimpleNamespace)
        modules['rclpy.time'].Time = object
        modules['tf2_ros'].TransformException = RuntimeError
        self.patch = patch.dict(sys.modules, modules)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        from planning_inflation import PlanningInflation
        self.inflation_patch = patch('planning_inflation.PlanningInflation',
            return_value=SimpleNamespace(active=lambda: True, apply=lambda radius: None))
        self.inflation_patch.start()
        self.addCleanup(self.inflation_patch.stop)
        self.publish_patch = patch.object(RosPlanningObstacles, '_publish',
            lambda gate, cloud: gate.publisher.publish(cloud))
        self.publish_patch.start()
        self.addCleanup(self.publish_patch.stop)

    def client(self, service, name, **kwargs):
        self.node.calls.append(name)
        def call(request):
            result = Future()
            if name.endswith('/get_parameters'):
                result.set_result(SimpleNamespace(values=self.values))
            else:
                self.node.clears.append(request.reset_distance)
                result.set_result(SimpleNamespace())
            return result
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    def test_only_global_costmap_is_used(self):
        gate = RosPlanningObstacles(self.node, self.tf)
        gate.receive(SimpleNamespace())
        gate.refresh()
        self.assertTrue(gate.state()['enabled'])
        self.assertEqual(self.node.calls, ['/global_costmap/global_costmap/get_parameters',
                                          '/global_costmap/clear_except_global_costmap'])

    def test_wrong_source_is_not_claimed_as_switchable(self):
        self.values[3].string_value = '/cloud_registered_body_1'
        gate = RosPlanningObstacles(self.node, self.tf)
        gate.receive(SimpleNamespace())
        gate.refresh()
        self.assertFalse(gate.state()['available'])

    def test_humble_uses_plugin_clearability(self):
        self.values[5] = SimpleNamespace(type=0)
        with patch.dict('os.environ', {'ROS_DISTRO': 'humble'}):
            gate = RosPlanningObstacles(self.node, self.tf)
            gate.receive(SimpleNamespace())
            gate.refresh()
        self.assertTrue(gate.state()['available'])

    def test_static_layer_is_not_clearable(self):
        self.values[5].string_array_value.append('static_layer')
        gate = RosPlanningObstacles(self.node, self.tf)
        gate.receive(SimpleNamespace())
        with self.assertRaises(RuntimeError):
            gate.set_enabled(False)
        self.assertEqual(self.node.clears, [])

    def test_empty_observation_does_not_mutate_source_cloud(self):
        cloud = SimpleNamespace(width=2, height=1, row_step=24, data=b'x'*24,
                                header=SimpleNamespace(frame_id='body', stamp=42), fields=['x', 'y', 'z'])
        empty = RosPlanningObstacles._empty(cloud)
        self.assertEqual((cloud.width, cloud.data), (2, b'x'*24))
        self.assertEqual((empty.width, empty.height, empty.data), (0, 1, b''))
        self.assertEqual((empty.header, empty.fields), (cloud.header, cloud.fields))

    def test_clear_waits_for_updated_global_costmap(self):
        gate = RosPlanningObstacles(self.node, self.tf)
        timer = threading.Timer(.01, lambda: gate.on_costmap(SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))))
        timer.start()
        gate._clear()
        gate._updated()
        timer.join()
        self.assertEqual(self.node.clears, [0.0])

    def test_both_switch_directions_wait_for_costmap_confirmation(self):
        gate = RosPlanningObstacles(self.node, self.tf)
        gate.validate = lambda: None
        gate.receive(SimpleNamespace())
        updates = []
        gate.updated = lambda: updates.append(gate.enabled)
        stop = threading.Event()
        worker = threading.Thread(target=lambda: self.produce(gate, stop))
        worker.start()
        try:
            gate.set_enabled(False)
            gate.set_enabled(True)
        finally:
            stop.set()
            worker.join()
        self.assertEqual(updates, [False, True])

    @staticmethod
    def produce(gate, stop):
        while not stop.wait(.005):
            gate.receive(SimpleNamespace())


class WebClientTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot/web'))
        from obstacle_control import PlanningObstacleClient
        self.client_type = PlanningObstacleClient
        self.backend = {'available': True, 'enabled': False, 'busy': False,
                        'revision': 7, 'session': 'existing-nav2-session', 'message': '',
                        'thresholds': dict(DEFAULT_THRESHOLDS), 'cloud': None}
        self.set_calls = []
        self.threshold_calls = []
        services = ModuleType('std_srvs.srv')
        services.SetBool = services.Trigger = SimpleNamespace(Request=SimpleNamespace)
        groups = ModuleType('rclpy.callback_groups')
        groups.ReentrantCallbackGroup = object
        interfaces = ModuleType('rcl_interfaces.srv')
        interfaces.SetParametersAtomically = SimpleNamespace(Request=SimpleNamespace)
        messages = ModuleType('rcl_interfaces.msg')
        messages.Parameter = messages.ParameterValue = SimpleNamespace
        messages.ParameterType = SimpleNamespace(PARAMETER_DOUBLE=3, PARAMETER_INTEGER=2)
        self.patch = patch.dict(sys.modules, {'std_srvs.srv': services, 'rclpy.callback_groups': groups,
                                             'rcl_interfaces.srv': interfaces, 'rcl_interfaces.msg': messages})
        self.patch.start()
        self.addCleanup(self.patch.stop)
        def create_client(service, name, **kwargs):
            def call(request):
                import json
                if name.endswith('/set_enabled'):
                    self.set_calls.append(request.data)
                    self.backend['enabled'] = request.data
                    self.backend['revision'] += 1
                result = Future()
                if name.endswith('/set_thresholds'):
                    values = {p.name: p.value.integer_value if p.name == 'min_points' else p.value.double_value for p in request.parameters}
                    self.threshold_calls.append(values)
                    self.backend['thresholds'] = values
                    self.backend['revision'] += 1
                    result.set_result(SimpleNamespace(result=SimpleNamespace(successful=True, reason=json.dumps(self.backend))))
                else:
                    result.set_result(SimpleNamespace(success=True, message=json.dumps(self.backend)))
                return result
            return SimpleNamespace(service_is_ready=lambda: True, call_async=call)
        # Deliberately provide no publisher/subscription API to the Web client.
        self.node = SimpleNamespace(create_client=create_client, create_timer=lambda *args: None)

    def test_web_restart_reads_existing_off_state_without_resetting_node(self):
        for _ in range(2):
            client = self.client_type(self.node)
            self.assertFalse(client.refresh()['enabled'])
        self.assertEqual(self.set_calls, [])
        self.assertEqual(self.backend['revision'], 7)

    def test_explicit_setting_uses_ros_service(self):
        client = self.client_type(self.node)
        self.assertTrue(client.set_enabled(True)['enabled'])
        self.assertEqual(self.set_calls, [True])

    def test_invalid_boolean_cannot_reach_ros(self):
        client = self.client_type(self.node)
        with self.assertRaises(ValueError):
            client.set_enabled('false')
        self.assertEqual(self.set_calls, [])

    def test_thresholds_use_atomic_service_and_survive_new_web_client(self):
        values = dict(DEFAULT_THRESHOLDS, min_points=4, neighbor_radius=.2)
        first = self.client_type(self.node)
        self.assertEqual(first.configure(values)['thresholds'], values)
        second = self.client_type(self.node)
        self.assertEqual(second.refresh()['thresholds'], values)
        self.assertEqual(self.threshold_calls, [values])

    def test_invalid_threshold_request_never_reaches_ros(self):
        client = self.client_type(self.node)
        with self.assertRaises(ValueError):
            client.configure(dict(DEFAULT_THRESHOLDS, min_points='3'))
        self.assertEqual(self.threshold_calls, [])

    def test_late_read_cannot_restore_old_cached_state(self):
        import json
        client = self.client_type(self.node)
        old = Future()
        started = threading.Event()
        def delayed(request):
            started.set()
            return old
        client.query.call_async = delayed
        worker = threading.Thread(target=client.refresh)
        worker.start()
        self.assertTrue(started.wait(1))
        newer = dict(self.backend, enabled=True, revision=8)
        current = Future()
        current.set_result(SimpleNamespace(success=True, message=json.dumps(newer)))
        client.query.call_async = lambda request: current
        client.refresh()
        old.set_result(SimpleNamespace(success=True, message=json.dumps(self.backend)))
        worker.join(1)
        self.assertTrue(client.state()['enabled'])
        self.assertEqual(client.state()['revision'], 8)


if __name__ == '__main__':
    unittest.main()
