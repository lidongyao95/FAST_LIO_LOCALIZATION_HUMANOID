#!/usr/bin/env python3
"""Exercise real/simulation launch composition without starting ROS (needs PyYAML)."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[2]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Node(Action):
    pass


class ExecuteProcess(Action):
    pass


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class ArchitectureTests(unittest.TestCase):
    def setUp(self):
        modules = {name: ModuleType(name) for name in (
            'launch', 'launch.actions', 'launch.conditions', 'launch.event_handlers',
            'launch.events', 'launch.substitutions', 'launch_ros',
            'launch_ros.actions', 'launch_ros.substitutions')}
        modules['launch'].LaunchDescription = Action
        for name in ('DeclareLaunchArgument', 'LogInfo', 'OpaqueFunction', 'RegisterEventHandler', 'TimerAction'):
            setattr(modules['launch.actions'], name, Action)
        modules['launch.actions'].ExecuteProcess = ExecuteProcess
        modules['launch.conditions'].IfCondition = Action
        modules['launch.event_handlers'].OnProcessExit = modules['launch.event_handlers'].OnShutdown = Action
        modules['launch.events'].Shutdown = Action
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch.substitutions'].PathJoinSubstitution = Action
        modules['launch_ros.actions'].Node = Node
        modules['launch_ros.substitutions'].FindPackageShare = Action
        stub = patch.dict(sys.modules, modules)
        stub.start()
        self.addCleanup(stub.stop)
        spec = importlib.util.spec_from_file_location('g1_navigation_under_test',
            ROOT / 'shared/nav/navigation.launch.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        original = tempfile.NamedTemporaryFile
        files = patch.object(self.module.tempfile, 'NamedTemporaryFile',
            side_effect=lambda *a, **kw: original(*a, **kw, dir=self.tmp.name))
        files.start()
        self.addCleanup(files.stop)
        self.run = patch.object(self.module.subprocess, 'run').start()
        self.addCleanup(patch.stopall)
        self.context = dict(repo=str(ROOT), mode='plan', use_sim_time='false',
            map_yaml='/field/map.yaml', params_file=str(ROOT / 'shared/nav/params.yaml'),
            robot_frame='body', rviz='false', profile='')

    def launch(self, **values):
        return self.module.launch_setup({**self.context, **values})

    @staticmethod
    def nodes(actions):
        return [a.kwargs['name'] for a in actions if isinstance(a, Node)]

    def test_real_session_has_planner_and_its_own_obstacle_input(self):
        actions = self.launch()
        self.assertEqual(self.nodes(actions), ['map_server', 'planner_server', 'g1_navigation_rviz'])
        commands = [a.kwargs['cmd'] for a in actions if isinstance(a, ExecuteProcess)]
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0][1].endswith('shared/nav/planning_obstacles.py'))
        self.assertIn('use_sim_time:=false', commands[0])
        self.assertTrue(any(a.args and isinstance(a.args[0], Action)
                            and a.args[0].kwargs.get('target_action') is next(
                                a for a in actions if isinstance(a, ExecuteProcess)) for a in actions))

    def test_simulation_keeps_full_control_stack_and_direct_input(self):
        actions = self.launch(mode='full', robot_frame='', map_yaml='', use_sim_time='')
        self.assertEqual(self.nodes(actions), ['map_server', 'controller_server', 'planner_server',
            'behavior_server', 'bt_navigator', 'g1_navigation_rviz'])
        self.assertFalse(any(isinstance(a, ExecuteProcess) for a in actions))
        params = yaml.safe_load(Path(next(a for a in actions if isinstance(a, Node)).kwargs['parameters'][0]).read_text())
        self.assertEqual(params['global_costmap']['global_costmap']['ros__parameters']
            ['obstacle_layer']['lidar']['topic'], '/cloud_registered_body_1')
        self.assertTrue(params['planner_server']['ros__parameters']['use_sim_time'])

    def test_plan_rewrites_only_global_obstacles_and_preserves_source_file(self):
        source = ROOT / 'shared/nav/params.yaml'
        before = source.read_bytes()
        original = yaml.safe_load(before)
        rewritten = yaml.safe_load(Path(self.module.rewrite_params(source, False, '/field/map.yaml',
            'body', planning_obstacles=True)).read_text())
        costmap = rewritten['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(costmap['robot_base_frame'], 'body')
        self.assertEqual(costmap['obstacle_layer']['lidar']['topic'], '/g1/planning_obstacles/cloud')
        self.assertEqual(costmap['clearable_layers'], ['obstacle_layer'])
        self.assertEqual(costmap['obstacle_layer']['observation_sources'], 'lidar clearing')
        self.assertFalse(costmap['obstacle_layer']['lidar']['clearing'])
        self.assertFalse(costmap['obstacle_layer']['clearing']['marking'])
        self.assertEqual(costmap['obstacle_layer']['clearing']['topic'], '/g1/planning_obstacles/clearing_cloud')
        self.assertEqual(rewritten['planner_server']['ros__parameters']['planner_plugins'], ['G1GridBased'])
        self.assertEqual(rewritten['planner_server']['ros__parameters']['G1GridBased'],
                         original['planner_server']['ros__parameters']['GridBased'])
        self.assertEqual(rewritten['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer'],
                         original['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer'])
        self.assertEqual(rewritten['map_server']['ros__parameters']['yaml_filename'], '/field/map.yaml')
        self.assertEqual(source.read_bytes(), before)

    def test_full_profile_replaces_the_controller_and_footprint(self):
        source = ROOT / 'shared/nav/params.yaml'
        before = source.read_bytes()
        rewritten = yaml.safe_load(Path(self.module.rewrite_params(
            source, True, '/sim/map.yaml', profile=ROOT / 'shared/nav/full.yaml')).read_text())
        controller = rewritten['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(controller['plugin'], 'nav2_mppi_controller::MPPIController')
        self.assertEqual(controller['motion_model'], 'Omni')
        # Nothing of the replaced DWB section may linger under the new plugin.
        self.assertNotIn('max_vel_x', controller)
        for name in ('local_costmap', 'global_costmap'):
            costmap = rewritten[name][name]['ros__parameters']
            self.assertNotIn('footprint', costmap)
            self.assertEqual(costmap['robot_radius'], 0.30)
            self.assertIn('obstacle_layer', costmap)
        self.assertEqual(source.read_bytes(), before)

    def test_unknown_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            self.launch(mode='full', use_sim_time='true', robot_frame='', profile='missing')

    def test_both_modes_run_process_and_ros_graph_preflight_before_starting(self):
        for mode in ('plan', 'full'):
            self.run.reset_mock()
            self.launch(mode=mode, robot_frame='')
            commands = [call.args[0] for call in self.run.call_args_list]
            self.assertEqual(len(commands), 2)
            self.assertTrue(commands[0][1].endswith('processes.py'))
            self.assertIn('--check', commands[0])
            self.assertIn('navigation', commands[0])
            self.assertIn('--wait-for-empty-graph', commands[1])

    def test_preflight_failure_stops_launch_setup(self):
        self.run.side_effect = subprocess.CalledProcessError(3, ['preflight'])
        with self.assertRaises(subprocess.CalledProcessError):
            self.launch()
        self.assertEqual(list(Path(self.tmp.name).iterdir()), [])

    def test_invalid_mode_or_implicit_real_map_and_clock_are_rejected(self):
        for values in ({'mode': 'removed'}, {'map_yaml': ''}, {'use_sim_time': ''}):
            with self.assertRaises(ValueError):
                self.launch(**values)
        self.run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
