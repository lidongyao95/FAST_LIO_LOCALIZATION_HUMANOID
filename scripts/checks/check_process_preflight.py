#!/usr/bin/env python3
"""Exercise process selection and cleanup with a synthetic /proc tree."""
import importlib.util
import os
from pathlib import Path
import signal
import sys
import tempfile
import unittest

path = Path(__file__).resolve().parents[1] / 'env/g1_processes.py'
spec = importlib.util.spec_from_file_location('g1_processes', path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.proc = self.root / 'proc'
        self.proc.mkdir()
        self.repo = self.root / 'g1'
        self.repo.mkdir()

    def process(self, pid, program, domain=223, cwd=None, start=50, parent=1):
        p = self.proc / str(pid)
        p.mkdir(exist_ok=True)
        (p / 'cmdline').write_bytes(b'\0'.join(os.fsencode(x) for x in program) + b'\0')
        (p / 'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0'.encode())
        # /proc/stat fields after comm: state, ppid ... starttime(field 22).
        fields = ['S', str(parent)] + ['0'] * 17 + [str(start)]
        (p / 'stat').write_text(f'{pid} (test command) ' + ' '.join(fields))
        target = cwd or self.repo
        if (p / 'cwd').is_symlink():
            (p / 'cwd').unlink()
        (p / 'cwd').symlink_to(target, target_is_directory=True)
        return p

    def scan(self, scope='navigation'):
        return module.scan(self.proc, self.repo, 223, scope, excluded=set())

    def test_navigation_preserves_lidar_lio_and_other_domains(self):
        self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'])
        self.process(101, ['/opt/ros/foxy/lib/fast_lio/fastlio_mapping'])
        self.process(102, ['/opt/ros/foxy/lib/livox_ros_driver2/livox_ros_driver2_node'])
        self.process(103, ['/opt/ros/foxy/lib/nav2_planner/planner_server'], domain=22)
        self.assertEqual([p.pid for p in self.scan()], [100])

    def test_scope_and_repository_are_both_required(self):
        other = self.root / 'g1-other'
        other.mkdir()
        self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'], cwd=other)
        self.process(101, ['/usr/bin/python3', str(self.repo / 'scripts/real_robot/web/server.py')], cwd=other)
        self.process(102, ['/usr/bin/python3', '-c', 'print("planner_server")'])
        self.assertEqual([p.pid for p in self.scan()], [101])

    def test_localization_preserves_running_fast_lio(self):
        self.process(100, ['/repo/install/open3d_loc/lib/open3d_loc/global_localization_node'])
        self.process(101, ['/repo/install/fast_lio/lib/fast_lio/fastlio_mapping'])
        self.process(102, ['/opt/ros/foxy/lib/tf2_ros/static_transform_publisher',
                           '0', '0', '0', '0', '0', '0', 'map', 'camera_init'])
        self.assertEqual([p.pid for p in self.scan('localization')], [100, 102])

    def test_pid_reuse_is_never_signalled(self):
        p = self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'])
        targets = self.scan()
        self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'], start=999)
        sent = []
        self.assertEqual(module.stop(targets, self.proc, send=lambda *a: sent.append(a), grace=0), [])
        self.assertEqual(sent, [])

    def test_changed_executable_is_never_signalled(self):
        self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'])
        targets = self.scan()
        self.process(100, ['/usr/bin/python3', '/other/job.py'])
        sent = []
        self.assertEqual(module.stop(targets, self.proc, send=lambda *args: sent.append(args), grace=0), [])
        self.assertEqual(sent, [])

    def test_explicit_stop_only_signals_selected_pids(self):
        self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'])
        self.process(101, ['/opt/ros/foxy/lib/fast_lio/fastlio_mapping'])
        sent = []
        def send(pid, sig):
            sent.append((pid, sig))
            (self.proc / str(pid) / 'stat').unlink()
        self.assertEqual(module.stop(self.scan(), self.proc, send=send, grace=0), [])
        self.assertEqual(sent, [(100, signal.SIGINT)])
        self.assertTrue((self.proc / '101/stat').exists())

    def test_stubborn_process_is_reported_without_sigkill(self):
        self.process(100, ['/opt/ros/foxy/lib/nav2_planner/planner_server'])
        sent = []
        left = module.stop(self.scan(), self.proc, send=lambda *args: sent.append(args), grace=0)
        self.assertEqual([p.pid for p in left], [100])
        self.assertEqual(sent, [(100, signal.SIGINT), (100, signal.SIGTERM)])

    def test_current_callers_can_be_excluded(self):
        self.process(100, ['/usr/bin/ros2', 'launch', str(self.repo / 'open3d_loc/launch/g1_navigation.launch.py')])
        self.assertEqual(module.scan(self.proc, self.repo, 223, 'navigation', excluded={100}), [])


if __name__ == '__main__':
    unittest.main()
