#!/usr/bin/env python3
"""List, check or explicitly stop this checkout's processes in one ROS domain."""
import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import sys
import time


@dataclass
class Process:
    pid: int
    start: str
    scope: str
    argv: list


def identity(directory):
    try:
        text = (directory / 'stat').read_text()
        fields = text[text.rfind(')') + 2:].split()
        return fields[19] if fields[0] not in ('Z', 'X') else None
    except (OSError, IndexError):
        return None


def category(argv):
    if not argv:
        return None
    names = [Path(arg).name for arg in argv]
    subject = names[1] if names[0].startswith('python') and len(names) > 1 else names[0]
    if subject == 'ros2' and 'run' in argv:
        index = argv.index('run')
        if len(argv) > index + 2:
            subject = argv[index + 2]
    if subject in ('map_server', 'planner_server', 'controller_server', 'bt_navigator',
                   'behavior_server', 'recoveries_server', 'velocity_smoother', 'planning_obstacles.py'):
        return 'navigation'
    if 'g1_navigation.launch.py' in names or any(
            arg.endswith(('scripts/real_robot/web/server.py', 'scripts/g1_nav_web/server.py')) for arg in argv):
        return 'navigation'
    # Retired bridges may still be running from an older deployment.
    if subject.startswith('g1_') and subject.endswith('_bridge.py'):
        return 'simulation' if subject == 'g1_command_bridge.py' else 'navigation'
    if subject in ('global_localization_node', 'global_initialize', 'capture_global_scans.py') or 'g1_walking_localization.launch.py' in names:
        return 'localization'
    if subject == 'static_transform_publisher':
        aliases = ('odom_to_camera_init', 'base_link_to_imu_link', 'base_link_to_motion_link')
        if ('map' in argv and 'camera_init' in argv) or any(
                arg.split(':=')[-1] in aliases for arg in argv):
            return 'localization'
    if subject == 'fastlio_mapping' or 'g1_walking_mapping.launch.py' in names or subject == 'record_g1_a_segment.sh':
        return 'mapping'
    if subject == 'livox_ros_driver2_node':
        return 'sensors'
    if subject in ('g1_ctrl', 'g1_walking_sim.py', 'g1_mujoco_sensors.py', 'g1_nav_tf.py') or 'g1_mujoco.launch.py' in names:
        return 'simulation'
    if subject == 'rviz2' and 'g1_navigation.rviz' in names:
        return 'navigation'
    return None


def under(path, root):
    try:
        Path(path).resolve().relative_to(root)
        return True
    except (ValueError, OSError):
        return False


def ancestors(proc_root):
    result = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        try:
            text = (proc_root / str(pid) / 'stat').read_text()
            pid = int(text[text.rfind(')') + 2:].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return result


def scan(proc_root, repo, domain, scope, excluded=None):
    repo = Path(repo).resolve()
    excluded = ancestors(proc_root) if excluded is None else excluded
    matches = []
    for directory in proc_root.glob('[0-9]*'):
        pid = int(directory.name)
        try:
            if pid in excluded or directory.stat().st_uid != os.getuid():
                continue
            argv = [os.fsdecode(arg) for arg in (directory / 'cmdline').read_bytes().split(b'\0') if arg]
            kind = category(argv)
            if not kind or scope not in ('all', kind):
                continue
            cwd = (directory / 'cwd').resolve()
            if not (under(cwd, repo) or any(arg.startswith('/') and under(arg, repo) for arg in argv)):
                continue
            env = dict(item.split(b'=', 1) for item in (directory / 'environ').read_bytes().split(b'\0') if b'=' in item)
            if int(env.get(b'ROS_DOMAIN_ID') or b'0') != domain:
                continue
            start = identity(directory)
            if start is not None:
                matches.append(Process(pid, start, kind, argv))
        except (OSError, ValueError):
            continue
    return sorted(matches, key=lambda process: process.pid)


def stop(targets, proc_root, send=os.kill, grace=3.0):
    def alive(process):
        directory = proc_root / str(process.pid)
        try:
            argv = [os.fsdecode(arg) for arg in (directory / 'cmdline').read_bytes().split(b'\0') if arg]
            return identity(directory) == process.start and argv == process.argv
        except OSError:
            return False
    for sig in (signal.SIGINT, signal.SIGTERM):
        for process in targets:
            if alive(process):
                try:
                    send(process.pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + grace
        while any(alive(process) for process in targets) and time.monotonic() < deadline:
            time.sleep(.1)
    return [process for process in targets if alive(process)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--list', action='store_true', help='List only (the default); do not stop anything')
    action.add_argument('--check', action='store_true', help='List conflicts and exit 3 if any exist')
    action.add_argument('--stop', action='store_true', help='Explicitly stop selected processes using INT then TERM')
    parser.add_argument('--scope', choices=('all', 'navigation', 'localization', 'mapping', 'sensors', 'simulation'), default='all')
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--domain', type=int, default=os.environ.get('ROS_DOMAIN_ID', '72'))
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('ROS domain must be between 0 and 232')
    proc_root = Path('/proc')
    if not proc_root.is_dir():
        parser.error('进程预检需要 Linux /proc；请在运行 ROS 的主机上执行')
    targets = scan(proc_root, args.repo, args.domain, args.scope)
    print(f'ROS_DOMAIN_ID={args.domain}，范围={args.scope}，仓库={args.repo.resolve()}')
    for process in targets:
        print(f'  PID {process.pid} [{process.scope}] {" ".join(process.argv)}')
    if not targets:
        print('没有本仓库的冲突进程')
        return 0
    if args.stop:
        left = stop(targets, proc_root)
        # A launch process may have created another child while stopping.
        remaining = scan(proc_root, args.repo, args.domain, args.scope)
        if left or remaining:
            print('仍有进程，请检查原启动终端；未发送 SIGKILL：', [p.pid for p in remaining or left])
            return 1
        print('所列进程已退出')
        return 0
    print(f'停止旧会话：scripts/env/g1_cleanup_stale.sh --stop --scope {args.scope} --domain {args.domain}')
    return 3 if args.check else 0


if __name__ == '__main__':
    sys.exit(main())
