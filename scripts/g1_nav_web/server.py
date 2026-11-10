#!/usr/bin/env python3
"""Loopback HTTP interface to G1 map, TF and ComputePathToPose."""
import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))


class Planner(Node):
    def __init__(self, robot_frame='base_footprint'):
        super().__init__('g1_nav_web')
        self.robot_frame = robot_frame
        self.grid = None
        self.costmap = None
        self.obstacles = None
        self.last_obstacles = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.action = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.create_subscription(
            Costmap, '/global_costmap/costmap_raw', self.on_costmap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.create_subscription(
            PointCloud2, '/cloud_registered_body_1', self.on_obstacles, qos_profile_sensor_data,
        )

    def on_obstacles(self, msg):
        now = time.monotonic()
        if not self.grid or now - self.last_obstacles < 0.5:
            return
        try:
            tf = self.buffer.lookup_transform(
                self.grid['frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp),
            ).transform
        except TransformException:
            return

        if not msg.width or not msg.height:
            return
        fields = {field.name: field for field in msg.fields}
        if any(name not in fields or fields[name].datatype != 7 for name in ('x', 'y', 'z')):
            return
        dtype = '>f4' if msg.is_bigendian else '<f4'
        points = np.column_stack([
            np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       offset=fields[name].offset, strides=(msg.row_step, msg.point_step)).ravel()
            for name in ('x', 'y', 'z')
        ])
        radius_sq = np.sum(points.astype(float) ** 2, axis=1)
        points = points[(radius_sq >= 0.4 ** 2) & (radius_sq <= 8.0 ** 2)]
        q, t = tf.rotation, tf.translation
        axis = np.array([q.x, q.y, q.z])
        cross = np.cross(axis, points)
        points = points + 2 * (q.w * cross + np.cross(axis, cross)) + [t.x, t.y, t.z]
        xy = points[(points[:, 2] >= 0.15) & (points[:, 2] <= 1.8), :2]
        _, indices = np.unique(np.floor(xy / 0.1), axis=0, return_index=True)
        self.obstacles = {'frame': self.grid['frame'], 'points': xy[indices].round(3).tolist()}
        self.last_obstacles = now

    def live_obstacles(self):
        if time.monotonic() - self.last_obstacles <= 2:
            return self.obstacles
        return None

    def on_costmap(self, msg):
        info = msg.metadata
        origin = info.origin
        self.costmap = {
            'frame': msg.header.frame_id, 'width': info.size_x, 'height': info.size_y,
            'resolution': info.resolution, 'x': origin.position.x, 'y': origin.position.y,
            'yaw': yaw(origin.orientation), 'data': list(msg.data),
            'revision': time.monotonic_ns(),
        }

    def on_map(self, msg):
        info = msg.info
        origin = info.origin
        self.grid = {
            'frame': msg.header.frame_id, 'width': info.width, 'height': info.height,
            'resolution': info.resolution, 'x': origin.position.x, 'y': origin.position.y,
            'yaw': yaw(origin.orientation), 'data': list(msg.data),
        }

    def pose(self):
        if not self.grid:
            return None
        try:
            tf = self.buffer.lookup_transform(self.grid['frame'], self.robot_frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return [t.x, t.y, yaw(q)]


def wait_for(future, seconds):
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(seconds):
        raise TimeoutError('规划器响应超时')
    return future.result()


def plan(node, data):
    grid = node.grid
    if not grid or node.pose() is None or not node.action.server_is_ready():
        raise ValueError('等待地图、定位 TF 和规划器就绪')
    x, y, heading = (float(data[key]) for key in ('x', 'y', 'yaw'))
    if not all(math.isfinite(n) for n in (x, y, heading)):
        raise ValueError('目标坐标无效')
    dx, dy = x - grid['x'], y - grid['y']
    c, s = math.cos(grid['yaw']), math.sin(grid['yaw'])
    u = (c * dx + s * dy) / grid['resolution']
    v = (-s * dx + c * dy) / grid['resolution']
    if not (0 <= u < grid['width'] and 0 <= v < grid['height']):
        raise ValueError('目标位于地图之外')

    goal = ComputePathToPose.Goal()
    goal.pose.header.frame_id = grid['frame']
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(heading / 2)
    goal.pose.pose.orientation.w = math.cos(heading / 2)
    if hasattr(goal, 'use_start'):
        goal.use_start = False
    handle = wait_for(node.action.send_goal_async(goal), 3)
    if not handle.accepted:
        raise ValueError('规划器拒绝目标')
    try:
        response = wait_for(handle.get_result_async(), 12)
    except TimeoutError:
        handle.cancel_goal_async()
        raise
    path = response.result.path
    if response.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
        raise ValueError(f'未返回可行路径（Nav2 状态 {response.status}）')
    if path.header.frame_id != grid['frame']:
        raise ValueError('路径与地图坐标系不一致')
    return [[p.pose.position.x, p.pose.position.y] for p in path.poses]


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def reply(self, data, status=200, kind='application/json'):
        body = data if isinstance(data, bytes) else json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def local_host(self):
        return self.headers.get('Host') in self.server.allowed_hosts

    def do_GET(self):
        if not self.local_host():
            return self.reply({'error': 'Invalid host'}, 403)
        route = urlsplit(self.path).path
        node = self.server.node
        if route == '/':
            return self.reply(self.server.page, kind='text/html; charset=utf-8')
        if route == '/api/state':
            return self.reply({'pose': node.pose(), 'planner': node.action.server_is_ready(),
                               'map': node.grid is not None, 'demo': self.server.demo,
                               'costmap': node.costmap and node.costmap['revision']})
        if route == '/api/obstacles':
            return self.reply(node.live_obstacles())
        if route == '/api/costmap':
            return self.reply(node.costmap)
        if route == '/api/map':
            return self.reply(node.grid)
        self.reply({'error': 'Unknown endpoint'}, 404)

    def do_POST(self):
        host = self.headers.get('Host')
        if not self.local_host() or self.headers.get('Origin') not in (None, f'http://{host}'):
            return self.reply({'error': 'Invalid origin'}, 403)
        if self.path != '/api/plan' or self.headers.get('Content-Type') != 'application/json':
            return self.reply({'error': 'Invalid endpoint or content type'}, 400)
        if not self.server.plan_lock.acquire(blocking=False):
            return self.reply({'error': '正在规划，请稍候'}, 409)
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024:
                raise ValueError('目标请求长度无效')
            data = json.loads(self.rfile.read(size))
            self.reply({'path': plan(self.server.node, data)})
        except (ValueError, TypeError, KeyError) as exc:
            self.reply({'error': str(exc)}, 422)
        except TimeoutError as exc:
            self.reply({'error': str(exc)}, 504)
        finally:
            self.server.plan_lock.release()

    def log_message(self, fmt, *args):
        if self.command == 'POST':
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=18081)
    parser.add_argument('--demo', nargs='?', const='测试地图', help='Label isolated demo data')
    parser.add_argument('--robot-frame', default='base_footprint', help='TF frame used for the robot marker')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Planner(args.robot_frame)
    with ThreadingHTTPServer(('127.0.0.1', args.port), Handler) as server:
        server.node = node
        server.demo = args.demo
        server.plan_lock = threading.Lock()
        server.page = Path(__file__).with_name('index.html').read_bytes()
        server.allowed_hosts = {f'{host}:{args.port}' for host in ('127.0.0.1', 'localhost')}
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f'G1 planning webpage listening on 127.0.0.1:{args.port}')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            server.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
