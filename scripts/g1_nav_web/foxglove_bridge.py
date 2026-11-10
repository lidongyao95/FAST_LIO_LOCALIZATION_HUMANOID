#!/usr/bin/env python3
"""Expose selected G1 ROS 2 visualization topics through Foxglove WebSocket."""
import argparse
import asyncio
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from foxglove_websocket.server import FoxgloveServer
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField


def schema(name):
    definitions = {}

    def read(qualified):
        if qualified in definitions:
            return
        package, _, kind = qualified.split('/')
        source = Path(get_package_share_directory(package)) / 'msg' / (kind + '.msg')
        definitions[qualified] = source.read_text().strip()
        for line in definitions[qualified].splitlines():
            fields = line.partition('#')[0].split()
            if len(fields) < 2:
                continue
            field = fields[0].partition('[')[0]
            if '/' in field:
                dep_package, dep_name = field.split('/')
                read(f'{dep_package}/msg/{dep_name}')
            elif field == 'Header':
                read('std_msgs/msg/Header')

    read(name)
    return definitions.pop(name) + ''.join(
        f'\n{"=" * 80}\nMSG: {kind}\n{body}' for kind, body in definitions.items()
    )


def read_cloud(path):
    with path.open('rb') as stream:
        header = {}
        while 'DATA' not in header:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header ended before DATA')
            key, _, value = line.decode().strip().partition(' ')
            header[key] = value
        if (header['DATA'], header['FIELDS'], header['SIZE'], header['TYPE']) != (
            'binary', 'x y z', '4 4 4', 'F F F'
        ):
            raise ValueError('Expected a binary XYZ float32 PCD')
        count = int(header['POINTS'])
        payload = stream.read()
    if len(payload) != count * 12:
        raise ValueError('PCD size does not match POINTS')
    cloud = PointCloud2(width=count, height=1, point_step=12, row_step=count * 12,
                        is_dense=True, data=payload)
    cloud.header.frame_id = 'map'
    cloud.fields = [PointField(name=axis, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, axis in enumerate('xyz')]
    return serialize_message(cloud)


async def run(node, args):
    static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = {
        '/map': ('nav_msgs/msg/OccupancyGrid', static),
        '/global_costmap/costmap': ('nav_msgs/msg/OccupancyGrid', static),
        '/tf_static': ('tf2_msgs/msg/TFMessage', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        '/tf': ('tf2_msgs/msg/TFMessage', qos_profile_sensor_data),
        '/plan': ('nav_msgs/msg/Path', qos_profile_sensor_data),
        '/localization/scan_cloud': ('sensor_msgs/msg/PointCloud2', qos_profile_sensor_data),
    }
    cached = {}
    tf_static = {}
    async with FoxgloveServer('127.0.0.1', args.port, args.name) as server:
        async def channel(topic, kind):
            return await server.add_channel({
                'topic': topic, 'encoding': 'cdr', 'schemaName': kind,
                'schemaEncoding': 'ros2msg', 'schema': schema(kind),
            })

        def callback(topic, channel_id):
            def receive(message):
                if topic == '/tf_static':
                    for transform in message.transforms:
                        tf_static[transform.child_frame_id] = transform
                    message.transforms = list(tf_static.values())
                    message = serialize_message(message)
                cached[channel_id] = message
            return receive

        for topic, (kind, qos) in topics.items():
            chan = await channel(topic, kind)
            node.create_subscription(get_message(kind), topic, callback(topic, chan), qos,
                                     raw=topic != '/tf_static')

        if args.cloud:
            cloud = await channel('/plaster/clean_cloud', 'sensor_msgs/msg/PointCloud2')
            cached[cloud] = read_cloud(args.cloud)

        sent = {}
        last_repeat = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            repeat = now - last_repeat >= 3
            for chan, data in cached.items():
                if repeat or sent.get(chan) is not data:
                    await server.send_message(chan, time.time_ns(), data)
                    sent[chan] = data
            if repeat:
                last_repeat = now
            await asyncio.sleep(0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--cloud', type=Path)
    parser.add_argument('--name', default='PC2 G1')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('g1_foxglove_visualization')
    try:
        asyncio.run(run(node, args))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
