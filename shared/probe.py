#!/usr/bin/env python3
"""Check the live ROS graph and received data without publishing commands."""

import argparse
import math
import os
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("check", choices=("free", "lidar", "lio", "tf", "localized", "nav", "clock"))
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--frame", default="body")
    parser.add_argument("--target", default="map")
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--node", action="append", default=[])
    parser.add_argument("--lidar-topic", default="")
    args = parser.parse_args()

    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=[])
    node = rclpy.create_node(f"g1_workflow_probe_{os.getpid()}")
    check = args.check
    seen = {}
    received = {}

    def watch(topic, msg_type, valid=lambda msg: True, qos=qos_profile_sensor_data):
        def callback(msg):
            if valid(msg):
                received[topic] = msg
                seen[topic] = time.monotonic()

        node.create_subscription(msg_type, topic, callback, qos)

    def fresh(topic):
        return time.monotonic() - seen.get(topic, -math.inf) < 2.5

    def transform():
        if buffer is None:
            return None
        try:
            return buffer.lookup_transform(args.target, args.frame, Time())
        except TransformException:
            return None

    buffer = None
    lidar_topics = list(dict.fromkeys(filter(None, (args.lidar_topic, "/livox/lidar", "/livox/custom_msg"))))
    clients = {}
    pending = {}
    states = {}
    if check == "lidar":
        from livox_ros_driver2.msg import CustomMsg

        for topic in lidar_topics:
            watch(topic, CustomMsg, lambda msg: msg.point_num > 0)
        watch("/livox/imu", Imu)
    if check == "lio":
        # FAST-LIO publishes both outputs RELIABLE; a best-effort sensor profile
        # drops the point cloud here and the readiness gate flaps.
        outputs = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        watch("/Odometry_loc", Odometry, lambda msg: msg.header.frame_id == "camera_init" and msg.child_frame_id == "body", outputs)
        watch("/cloud_registered_body_1", PointCloud2, lambda msg: msg.width > 0 and msg.header.frame_id == "body", outputs)
    if check == "clock":
        watch("/clock", Clock, lambda msg: msg.clock.sec != 0 or msg.clock.nanosec != 0)
    if check == "localized":
        watch("/localization_3d_registration_ok", Bool)
    if check in ("tf", "localized"):
        buffer = Buffer()
        listener = TransformListener(buffer, node)
    if check == "nav":
        clients = {name: node.create_client(GetState, f"/{name}/get_state") for name in args.node}
        watch("/map", OccupancyGrid, lambda msg: msg.info.width > 0,
              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def occupied():
        names = set(node.get_node_names())
        found = [name for name in args.node if name in names]
        found.extend(topic for topic in args.topic if node.count_publishers(topic))
        return found

    duration = args.seconds if args.seconds is not None else (2 if check == "free" else 25)
    deadline = time.monotonic() + duration
    current_tf = None
    first_stamp = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
            if check == "free":
                conflicts = occupied()
                if conflicts:
                    raise RuntimeError(f"域内已有节点或发布端：{', '.join(conflicts)}")
            elif check == "lidar":
                publishers = [topic for topic in lidar_topics if node.count_publishers(topic)]
                if len(publishers) > 1 or any(node.count_publishers(topic) > 1 for topic in publishers + ["/livox/imu"]):
                    raise RuntimeError("发现多路 Livox 雷达或 IMU 发布者")
                if publishers and args.lidar_topic in ("", publishers[0]) and all(map(fresh, [*publishers, "/livox/imu"])):
                    print(publishers[0])
                    return 0
            elif check == "lio":
                topics = ("/Odometry_loc", "/cloud_registered_body_1")
                if any(node.count_publishers(topic) > 1 for topic in topics):
                    raise RuntimeError("发现重复 FAST-LIO 输出")
                if all(map(fresh, topics)):
                    print("FAST-LIO 里程计与机体系点云就绪")
                    return 0
            elif check == "clock" and fresh("/clock"):
                return 0
            elif check in ("tf", "localized"):
                current_tf = transform()
                if current_tf is None:
                    continue
                stamp = (current_tf.header.stamp.sec, current_tf.header.stamp.nanosec)
                if first_stamp is None:
                    first_stamp = stamp
                registered = check == "tf" or (fresh("/localization_3d_registration_ok") and
                                               received["/localization_3d_registration_ok"].data)
                if registered and stamp > first_stamp:
                    p = current_tf.transform.translation
                    if not all(map(math.isfinite, (p.x, p.y, p.z))):
                        raise RuntimeError("TF 位姿不是有限数")
                    print(f"{args.target} → {args.frame}: {p.x:.3f}, {p.y:.3f}, {p.z:.3f} m")
                    return 0
            elif check == "nav":
                if node.count_publishers("/map") > 1 or node.count_publishers("/global_costmap/costmap_raw") > 1:
                    raise RuntimeError("发现重复地图或全局代价地图发布者")
                for name, client in clients.items():
                    future, sent = pending.get(name, (None, 0.))
                    if future is not None and future.done():
                        result = future.result()
                        states[name] = result.current_state.id if result else -1
                        del pending[name]
                    elif future is not None and time.monotonic() - sent > 2.:
                        # A node busy configuring can drop the response; ask again.
                        client.remove_pending_request(future)
                        del pending[name]
                    if states.get(name) != 3 and name not in pending and client.service_is_ready():
                        pending[name] = (client.call_async(GetState.Request()), time.monotonic())
                if all(states.get(name) == 3 for name in clients) and "/map" in received:
                    print("Nav2 生命周期 active，已收到 /map")
                    return 0
        if check == "free":
            return 0
        if check == "lidar" and not any(node.count_publishers(t) for t in [*lidar_topics, "/livox/imu"]):
            return 3
        if check == "nav":
            print(f"生命周期状态：{states}；已收到地图：{'/map' in received}", file=sys.stderr)
        if check in ("tf", "localized"):
            print(f"{args.target} → {args.frame} 持续更新或配准健康检查未通过", file=sys.stderr)
        print(f"{check} 超时，已收到：{', '.join(received) or '无'}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        # Foxy try_shutdown only closes the context; shutdown also releases
        # the global executor before Python tears down the signal module.
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, ImportError) as error:
        sys.exit(str(error))
