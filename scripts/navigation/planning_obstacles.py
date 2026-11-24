#!/usr/bin/env python3
"""Nav2-owned real obstacle input for global planning; independent of the Web server."""
import argparse
import copy
import json
import math
import os
import threading
import time
import uuid

OBSTACLE_TOPIC = '/g1/planning_obstacles/cloud'
STATE_SERVICE = '/g1/planning_obstacles/get_state'
SET_SERVICE = '/g1/planning_obstacles/set_enabled'
THRESHOLD_SERVICE = '/g1/planning_obstacles/set_thresholds'
CLEARING_TOPIC = '/g1/planning_obstacles/clearing_cloud'
DEFAULT_THRESHOLDS = dict(neighbor_radius=0.15, min_points=1, inflation_radius=0.6)


def validate_thresholds(values):
    if not isinstance(values, dict) or set(values) != set(DEFAULT_THRESHOLDS):
        raise ValueError('阈值需要 neighbor_radius、min_points、inflation_radius 三项')
    limits = dict(neighbor_radius=(0.03, 0.5), inflation_radius=(0.0, 1.5))
    for name, (low, high) in limits.items():
        value = values[name]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} 需要 {low}–{high} 米的有限数值')
    if type(values['min_points']) is not int or not 1 <= values['min_points'] <= 50:
        raise ValueError('min_points 需要 1–50 的整数')
    return dict(neighbor_radius=float(values['neighbor_radius']), min_points=values['min_points'],
                inflation_radius=float(values['inflation_radius']))


class ObstacleGate:
    """Serialize input changes and acknowledge fresh input plus costmap clearing."""

    def __init__(self, validate, clear, publish, empty, *, updated=lambda: None, set_inflation=lambda value: None, timeout=3.0):
        self.validate = validate
        self.clear = clear
        self.publish = publish
        self.empty = empty
        self.updated = updated
        self.set_inflation = set_inflation
        self.thresholds = dict(DEFAULT_THRESHOLDS)
        self.flushing = False
        self.input_error = None
        self.recovery_error = None
        self.cloud_stats = None
        self.timeout = timeout
        self.condition = threading.Condition(threading.RLock())
        self.enabled = True
        self.busy = False
        self.ready = False
        self.error = '等待规划实时障碍服务'
        self.last_input = 0.0
        self.checked = 0.0
        self.received = 0
        self.revision = 0
        self.session = uuid.uuid4().hex

    def receive(self, cloud):
        with self.condition:
            try:
                self.publish(cloud if self.enabled and not self.flushing else self.empty(cloud))
            except (ValueError, RuntimeError, TimeoutError) as exc:
                self.input_error = str(exc)
                self.ready = False
                return
            self.input_error = None
            self.last_input = time.monotonic()
            self.received += 1
            self.condition.notify_all()

    def refresh(self):
        with self.condition:
            if self.busy:
                return
            revision = self.revision
        try:
            self.validate()
            error = None
        except (RuntimeError, TimeoutError) as exc:
            error = str(exc)
        with self.condition:
            if not self.busy and revision == self.revision:
                self.error = error
                self.ready = error is None
                self.checked = time.monotonic()

    def state(self):
        with self.condition:
            fresh = time.monotonic() - self.last_input <= 2.5
            available = self.ready and fresh and not self.input_error and not self.recovery_error and time.monotonic() - self.checked <= 5
            message = self.recovery_error or self.input_error or self.error or ('' if fresh else '等待新鲜的实时雷达点云')
            if not message and not available:
                message = '等待规划实时障碍状态更新'
            return {'available': available, 'enabled': self.enabled if available and not self.busy else None,
                    'busy': self.busy, 'message': message,
                    'revision': self.revision, 'session': self.session,
                    'thresholds': dict(self.thresholds), 'cloud': self.cloud_stats}

    def set_enabled(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('enabled 必须为布尔值')
        with self.condition:
            if self.busy:
                raise RuntimeError('正在切换规划实时障碍，请稍候')
            previous = self.enabled
            self.busy = True
        try:
            self.validate()
            with self.condition:
                if time.monotonic() - self.last_input > 2.5:
                    raise RuntimeError('实时雷达点云未就绪，未切换避障')
                changed = enabled != self.enabled
                if changed:
                    self.enabled = enabled
                    self.revision += 1
                    before = self.received
                    # Two new empty clouds replace buffered observations before
                    # clearing; enabling waits for an actual new real cloud.
                    frames = 1 if enabled else 2
                    if not self.condition.wait_for(
                            lambda: self.received >= before + frames, self.timeout):
                        raise TimeoutError('等待规划实时障碍输入更新超时')
            # Keep receiving empty observations while ROS services and fresh
            # costmap publications are pending. Never hold the input lock here.
            if changed and not enabled:
                self.clear()
            if changed:
                self.updated()
            with self.condition:
                self.error = None
                self.ready = True
                self.checked = time.monotonic()
        except (RuntimeError, TimeoutError) as exc:
            with self.condition:
                if self.enabled != previous:
                    self.revision += 1
                self.enabled = previous
                self.error = str(exc)
                self.ready = False
            raise
        finally:
            with self.condition:
                self.busy = False
        return self.state()


    def configure(self, values):
        values = validate_thresholds(values)
        with self.condition:
            if self.busy:
                raise RuntimeError('正在更新规划障碍设置，请稍候')
            if values == self.thresholds:
                return self.state()
            self.busy = True
            previous = dict(self.thresholds)
        inflation_attempted = False
        changed = False
        try:
            self.validate()
            with self.condition:
                if time.monotonic() - self.last_input > 2.5:
                    raise RuntimeError('实时点云未就绪，未修改阈值')
                self.revision += 1
                changed = True
                self.flushing = True
                before = self.received
                if not self.condition.wait_for(lambda: self.received >= before + 2, self.timeout):
                    raise TimeoutError('等待空观测替换旧障碍输入超时')
            if values['inflation_radius'] != previous['inflation_radius']:
                inflation_attempted = True
                self.set_inflation(values['inflation_radius'])
            self.clear()
            with self.condition:
                self.thresholds = values
                self.flushing = False
                before = self.received
                if not self.condition.wait_for(lambda: self.received >= before + 2, self.timeout):
                    raise TimeoutError('等待新阈值下的点云超时')
            self.updated()
            self.validate()
            with self.condition:
                self.error, self.ready, self.checked = None, True, time.monotonic()
                self.recovery_error = None
        except (RuntimeError, TimeoutError) as exc:
            with self.condition:
                self.thresholds = previous
                self.flushing = changed
                self.revision += 1
                self.error, self.ready = str(exc), False
            if changed:
                try:
                    if inflation_attempted:
                        self.set_inflation(previous['inflation_radius'])
                    with self.condition:
                        before = self.received
                        if not self.condition.wait_for(lambda: self.received >= before + 2, self.timeout):
                            raise TimeoutError('等待恢复观测超时')
                    self.clear()
                    with self.condition:
                        self.flushing = False
                        before = self.received
                        if not self.condition.wait_for(lambda: self.received >= before + 2, self.timeout):
                            raise TimeoutError('等待原设置下的点云超时')
                    self.updated()
                except (RuntimeError, TimeoutError) as restore:
                    with self.condition:
                        self.error += f'；原设置恢复未确认：{restore}'
                        self.recovery_error = self.error + '；请检查并重启 Nav2 会话'
            raise
        finally:
            with self.condition:
                self.flushing = False
                self.busy = False
        return self.state()


def wait_all(futures, seconds=2.0):
    deadline = time.monotonic() + seconds
    results = []
    for future in futures:
        event = threading.Event()
        future.add_done_callback(lambda _, event=event: event.set())
        if not event.wait(max(0.0, deadline - time.monotonic())):
            raise TimeoutError('Nav2 规划实时障碍服务响应超时')
        try:
            result = future.result()
        except Exception as exc:
            raise RuntimeError(f'Nav2 规划实时障碍服务失败：{exc}') from exc
        if result is None:
            raise RuntimeError('Nav2 规划实时障碍服务未返回结果')
        results.append(result)
    return results


class RosPlanningObstacles(ObstacleGate):
    def __init__(self, node, tf_buffer):
        from nav2_msgs.msg import Costmap
        from nav2_msgs.srv import ClearCostmapExceptRegion
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
        from planning_inflation import PlanningInflation
        from sensor_msgs.msg import PointCloud2
        from std_srvs.srv import SetBool, Trigger

        self.node = node
        self.tf_buffer = tf_buffer
        self.map_condition = threading.Condition()
        self.map_stamp = -1
        self.publisher = node.create_publisher(PointCloud2, OBSTACLE_TOPIC, 1)
        self.clearing_publisher = node.create_publisher(PointCloud2, CLEARING_TOPIC, 1)
        self.geometry = None
        group = ReentrantCallbackGroup()
        self.parameters = node.create_client(GetParameters, '/global_costmap/global_costmap/get_parameters', callback_group=group)
        self.clear_service = node.create_client(ClearCostmapExceptRegion, '/global_costmap/clear_except_global_costmap', callback_group=group)
        self.inflation = PlanningInflation(node, group, wait_all)
        super().__init__(self._validate, self._clear, self._publish, self._empty,
                         updated=self._updated, set_inflation=self.inflation.apply)
        node.create_subscription(PointCloud2, '/cloud_registered_body_1', self.receive, 1, callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(Costmap, '/global_costmap/costmap_raw', self.on_costmap, 1, callback_group=group)
        node.create_service(Trigger, STATE_SERVICE, self.get_state, callback_group=group)
        node.create_service(SetBool, SET_SERVICE, self.set_state, callback_group=group)
        node.create_service(SetParametersAtomically, THRESHOLD_SERVICE, self.configure_state, callback_group=group)
        node.create_timer(1.0, self.refresh)

    @staticmethod
    def _empty(cloud):
        empty = copy.copy(cloud)
        empty.height, empty.width, empty.row_step, empty.data = 1, 0, 0, b''
        return empty

    def _publish(self, cloud):
        from obstacle_cloud import filter_cloud
        from rclpy.time import Time
        from tf2_ros import TransformException
        if cloud.width == 0:
            self.publisher.publish(cloud)
            self.clearing_publisher.publish(cloud)
            return
        if self.geometry is None:
            raise RuntimeError('等待代价地图的高度与距离配置')
        frame, low, high, distance = self.geometry
        try:
            tf = self.tf_buffer.lookup_transform(frame, cloud.header.frame_id, Time.from_msg(cloud.header.stamp))
        except TransformException as exc:
            raise RuntimeError('等待当前点云时间戳的定位 TF') from exc
        t, q = tf.transform.translation, tf.transform.rotation
        marked, rays, stats = filter_cloud(cloud, self.thresholds,
            ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]), low, high, distance)
        self.publisher.publish(marked)
        self.clearing_publisher.publish(rays)
        self.cloud_stats = stats

    def configure_state(self, request, response):
        from rcl_interfaces.msg import ParameterType
        try:
            values = {}
            for parameter in request.parameters:
                kind = ParameterType.PARAMETER_INTEGER if parameter.name == 'min_points' else ParameterType.PARAMETER_DOUBLE
                if parameter.name in values or parameter.value.type != kind:
                    raise ValueError('阈值参数重复或类型错误')
                values[parameter.name] = parameter.value.integer_value if kind == ParameterType.PARAMETER_INTEGER else parameter.value.double_value
            self.configure(values)
            response.result.successful = True
        except (ValueError, RuntimeError, TimeoutError) as exc:
            response.result.successful = False
            response.result.reason = str(exc)
            return response
        response.result.reason = json.dumps(self.state(), ensure_ascii=False)
        return response

    def on_costmap(self, msg):
        stamp = msg.header.stamp
        with self.map_condition:
            self.map_stamp = stamp.sec * 1_000_000_000 + stamp.nanosec
            self.map_condition.notify_all()

    def get_state(self, request, response):
        response.success = True
        response.message = json.dumps(self.state(), ensure_ascii=False)
        return response

    def set_state(self, request, response):
        try:
            state = self.set_enabled(request.data)
            response.success = True
        except (ValueError, RuntimeError, TimeoutError):
            state = self.state()
            response.success = False
        response.message = json.dumps(state, ensure_ascii=False)
        return response

    def _validate(self):
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from rclpy.time import Time
        from tf2_ros import TransformException

        if any(self.node.count_publishers(topic) != 1 for topic in (OBSTACLE_TOPIC, CLEARING_TOPIC)):
            raise RuntimeError('规划障碍输入存在重复发布者')
        if any(publisher.get_subscription_count() < 1 for publisher in (self.publisher, self.clearing_publisher)):
            raise RuntimeError('等待全局代价地图接入规划障碍输入')
        if not self.parameters.service_is_ready() or not self.clear_service.service_is_ready():
            raise RuntimeError('全局代价地图服务未就绪')
        plugin_types = {'static_layer': 'nav2_costmap_2d::StaticLayer',
                        'obstacle_layer': 'nav2_costmap_2d::ObstacleLayer',
                        'inflation_layer': 'nav2_costmap_2d::InflationLayer'}
        names = ['plugins', 'obstacle_layer.enabled', 'obstacle_layer.observation_sources',
                 'obstacle_layer.lidar.topic', 'obstacle_layer.lidar.observation_persistence',
                 'clearable_layers', *[name + '.plugin' for name in plugin_types],
                 'global_frame', 'robot_base_frame', 'inflation_layer.inflation_radius',
                 'obstacle_layer.lidar.min_obstacle_height', 'obstacle_layer.lidar.max_obstacle_height',
                 'obstacle_layer.lidar.obstacle_range', 'obstacle_layer.lidar.obstacle_max_range',
                 'obstacle_layer.lidar.marking', 'obstacle_layer.lidar.clearing',
                 'obstacle_layer.clearing.topic', 'obstacle_layer.clearing.marking',
                 'obstacle_layer.clearing.clearing', 'obstacle_layer.clearing.observation_persistence']
        request = GetParameters.Request()
        request.names = names
        values = wait_all([self.parameters.call_async(request)])[0].values
        if len(values) != len(names):
            raise RuntimeError('全局代价地图参数响应不完整')
        clearable = ((values[5].type == ParameterType.PARAMETER_STRING_ARRAY
                      and list(values[5].string_array_value) == ['obstacle_layer'])
                     or (os.environ.get('ROS_DISTRO') == 'humble'
                         and values[5].type == ParameterType.PARAMETER_NOT_SET))
        classes = dict(zip(plugin_types, values[6:9]))
        valid = (values[0].type == ParameterType.PARAMETER_STRING_ARRAY
                 and set(values[0].string_array_value) == set(plugin_types)
                 and all(classes[name].type == ParameterType.PARAMETER_STRING
                         and classes[name].string_value == plugin_types[name] for name in plugin_types)
                 and values[1].type == ParameterType.PARAMETER_BOOL and values[1].bool_value
                 and values[2].type == ParameterType.PARAMETER_STRING and values[2].string_value == 'lidar clearing'
                 and values[3].type == ParameterType.PARAMETER_STRING and values[3].string_value == OBSTACLE_TOPIC
                 and values[4].type == ParameterType.PARAMETER_DOUBLE and values[4].double_value == 0.0
                 and clearable and all(v.type == ParameterType.PARAMETER_STRING for v in values[9:11])
                 and values[11].type == ParameterType.PARAMETER_DOUBLE
                 and all(v.type == ParameterType.PARAMETER_DOUBLE for v in values[12:14])
                 and all(values[i].type == ParameterType.PARAMETER_BOOL for i in (16, 17, 19, 20))
                 and values[18].type == ParameterType.PARAMETER_STRING
                 and values[21].type == ParameterType.PARAMETER_DOUBLE
                 and values[16].bool_value and not values[17].bool_value
                 and values[18].string_value == CLEARING_TOPIC
                 and not values[19].bool_value and values[20].bool_value
                 and values[21].double_value == 0.0)
        if not valid:
            raise RuntimeError('全局代价地图配置不匹配，请使用更新后的仅规划入口')
        distance = values[14] if os.environ.get('ROS_DISTRO') == 'foxy' else values[15]
        if distance.type != ParameterType.PARAMETER_DOUBLE or not math.isfinite(distance.double_value) or distance.double_value <= 0:
            raise RuntimeError('代价地图距离参数无效')
        self.geometry = (values[9].string_value, values[12].double_value, values[13].double_value, distance.double_value)
        actual = values[11].double_value
        if not math.isfinite(actual) or not 0 <= actual <= 1.5:
            raise RuntimeError('当前膨胀距离不在 Web 支持的 0–1.5 米范围')
        if not all(math.isfinite(v.double_value) for v in values[12:14]) or values[12].double_value >= values[13].double_value:
            raise RuntimeError('障碍高度范围无效')
        with self.condition:
            if not self.busy and self.thresholds['inflation_radius'] != actual:
                self.thresholds = dict(self.thresholds, inflation_radius=actual)
                self.revision += 1
            elif self.busy and not self.flushing and self.thresholds['inflation_radius'] != actual:
                raise RuntimeError('膨胀距离读回不一致，更新未确认')
        if not self.inflation.active():
            raise RuntimeError('等待全局规划器 active')
        try:
            self.tf_buffer.lookup_transform(values[9].string_value, values[10].string_value, Time())
        except TransformException as exc:
            raise RuntimeError('等待全局规划定位 TF 恢复') from exc

    def _clear(self):
        from nav2_msgs.srv import ClearCostmapExceptRegion
        request = ClearCostmapExceptRegion.Request()
        request.reset_distance = 0.0
        wait_all([self.clear_service.call_async(request)])

    def _updated(self):
        after = self.node.get_clock().now().nanoseconds
        with self.map_condition:
            if not self.map_condition.wait_for(lambda: self.map_stamp > after, 3.0):
                raise TimeoutError('全局代价地图尚未更新，规划障碍输入已恢复原状态')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    _, ros_args = parser.parse_known_args()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from tf2_ros import Buffer, TransformListener
    rclpy.init(args=ros_args)
    node = rclpy.create_node('g1_planning_obstacles')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    control = RosPlanningObstacles(node, buffer)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.get_logger().info('全局规划实时障碍输入已启动，默认开启；状态由 ROS 服务读取')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
