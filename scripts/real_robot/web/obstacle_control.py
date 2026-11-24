"""Read and set the Nav2-owned planning obstacle node; never relay point clouds."""
import json
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'navigation'))
from planning_obstacles import SET_SERVICE, STATE_SERVICE, THRESHOLD_SERVICE, validate_thresholds, wait_all


def decode_state(message):
    state = json.loads(message)
    if (not isinstance(state, dict) or type(state.get('available')) is not bool
            or type(state.get('busy')) is not bool or type(state.get('revision')) is not int
            or not isinstance(state.get('session'), str) or 'enabled' not in state
            or state.get('enabled') is not None and type(state['enabled']) is not bool):
        raise ValueError('规划障碍节点返回的状态无效')
    if state['available']:
        validate_thresholds(state.get('thresholds'))
    return state


class PlanningObstacleClient:
    def __init__(self, node):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_srvs.srv import SetBool, Trigger
        from rcl_interfaces.srv import SetParametersAtomically
        group = ReentrantCallbackGroup()
        self.query = node.create_client(Trigger, STATE_SERVICE, callback_group=group)
        self.setting = node.create_client(SetBool, SET_SERVICE, callback_group=group)
        self.threshold_setting = node.create_client(SetParametersAtomically, THRESHOLD_SERVICE, callback_group=group)
        self.lock = threading.Lock()
        self.pending = False
        self.version = 0
        self.read_sequence = 0
        self.accepted_read = 0
        self.last_read = 0.0
        self.cached = self.unknown('等待规划障碍节点')
        node.create_timer(1.0, self.refresh)

    @staticmethod
    def unknown(message):
        return {'available': False, 'enabled': None, 'busy': False,
                'revision': -1, 'session': '', 'message': message, 'thresholds': None, 'cloud': None}

    def state(self):
        with self.lock:
            state = dict(self.cached) if time.monotonic() - self.last_read <= 5 else self.unknown('规划障碍状态连接中断')
            if self.pending:
                state['busy'], state['enabled'] = True, None
            return state

    def refresh(self):
        from std_srvs.srv import Trigger
        with self.lock:
            version = self.version
            self.read_sequence += 1
            sequence = self.read_sequence
        try:
            if not self.query.service_is_ready():
                raise RuntimeError('等待 Nav2 的规划障碍节点')
            response = wait_all([self.query.call_async(Trigger.Request())])[0]
            if not response.success:
                raise RuntimeError('无法读取规划障碍状态')
            state = decode_state(response.message)
        except (ValueError, RuntimeError, TimeoutError) as exc:
            state = self.unknown(str(exc))
        with self.lock:
            if version == self.version and not self.pending and sequence >= self.accepted_read:
                self.cached = state
                self.accepted_read = sequence
                self.last_read = time.monotonic()
        return self.state()

    def set_enabled(self, enabled):
        from std_srvs.srv import SetBool
        if type(enabled) is not bool:
            raise ValueError('enabled 必须为布尔值')
        with self.lock:
            if self.pending:
                raise RuntimeError('正在切换规划障碍输入，请稍候')
            self.pending = True
            self.version += 1
        try:
            if not self.setting.service_is_ready():
                raise RuntimeError('规划障碍节点未就绪')
            request = SetBool.Request()
            request.data = enabled
            response = wait_all([self.setting.call_async(request)], seconds=12.0)[0]
            state = decode_state(response.message)
            if not response.success or not state['available'] or state['enabled'] is not enabled:
                raise RuntimeError(state.get('message') or '规划障碍设置尚未确认')
            with self.lock:
                self.cached = state
                self.last_read = time.monotonic()
        except (ValueError, RuntimeError, TimeoutError) as exc:
            with self.lock:
                self.cached = self.unknown(str(exc))
                self.last_read = time.monotonic()
            raise
        finally:
            with self.lock:
                self.pending = False
                self.version += 1
        return self.state()

    def configure(self, values):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParametersAtomically
        values = validate_thresholds(values)
        with self.lock:
            if self.pending:
                raise RuntimeError('正在更新规划障碍设置，请稍候')
            self.pending = True
            self.version += 1
        try:
            if not self.threshold_setting.service_is_ready():
                raise RuntimeError('规划障碍阈值服务未就绪，请更新 Nav2 会话')
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name=name, value=(
                ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
                if name == 'min_points' else
                ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)))
                for name, value in values.items()]
            response = wait_all([self.threshold_setting.call_async(request)], seconds=75.0)[0]
            if not response.result.successful:
                raise RuntimeError(response.result.reason or '阈值更新失败')
            state = decode_state(response.result.reason)
            if not state['available'] or state['busy'] or state['thresholds'] != values:
                raise RuntimeError('阈值读回尚未确认')
            with self.lock:
                self.cached, self.last_read = state, time.monotonic()
        except (ValueError, RuntimeError, TimeoutError) as exc:
            with self.lock:
                self.cached, self.last_read = self.unknown(str(exc)), time.monotonic()
            raise
        finally:
            with self.lock:
                self.pending = False
                self.version += 1
        return self.state()
