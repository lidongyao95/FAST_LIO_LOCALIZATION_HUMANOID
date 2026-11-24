"""Apply inflation to the real global costmap, including Foxy's configure-only plugin."""
import os


class PlanningInflation:
    def __init__(self, node, group, wait):
        from lifecycle_msgs.srv import ChangeState, GetState
        from rcl_interfaces.srv import SetParametersAtomically
        self.wait = wait
        self.setting = node.create_client(SetParametersAtomically,
            '/global_costmap/global_costmap/set_parameters_atomically', callback_group=group)
        self.state = node.create_client(GetState, '/planner_server/get_state', callback_group=group)
        self.change = node.create_client(ChangeState, '/planner_server/change_state', callback_group=group)

    def active(self):
        from lifecycle_msgs.srv import GetState
        if not self.state.service_is_ready():
            raise RuntimeError('规划器生命周期服务未就绪')
        return self.wait([self.state.call_async(GetState.Request())])[0].current_state.id == 3

    def transition(self, identifier):
        from lifecycle_msgs.srv import ChangeState
        request = ChangeState.Request()
        request.transition.id = identifier
        if not self.change.service_is_ready() or not self.wait(
                [self.change.call_async(request)], seconds=5.0)[0].success:
            raise RuntimeError('规划器重载失败，请检查 Nav2 日志')

    def apply(self, radius):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParametersAtomically
        if os.environ.get('ROS_DISTRO') not in ('foxy', 'humble'):
            raise RuntimeError('膨胀距离调节仅支持 Foxy/Humble')
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name='inflation_layer.inflation_radius',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=radius))]
        if not self.setting.service_is_ready():
            raise RuntimeError('全局代价地图参数服务未就绪')
        response = self.wait([self.setting.call_async(request)])[0]
        if not response.result.successful:
            raise RuntimeError(response.result.reason or '膨胀距离设置被拒绝')
        if os.environ.get('ROS_DISTRO') == 'foxy':
            # Restore from any partially completed previous reload as well.
            from lifecycle_msgs.srv import GetState
            state = self.wait([self.state.call_async(GetState.Request())])[0].current_state.id
            if state == 3:
                self.transition(4)  # deactivate
                state = 2
            if state == 2:
                self.transition(2)  # cleanup: discard configure-only inflation cache
                state = 1
            if state != 1:
                raise RuntimeError('规划器状态异常，无法重载膨胀距离')
            self.transition(1)  # configure reads the actual new radius
            self.transition(3)  # activate
        if not self.active():
            raise RuntimeError('等待规划器恢复 active')
