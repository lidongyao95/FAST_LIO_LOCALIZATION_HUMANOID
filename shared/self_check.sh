#!/usr/bin/env bash
# Check that the dependencies and build outputs needed by the simulation or the robot are present.
workflow=check
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

profile=${1:-sim}
case "$profile" in
    sim|robot) ;;
    *) echo "用法：self_check.sh sim|robot"; exit 2 ;;
esac

if [[ "$profile" == sim ]]; then
    log_session "$repo/sim/generated/logs"
    load_ros 74
    ros_packages fast_lio open3d_loc nav2_map_server nav2_planner nav2_controller nav2_behaviors nav2_bt_navigator
    for file in .deps/controller-build/g1_ctrl .deps/g1-controller/config.yaml .deps/venv/bin/python; do
        [[ -s "$repo/$file" ]] || fail "缺少 ${file}；运行 ./g1 setup"
    done
    "$repo/.deps/venv/bin/python" -c 'import mujoco, rclpy' || fail "仿真 Python 环境不能导入 mujoco 或 rclpy"
    /usr/bin/python3 -c 'import numpy, cv2' || fail "系统 Python 缺少 NumPy 或 OpenCV：sudo apt install python3-numpy python3-opencv"
    say "仿真环境检查通过。下一步：./g1 sim map"
else
    log_session
    load_ros 223
    ros_packages fast_lio open3d_loc nav2_map_server nav2_planner nav2_controller nav2_bt_navigator livox_ros_driver2
    [[ -x "$(ros2 pkg prefix open3d_loc)/lib/open3d_loc/global_initialize" ]] ||
        fail "缺少自动重定位程序；重新运行 ./g1 robot deploy build"
    bash "$repo/shared/open3d_probe.sh" >/dev/null ||
        fail "Open3D 运行库不完整；运行 bash shared/open3d_probe.sh 查看缺少的库"
    /usr/bin/python3 -c 'import numpy, cv2; from scipy.spatial import cKDTree' ||
        fail "系统 Python 缺少依赖：sudo apt install python3-numpy python3-scipy python3-opencv"
    for tool in g1_navigation_sdk g1_fsm; do
        [[ -x "$G1_DATA/bin/$tool" ]] || fail "缺少 $G1_DATA/bin/$tool；运行 bash robot/sdk/build.sh"
    done
    say "PC2 环境检查通过。下一步：./g1 robot map 地图名"
fi
