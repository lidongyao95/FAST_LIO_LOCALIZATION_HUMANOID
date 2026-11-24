#!/usr/bin/env bash
workflow=self-check
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

case "${1:-}" in
    -h|--help)
        echo "用法：$0 [offline|sim|robot]"
        echo '默认按本机选择（macOS: offline；Linux: sim）。自检只查依赖、构建结果和环境。'
        exit 0 ;;
esac
(( $# <= 1 )) || usage_error "只接受一个自检模式"
default=offline
[[ "$(uname -s)" == Linux ]] && default=sim
profile=${1:-$default}
case "$profile" in offline|sim|robot) ;; *) usage_error "未知模式 $profile" ;; esac
say "主机 $(hostname)，系统 $(uname -sm)，模式 $profile"

if [[ "$profile" == offline ]]; then
    "${G1_PYTHON:-python3}" -c 'import numpy, cv2; print("NumPy", numpy.__version__, "OpenCV", cv2.__version__)' ||
        fail "地图导出需要 NumPy 与 OpenCV；用 G1_PYTHON 指向已装依赖的解释器"
    "${G1_PYTHON:-python3}" -c 'import scipy, skimage, matplotlib' 2>/dev/null ||
        say "可选石膏预览依赖缺失：scipy、scikit-image、matplotlib"
    "${G1_PYTHON:-python3}" -c 'from dufomap import dufomap' 2>/dev/null ||
        say "可选 DUFOMap 依赖缺失；见工作流说明中的固定版本"
else
    if [[ "$profile" == sim ]]; then
        log_session "$repo/sim/generated/workflow_logs"
        load_ros 74
    else
        log_session
        load_ros 223
    fi
    ros_packages fast_lio open3d_loc nav2_map_server nav2_planner
    if [[ "$profile" == sim ]]; then
        ros_packages nav2_controller nav2_behaviors nav2_bt_navigator
        for file in .deps/controller-build/g1_ctrl .deps/g1-controller/config.yaml .deps/venv/bin/python; do
            [[ -s "$repo/$file" ]] || fail "缺少 ${file}；按环境文档安装依赖并构建仿真控制器"
        done
        "$repo/.deps/venv/bin/python" -c 'import mujoco, rclpy' ||
            fail "仿真 Python 无法导入 mujoco / rclpy"
        [[ -n "${DISPLAY:-}" ]] || say "未设置 DISPLAY，建图可运行 scripts/simulation/03_simulation.sh map sim/generated/room_run01 --headless（目录须不存在）"
    else
        ros_packages livox_ros_driver2
        /usr/bin/python3 -c 'import numpy; from scipy.spatial import cKDTree' || fail "点云障碍分析需要 python3-numpy、python3-scipy"
    fi
fi
say "$profile 自检通过"
