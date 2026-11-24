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
    export G1_PYTHON=$(offline_python)
    "${G1_PYTHON:-python3}" -c 'import numpy, cv2; print("NumPy", numpy.__version__, "OpenCV", cv2.__version__)' ||
        fail "地图导出需要 NumPy 与 OpenCV；用 G1_PYTHON 指向已装依赖的解释器"
    preview_missing=false
    for module in scipy skimage matplotlib; do
        if details=$("${G1_PYTHON:-python3}" -c 'import importlib,sys; m=importlib.import_module(sys.argv[1]); print(getattr(m,"__version__","版本未知"))' "$module" 2>&1); then
            say "石膏预览依赖 $module 可用：$details"
        else
            preview_missing=true
            say "石膏预览依赖 $module 导入失败：${details##*$'\n'}"
        fi
    done
    dufo_missing=false
    if "${G1_PYTHON:-python3}" -c 'from dufomap import dufomap' 2>/dev/null; then
        say '可选 DUFOMap 依赖可用'
    else
        dufo_missing=true
        say '可选 DUFOMap 依赖缺失（不影响普通导出和石膏预览）'
    fi
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
        [[ -x "$(ros2 pkg prefix open3d_loc)/lib/open3d_loc/global_initialize" ]] ||
            fail "缺少自动重定位程序；请重新构建 open3d_loc"
        # 解包式 Open3D 的 lapacke/lapack/tmglib 只在 .deps 本地目录可见；
        # 这里在启动定位前确认依赖树完整，避免构建或运行时才暴露缺库。
        if ! bash "$repo/scripts/env/open3d_runtime_probe.sh" >/dev/null; then
            fail "Open3D 本地运行库依赖不完整（修复命令见上方）；否则 open3d_loc 构建或启动会缺库"
        fi
        /usr/bin/python3 -c 'import numpy; from scipy.spatial import cKDTree' || fail "点云障碍分析需要 python3-numpy、python3-scipy"
    fi
fi
say "$profile 自检通过"

if [[ "$profile" == offline ]] && [[ "$(uname -s)" == Linux ]] &&
    "$preview_missing"; then
    say '可选石膏预览依赖安装命令（清华 PyPI，独立环境，不改动 ROS 系统 Python）：'
    cat >&2 <<'INSTALL'
cd "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
/usr/bin/python3 -m venv --without-pip --system-site-packages .deps/offline-venv
.deps/offline-venv/bin/python -m pip install --only-binary=:all: -i https://pypi.tuna.tsinghua.edu.cn/simple "numpy==1.24.4" "scipy==1.10.1" "scikit-image==0.21.0" "matplotlib==3.7.5"
export G1_PYTHON="$PWD/.deps/offline-venv/bin/python"
./scripts/workflows/01_self_check.sh offline
INSTALL
fi

if [[ "$profile" == offline ]] && "$dufo_missing"; then
    if [[ "$(uname -m)" != x86_64 || "$(uname -s)" != Linux ]]; then
        say 'DUFOMap 1.1.1 没有本机架构安装包；以下命令需在 Linux x86_64 处理机的仓库根目录执行，不能直接在 PC2 ARM64 上安装。'
    else
        say 'DUFOMap 安装命令（清华源；在已具备离线导出依赖的处理机执行）：'
    fi
    cat >&2 <<'DUFO_INSTALL'
python3 -m venv --system-site-packages .deps/dufomap-venv
.deps/dufomap-venv/bin/python -m pip install --only-binary=:all: -i https://pypi.tuna.tsinghua.edu.cn/simple "dufomap==1.1.1"
# 验证 DUFOMap 导入：
.deps/dufomap-venv/bin/python -c 'from dufomap import dufomap; print("DUFOMap 可用")'
DUFO_INSTALL
fi

if [[ "$profile" == robot ]]; then
    # Read the live interface, never reuse an address from a previous DHCP lease.
    wireless_ip=$(ip -4 -o addr show dev wlan0 scope global 2>/dev/null |
        awk 'NR == 1 { split($4, address, "/"); print address[1] }') || wireless_ip=
    if [[ -n "$wireless_ip" ]]; then
        say "当前无线 IPv4：$wireless_ip（DHCP 地址可能变化）"
        say '下一步：在操作电脑另开终端，使用下列命令连接 PC2：'
        printf 'ssh unitree@%s\n' "$wireless_ip" >&2
        say '确认无线 SSH 登录成功后，断开外部电脑连接 PC2 的有线网线，再从无线会话继续 02 建图。'
        say '无线连接失败时保留网线排查；不要断开机器人内部或雷达网线。'
        say '无线登录后，在新的 PC2 终端依次执行：'
        cat >&2 <<'COMMANDS'
cd "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export G1_DATA="$HOME/ldy/g1_real_data"
export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
# 然后开始 02 建图，RUN 换成本次未使用的名称：
# ./scripts/workflows/02_mapping.sh record RUN
COMMANDS
    else
        say '未检测到 wlan0 可用 IPv4。请保留有线网线，先连接 sic-guest，再重新执行 01 自检获取无线命令。'
    fi
fi
