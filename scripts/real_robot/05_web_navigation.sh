#!/usr/bin/env bash
workflow=web-navigation
source "$(dirname -- "${BASH_SOURCE[0]}")/../workflows/lib.sh"
source "$repo/scripts/real_robot/live_env.sh"

usage() {
    cat <<'EOF'
用法：05_web_navigation.sh MAP.yaml|MAP_DIR [--port 18081] [--plan-only] [--review]
--review：仅检查导出地图，无需定位、ROS 或 Nav2；缩放/平移/点击查询栅格。
先让 04_robot_ready.sh localize 保持运行，再启动本入口。
默认启动实机 Nav2；网页先“规划路径”，再明确点击“开始实机导航”才接入 SDK。
--plan-only：只提供路径规划，不启动控制器和实机 SDK。
网页“规划考虑实时障碍”默认开启，只影响全局规划；静态地图障碍始终保留。
HTTP 仅监听本机回环，可通过 SSH 转发；Ctrl+C 停止本次 Web 与 Nav2。
EOF
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
map=$(absolute "$1")
shift
[[ ! -d "$map" ]] || map="$map/map.yaml"
port=18081
review=false
mode=real
sdk="${G1_DATA:-$HOME/ldy/g1_real_data}/bin/g1_navigation_sdk"
interface=${G1_ROBOT_INTERFACE:-eth0}
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --plan-only) mode=plan; shift ;;
        --review) review=true; shift ;;
        --port) need_args 2 "$#" "$1"; port=$2; shift 2 ;;
        *) usage_error "未知参数 $1" ;;
    esac
done
[[ "$port" =~ ^[1-9][0-9]{0,4}$ ]] && ((port <= 65535)) || usage_error "端口需要 1–65535 的整数"
check_file "$map"
if "$review"; then
    say "地图检查模式：仅回环监听，网页端口 ${port}"
    print_web_access "$port"
    exec /usr/bin/python3 "$repo/scripts/real_robot/web/map_review.py" "$map" --port "$port"
fi
log_session
load_ros 223
/usr/bin/python3 -c 'import numpy; from scipy.spatial import cKDTree' || fail "点云障碍分析需要 python3-numpy、python3-scipy"
preflight_processes navigation
require_real_graph
ros_packages open3d_loc nav2_map_server nav2_planner
args=(--node map_server --node planner_server)
web_args=()
ground_z=0.0
if [[ "$mode" == real ]]; then
    ros_packages nav2_controller nav2_bt_navigator
    [[ -x "$sdk" ]] || fail "缺少导航 SDK；先运行 ./scripts/real_robot/build_navigation_sdk.sh"
    [[ -d "/sys/class/net/$interface" ]] || fail "机器人网口不存在：$interface；可用 G1_ROBOT_INTERFACE 指定"
    ground_z=$(/usr/bin/python3 - "$map" <<'FLOOR'
import json, math, sys
from pathlib import Path
value = float(json.loads(Path(sys.argv[1]).with_name('metadata.json').read_text())['floor_z'])
if not math.isfinite(value): raise ValueError('floor_z 必须有限')
print(value)
FLOOR
    ) || fail "实机导航需要配套 metadata.json 中的 floor_z；请使用 02 导出的地图"
    args+=(--node controller_server --node bt_navigator)
    web_args=(--real-navigation --sdk-executable "$sdk" --robot-interface "$interface")
fi
if ! /usr/bin/python3 - "$map" "$port" <<'PY'
from pathlib import Path
import socket
import sys
import yaml

path = Path(sys.argv[1])
try:
    data = yaml.safe_load(path.read_text())
    image = path.parent / data["image"]
    if not image.is_file() or not image.stat().st_size:
        raise ValueError(f"地图图像缺失或为空：{image}")
    with socket.socket() as server:
        server.bind(("127.0.0.1", int(sys.argv[2])))
except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
    sys.exit(str(error))
PY
then fail "地图 YAML、图像或本地端口检查失败"; fi
lock_run "web-$ROS_DOMAIN_ID"
nav_graph_empty || fail "检测到旧 Nav2 节点或地图发布者；先结束旧 Web/Nav2 会话"
if probe free --topic /Odometry_loc --topic /cloud_registered_body_1 2>/dev/null; then
    fail "未检测到 FAST-LIO 输出（/Odometry_loc、/cloud_registered_body_1 无发布端）；先运行 04_robot_ready.sh localize 并保持该会话运行"
fi
wait_ready lio --seconds 12
wait_ready localized --seconds 12
start_bg nav2 ros2 launch "$repo/open3d_loc/launch/g1_navigation.launch.py" \
    "repo:=$repo" "map_yaml:=$map" "mode:=$mode" "ground_z:=$ground_z" use_sim_time:=false robot_frame:=body rviz:=false
wait_ready nav "${args[@]}" --seconds 55
start_bg web /usr/bin/python3 "$repo/scripts/real_robot/web/server.py" --port "$port" --robot-frame body "${web_args[@]}"
url="http://127.0.0.1:$port"
if ! /usr/bin/python3 - "$url" <<'PY'
import json
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

def get(route):
    with urlopen(sys.argv[1] + route, timeout=1) as response:
        return json.load(response)

state = {}
obstacles = None
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    try:
        state = get("/api/state")
        obstacles = get("/api/obstacles")
        obstacles_mode = state.get('planning_obstacles', {})
        if (all(state.get(key) for key in ("map", "pose", "planner", "costmap"))
                and obstacles is not None and obstacles_mode.get('available') and not obstacles_mode.get('busy')):
            print(f"地图、位姿、代价地图、规划器和实时雷达图层就绪（{len(obstacles['points'])} 个点）")
            break
    except (URLError, TimeoutError, ValueError):
        pass
    time.sleep(.5)
else:
    print(f"Web 数据超时；状态 {state}，实时雷达图层收到：{obstacles is not None}", file=sys.stderr)
    sys.exit(1)
PY
then
    assert_running
    fail "网页数据未就绪；检查 web.log、nav2.log 和定位数据"
fi
say "Web 就绪：${url}"
if [[ "$mode" == real ]]; then
    say '在网页点选目标 → 规划路径 → 开始实机导航；取消并停车可终止任务。'
    say 'SDK 仅在点击开始后运行；需要机器人已由操作员进入 FSM 501。网页断连或定位失效会停止任务。'
else
    say '当前仅规划：点击目标生成路径。需要实机导航时去掉 --plan-only 重新启动 05。'
fi
print_web_access "$port"
session_wait
