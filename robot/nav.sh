#!/usr/bin/env bash
# Real-robot navigation: localize on a saved map, start Nav2 and serve the navigation web page.
workflow=robot-nav
source "$(dirname -- "${BASH_SOURCE[0]}")/../shared/lib.sh"
source "$repo/robot/live_env.sh"

usage() {
    cat <<'USAGE'
用法：./g1 robot nav 地图名 [--plan-only] [--port 18081]

自动完成定位、启动 Nav2 和导航网页。定位期间保持机器人静止。
网页里点目标 → 规划路径 → 开始实机导航；“取消并停车”随时终止。
开始导航前，机器人要处于运控模式 501（./g1 robot fsm get 查看）。

--plan-only  只规划路径，不让机器人走。
Ctrl+C 结束定位、Nav2 和网页。
USAGE
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
G1_DATA="${G1_DATA:-$HOME/ldy/g1_real_data}"
map_dir="$G1_DATA/maps/$1"
[[ -d "$map_dir" ]] || map_dir=$(absolute "$1")
shift
map="$map_dir/map.yaml"
port=18081
mode=real
sdk="${G1_DATA:-$HOME/ldy/g1_real_data}/bin/g1_navigation_sdk"
interface=${G1_ROBOT_INTERFACE:-eth0}
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --plan-only) mode=plan; shift ;;
        --port) need_args 2 "$#" "$1"; port=$2; shift 2 ;;
        *) usage_error "未知参数 $1" ;;
    esac
done
[[ "$port" =~ ^[1-9][0-9]{0,4}$ ]] && ((port <= 65535)) || usage_error "端口需要 1–65535 的整数"
check_file "$map"
check_file "$map_dir/map.ply"
log_session
load_ros 223
/usr/bin/python3 -c 'import numpy; from scipy.spatial import cKDTree' || fail "缺少依赖：sudo apt install python3-numpy python3-scipy"
preflight_processes navigation
require_real_graph
ros_packages open3d_loc nav2_map_server nav2_planner
args=(--node map_server --node planner_server)
web_args=()
ground_z=0.0
if [[ "$mode" == real ]]; then
    ros_packages nav2_controller nav2_bt_navigator
    [[ -x "$sdk" ]] || fail "缺少导航 SDK；先运行 bash robot/sdk/build.sh"
    [[ -d "/sys/class/net/$interface" ]] || fail "机器人网口不存在：$interface；可用 G1_ROBOT_INTERFACE 指定"
    ground_z=$(/usr/bin/python3 - "$map" <<'FLOOR'
import json, math, sys
from pathlib import Path
value = float(json.loads(Path(sys.argv[1]).with_name('metadata.json').read_text())['floor_z'])
if not math.isfinite(value): raise ValueError('floor_z 必须有限')
print(value)
FLOOR
    ) || fail "地图目录缺少 metadata.json；用 ./g1 robot export 重新导出"
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
nav_graph_empty || fail "已有 Nav2 在运行；先结束上一次导航"
if probe localized --seconds 3 >/dev/null 2>&1; then
    say "定位已在运行，直接使用。"
else
    say "正在定位：保持机器人静止，最长约 4 分钟。进度见 $run_dir/localization.log"
    start_bg localization "$repo/robot/localize.sh" localize --map "$map_dir/map.ply"
    deadline=$((SECONDS + 260))
    until probe localized --seconds 5 >/dev/null 2>&1; do
        assert_running
        ((SECONDS < deadline)) || fail "定位超时，查看 localization.log"
    done
    say "定位完成。"
fi
wait_ready lio --seconds 12
wait_ready localized --seconds 12
start_bg nav2 ros2 launch "$repo/shared/nav/navigation.launch.py" \
    "repo:=$repo" "map_yaml:=$map" "mode:=$mode" "ground_z:=$ground_z" use_sim_time:=false robot_frame:=body rviz:=false
wait_ready nav "${args[@]}" --seconds 55
start_bg web /usr/bin/python3 "$repo/robot/web/server.py" --port "$port" --robot-frame body "${web_args[@]}"
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
    say '在网页点目标 → 规划路径 → 开始实机导航；“取消并停车”终止。机器人需处于运控模式 501。'
else
    say '当前只规划路径。要让机器人走，去掉 --plan-only 重新运行。'
fi
print_web_access "$port"
session_wait
