#!/usr/bin/env bash
workflow=web-navigation
source "$(dirname -- "${BASH_SOURCE[0]}")/../workflows/lib.sh"
source "$repo/scripts/real_robot/live_env.sh"

usage() {
    cat <<'EOF'
用法：05_web_navigation.sh MAP.yaml|MAP_DIR [--port 18081]
先让 04_robot_ready.sh localize 保持运行，再启动本入口。
Web 点选目标仅请求路径，不启动实机控制器或速度桥。
网页“规划考虑实时障碍”默认开启，只影响全局规划；静态地图障碍始终保留。
HTTP 仅监听本机回环，可通过 SSH 转发；Ctrl+C 停止本次 Web 与 Nav2。
EOF
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
map=$(absolute "$1")
shift
[[ ! -d "$map" ]] || map="$map/map.yaml"
port=18081
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --port) need_args 2 "$#" "$1"; port=$2; shift 2 ;;
        *) usage_error "未知参数 $1" ;;
    esac
done
[[ "$port" =~ ^[1-9][0-9]{0,4}$ ]] && ((port <= 65535)) || usage_error "端口需要 1–65535 的整数"
check_file "$map"
log_session
load_ros 223
/usr/bin/python3 -c 'import numpy; from scipy.spatial import cKDTree' || fail "点云障碍分析需要 python3-numpy、python3-scipy"
preflight_processes navigation
require_real_graph
ros_packages open3d_loc nav2_map_server nav2_planner
args=(--node map_server --node planner_server)
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
wait_ready lio --seconds 12
wait_ready localized --seconds 12
start_bg nav2 ros2 launch "$repo/open3d_loc/launch/g1_navigation.launch.py" \
    "repo:=$repo" "map_yaml:=$map" mode:=plan use_sim_time:=false robot_frame:=body rviz:=false
wait_ready nav "${args[@]}" --seconds 55
start_bg web /usr/bin/python3 "$repo/scripts/real_robot/web/server.py" --port "$port" --robot-frame body
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
say "Web 就绪：${url}（点击目标生成路径）"
say "远程浏览在访问端运行：ssh -N -L $port:127.0.0.1:$port 用户@PC2"
session_wait
