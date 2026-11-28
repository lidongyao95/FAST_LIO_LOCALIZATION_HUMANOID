#!/usr/bin/env bash
# Simulation entry: build a map in the virtual room, or localize and navigate on a saved map.
workflow=sim
source "$(dirname -- "${BASH_SOURCE[0]}")/../shared/lib.sh"
sim="$repo/sim"
maps="$repo/sim/generated/maps"
headless=false
start_x=0.0 start_y=0.0 start_yaw=0.0
goal=() obstacle=() test_options=()

usage() {
    cat <<'EOF'
用法：
  ./g1_slam sim map [地图名]
  ./g1_slam sim nav [地图名] [--goal X Y]

地图名默认是 room，地图保存在 sim/generated/maps/ 下。
没有图形界面时自动以无窗口方式运行，也可以加 --headless 强制无窗口。

map：按住 w/a/s/d 前后左右移动，q/e 转向，松手即停，回车保存。
nav：启动后在 RViz 里用 Nav2 Goal 点目标；--goal X Y 直接发一个目标并等待结果。

nav 的其他参数（主要给测试用）：
  --start X Y            机器人起点
  --start-yaw RAD        机器人起始朝向
  --moving-obstacle      场景里加一块来回移动的挡板
  --obstacle-after N     发出目标 N 秒后放入障碍
  --obstacle-clear-after N  放入障碍 N 秒后移走
  --cancel-after N       N 秒后取消目标
EOF
}

parse_options() {
    local injected=false clearing=false
    while (($#)); do
        case "$1" in
            -h|--help) usage; exit 0 ;;
            --headless) headless=true; shift ;;
            --start|--goal)
                need_args 3 "$#" "$1"
                if [[ "$1" == --start ]]; then start_x=$2; start_y=$3; else goal=(--x "$2" --y "$3"); fi
                shift 3 ;;
            --start-yaw) need_args 2 "$#" "$1"; start_yaw=$2; shift 2 ;;
            --moving-obstacle) obstacle=(--moving-obstacle); shift ;;
            --obstacle-after|--cancel-after|--obstacle-clear-after)
                need_args 2 "$#" "$1"
                if [[ "$1" == --obstacle-after ]]; then injected=true; fi
                if [[ "$1" == --obstacle-clear-after ]]; then clearing=true; fi
                test_options+=("$1" "$2"); shift 2 ;;
            *) usage_error "未知参数 $1" ;;
        esac
    done
    [[ "$stage" == nav ]] || ((${#goal[@]} + ${#obstacle[@]} + ${#test_options[@]} == 0)) ||
        usage_error "map 只接受 --headless"
    ((${#test_options[@]} == 0 || ${#goal[@]} != 0)) || usage_error "障碍和取消参数需要同时给 --goal"
    if "$clearing" && ! "$injected"; then usage_error "--obstacle-clear-after 需要 --obstacle-after"; fi
    local numbers
    numbers=$(/usr/bin/python3 -c '
import math, sys
values = [float(v) for v in sys.argv[1:] if not v.startswith("--")]
assert all(map(math.isfinite, values))
x, y, yaw = values[:3]
print(format(x, ".9f"), format(y, ".9f"), format(yaw, ".9f"))
' "$start_x" "$start_y" "$start_yaw" "${goal[@]}" "${test_options[@]}" 2>/dev/null) ||
        usage_error "坐标、角度和时间需要是数字"
    read -r start_x start_y start_yaw <<<"$numbers"
    [[ -n "${DISPLAY:-}" ]] || headless=true
}

preflight() {
    log_session "$repo/sim/generated/logs"
    export ROS_LOCALHOST_ONLY=1
    load_ros 74
    ros_packages fast_lio open3d_loc
    "$headless" || ros_packages rviz2
    local artifact
    for artifact in .deps/venv/bin/python .deps/controller-build/g1_ctrl .deps/controller-build/libg1_dds_bridge.so .deps/g1-controller/config.yaml; do
        [[ -s "$repo/$artifact" ]] || fail "缺少 ${artifact}；先运行 ./g1_slam setup"
    done
    preflight_processes all
    lock_run "simulation-$ROS_DOMAIN_ID"
    probe free --node g1_mujoco_sensors --node global_localization_node \
        --topic /clock --topic /Odometry_loc --topic /livox/imu --topic /g1_sim/ground_truth \
        --topic /cmd_vel --topic /cmd_vel_nav ||
        fail "已有一轮仿真在运行；先在它的终端按 Ctrl+C"
    nav_graph_empty || fail "已有 Nav2 在运行；先结束上一轮仿真"
}

operator() { /usr/bin/python3 "$sim/operator_cli.py" "$@"; }

start_robot() {
    local -a sim_args=(--start-x "$start_x" --start-y "$start_y" --start-yaw "$start_yaw" "${obstacle[@]}")
    if "$headless"; then sim_args+=(--headless); fi
    start_bg mujoco "$sim/start.sh" simulator "${sim_args[@]}"
    wait_ready clock --seconds 25
    start_bg controller "$sim/start.sh" controller
    # Later stages check this lock to know the controller is alive.
    local attempt
    for attempt in {1..100}; do
        if ! flock -n "$repo/.deps/walking-controller-${ROS_DOMAIN_ID}.lock" true; then return 0; fi
        assert_running
        sleep .1
    done
    fail "控制器 10 秒内没有启动，查看 controller.log"
}

stand_up() {
    timeout 35 /usr/bin/python3 "$sim/operator_cli.py" prepare >"$run_dir/prepare.log" 2>&1 ||
        fail "机器人没有站稳，查看 prepare.log 和 controller.log"
}

run_mapping() {
    local direction duration extra status
    preflight
    rm -rf -- "$map_dir"
    mkdir -p -- "$map_dir"
    start_robot
    stand_up
    start_bg mapping "$sim/start.sh" mapping "rviz:=$rviz" "map_file:=$map_dir/map.pcd" "debug_dir:=$run_dir"
    wait_ready lio --seconds 45
    if [[ -t 0 ]]; then
        say "走遍要导航的区域：按住 w/a/s/d 移动，q/e 转向，松手即停，回车保存。"
        operator teleop || fail "键盘控制中断，查看 controller.log"
        assert_running
    else
        # Piped input, used by tests: one "direction seconds" per line, empty line saves.
        while true; do
            while true; do
                if read -r -t 1 direction duration extra; then break; else status=$?; fi
                ((status > 128)) || fail "输入已关闭；用空行结束采集"
                assert_running
            done
            [[ -n "$direction" ]] || break
            duration=${duration:-2}
            [[ "$direction" =~ ^[wasdqe]$ && "$duration" =~ ^([1-9]|[12][0-9]|30)$ && -z "$extra" ]] ||
                fail "输入格式是“方向 秒数”，比如 w 5"
            timeout "$((duration + 8))" /usr/bin/python3 "$sim/operator_cli.py" drive "$direction" --seconds "$duration" \
                >>"$run_dir/drive.log" 2>&1 || fail "行走指令失败，查看 drive.log 和 controller.log"
            assert_running
        done
    fi
    if ! timeout 60 ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/map_save.log" 2>&1 ||
        ! grep -Eq 'success=(True|true)|success: (True|true)' "$run_dir/map_save.log"; then
        fail "地图没有保存成功，查看 map_save.log 和 mapping.log"
    fi
    check_file "$map_dir/map.pcd"
    stop_owned
    pids=()
    /usr/bin/python3 "$repo/shared/map/export_map.py" "$map_dir/map.pcd" --out "$map_dir" \
        >"$run_dir/export.log" 2>&1 || fail "地图导出失败，查看 export.log"
    check_file "$map_dir/map.ply"
    check_file "$map_dir/map.yaml"
    say "建图完成：$map_dir"
    say "下一步：./g1_slam sim nav $map_name"
}

run_navigation() {
    check_file "$map_dir/map.ply"
    check_file "$map_dir/map.yaml"
    preflight
    ros_packages nav2_map_server nav2_planner nav2_controller nav2_behaviors nav2_bt_navigator
    start_robot
    start_bg localization "$sim/start.sh" localization "map_ply:=$map_dir/map.ply" \
        "initial_x:=$start_x" "initial_y:=$start_y" initial_z:=0.0 \
        "initial_yaw:=$(/usr/bin/python3 -c 'import math,sys; print(math.degrees(float(sys.argv[1])))' "$start_yaw")"
    wait_ready localized --frame base_footprint --seconds 75
    stand_up
    start_bg nav2 "$sim/start.sh" navigation "map_yaml:=$map_dir/map.yaml" "rviz:=$rviz"
    wait_ready nav --node map_server --node planner_server --node controller_server --node bt_navigator --seconds 60
    timeout 15 /usr/bin/python3 "$sim/operator_cli.py" enable-nav >"$run_dir/nav_enable.log" 2>&1 ||
        fail "导航没有接管机器人，查看 nav_enable.log"
    say "定位和导航已就绪。在 RViz 里用 Nav2 Goal 点目标；Ctrl+C 结束。"
    if ((${#goal[@]})); then
        /usr/bin/python3 "$repo/tests/scenarios/nav_goal.py" "${goal[@]}" "${test_options[@]}" --keep-nav \
            --output "$run_dir/nav_goal.json" 2>&1 | tee "$run_dir/nav_goal.log" ||
            fail "目标没有按预期完成，查看 nav_goal.log 和 nav_goal.json"
        say "目标完成，结果在 $run_dir/nav_goal.json"
        if "$headless"; then return; fi
    fi
    session_wait
}

case "${1:-}" in
    ''|-h|--help) usage; exit 0 ;;
    map|nav) stage=$1; shift ;;
    *) usage_error "请选择 map 或 nav" ;;
esac
map_name=room
if [[ -n "${1:-}" && "$1" != -* ]]; then map_name=$1; shift; fi
[[ "$map_name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || usage_error "地图名只能用字母、数字、下划线和连字符"
map_dir="$maps/$map_name"
parse_options "$@"
rviz=true
if "$headless"; then rviz=false; fi
if [[ "$stage" == map ]]; then run_mapping; else run_navigation; fi
