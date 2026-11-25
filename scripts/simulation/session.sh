#!/usr/bin/env bash
# Shared startup for the built-in room and the recorded-scan preview.
sim="$repo/scripts/simulation"
headless=false rviz=true
start_x=0.0 start_y=0.0 start_yaw=0.0
model_path=
goal=() obstacle=() test_options=()

navigation_options() {
    cat <<'EOF'
  --headless                       关闭 MuJoCo 和 RViz 窗口
  --start X Y --start-yaw RAD       仿真起点和航向
  --initial-z M                    Open3D 配准初始高度
  --goal X Y                       运行一次导航目标验证
  --obstacle-after N               N 秒后插入动态障碍
  --obstacle-clear-after N         插入后 N 秒移除
  --cancel-after N                 N 秒后取消目标并验证取消状态
EOF
}

parse_navigation() {
    local options
    local injected=false clearing=false
    while (($#)); do
        case "$1" in
            -h|--help) usage; exit 0 ;;
            --headless) headless=true; rviz=false; shift ;;
            --start|--goal)
                need_args 3 "$#" "$1"
                if [[ "$1" == --start ]]; then start_x=$2; start_y=$3; else goal=(--x "$2" --y "$3"); fi
                shift 3 ;;
            --start-yaw|--initial-z)
                need_args 2 "$#" "$1"
                if [[ "$1" == --start-yaw ]]; then start_yaw=$2; else initial_z=$2; fi
                shift 2 ;;
            --moving-obstacle)
                [[ -z "$model_path" ]] || usage_error "移动障碍预设用于内置房间；扫描场景请用 --obstacle-after"
                obstacle=(--moving-obstacle); shift ;;
            --obstacle-after|--cancel-after|--obstacle-clear-after)
                need_args 2 "$#" "$1"
                if [[ "$1" == --obstacle-after ]]; then injected=true; fi
                if [[ "$1" == --obstacle-clear-after ]]; then clearing=true; fi
                test_options+=("$1" "$2"); shift 2 ;;
            *) usage_error "未知导航参数 $1" ;;
        esac
    done
    ((${#test_options[@]} == 0 || ${#goal[@]} != 0)) || usage_error "验证参数要求 --goal"
    if "$clearing" && ! "$injected"; then usage_error "--obstacle-clear-after 需要 --obstacle-after"; fi
    if ! options=$("${G1_PYTHON:-python3}" -c '
import math
import sys
x, y, yaw, z = map(float, sys.argv[1:5])
for key, raw in zip(sys.argv[5::2], sys.argv[6::2]):
    number = float(raw)
    assert math.isfinite(number) and (key in ("--x", "--y") or number >= 0)
values = [x, y, yaw, z, math.degrees(yaw)]
assert all(map(math.isfinite, values))
print(*(format(value, ".9f") for value in values))
' "$start_x" "$start_y" "$start_yaw" "$initial_z" "${goal[@]}" "${test_options[@]}" 2>/dev/null); then
        usage_error "坐标、角度、高度和时间需要有限数，时间须非负"
    fi
    read -r start_x start_y start_yaw initial_z initial_yaw <<<"$options"
}

simulation_preflight() {
    log_session "$repo/sim/generated/workflow_logs"
    export ROS_LOCALHOST_ONLY=1
    load_ros 74
    if [[ ! "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || ((10#$ROS_DOMAIN_ID < 1 || 10#$ROS_DOMAIN_ID > 232 || 10#$ROS_DOMAIN_ID == 223)); then
        fail "请选择 1–232 的独立仿真域，PC2 默认域为 223"
    fi
    ros_packages fast_lio open3d_loc
    if "$rviz"; then
        ros_packages rviz2
        [[ -n "${DISPLAY:-}" ]] || fail "没有 DISPLAY；请选择图形桌面或使用 --headless"
        if [[ -n "$model_path" ]]; then
            local graphics_report renderer
            graphics_report=$(glxinfo -B 2>&1) || fail "无法检查 OpenGL 渲染器：$graphics_report"
            renderer=$(printf '%s\n' "$graphics_report" | sed -n 's/^OpenGL renderer string: //p')
            [[ "$renderer" == *NVIDIA* ]] || fail "当前图形渲染器不是 NVIDIA：$renderer"
            say "GPU 图形渲染：$renderer"
        fi
    fi
    for artifact in .deps/venv/bin/python .deps/controller-build/g1_ctrl .deps/controller-build/libg1_dds_bridge.so .deps/g1-controller/config.yaml; do
        check_file "$repo/$artifact"
    done
    preflight_processes all
    lock_run simulation
    probe free --node g1_walking_sim --node global_localization_node \
        --topic /clock --topic /Odometry_loc --topic /livox/imu --topic /g1_sim/ground_truth \
        --topic /cmd_vel --topic /cmd_vel_nav ||
        fail "仿真域已被占用，换独立域或先结束原会话"
    nav_graph_empty || fail "已有 Nav2；请结束原会话或使用其他仿真域"
}

start_simulator() {
    local -a sim_args=(--start-x "$start_x" --start-y "$start_y" --start-yaw "$start_yaw" "${obstacle[@]}")
    if "$headless"; then sim_args+=(--headless); fi
    if [[ -n "$model_path" ]]; then sim_args+=(--model-path "$model_path"); fi
    start_bg mujoco "$sim/start_walking_sim.sh" "${sim_args[@]}"
    wait_ready clock --seconds 25
    start_bg controller "$sim/start_g1_controller.sh"
    # 下游脚本以 walking-controller 锁判断控制器是否在线，必须等控制进程真正持锁再继续。
    for attempt in {1..100}; do
        if ! flock -n "$repo/.deps/walking-controller-${ROS_DOMAIN_ID}.lock" true; then return 0; fi
        assert_running
        sleep .1
    done
    fail "控制器启动后 10 秒内未持有 walking-controller 锁"
}

prepare_operator() {
    timeout 35 "$sim/g1_operator.sh" prepare >"$run_dir/prepare.log" 2>&1 ||
        fail "仿真控制器准备失败，查看 prepare.log 与 controller.log"
}

run_navigation() {
    local map_ply="$map_dir/map.ply" map_yaml="$map_dir/map.yaml"
    check_file "$map_ply"
    check_file "$map_yaml"
    if [[ -n "$model_path" ]]; then check_file "$model_path"; fi
    simulation_preflight
    ros_packages nav2_map_server nav2_planner nav2_controller nav2_behaviors nav2_bt_navigator
    start_simulator
    start_bg localization "$sim/start_g1_localization.sh" "map_ply:=$map_ply" \
        "initial_x:=$start_x" "initial_y:=$start_y" "initial_z:=$initial_z" "initial_yaw:=$initial_yaw"
    wait_ready localized --frame base_footprint --seconds 75
    prepare_operator
    start_bg nav2 "$sim/start_g1_navigation.sh" "map_yaml:=$map_yaml" "rviz:=$rviz"
    wait_ready nav --node map_server --node planner_server --node controller_server --node bt_navigator --seconds 60
    timeout 15 "$sim/g1_operator.sh" enable-nav >"$run_dir/nav_enable.log" 2>&1 ||
        fail "仿真导航门禁未放行，查看 nav_enable.log"
    say "加载地图 ${map_dir}。仿真定位、Nav2 和速度门禁就绪。Ctrl+C 停止本轮。"
    if ((${#goal[@]})); then
        "$repo/scripts/checks/validate_nav_goal.sh" "${goal[@]}" "${test_options[@]}" --keep-nav \
            --output "$run_dir/nav_goal.json" 2>&1 | tee "$run_dir/nav_goal.log" ||
            fail "仿真目标验证失败，查看 nav_goal.log 和 nav_goal.json"
        if ! /usr/bin/python3 - "$run_dir/nav_goal.json" "${test_options[@]}" <<'PY'
import json
import sys

with open(sys.argv[1]) as stream:
    result = json.load(stream)
args = sys.argv[2:]
expected = 5 if "--cancel-after" in args else 4
if result.get("goal_status") != expected or result.get("timeout"):
    sys.exit(f"目标结果异常，状态 {result.get('goal_status')}，预期 {expected}")
if "--obstacle-after" in args and not result.get("obstacle_added"):
    sys.exit("仿真未成功插入障碍")
if "--obstacle-clear-after" in args and not result.get("obstacle_removed"):
    sys.exit("仿真未成功移除障碍")
print(f"目标状态 {expected}，非零速度指令 {result['nonzero_commands']} 条，路径更新 {result['plan_updates']} 次")
PY
        then fail "仿真结果未满足预期，查看 nav_goal.json"; fi
        say "仿真目标验证通过，结果 $run_dir/nav_goal.json；导航门禁保持开启，可直接在 RViz 用 Nav2 Goal 继续下发目标"
        if "$headless"; then return; fi
    fi
    session_wait
}


load_plaster_scene() {
    model_path="$map_dir/scene.xml"
    initial_z=0.793
    local spawn scene_x scene_y
spawn=$("${G1_PYTHON:-python3}" - "$map_dir/metadata.json" <<'PYSPAWN'
import json, math, sys
from pathlib import Path
import xml.etree.ElementTree as ET
metadata = Path(sys.argv[1])
info = json.loads(metadata.read_text())
if info.get('scene_representation') != 'continuous plaster mesh':
    raise SystemExit('所选目录是旧版或非石膏场景；请用 03_generate_scene.sh 生成石膏场景，勿使用旧 _preview 目录')
root = ET.parse(metadata.with_name('scene.xml')).getroot()
geoms = list(root.iter('geom'))
if any(g.get('name', '').startswith(('scan_visual_', 'scan_collision_')) for g in geoms):
    raise SystemExit('检测到旧扫描体素几何，已阻止启动')
contacts = [g for g in geoms if g.get('name', '').startswith('plaster_collision_')]
if not contacts or any(g.get('type') != 'mesh' or g.get('group') != '3' for g in contacts):
    raise SystemExit('缺少沿石膏表面的 mesh 碰撞几何，已阻止启动')
print(f"[场景] {metadata.parent}；连续石膏网格，{len(contacts)} 个表面三角碰撞片，无扫描体素盒", file=sys.stderr)
spawn = info['spawn_xy_m']
if len(spawn) != 2 or not all(math.isfinite(float(v)) for v in spawn):
    raise SystemExit('Invalid spawn_xy_m')
print(*(float(v) for v in spawn))
PYSPAWN
) || fail "场景或出生点检查失败，请按上方提示处理"
read -r scene_x scene_y <<<"$spawn"
parse_navigation --start "$scene_x" "$scene_y" "$@"
}
