#!/usr/bin/env bash
workflow=robot-ready
source "$(dirname -- "${BASH_SOURCE[0]}")/../workflows/lib.sh"
source "$repo/scripts/real_robot/live_env.sh"

usage() {
    cat <<'EOF'
用法：
  04_robot_ready.sh [status] [--lidar-topic TOPIC] [--interface NIC] [--sdk-preflight EXE]
  04_robot_ready.sh localize --map MAP.ply [--initial X Y Z ROLL PITCH YAW] [--lidar-topic TOPIC]
status 仅读取雷达、FAST-LIO、Open3D 和可选的运控模式。
localize 默认自动全局重定位：保持机器人静止，无需手动初值。
--initial 仅用于显式调试，表示 map ← camera_init 变换（米、度）。
EOF
}

mode=status
case "${1:-}" in
    status|localize) mode=$1; shift ;;
    -h|--help) usage; exit 0 ;;
esac
map= lidar= nic= sdk= initial=()
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --map) need_args 2 "$#" "$1"; map=$(absolute "$2"); shift 2 ;;
        --lidar-topic) need_args 2 "$#" "$1"; lidar=$2; shift 2 ;;
        --interface) need_args 2 "$#" "$1"; nic=$2; shift 2 ;;
        --sdk-preflight) need_args 2 "$#" "$1"; sdk=$(absolute "$2"); shift 2 ;;
        --initial) need_args 7 "$#" "$1"; initial=("$2" "$3" "$4" "$5" "$6" "$7"); shift 7 ;;
        *) usage_error "未知参数 $1" ;;
    esac
done
if [[ "$mode" == localize ]]; then
    [[ -n "$map" ]] || usage_error "localize 需要 --map"
    check_file "$map"
    [[ -z "$nic$sdk" ]] || usage_error "SDK 查询参数用于 status"
    if ((${#initial[@]})); then
        ((${#initial[@]} == 6)) || usage_error "初值需要六个数"
        values=$("${G1_PYTHON:-python3}" -c 'import math,sys; v=list(map(float,sys.argv[1:])); assert all(map(math.isfinite,v)); print(*(format(x,".9f") for x in v))' "${initial[@]}") ||
            usage_error "初值必须为有限数"
        read -r -a initial <<<"$values"
    fi
else
    [[ -z "$map" ]] && ((${#initial[@]} == 0)) || usage_error "地图和初值用于 localize"
    [[ -z "$sdk" || -n "$nic" ]] || usage_error "--sdk-preflight 需要 --interface NIC"
fi
log_session
load_ros 223
require_real_graph
if [[ "$mode" == status ]]; then
    bad=0
    probe lidar --seconds 6 --lidar-topic "$lidar" 2>&1 | tee "$run_dir/lidar.log" || { say "雷达或 IMU 检查失败"; bad=1; }
    probe lio --seconds 6 2>&1 | tee "$run_dir/lio.log" || { say "FAST-LIO 检查失败"; bad=1; }
    probe localized --seconds 6 2>&1 | tee "$run_dir/localization.log" || { say "定位健康或 map → body 检查失败"; bad=1; }
    if [[ -n "$nic" ]]; then
        sdk=${sdk:-${G1_SDK_PREFLIGHT:-$G1_DATA/bin/g1_loco_preflight}}
        if [[ ! -x "$sdk" ]]; then
            say "缺少只读 SDK 查询工具 ${sdk}，见 PC2 调试记录的构建命令"
            bad=1
        elif ! timeout 8 "$sdk" "$nic" 2>&1 | tee "$run_dir/sdk.log"; then
            say "运控模式查询失败，查看 sdk.log"
            bad=1
        fi
    else
        say "运控模式未查询；可传 --interface NIC 检查"
    fi
    ((bad == 0)) || fail "一项或多项状态检查失败，按上方检查项定位"
    say "所选状态检查通过"
    exit 0
fi

preflight_processes localization
ros_packages fast_lio livox_ros_driver2 open3d_loc tf2_ros
lock_run "real-lio-$ROS_DOMAIN_ID"
probe free --node global_localization_node --topic /localization_3d_registration_ok ||
    fail "检测到持续配准会话；先结束该会话或运行 status"
ensure_livox "$lidar"
if probe free --topic /Odometry_loc --topic /cloud_registered_body_1; then
    mkdir -p "$run_dir/lio_debug"
    start_bg fast-lio ros2 run fast_lio fastlio_mapping --ros-args \
        --params-file "$repo/FAST_LIO/config/mid360.yaml" \
        -p use_sim_time:=false -p "common.lid_topic:=$lidar_topic" \
        -p "common.input_roll_180:=${G1_INPUT_ROLL_180:-true}" \
        -p common.check_imu_z_up:=true -p pcd_save.pcd_save_en:=false \
        -p "debug_log_dir:=$run_dir/lio_debug"
fi
wait_ready lio --seconds "${G1_LIO_WAIT_SECONDS:-45}"
if probe tf --target map --frame body --seconds 3; then
    fail "已有 map → body TF；检查是否仍运行静态对齐或旧定位会话"
fi

if ((${#initial[@]} == 0)); then
    initializer="$(ros2 pkg prefix open3d_loc)/lib/open3d_loc/global_initialize"
    [[ -x "$initializer" ]] || fail "缺少自动重定位程序；请先重新构建 open3d_loc"
    say '正在自动检测地图位置与朝向，请保持机器人静止；无需输入初值。'
    /usr/bin/python3 "$repo/scripts/real_robot/capture_global_scans.py" "$run_dir/global-scans" 2>&1 |
        tee "$run_dir/global-capture.log" || fail "自动重定位扫描采集失败；未启动地图定位"
    say '全图搜索与多候选/三组扫描验证中，最长 120 秒。'
    timeout --foreground 120 "$initializer" "$map" \
        "$run_dir/global-scans/scan1.ply" "$run_dir/global-scans/scan2.ply" "$run_dir/global-scans/scan3.ply" \
        "$run_dir/global-initialization.json" 2>&1 | tee "$run_dir/global-search.log" ||
        fail "自动重定位失败、存在位置歧义或搜索超时；查看 global-search.log；未发布候选地图位姿"
    values=$(/usr/bin/python3 - "$run_dir/global-initialization.json" <<'AUTO_INITIAL'
import json,math,sys
result=json.load(open(sys.argv[1]))
values=result['initial']
assert result['accepted'] is True and len(values)==6 and all(math.isfinite(v) for v in values)
print(*values)
AUTO_INITIAL
    ) || fail "自动重定位输出无效"
    read -r -a initial <<<"$values"
    say "自动重定位已通过多组扫描验证，开始持续跟踪。结果记录在 $run_dir/global-initialization.json"
fi

start_bg open3d ros2 launch "$repo/open3d_loc/launch/g1_walking_localization.launch.py" \
    "repo:=$repo" "map_ply:=$map" real_robot:=true start_lio:=false use_sim_time:=false \
    "initial_x:=${initial[0]}" "initial_y:=${initial[1]}" "initial_z:=${initial[2]}" \
    "initial_roll:=${initial[3]}" "initial_pitch:=${initial[4]}" "initial_yaw:=${initial[5]}"
wait_ready localized --seconds 75
say "Open3D 已配准，map → body 持续更新。保留本终端，再开一个终端启动 Web。"
print_web_next_steps "$map" 2>&1 | tee "$run_dir/next-web.txt"
session_wait
