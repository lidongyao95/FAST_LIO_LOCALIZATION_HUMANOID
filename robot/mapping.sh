#!/usr/bin/env bash
# Real-robot mapping: record with FAST-LIO, then export the 3D and 2D maps used for navigation.
workflow=robot-map
source "$(dirname -- "${BASH_SOURCE[0]}")/../shared/lib.sh"

usage() {
    cat <<'EOF'
用法：
  ./g1 robot map 地图名 [--seconds N]
      启动雷达和 FAST-LIO，带着机器人走一圈，按回车保存并导出地图。
      --seconds N：不等回车，采集 N 秒后自动保存。
  ./g1 robot export 地图名 [--resolution 米] [--voxel 米] [--max-range 米]
      用已有的点云重新导出地图（想换分辨率或裁剪范围时用）。

现场有人走动时，用 DUFOMap 去掉动态点（可选）：
  ./g1 robot scans 名称 --lidar-in-body X Y Z [--frames N]
      在 FAST-LIO 运行时逐帧采集扫描，默认 120 帧。
  ./g1 robot clean 扫描目录 地图名
      在装有 dufomap 的 x86 电脑上清理动态点并导出地图。

地图保存在 $G1_DATA/maps/地图名/，默认 G1_DATA=~/ldy/g1_real_data。
EOF
}

export_map() {
    local input=$1 out=$2
    shift 2
    "$python" -c 'import numpy, cv2' || fail "地图导出需要 NumPy 和 OpenCV"
    mkdir -p "$out"
    "$python" "$repo/shared/map/export_map.py" "$input" --out "$out" --floor-model plane --known-free-radius 0 "$@" \
        2>&1 | tee "$run_dir/export.log" || fail "地图导出失败，查看 export.log"
    check_file "$out/map.ply"
    check_file "$out/map.yaml"
    say "地图已导出：$out"
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
mode=$1
shift
for arg in "$@"; do if [[ "$arg" == --help || "$arg" == -h ]]; then usage; exit 0; fi; done
python=/usr/bin/python3
G1_DATA="${G1_DATA:-$HOME/ldy/g1_real_data}"
resolution=.08 voxel=.06

case "$mode" in
    map)
        need_args 1 "$#" "$mode"
        name=$1; shift
        [[ "$name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || usage_error "地图名只能用字母、数字、下划线和连字符"
        seconds=
        while (($#)); do
            case "$1" in
                --seconds) need_args 2 "$#" "$1"; seconds=$2; shift 2 ;;
                *) usage_error "未知参数 $1" ;;
            esac
        done
        [[ -z "$seconds" || "$seconds" =~ ^[1-9][0-9]*$ ]] || usage_error "--seconds 需要正整数"
        [[ -n "$seconds" || -t 0 ]] || usage_error "不在终端里运行时要加 --seconds N"
        log_session
        source "$repo/robot/live_env.sh"
        load_ros 223
        preflight_processes mapping
        preflight_processes localization
        require_real_graph
        lock_run "real-lio-$ROS_DOMAIN_ID"
        ros_packages fast_lio ros2bag livox_ros_driver2
        for output in "$G1_DATA/bags/$name" "$G1_DATA/maps/$name.pcd" "$G1_DATA/maps/$name" "$G1_DATA/records/$name"; do
            [[ ! -e "$output" && ! -L "$output" ]] || fail "地图名 $name 已经用过（$output）；换一个名字"
        done
        probe free --topic /Odometry_loc --topic /cloud_registered_body_1 ||
            fail "FAST-LIO 已在运行；先结束正在运行的定位或建图"
        ensure_livox ""
        export G1_LIDAR_TOPIC="$lidar_topic" G1_RECORD_SUPERVISED=1
        start_bg record "$repo/robot/record.sh" "$name"
        wait_ready lio --seconds "${G1_LIO_WAIT_SECONDS:-45}"
        if [[ -n "$seconds" ]]; then
            say "采集 $seconds 秒后保存"
            end=$((SECONDS + seconds))
            while ((SECONDS < end)); do assert_running; sleep 1; done
        else
            say "开始建图：带着机器人走遍要导航的区域，回到起点后按回车保存。"
            while true; do
                if IFS= read -r -t 1; then break; else status=$?; fi
                ((status > 128)) || fail "终端输入已关闭"
                assert_running
            done
        fi
        if ! timeout 60 ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/map_save.log" 2>&1 ||
            ! grep -Eq 'success=(True|true)|success: (True|true)' "$run_dir/map_save.log"; then
            fail "地图没有保存成功，查看 map_save.log 和 $G1_DATA/records/$name/lio.log"
        fi
        check_file "$G1_DATA/maps/$name.pcd"
        stop_owned
        pids=()
        export_map "$G1_DATA/maps/$name.pcd" "$G1_DATA/maps/$name" --resolution "$resolution" --voxel "$voxel"
        say "下一步：./g1 robot nav $name"
        ;;
    export)
        need_args 1 "$#" "$mode"
        name=$1; shift
        options=()
        while (($#)); do
            need_args 2 "$#" "$1"
            case "$1" in
                --resolution) resolution=$2 ;;
                --voxel) voxel=$2 ;;
                --max-range) options+=("$1" "$2") ;;
                *) usage_error "未知参数 $1" ;;
            esac
            shift 2
        done
        check_file "$G1_DATA/maps/$name.pcd"
        log_session
        export_map "$G1_DATA/maps/$name.pcd" "$G1_DATA/maps/$name" --resolution "$resolution" --voxel "$voxel" "${options[@]}"
        ;;
    scans)
        need_args 1 "$#" "$mode"
        name=$1; shift
        offset=() frames=120
        while (($#)); do
            case "$1" in
                --lidar-in-body) need_args 4 "$#" "$1"; offset=("$2" "$3" "$4"); shift 4 ;;
                --frames) need_args 2 "$#" "$1"; frames=$2; shift 2 ;;
                *) usage_error "未知参数 $1" ;;
            esac
        done
        ((${#offset[@]} == 3)) || usage_error "需要 --lidar-in-body X Y Z（雷达在机体坐标系里的位置，米）"
        out="$G1_DATA/scans/$name"
        [[ ! -e "$out" ]] || fail "$out 已存在；换一个名称"
        log_session
        load_ros 223
        wait_ready lio
        mkdir -p "$G1_DATA/scans"
        "$python" "$repo/shared/map/record_dufomap_scans.py" "$out" --lidar-in-body "${offset[@]}" --frames "$frames" \
            2>&1 | tee "$run_dir/capture.log" || fail "扫描采集失败，查看 capture.log"
        say "扫描已保存：$out"
        ;;
    clean)
        need_args 2 "$#" "$mode"
        scans=$(absolute "$1"); name=$2
        [[ -d "$scans" ]] || fail "找不到扫描目录 $scans"
        if [[ -x "$repo/.deps/dufomap-venv/bin/python" ]]; then python="$repo/.deps/dufomap-venv/bin/python"; fi
        "$python" -c 'from dufomap import dufomap' ||
            fail "缺少 dufomap：python3 -m venv --system-site-packages .deps/dufomap-venv && .deps/dufomap-venv/bin/pip install dufomap==1.1.1"
        log_session
        out="$G1_DATA/maps/$name"
        [[ ! -e "$out" ]] || fail "$out 已存在；换一个地图名"
        mkdir -p "$out"
        "$python" "$repo/shared/map/dufomap_clean.py" "$scans" "$out/clean.pcd" 2>&1 | tee "$run_dir/dufo.log" ||
            fail "动态点清理失败，查看 dufo.log"
        export_map "$out/clean.pcd" "$out" --resolution "$resolution" --voxel "$voxel"
        ;;
    *) usage_error "未知命令 $mode" ;;
esac
