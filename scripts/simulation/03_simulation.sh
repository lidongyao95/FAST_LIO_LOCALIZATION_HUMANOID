#!/usr/bin/env bash
workflow=simulation
source "$(dirname -- "${BASH_SOURCE[0]}")/../workflows/lib.sh"
source "$repo/scripts/simulation/session.sh"

usage() {
    cat <<'EOF'
用法：
  03_simulation.sh map OUTPUT_DIR [--headless]
  03_simulation.sh nav MAP_DIR [导航参数]
先在内置 MuJoCo 房间中采集虚拟 LiDAR/IMU。地图目录生成后重启做定位和导航。
默认仿真域 74，仅本机通信。Ctrl+C 停止本轮进程。
实机地图预演请用 Mac 侧入口：scripts/simulation/03_preview.sh prepare|play。

导航参数：
EOF
    navigation_options
    echo '  --moving-obstacle                在内置房间启用巡逻障碍'
}


run_mapping() {
    local direction duration extra status
    local export_python="${G1_PYTHON:-/usr/bin/python3}"
    [[ ! -e "$map_dir" && ! -L "$map_dir" ]] || fail "输出目录已存在：$map_dir"
    simulation_preflight
    "$export_python" -c 'import numpy, cv2' 2>/dev/null ||
        fail "导出地图需要 NumPy 和 OpenCV；可设置 G1_PYTHON"
    mkdir -p "$(dirname -- "$map_dir")"
    mkdir -- "$map_dir" || fail "不能创建新地图目录 $map_dir"
    start_simulator
    prepare_operator
    start_bg mapping "$sim/start_g1_mapping.sh" "rviz:=$rviz" "map_file:=$map_dir/map.pcd" "debug_dir:=$run_dir"
    wait_ready lio --seconds 45
    if [[ -t 0 ]]; then
        # 交互终端：实时单键控制，按键即时生效，不需要回车和秒数。
        say "走遍准备导航的区域。实时控制：按住 w/a/s/d 前后左右、q/e 转向，松手即停，回车保存并结束。"
        if ! "$sim/g1_operator.sh" teleop; then
            fail "仿真实时控制失败，查看 controller.log"
        fi
        assert_running
    else
        say "走遍准备导航的区域。非交互输入：w/a/s/d 前后左右，q/e 转向，例如 w 5；支持 1–30 秒，默认 2 秒。空行保存。"
        while true; do
            printf '建图控制> ' >&2
            while true; do
                if read -r -t 1 direction duration extra; then break; else status=$?; fi
                ((status > 128)) || fail "输入已关闭；重新运行并用空行结束采集"
                assert_running
            done
            [[ -n "$direction" ]] || break
            duration=${duration:-2}
            if [[ ! "$direction" =~ ^[wasdqe]$ || ! "$duration" =~ ^([1-9]|[12][0-9]|30)$ || -n "$extra" ]]; then
                say "请输入一个方向字母和可选时长，比如 q 3"
                continue
            fi
            if ! timeout "$((duration + 8))" "$sim/g1_operator.sh" drive "$direction" --seconds "$duration" >>"$run_dir/drive.log" 2>&1; then
                fail "仿真手动指令失败，查看 drive.log 和 controller.log"
            fi
            assert_running
        done
    fi
    if ! timeout 60 ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/map_save.log" 2>&1 ||
        ! grep -Eq 'success=(True|true)|success: (True|true)' "$run_dir/map_save.log"; then
        fail "/map_save 未返回成功，查看 map_save.log 和 mapping.log"
    fi
    check_file "$map_dir/map.pcd"
    stop_owned
    pids=()
    "$export_python" "$repo/scripts/mapping/export_g1_map.py" "$map_dir/map.pcd" --out "$map_dir" \
        2>&1 | tee "$run_dir/export.log" || fail "地图导出失败，查看 export.log；PCD 已保留"
    check_file "$map_dir/map.ply"
    check_file "$map_dir/map.yaml"
    check_file "$map_dir/map.pgm"
    say "建图完成：$map_dir"
    printf '下一条命令：%q nav %q\n' "$sim/03_simulation.sh" "$map_dir" >&2
}

case "${1:-}" in
    ''|-h|--help) usage; exit 0 ;;
    map|nav) stage=$1; shift ;;
    *) usage_error "请选择 map（建图）或 nav（从已建地图导航）" ;;
esac
case "${1:-}" in -h|--help) usage; exit 0 ;; esac
[[ -n "${1:-}" && "$1" != --* ]] || usage_error "$stage 需要地图目录"
map_dir=$(absolute "$1")
shift
if [[ "$stage" == map ]]; then
    if [[ "${1:-}" == --headless ]]; then headless=true; rviz=false; shift; fi
    (($# == 0)) || usage_error "未知建图参数 $1"
    run_mapping
else
    if [[ -f "$map_dir/scene.xml" ]]; then
        exec bash "$repo/scripts/simulation/scan_preview.sh" "$map_dir" "$@"
    fi
    check_file "$map_dir/map.pcd"
    initial_z=0.0
    parse_navigation "$@"
    run_navigation
fi
