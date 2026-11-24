#!/usr/bin/env bash
workflow=mapping
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

usage() {
    unqualified="${0##*/}"
    cat <<EOF
用法：
  $unqualified record RUN [--seconds N] [--lidar-topic TOPIC]
      [--point-filter N] [--surface-voxel M] [--mapping-voxel M]
  $unqualified capture-scans OUT --lidar-in-body X Y Z [--frames N | --idle-timeout N]
  $unqualified export INPUT.pcd OUT [--preset detail|standard|quick] [--voxel M]
      [--resolution M] [--floor-model plane|percentile] [--max-range M] [--plaster]
      [--plaster-grid M] [--plaster-ground-cut M] [--plaster-radius M] [--plaster-height M]
  $unqualified dufo SCANS OUT [以上 export 参数] [--d-s M] [--d-p N] [--revisit-after STAMP_NS]
record 默认按 Enter 保存；capture-scans 默认采 120 个精确配对帧。
export 和 dufo 生成同坐标系的 map.ply、map.yaml、map.pgm 与 metadata.json。
EOF
}

print_map_review() {
    local out=$1 pcd=${2:-} wireless_ip name
    if [[ -s "$out/metadata.json" ]]; then
        pcd=$(/usr/bin/python3 - "$out/metadata.json" "$pcd" <<'MAP_SOURCE'
import json,sys
try:
    print(json.load(open(sys.argv[1])).get('source_pcd') or sys.argv[2])
except (OSError,ValueError):
    print(sys.argv[2])
MAP_SOURCE
        )
    fi
    say '下一步：在本机终端启动网页检查这张地图（保持该终端运行）：'
    printf 'cd %q
' "$repo" >&2
    printf './scripts/real_robot/05_web_navigation.sh %q --review --port 18082
' "$out" >&2
    wireless_ip=
    if command -v ip >/dev/null 2>&1; then
        wireless_ip=$(ip -4 -o addr show dev wlan0 scope global 2>/dev/null |
            awk 'NR == 1 { split($4, address, "/"); print address[1] }') || wireless_ip=
    fi
    say '在浏览器所在电脑另开终端执行端口转发（保持该终端运行）：'
    if [[ -n "$wireless_ip" ]]; then
        printf 'ssh -N -L 18082:127.0.0.1:18082 unitree@%s
' "$wireless_ip" >&2
        say '上述无线 IP 来自当次 wlan0，重连后需重新核对。'
    else
        printf 'ssh -N -L 18082:127.0.0.1:18082 用户@运行导出的主机IP
' >&2
        say '未检测到 wlan0 地址，请将用户和主机 IP 替换为实际连接目标；本机浏览无需转发。'
    fi
    say '浏览器打开：http://127.0.0.1:18082/'
    say '检查墙、门口、重影及起点栅格；完成后在网页服务终端按 Ctrl+C。'
    say '巡检完成后二选一：'
    say '（A）直接在 PC2 启动持续定位（实机导航使用这张地图）：'
    printf './scripts/real_robot/04_robot_ready.sh localize --map %q/map.ply
' "$out" >&2
    say '（B）可选：先在仿真服务器上做导航预演，确认路线可通过后再上实机。'
    say '    Mac 只负责发起：03_preview.sh 通过 SSH 把地图传到服务器，MuJoCo、定位与 Nav2 都在服务器上运行，Mac 不跑仿真。'
    say '    默认服务器为 4622 的 sjtu@202.121.181.124；换机器用环境变量 G1_SIM_HOST / G1_SIM_PORT 指定。'
    if [[ -n "$pcd" ]]; then
        name="$(basename "${pcd%.pcd}")_preview_$(date +%Y%m%d_%H%M%S)"
        local pc2_target="unitree@<当次无线IP>"
        [[ -n "$wireless_ip" ]] && pc2_target="unitree@$wireless_ip"
        say '在 Mac 仓库根执行一条命令：读取 PC2 的 PCD → 传输并校验 → 自动选出生点 → 生成场景 → 启动服务器预演：'
        printf './scripts/simulation/03_preview.sh play %q --name %q --pc2 %s\n' "pc2:$pcd" "$name" "$pc2_target" >&2
        say '仅准备不启动时，将 play 换成 prepare；随后按 03 输出的命令 launch。'
        say '无图形界面时在命令末尾加 -- --headless --goal X Y；目标在派生仿真地图中选择。'
        say '默认仿真域 75，与实机域 223 分离。预演后回到 PC2 执行上面的 04，不把仿真地图拷回实机。'
    else
        say '本次未拿到采集 PCD 路径，B 分支请直接用实机 PCD 运行 03_preview.sh prepare。'
    fi
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
mode=$1
shift
for arg in "$@"; do if [[ "$arg" == --help || "$arg" == -h ]]; then usage; exit 0; fi; done
mapping="$repo/scripts/mapping"
python="${G1_PYTHON:-python3}"
if [[ "$mode" == export || "$mode" == dufo ]]; then python=$(offline_python); fi

case "$mode" in
    record)
        need_args 1 "$#" "$mode"
        run_name=$1; shift
        [[ "$run_name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || usage_error "RUN 只能包含英数字、下划线和连字符，并以英数字开头"
        seconds= lidar=
        while (($#)); do
            need_args 2 "$#" "$1"
            case "$1" in
                --seconds) seconds=$2 ;;
                --lidar-topic) lidar=$2 ;;
                --point-filter) [[ $2 =~ ^[1-9][0-9]*$ ]] || usage_error "point-filter 需要正整数"; export G1_POINT_FILTER_NUM=$2 ;;
                --surface-voxel) export G1_FILTER_SIZE_SURF=$2 ;;
                --mapping-voxel) export G1_FILTER_SIZE_MAP=$2 ;;
                *) usage_error "未知参数 $1" ;;
            esac
            shift 2
        done
        [[ -z "$seconds" || "$seconds" =~ ^[1-9][0-9]*$ ]] || usage_error "--seconds 需要正整数"
        [[ -n "$seconds" || -t 0 ]] || usage_error "非交互运行须传 --seconds N"
        if ! sizes=$("$python" -c 'import math,sys; v=list(map(float,sys.argv[1:])); assert all(math.isfinite(x) and x>=1e-9 for x in v); print(*(format(x,".12f") for x in v))' \
            "${G1_FILTER_SIZE_SURF:-0.5}" "${G1_FILTER_SIZE_MAP:-0.5}"); then
            usage_error "建图体素尺寸需要正数（米）"
        fi
        read -r G1_FILTER_SIZE_SURF G1_FILTER_SIZE_MAP <<<"$sizes"
        export G1_FILTER_SIZE_SURF G1_FILTER_SIZE_MAP
        log_session
        source "$repo/scripts/real_robot/live_env.sh"
        load_ros 223
        preflight_processes mapping
        preflight_processes localization
        require_real_graph
        lock_run "real-lio-$ROS_DOMAIN_ID"
        ros_packages fast_lio ros2bag livox_ros_driver2
        for output in "$G1_DATA/bags/$run_name" "$G1_DATA/maps/$run_name.pcd" "$G1_DATA/records/$run_name"; do
            [[ ! -e "$output" && ! -L "$output" ]] || fail "采集名称已使用：$output"
        done
        probe free --topic /Odometry_loc --topic /cloud_registered_body_1 ||
            fail "已存在 FAST-LIO；停止原定位会话后再录制"
        ensure_livox "$lidar"
        export G1_LIDAR_TOPIC="$lidar_topic" G1_RECORD_SUPERVISED=1
        start_bg record "$repo/scripts/real_robot/record_g1_a_segment.sh" "$run_name"
        wait_ready lio --seconds "${G1_LIO_WAIT_SECONDS:-45}"
        if [[ -n "$seconds" ]]; then
            say "采集 $seconds 秒后保存地图"
            end=$((SECONDS + seconds))
            while ((SECONDS < end)); do assert_running; sleep 1; done
        else
            say "现在可按现场流程采集；到达结束位置后按 Enter 保存"
            while true; do
                if IFS= read -r -t 1; then break; else status=$?; fi
                ((status > 128)) || fail "终端输入已关闭，采集结束；下次可用 --seconds"
                assert_running
            done
        fi
        save_log="$run_dir/map_save.log"
        if ! timeout 60 ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$save_log" 2>&1 ||
            ! grep -Eq 'success=(True|true)|success: (True|true)' "$save_log"; then
            fail "/map_save 未成功，查看 $save_log 与 $G1_DATA/records/$run_name/lio.log"
        fi
        check_file "$G1_DATA/maps/$run_name.pcd"
        stop_owned
        pids=()
        metadata="$G1_DATA/bags/$run_name/metadata.yaml"
        check_file "$metadata"
        if ! /usr/bin/python3 - "$metadata" "$lidar_topic" <<'PY'
from pathlib import Path
import sys
import yaml

bag = yaml.safe_load(Path(sys.argv[1]).read_text())["rosbag2_bagfile_information"]
counts = {item["topic_metadata"]["name"]: item["message_count"]
          for item in bag["topics_with_message_count"]}
missing = [name for name in (sys.argv[2], "/livox/imu") if not counts.get(name)]
if missing:
    sys.exit(f"录包没有这些话题的数据：{', '.join(missing)}")
print(f"录包消息数：{counts}")
PY
        then fail "录包元数据或消息数异常，请检查 $G1_DATA/records/$run_name/record.log"; fi
        say "采集完成，地图 $G1_DATA/maps/$run_name.pcd，录包 $G1_DATA/bags/$run_name"
        say '下一步导出地图；导出后会给出可选 03 预演与直接 04 定位两条分支：'
        printf './scripts/workflows/02_mapping.sh export %q %q --plaster\n' "$G1_DATA/maps/$run_name.pcd" "$G1_DATA/maps/${run_name}_map" >&2
        ;;
    capture-scans)
        need_args 1 "$#" "$mode"
        out=$(absolute "$1"); shift
        offset=() end=() capture_timeout=()
        while (($#)); do
            case "$1" in
                --lidar-in-body) need_args 4 "$#" "$1"; offset=("$2" "$3" "$4"); shift 4 ;;
                --frames|--idle-timeout)
                    need_args 2 "$#" "$1"; ((${#end[@]} == 0)) || usage_error "只能指定一种采集结束条件"
                    end=("$1" "$2"); shift 2 ;;
                --timeout) need_args 2 "$#" "$1"; capture_timeout=("$1" "$2"); shift 2 ;;
                *) usage_error "未知参数 $1" ;;
            esac
        done
        ((${#offset[@]} == 3)) || usage_error "需要实测 --lidar-in-body X Y Z（米）"
        if ((${#end[@]} == 0)); then end=(--frames 120); fi
        log_session
        load_ros 223
        wait_ready lio
        /usr/bin/python3 "$mapping/record_dufomap_scans.py" "$out" --lidar-in-body "${offset[@]}" \
            "${end[@]}" "${capture_timeout[@]}" 2>&1 | tee "$run_dir/capture.log" ||
            fail "扫描采集失败，见 capture.log；检查同步扫描与里程计时间戳"
        say "精确配对扫描已保存到 $out"
        ;;
    export|dufo)
        need_args 2 "$#" "$mode"
        input=$(absolute "$1"); out=$(absolute "$2"); shift 2
        preset=standard voxel= resolution= floor_model=plane plaster=false
        preview_grid=.08 ground_cut=.16 preview_radius=15 preview_height=2.8
        export_args=() dufo_args=()
        while (($#)); do
            if [[ "$1" == --plaster ]]; then plaster=true; shift; continue; fi
            need_args 2 "$#" "$1"
            case "$1" in
                --preset) preset=$2 ;;
                --voxel) voxel=$2 ;;
                --resolution) resolution=$2 ;;
                --floor-model) floor_model=$2 ;;
                --max-range|--known-free-radius|--floor-band|--obstacle-min|--obstacle-max) export_args+=("$1" "$2") ;;
                --plaster-grid) preview_grid=$2 ;;
                --plaster-ground-cut) ground_cut=$2 ;;
                --plaster-radius) preview_radius=$2 ;;
                --plaster-height) preview_height=$2 ;;
                --d-s|--d-p|--revisit-after)
                    [[ "$mode" == dufo ]] || usage_error "$1 仅用于 dufo"
                    dufo_args+=("$1" "$2") ;;
                *) usage_error "未知参数 $1" ;;
            esac
            shift 2
        done
        case "$preset" in
            detail) voxel=${voxel:-.03}; resolution=${resolution:-.05} ;;
            standard) voxel=${voxel:-.06}; resolution=${resolution:-.08} ;;
            quick) voxel=${voxel:-.12}; resolution=${resolution:-.12} ;;
            *) usage_error "未知降采样预设 $preset" ;;
        esac
        [[ "$floor_model" == plane || "$floor_model" == percentile ]] || usage_error "floor-model 只能是 plane 或 percentile"
        log_session
        if [[ -e "$out" || -L "$out" ]]; then
            if [[ -d "$out" ]] && "$python" - "$out" "$repo" <<'EXISTING_MAP'
import sys
sys.path.insert(0, sys.argv[2] + '/scripts/real_robot/web')
from map_review import load_map
try:
    load_map(sys.argv[1])
except Exception as error:
    sys.exit(f'已有目录无法预览：{error}')
EXISTING_MAP
            then
                say "输出目录已存在：${out}；保留已有地图，本次未重新导出或应用导出参数。"
                print_map_review "$out" "$input"
                say '如果网页服务和 SSH 转发已在运行，直接打开上述链接；需要重新导出时请选择新目录。'
                exit 3
            fi
            fail "输出路径已存在且不是可预览的有效地图：${out}；请选择新目录"
        fi
        "$python" -c 'import numpy, cv2' || fail "地图导出需要 NumPy 和 OpenCV，可设置 G1_PYTHON"
        if "$plaster"; then
            "$python" -c 'import scipy, skimage, matplotlib' || fail "石膏预览需要 scipy、scikit-image、matplotlib"
        fi
        if [[ "$mode" == dufo ]]; then
            [[ -d "$input" ]] || fail "DUFOMap 输入须为带时间戳和雷达原点的 NPZ 扫描目录"
            "$python" -c 'from dufomap import dufomap' || fail "未找到 DUFOMap Python 模块，见建图文档"
        else
            check_file "$input"
        fi
        mkdir -p "$(dirname "$out")"
        mkdir "$out" || fail "输出路径不可创建或已存在：${out}；请选择新目录"
        if [[ "$mode" == dufo ]]; then
            say "运行 DUFOMap"
            "$python" "$mapping/dufomap_clean.py" "$input" "$out/clean.pcd" "${dufo_args[@]}" 2>&1 | tee "$run_dir/dufo.log" ||
                fail "DUFOMap 清洗失败，见 dufo.log"
            input="$out/clean.pcd"
        fi
        say "导出 3D 体素 $voxel m、2D 栅格 $resolution m"
        "$python" "$mapping/export_g1_map.py" "$input" --out "$out" --resolution "$resolution" --voxel "$voxel" \
            --floor-model "$floor_model" --known-free-radius 0 "${export_args[@]}" 2>&1 | tee "$run_dir/export.log" ||
            fail "地图导出失败，检查 export.log 中的点云格式、地面拟合或地图尺寸"
        if "$plaster"; then
            "$python" "$mapping/plaster_surface.py" "$input" "$out" --grid "$preview_grid" --ground-cut "$ground_cut" \
                --radius "$preview_radius" --height "$preview_height" 2>&1 |
                tee "$run_dir/plaster.log" || fail "定位地图已导出；石膏预览生成失败，见 plaster.log"
        fi
        say "地图生成完成：$out"
        print_map_review "$out" "$input"

        ;;
    *) usage_error "未知建图任务 $mode" ;;
esac
