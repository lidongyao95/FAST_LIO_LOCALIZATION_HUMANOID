#!/usr/bin/env bash
# Operator computer entry: move a real-robot PCD to the simulation host, build the
# scan scene there, and optionally start the MuJoCo navigation rehearsal.
set -euo pipefail

workflow=preview
say() { printf '[%s] %s\n' "$workflow" "$*" >&2; }
fail() {
    say "失败：$*"
    exit 1
}
usage_error() { say "$*（用 --help 查看用法）"; exit 2; }
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)

usage() {
    cat <<'EOF'
用法：
  03_preview.sh prepare SOURCE [--name NAME] [--radius M] [--start X Y] [--force] [--pc2 USER@HOST]
  03_preview.sh launch SCENE [-- 导航参数]
  03_preview.sh play SOURCE [--name NAME] [--radius M] [--start X Y] [--force] [--pc2 USER@HOST] [-- 导航参数]

在操作电脑（Mac）的仓库根目录运行：把实机地图送到 Ubuntu 仿真机并生成
sim/generated/NAME 场景；play 对 PCD 来源先准备再启动；传已有场景目录时直接启动 scan_preview.sh 预演。
MuJoCo、定位与 Nav2 全部在仿真机上运行，本机不跑仿真。
SOURCE 可为 pc2:/绝对/路径.pcd（从 PC2 读取）或本机 PCD 路径。
默认自动选择离扫描原点最近的空闲出生点，--start X Y 可显式指定（米）。
输出目录已存在时报错；--force 覆盖同一轮的场景资产。

环境变量：
  G1_SIM_HOST   仿真机 ssh 目标（默认 sjtu@202.121.181.124）
  G1_SIM_PORT   仿真机 ssh 端口（默认 4622）
  G1_SIM_REPO   仿真机仓库目录（默认 ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID）
  G1_SIM_DATA   仿真机数据目录（默认 ~/ldy/g1_real_data）
  G1_PC2        PC2 ssh 目标（默认 unitree@192.168.123.164；无线时显式覆盖）
  G1_SIM_DISPLAY 仿真机 X 显示（默认自动探测 /tmp/.X11-unix，优先 :11，其次 :0）
  G1_SIM_DOMAIN / ROS_DOMAIN_ID 仿真域（默认 75，不得使用实机域 223）
默认 4622 不可达或显存不足 2 GiB 时尝试 4322；显式 G1_SIM_PORT 不自动切换。
EOF
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac

SIM_HOST=${G1_SIM_HOST:-sjtu@202.121.181.124}
SIM_PORT=${G1_SIM_PORT:-4622}
PC2=${G1_PC2:-unitree@192.168.123.164}
SIM_DISPLAY=${G1_SIM_DISPLAY:-}

sim_ssh() { ssh -o LogLevel=ERROR -o BatchMode=yes -o ConnectTimeout=10 -p "$SIM_PORT" "$SIM_HOST" "mkdir -p ~/ldy && cd ~/ldy && $*"; }
pc2_ssh() { ssh -o LogLevel=ERROR -o BatchMode=yes -o ConnectTimeout=10 "$PC2" "$@"; }
quote() { printf '%q' "$1"; }

SIM_DOMAIN=${G1_SIM_DOMAIN:-${ROS_DOMAIN_ID:-75}}
[[ "$SIM_DOMAIN" =~ ^[0-9]+$ ]] && ((10#$SIM_DOMAIN>=1 && 10#$SIM_DOMAIN<=232 && 10#$SIM_DOMAIN!=223)) || usage_error '请选择 1–232 的仿真域，不得使用实机域 223'
ports=("$SIM_PORT")
if [[ -z "${G1_SIM_PORT:-}" && "$SIM_HOST" == sjtu@202.121.181.124 ]]; then ports=(4622 4322); fi
SIM_GPU=
for candidate_port in "${ports[@]}"; do
    SIM_PORT=$candidate_port
    say "检查仿真服务器 ${SIM_HOST}:${SIM_PORT}"
    if gpu_report=$(sim_ssh 'nvidia-smi --query-gpu=index,memory.free,utilization.gpu --format=csv,noheader,nounits'); then
        SIM_GPU=$(printf '%s\n' "$gpu_report" | python3 -c 'import sys; rows=[[int(v.strip()) for v in line.split(",")] for line in sys.stdin if line.strip()]; rows=[r for r in rows if r[1]>=2048]; rows.sort(key=lambda r:(r[2],-r[1])); print(rows[0][0] if rows else "")')
        [[ -z "$SIM_GPU" ]] || break
        say "显存余量不足 2 GiB，未启动任务"
    fi
done
[[ -n "$SIM_GPU" ]] || fail '仿真服务器无法连接或没有足够空闲显存；未终止任何已有任务'
server_home=$(sim_ssh 'printf %s "$HOME"') || fail "无法连接仿真机 $SIM_HOST:$SIM_PORT"
say "使用服务器 ${SIM_HOST}:${SIM_PORT} / GPU ${SIM_GPU} / 仿真域 ${SIM_DOMAIN}"
SIM_REPO=${G1_SIM_REPO:-$server_home/ldy/FAST_LIO_LOCALIZATION_HUMANOID}
SIM_DATA=${G1_SIM_DATA:-$server_home/ldy/g1_real_data}

# 仿真机的 X 显示不一定是 :0（本机实测为 :11）。未显式指定时按实际 socket 自动选择。
if [[ -z "$SIM_DISPLAY" ]]; then
    sockets=$(sim_ssh 'ls /tmp/.X11-unix 2>/dev/null' || true)
    pick=
    for n in 11 0 1 2 3; do
        if printf '%s\n' "$sockets" | grep -qx "X$n"; then pick=$n; break; fi
    done
    if [[ -z "$pick" ]]; then
        pick=$(printf '%s\n' "$sockets" | sed -n 's/^X\([0-9][0-9]*\)$/\1/p' | head -n 1)
    fi
    SIM_DISPLAY=":${pick:-0}"
    say "仿真机图形显示：${SIM_DISPLAY}（可用 G1_SIM_DISPLAY 覆盖）"
fi

preflight_sim() {
    say "预检仿真机 $SIM_HOST:${SIM_PORT}（只读）"
    local report
    report=$(sim_ssh "bash -s" <<REMOTE
set -u
repo=$(quote "$SIM_REPO")
cd "\$repo" 2>/dev/null || { echo "PROBLEM 仿真机缺少仓库目录：\$repo"; exit 0; }
for file in scripts/mapping/g1_scan_scene.py scripts/simulation/scan_preview.sh scripts/env/walking_env.sh; do
    [ -f "\$repo/\$file" ] || echo "PROBLEM 仿真机缺少 \$file"
done
[ -x "\$repo/.deps/venv/bin/python" ] || echo "PROBLEM 仿真机缺少 .deps/venv/bin/python"
[ -f "\$repo/.deps/src/unitree_mujoco/unitree_robots/g1/g1_29dof.xml" ] || echo "PROBLEM 仿真机缺少官方 G1 MuJoCo 模型（.deps/src/unitree_mujoco）"
free_kb=\$(df -Pk "\$repo" | awk 'NR == 2 {print \$4}')
if [ "\${free_kb:-0}" -lt 1048576 ]; then echo "PROBLEM 仿真机仓库分区剩余不足 1 GiB（\${free_kb:-?} KiB）"; fi
if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_free=\$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | sort -n | tail -1)
    if [ "\${gpu_free:-0}" -lt 2048 ]; then echo "WARN 仿真机显存余量 \${gpu_free:-?} MiB；无窗口模式可能仍可运行"; fi
fi
ls -d /opt/ros/*/setup.bash >/dev/null 2>&1 || echo "PROBLEM 仿真机未安装 ROS 2"
for pkg in nav2_map_server nav2_bringup; do
    ls /opt/ros/*/share/\$pkg >/dev/null 2>&1 || echo "WARN 仿真机缺少 ROS 包 \${pkg}（预演导航必需）；在仿真机执行：sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup"
done
ls /opt/ros/*/share/rviz2 >/dev/null 2>&1 || echo "WARN 仿真机未安装 rviz2；安装：sudo apt-get install -y ros-humble-rviz2，或用 -- --headless 无窗口运行"
echo "OK \$repo"
REMOTE
    ) || fail "仿真机预检命令执行失败"
    printf '%s\n' "$report" >&2
    if printf '%s\n' "$report" | grep -q '^PROBLEM'; then
        fail "仿真机环境不满足预演条件，先按上述清单补齐"
    fi
}

source_size() {
    local path=$1
    if [[ "$path" == pc2:* ]]; then
        local remote=${path#pc2:}
        pc2_ssh "test -s $(quote "$remote")" || fail "PC2 上不存在或为空：$remote"
        pc2_ssh "stat -c %s $(quote "$remote")" || fail "读取 PC2 文件大小失败：$remote"
    else
        [[ -s "$path" ]] || fail "本机不存在或为空：$path"
        wc -c <"$path" | tr -d ' '
    fi
}

send_pcd() {
    local source=$1 dest=$2
    if [[ "$source" == pc2:* ]]; then
        local remote=${source#pc2:}
        pc2_ssh "cat $(quote "$remote")" | sim_ssh "cat > $(quote "$dest")"
    else
        cat "$source" | sim_ssh "cat > $(quote "$dest")"
    fi
}

prepare_scene() {
    local source=$1 name=$2 radius=$3 size spawn_args
    [[ "$name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || fail "NAME 只能包含英数字、下划线和连字符，并以英数字开头"
    [[ "$radius" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "RADIUS 需要正数（米）"

    size=$(source_size "$source")
    if ((size <= 0)); then fail "来源 PCD 为空：$source"; fi
    say "来源 PCD 大小 $((size / 1024)) KiB"

    local pcd="$SIM_DATA/replay/$name.pcd"
    local scene="$SIM_REPO/sim/generated/$name"
    if sim_ssh "test -e $(quote "$scene")"; then
        if [[ "$force" == true ]]; then
            sim_ssh "rm -rf $(quote "$scene")"
        else
            fail "场景目录已存在：${scene}；换名或加 --force 覆盖同一轮资产"
        fi
    fi

    say "传输 PCD 到仿真机：$pcd"
    sim_ssh "mkdir -p $(quote "$SIM_DATA/replay")"
    send_pcd "$source" "$pcd"
    local remote_size
    remote_size=$(sim_ssh "stat -c %s $(quote "$pcd")") || fail "仿真机未收到 PCD"
    [[ "$remote_size" == "$size" ]] || fail "传输大小不一致（本机 ${size}，仿真机 ${remote_size}）"
    local source_hash remote_hash
    if [[ "$source" == pc2:* ]]; then
        source_hash=$(pc2_ssh "sha256sum $(quote "${source#pc2:}")")
    else
        source_hash=$(python3 -c 'import hashlib,sys; h=hashlib.sha256(); f=open(sys.argv[1],"rb"); [h.update(chunk) for chunk in iter(lambda:f.read(1024*1024),b"")]; print(h.hexdigest())' "$source")
    fi
    remote_hash=$(sim_ssh "sha256sum $(quote "$pcd")")
    [[ "${source_hash%% *}" == "${remote_hash%% *}" ]] || fail 'PCD SHA-256 校验失败'
    say 'PCD SHA-256 校验通过' 

    if [[ -n "$start_x" ]]; then
        spawn_args="--start $(quote "$start_x") $(quote "$start_y")"
        say "使用指定出生点 $start_x $start_y"
    else
        spawn_args=--auto-start
    fi

    say "在仿真机生成场景：${scene}（半径 $radius m）"
    local task_dir="$SIM_DATA/records/preview/$name-$(date +%Y%m%d-%H%M%S)-$$" generation_cmd job report previous_bytes=0
    sim_ssh "mkdir -p $(quote "$task_dir")"
    generation_cmd="cd $(quote "$SIM_REPO") && OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 .deps/venv/bin/python scripts/mapping/g1_scan_scene.py --pcd $(quote "$pcd") --out $(quote "$scene") --radius $(quote "$radius") $spawn_args"
    printf -v job '%s; code=$?; printf "%%s\\n" "$code" > %q; exit "$code"' "$generation_cmd" "$task_dir/status"
    sim_ssh "{ nohup bash -c $(quote "$job") > $(quote "$task_dir/generate.log") 2>&1 < /dev/null & echo \$! > $(quote "$task_dir/pid"); }"
    say "服务器任务证据：${task_dir}（中断后先检查 status，不重复启动）"
    while :; do
        report=$(sim_ssh "bash -s" <<REMOTE
set -e
run=$(quote "$task_dir")
if [ -f "\$run/status" ]; then
    printf 'FINISHED %s\n' "\$(cat "\$run/status")"
    tail -20 "\$run/generate.log"
elif kill -0 "\$(cat "\$run/pid")" 2>/dev/null; then
    bytes=\$(stat -c %s "\$run/generate.log")
    printf 'RUNNING pid=%s bytes=%s growth=%s\n' "\$(cat "\$run/pid")" "\$bytes" "\$((bytes-$previous_bytes))"
    tail -3 "\$run/generate.log"
else
    echo 'DEAD without status'
    tail -20 "\$run/generate.log"
fi
REMOTE
        ) || fail "服务器进度检查中断，先查看 $task_dir 的 pid/status/generate.log"
        printf '%s\n' "$report" >&2
        case "$report" in
            'FINISHED 0'*) break ;;
            FINISHED*|DEAD*) fail "场景生成/编译失败，查看 $task_dir/generate.log" ;;
        esac
        previous_bytes=$(printf '%s\n' "$report" | head -n 1 | sed -n 's/.* bytes=\([0-9]*\).*/\1/p')
        sleep 25
    done

    local missing
    missing=$(sim_ssh "bash -s" <<REMOTE
set -u
scene=$(quote "$scene")
for file in scene.xml map.ply map.yaml map.pgm metadata.json; do
    [ -s "\$scene/\$file" ] || echo "\$file"
done
REMOTE
    ) || fail "场景文件校验失败"
    if [[ -n "$missing" ]]; then fail "场景生成不完整，缺少：$missing"; fi
    say "场景就绪：$scene"
    printf '下一步启动预演：\n  G1_SIM_HOST=%q G1_SIM_PORT=%q G1_SIM_DOMAIN=%q ./scripts/simulation/03_preview.sh launch %q\n' "$SIM_HOST" "$SIM_PORT" "$SIM_DOMAIN" "sim/generated/$name" >&2
    printf '预演结束后回到 PC2 继续 04 持续定位；B4/B5 仍使用实机导出的地图。\n' >&2
}

scene_path() {
    local scene=$1
    case "$scene" in
        /*) printf '%s' "$scene" ;;
        *) printf '%s/%s' "$SIM_REPO" "$scene" ;;
    esac
}

launch_scene() {
    local remote_scene nav="" remote_cmd
    remote_scene=$(scene_path "$1")
    if ((${#navargs[@]})); then nav=$(printf '%q ' "${navargs[@]}"); fi
    sim_ssh "bash -s" <<REMOTE || fail "场景文件不完整：$remote_scene"
set -u
scene=$(quote "$remote_scene")
for file in scene.xml map.ply map.yaml; do
    [ -s "\$scene/\$file" ] || { echo "缺少 \$scene/\$file" >&2; exit 1; }
done
REMOTE
    if [[ " $nav " != *--start\ * ]]; then
        local spawn
        spawn=$(sim_ssh "python3 -c $(quote 'import json,sys; print(*json.load(open(sys.argv[1])).get("spawn_xy_m",[0.,0.]))') $(quote "$remote_scene/metadata.json")") || fail '无法读取场景起点'
        local spawn_x spawn_y
        read -r spawn_x spawn_y <<<"$spawn"
        nav="--start $(quote "$spawn_x") $(quote "$spawn_y") $nav"
    fi
    remote_cmd="mkdir -p ~/ldy && cd $(quote "$SIM_REPO") && export DISPLAY=$(quote "$SIM_DISPLAY") CUDA_VISIBLE_DEVICES=$(quote "$SIM_GPU") ROS_DOMAIN_ID=$(quote "$SIM_DOMAIN") ROS_LOCALHOST_ONLY=1 && ./scripts/simulation/scan_preview.sh $(quote "$remote_scene") $nav"
    local have_rviz
    have_rviz=$(sim_ssh "ls -d /opt/ros/*/share/rviz2 >/dev/null 2>&1 && echo yes || echo no")
    if [[ "$have_rviz" != yes && "$nav" != *--headless* ]]; then
        say '仿真机未安装 rviz2，图形化预演不可用。可任选一种处理：'
        printf 'ssh -p %s %s "sudo apt-get install -y ros-humble-rviz2"
' "$SIM_PORT" "$SIM_HOST" >&2
        say '或改用无窗口模式：命令末尾加 -- --headless（导航仍在仿真机运行，只是没有 RViz 窗口）'
        fail '缺少 rviz2，未启动预演'
    fi
    # Existing 4622 deployment keeps Open3D/PCL/Nav2 dependencies in a read-only overlay.
    local runtime_report
    runtime_report=$(sim_ssh "bash -s" <<REMOTE
cd $(quote "$SIM_REPO")
source scripts/env/walking_env.sh
missing=0
for binary in install/fast_lio/lib/fast_lio/fastlio_mapping install/open3d_loc/lib/open3d_loc/global_localization_node; do
    [ -x "\$binary" ] || { echo "缺少本机编译产物：\$binary" >&2; exit 1; }
    result=\$(ldd "\$binary" 2>&1)
    if printf '%s\n' "\$result" | grep -q 'not found'; then missing=1; fi
done
for pkg in nav2_map_server nav2_planner nav2_controller nav2_bt_navigator; do
    ros2 pkg prefix "\$pkg" >/dev/null 2>&1 || missing=1
done
[ "\$missing" = 0 ] || echo NEED_OVERLAY
REMOTE
    ) || fail '仿真运行环境检查失败'
    if [[ "$runtime_report" == *NEED_OVERLAY* ]]; then
        local validation
        validation="cd $(quote "$SIM_REPO") && source scripts/env/walking_env.sh && ! { ldd install/fast_lio/lib/fast_lio/fastlio_mapping; ldd install/open3d_loc/lib/open3d_loc/global_localization_node; } | grep 'not found'"
        sim_ssh "cd $(quote "$SIM_REPO") && bash scripts/env/ros_overlay_ns.sh bash -c $(quote "$validation")" ||
            fail '缺少仿真动态库且已有 ROS 覆盖层不可用；先完成服务器 A1 部署'
        say '自动使用服务器已有只读 ROS 覆盖层，不修改系统依赖'
        remote_cmd="mkdir -p ~/ldy && cd $(quote "$SIM_REPO") && export DISPLAY=$(quote "$SIM_DISPLAY") CUDA_VISIBLE_DEVICES=$(quote "$SIM_GPU") ROS_DOMAIN_ID=$(quote "$SIM_DOMAIN") ROS_LOCALHOST_ONLY=1 && bash scripts/env/ros_overlay_ns.sh ./scripts/simulation/scan_preview.sh $(quote "$remote_scene") $nav"
    fi
    say "在仿真机启动预演（Ctrl+C 结束本轮）"
    local terminal=-t
    [[ " $nav " != *--headless* ]] || terminal=-T
    ssh "$terminal" -o BatchMode=yes -o ServerAliveInterval=15 -o LogLevel=ERROR -p "$SIM_PORT" "$SIM_HOST" "$remote_cmd"
    say '预演结束。回到 PC2 继续 04 持续定位；预演地图不能用于实机导航。'
}

case "${1:-}" in ''|-h|--help) usage; exit 0 ;; esac
mode=$1
shift
case "$mode" in prepare|launch|play) ;; *) usage_error "请选择 prepare、launch 或 play" ;; esac

source= name= radius=8.0 force=false start_x= start_y=
navargs=()
while (($#)); do
    case "$1" in
        --name) (($# >= 2)) || usage_error "--name 需要参数"; name=$2; shift 2 ;;
        --radius) (($# >= 2)) || usage_error "--radius 需要参数"; radius=$2; shift 2 ;;
        --start) (($# >= 3)) || usage_error "--start 需要 X Y"; start_x=$2; start_y=$3; shift 3 ;;
        --pc2) (($# >= 2)) || usage_error "--pc2 需要 USER@HOST"; PC2=$2; shift 2 ;;
        --force) force=true; shift ;;
        --) shift; navargs=("$@"); break ;;
        -h|--help) usage; exit 0 ;;
        -*) usage_error "未知参数 $1" ;;
        *)
            [[ -z "$source" ]] || usage_error "只能指定一个 SOURCE 或 SCENE"
            source=$1
            shift
            ;;
    esac
done
[[ -n "$source" ]] || usage_error "$mode 需要 SOURCE 或 SCENE"
if [[ "$mode" == prepare ]] && ((${#navargs[@]})); then usage_error "prepare 不接受 -- 之后的导航参数"; fi

if [[ "$mode" == launch ]]; then
    launch_scene "$source"
    exit 0
fi

if [[ "$mode" == play && "$source" != pc2:* && "$source" != *.pcd ]]; then
    # Existing scene directories remain a supported direct-launch form.
    preflight_sim
    launch_scene "$source"
    exit 0
fi

if [[ -z "$name" ]]; then
    name=$(basename -- "$source")
    name=${name%.pcd}
fi
preflight_sim
prepare_scene "$source" "$name" "$radius"
if [[ "$mode" == play ]]; then launch_scene "sim/generated/$name"; fi
