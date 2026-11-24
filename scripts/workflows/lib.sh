#!/usr/bin/env bash
set -eE -o pipefail

workflow_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$workflow_dir/../.." && pwd)"
run_dir=
pids=()
labels=()

say() { printf '[%s] %s\n' "$workflow" "$*" >&2; }
fail() {
    say "失败：$*"
    if [[ -n "$run_dir" ]]; then say "日志目录：$run_dir"; fi
    exit 1
}
usage_error() { say "$*（用 --help 查看用法）"; exit 2; }
need_args() {
    local expected=$1 remaining=$2 option=$3
    ((remaining >= expected)) || usage_error "$option 参数不足"
}
absolute() { "${G1_PYTHON:-python3}" -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().resolve())' "$1"; }
check_file() { [[ -s "$1" ]] || fail "缺少文件或文件为空：$1"; }

log_session() {
    local default_root=${1:-${G1_DATA:-$HOME/ldy/g1_real_data}/records/workflows}
    local root="${G1_WORKFLOW_LOGS:-$default_root}"
    root="$(absolute "$root")"
    mkdir -p "$root"
    run_dir="$(mktemp -d "$root/$workflow-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
    say "日志目录：$run_dir"
    trap 'fail "第 $LINENO 行执行失败（退出码 $?）"' ERR
}

stop_owned() {
    local signal pid active attempt
    for signal in INT TERM KILL; do
        active=0
        for pid in "${pids[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then
                kill "-$signal" -- "-$pid" 2>/dev/null || true
                active=1
            fi
        done
        ((active)) || break
        [[ "$signal" != KILL ]] || break
        for attempt in {1..30}; do
            active=0
            for pid in "${pids[@]}"; do
                if kill -0 -- "-$pid" 2>/dev/null; then active=1; fi
            done
            ((active)) || break
            sleep .2
        done
    done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}

load_ros() {
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$1}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
    export ROS_LOG_DIR="$run_dir/ros"
    if [[ -n "${G1_DATA:-}" ]]; then export G1_DATA="$(absolute "$G1_DATA")"; fi
    source "$repo/scripts/env/walking_env.sh" || fail "ROS 环境加载失败；先运行 01_self_check.sh"
    command -v setsid >/dev/null || fail "缺少 setsid（util-linux）"
    command -v flock >/dev/null || fail "缺少 flock（util-linux）"
    trap stop_owned EXIT
    trap 'say "收到中断，正在停止本次启动的进程"; exit 130' INT
    trap 'say "收到停止信号，正在停止本次启动的进程"; exit 143' TERM
    say "ROS ${ROS_DISTRO}，域 ${ROS_DOMAIN_ID}，localhost=$ROS_LOCALHOST_ONLY"
}

lock_run() {
    mkdir -p "$repo/.deps"
    exec 7>"$repo/.deps/workflow-$1.lock"
    flock -n 7 || fail "已有同类工作流运行（锁 $1）"
}

ros_packages() {
    local package
    for package in "$@"; do
        if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
            fail "ROS 找不到 ${package}；检查安装或运行 01_self_check.sh"
        fi
    done
}

start_bg() {
    local name=$1
    shift
    say "启动 ${name}，日志 $run_dir/$name.log"
    setsid -- "$@" 7>&- </dev/null >"$run_dir/$name.log" 2>&1 &
    pids+=("$!")
    labels+=("$name")
}

assert_running() {
    local index pid status
    for index in "${!pids[@]}"; do
        pid=${pids[$index]}
        if ! kill -0 "$pid" 2>/dev/null; then
            status=0
            wait "$pid" || status=$?
            fail "${labels[$index]} 已退出（$status），查看 $run_dir/${labels[$index]}.log"
        fi
    done
}

session_wait() { while true; do assert_running; sleep 1; done; }

probe() { /usr/bin/python3 "$workflow_dir/probe.py" "$@"; }
wait_ready() {
    if ! probe "$@" 2>&1 | tee "$run_dir/probe-$1.log"; then
        assert_running
        fail "未达到就绪条件：$*，查看 probe-$1.log"
    fi
    assert_running
}

nav_graph_empty() {
    probe free --node map_server --node planner_server --node controller_server \
        --node bt_navigator --node behavior_server --node recoveries_server \
        --topic /map --topic /global_costmap/costmap_raw --topic /cmd_vel_nav
}

preflight_processes() {
    say "进程预检（$1）：核对当前 ROS 域与本仓库，不自动停止进程"
    /usr/bin/python3 "$repo/scripts/env/g1_processes.py" --check --scope "$1" --repo "$repo" ||
        fail "有冲突进程或无法完成检查；按上方清单结束旧会话，再重试"
}
