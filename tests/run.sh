#!/usr/bin/env bash
# Test entry. `unit` needs no simulator; `sim` and `walk` start the headless simulation.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
out="$repo/sim/generated/test"

usage() {
    cat <<'EOF'
用法：
  ./g1 test          单元测试（不需要仿真器，约 10 秒）
  ./g1 test sim      仿真场景测试：建图、带障碍导航、取消目标、高速行走时的定位精度（约 10 分钟）
  ./g1 test walk     行走速度扫描：各方向指令速度与实际速度对比（约 7 分钟）
结果保存在 sim/generated/test/。
EOF
}

ros_env() {
    set +u
    source "$repo/shared/env.sh"
    set -u
    export ROS_DOMAIN_ID=${G1_TEST_DOMAIN:-74} ROS_LOCALHOST_ONLY=1
}

step() { printf '\n=== %s ===\n' "$*"; }

wait_for_line() {
    local file=$1 pattern=$2 seconds=$3 pid=$4 end=$((SECONDS + $3))
    until grep -q -- "$pattern" "$file" 2>/dev/null; do
        kill -0 "$pid" 2>/dev/null || { tail -20 "$file"; echo "进程提前退出：$file"; return 1; }
        ((SECONDS < end)) || { tail -20 "$file"; echo "等待超时：$pattern"; return 1; }
        sleep 1
    done
}

case "${1:-unit}" in
    -h|--help) usage ;;
    unit)
        ros_env
        /usr/bin/python3 -m pytest "$repo/tests/unit" -q ;;
    sim)
        mkdir -p "$out"
        step '建图'
        "$repo/g1" sim map test --headless < <(printf 'w 10\nq 30\nw 10\nq 30\nw 6\n\n'; sleep 600) &
        wait $!
        step '导航到目标，途中放入障碍'
        "$repo/g1" sim nav test --headless --goal 3.0 0.0 --obstacle-after 2
        step '取消目标'
        "$repo/g1" sim nav test --headless --goal 3.0 0.0 --cancel-after 4
        step '高速行走时的定位精度'
        setsid "$repo/g1" sim nav test --headless >"$out/nav_session.log" 2>&1 < <(sleep 900) &
        session=$!
        trap 'kill -INT -- "-$session" 2>/dev/null || true; wait "$session" 2>/dev/null || true' EXIT
        wait_for_line "$out/nav_session.log" '定位和导航已就绪' 240 "$session"
        ros_env
        /usr/bin/python3 "$repo/tests/scenarios/localization_speed.py" --output "$out/localization_speed.json"
        step '仿真场景测试全部通过' ;;
    walk)
        mkdir -p "$out"
        ros_env
        export ROS_DOMAIN_ID=${G1_TEST_DOMAIN:-81}
        setsid "$repo/sim/start.sh" simulator --headless >"$out/walk_simulator.log" 2>&1 &
        simulator=$!
        trap 'kill -INT -- "-$simulator" "-${controller:-0}" 2>/dev/null || true; wait 2>/dev/null || true' EXIT
        sleep 6
        setsid "$repo/sim/start.sh" controller >"$out/walk_controller.log" 2>&1 </dev/null &
        controller=$!
        sleep 4
        /usr/bin/python3 "$repo/tests/scenarios/walking_sweep.py" --output "$out/walking_sweep.json" ;;
    *) usage; exit 2 ;;
esac
