#!/usr/bin/env bash
# Start one simulation component. Called by sim/run.sh; each component holds a lock
# so a second copy cannot start and later components can check their prerequisites.
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/../shared/env.sh"
component=${1:?用法：start.sh simulator|controller|mapping|localization|navigation [参数]}
shift
lock() { printf '%s/.deps/walking-%s-%s.lock' "$G1_REPO" "$1" "$ROS_DOMAIN_ID"; }
hold() {
    exec 9>"$(lock "$1")"
    flock -n 9 || { echo "$1 已在运行" >&2; exit 1; }
}
require() {
    local name
    for name in "$@"; do
        if flock -n "$(lock "$name")" true; then
            echo "需要先启动 $name" >&2
            exit 1
        fi
    done
}

case "$component" in
    simulator)
        hold sim
        exec "$G1_REPO/.deps/venv/bin/python" "$G1_REPO/sim/simulator.py" --domain "$ROS_DOMAIN_ID" "$@" ;;
    controller)
        # The previous controller may still be shutting down; wait for its lock.
        exec 9>"$(lock controller)"
        for attempt in {1..60}; do
            if flock -n 9; then break; fi
            if ((attempt == 60)); then echo 'controller 已在运行' >&2; exit 1; fi
            sleep .25
        done
        require sim
        command_file="$G1_REPO/.deps/g1_command-${ROS_DOMAIN_ID}.txt"
        printf '%s 1 0 0 0\n' "$(date +%s.%N)" > "$command_file"
        /usr/bin/python3 "$G1_REPO/sim/command_bridge.py" &
        bridge=$!
        trap 'kill -INT "$bridge" 2>/dev/null || true' EXIT INT TERM
        "$G1_REPO/.deps/controller-build/g1_ctrl" "$G1_REPO/.deps/g1-controller/config.yaml" "$ROS_DOMAIN_ID" "$command_file" ;;
    mapping)
        hold lio
        require sim controller
        exec ros2 launch "$G1_REPO/sim/mapping.launch.py" "repo:=$G1_REPO" "$@" ;;
    localization)
        hold lio
        exec 8>"$(lock localization)"
        flock -n 8 || { echo 'localization 已在运行' >&2; exit 1; }
        require sim controller
        exec ros2 launch "$G1_REPO/shared/localization.launch.py" "repo:=$G1_REPO" "$@" ;;
    navigation)
        hold navigation
        require sim controller localization
        exec ros2 launch "$G1_REPO/shared/nav/navigation.launch.py" "repo:=$G1_REPO" "$@" ;;
    *) echo "未知组件 $component" >&2; exit 2 ;;
esac
