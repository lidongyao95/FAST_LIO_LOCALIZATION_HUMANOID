#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/../env/walking_env.sh"
LOCK="$G1_REPO/.deps/walking-controller-${ROS_DOMAIN_ID}.lock"
exec 9>"$LOCK"
# 上一轮的控制器可能还在收尾；等它真正释放锁，避免"刚停就重开"直接失败。
for attempt in {1..60}; do
    if flock -n 9; then break; fi
    if ((attempt == 60)); then
        echo "G1 controller already running（锁 $LOCK 被占用，持有者：$(fuser "$LOCK" 2>/dev/null || echo 未知)）；先结束原会话或等待收尾" >&2
        exit 1
    fi
    sleep .25
done
if flock -n "$G1_REPO/.deps/walking-sim-${ROS_DOMAIN_ID}.lock" true; then
    echo 'Walking simulation is not running; start scripts/simulation/start_walking_sim.sh first' >&2
    exit 1
fi
COMMAND_FILE="$G1_REPO/.deps/g1_command-${ROS_DOMAIN_ID}.txt"
printf '%s 1 0 0 0\n' "$(date +%s.%N)" > "$COMMAND_FILE"
/usr/bin/python3 scripts/simulation/g1_command_bridge.py &
BRIDGE_PID=$!
trap 'kill -INT "$BRIDGE_PID" 2>/dev/null || true' EXIT INT TERM
"$G1_REPO/.deps/controller-build/g1_ctrl" "$G1_REPO/.deps/g1-controller/config.yaml" "$ROS_DOMAIN_ID" "$COMMAND_FILE"
