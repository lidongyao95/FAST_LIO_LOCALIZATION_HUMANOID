#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/walking_env.sh"
exec 9>"$G1_REPO/.deps/walking-controller-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'G1 controller already running'; exit 1; }
if flock -n "$G1_REPO/.deps/walking-sim-${ROS_DOMAIN_ID}.lock" true; then
    echo 'Walking simulation is not running; start scripts/start_walking_sim.sh first' >&2
    exit 1
fi
COMMAND_FILE="$G1_REPO/.deps/g1_command-${ROS_DOMAIN_ID}.txt"
printf '%s 1 0 0 0\n' "$(date +%s.%N)" > "$COMMAND_FILE"
/usr/bin/python3 scripts/g1_command_bridge.py &
BRIDGE_PID=$!
trap 'kill -INT "$BRIDGE_PID" 2>/dev/null || true' EXIT INT TERM
"$G1_REPO/.deps/controller-build/g1_ctrl" "$G1_REPO/.deps/g1-controller/config.yaml" "$ROS_DOMAIN_ID" "$COMMAND_FILE"
