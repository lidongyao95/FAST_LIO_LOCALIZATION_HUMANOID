#!/usr/bin/env bash
set -e
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$REPO_ROOT/.deps/g1-demo-${ROS_DOMAIN_ID:-71}.pid"
[[ -f "$PID_FILE" ]] || { echo 'No recorded G1 demo process'; exit 0; }
read -r DEMO_PID < "$PID_FILE"
[[ "$DEMO_PID" =~ ^[0-9]+$ ]] || exit 1
if [[ -r "/proc/$DEMO_PID/cmdline" ]] &&
   tr '\0' '\n' < "/proc/$DEMO_PID/cmdline" | /usr/bin/grep -qx 'g1_mujoco.launch.py'; then
    kill -INT "$DEMO_PID"
    echo "Stopping G1 demo (PID $DEMO_PID)"
else
    echo 'The recorded G1 demo is no longer running'
fi
