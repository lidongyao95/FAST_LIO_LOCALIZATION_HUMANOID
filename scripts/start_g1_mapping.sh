#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/walking_env.sh"
exec 8>"$G1_REPO/.deps/walking-lio-${ROS_DOMAIN_ID}.lock"
flock -n 8 || { echo 'FAST-LIO mapping or localization already running'; exit 1; }
for required in walking-sim walking-controller; do
    if flock -n "$G1_REPO/.deps/${required}-${ROS_DOMAIN_ID}.lock" true; then
        echo "Required ${required#walking-} process is not running" >&2
        exit 1
    fi
done
mkdir -p sim/generated/mapping sim/generated/logs
exec ros2 launch open3d_loc g1_walking_mapping.launch.py "repo:=$G1_REPO" "$@"
