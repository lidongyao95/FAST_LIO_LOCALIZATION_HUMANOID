#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/walking_env.sh"
exec 9>"$G1_REPO/.deps/walking-navigation-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'G1 navigation already running'; exit 1; }
for required in walking-sim walking-controller walking-localization; do
    if flock -n "$G1_REPO/.deps/${required}-${ROS_DOMAIN_ID}.lock" true; then
        echo "Required ${required#walking-} process is not running" >&2
        exit 1
    fi
done
test -s sim/generated/room.yaml || { echo 'Occupancy map missing; generate scene first'; exit 1; }
exec ros2 launch open3d_loc g1_navigation.launch.py "repo:=$G1_REPO" "$@"
