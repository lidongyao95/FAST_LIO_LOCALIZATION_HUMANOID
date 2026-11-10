#!/usr/bin/env bash
# Remove ROS processes left behind by earlier sessions in the same ROS_DOMAIN_ID.
# Restarting a launch with plain `kill` can orphan its static TF publishers and
# Nav2 nodes; duplicates then fight over the same frames. Run this before
# starting a new session, or set G1_CLEAN_STALE=1 for the start_* scripts.
set -uo pipefail

domain="${ROS_DOMAIN_ID:-72}"
dry_run="${1:-}"

pattern='static_transform_publisher|g1_nav_tf\.py|global_localization_node'
pattern+='|g1_real_velocity_bridge\.py|g1_navigation\.launch|g1_walking_localization\.launch'
pattern+='|fastlio_mapping|livox_ros_driver2_node|nav2_map_server|nav2_planner'
pattern+='|nav2_controller|nav2_bt_navigator|velocity_smoother|g1_mujoco_sensors'
pattern+='|g1_walking_sim|g1_command_bridge'

self=$$
targets=()
for pid in $(pgrep -f "$pattern" 2>/dev/null); do
    [[ "$pid" == "$self" ]] && continue
    cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    [[ "$cmdline" == *g1_cleanup_stale* ]] && continue
    pid_domain=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null |
                 sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
    if [[ -n "$pid_domain" && "$pid_domain" != "$domain" ]]; then
        continue
    fi
    targets+=("$pid")
done

if (( ${#targets[@]} == 0 )); then
    echo "No stale ROS processes in ROS_DOMAIN_ID=$domain"
    exit 0
fi

if [[ "$dry_run" == "--dry-run" ]]; then
    echo "Would stop ${#targets[@]} process(es) in ROS_DOMAIN_ID=$domain:"
    for pid in "${targets[@]}"; do
        printf '  %s  %s\n' "$pid" "$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-90)"
    done
    exit 0
fi

kill "${targets[@]}" 2>/dev/null
sleep 2
survivors=()
for pid in "${targets[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
        survivors+=("$pid")
    fi
done
if (( ${#survivors[@]} )); then
    kill -9 "${survivors[@]}" 2>/dev/null
    sleep 1
fi
echo "Stopped ${#targets[@]} stale process(es) in ROS_DOMAIN_ID=$domain"
