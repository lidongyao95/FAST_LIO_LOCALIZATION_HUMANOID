#!/usr/bin/env bash
# Supervise one PC2 A-segment mapping and recording run. Start the driver first.
set -euo pipefail

if [[ $# != 1 || ! $1 =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
    echo "Usage: $0 RUN_NAME" >&2
    exit 2
fi
run_name=$1
checkpoint_seconds="${G1_MAP_CHECKPOINT_SECONDS:-0}"
if [[ ! "$checkpoint_seconds" =~ ^(0|[1-9][0-9]*)$ ]]; then
    echo "G1_MAP_CHECKPOINT_SECONDS must be a nonnegative integer." >&2
    exit 2
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/walking_env.sh"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
records="$G1_DATA/records/$run_name"
bag="$G1_DATA/bags/$run_name"
map="$G1_DATA/maps/$run_name.pcd"

mkdir -p "$G1_DATA"/{bags,maps,records}
if [[ -e "$bag" || -e "$map" || -L "$bag" || -L "$map" ]]; then
    echo "Bag or map already exists for $run_name" >&2
    exit 1
fi
mkdir -- "$records"

# Job control lets children handle SIGINT even when the supervisor is detached.
set -m
mapper_pid=
recorder_pid=
checkpoint_pid=
stop_children() {
    trap '' INT TERM
    if [[ -n "$checkpoint_pid" ]]; then
        kill -TERM -- "-$checkpoint_pid" 2>/dev/null || true
        wait "$checkpoint_pid" 2>/dev/null || true
    fi
    for pid in "$recorder_pid" "$mapper_pid"; do
        if [[ -n "$pid" ]]; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    for pid in "$recorder_pid" "$mapper_pid"; do
        if [[ -n "$pid" ]]; then
            wait "$pid" || true
        fi
    done
}
trap stop_children EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

mapper="$(ros2 pkg prefix fast_lio)/lib/fast_lio/fastlio_mapping"
ros2 bag record -o "$bag" /livox/lidar /livox/imu >"$records/record.log" 2>&1 &
recorder_pid=$!
"$mapper" --ros-args --params-file "$G1_REPO/FAST_LIO/config/mid360.yaml" \
    -r /livox/custom_msg:=/livox/lidar -p use_sim_time:=false \
    -p "common.input_roll_180:=${G1_INPUT_ROLL_180:-true}" -p common.check_imu_z_up:=true \
    -p "map_file_path:=$map" -p "debug_log_dir:=$records" >"$records/lio.log" 2>&1 &
mapper_pid=$!

if [[ "$checkpoint_seconds" != 0 ]]; then
    (
        # Keep sleep and the service client in the checkpoint job's group.
        set +m
        while sleep "$checkpoint_seconds"; do
            date -Is
            timeout --foreground 15 ros2 service call /map_save std_srvs/srv/Trigger '{}' ||
                echo "Checkpoint service call failed or timed out."
        done
    ) >"$records/checkpoint.log" 2>&1 &
    checkpoint_pid=$!
fi

printf 'supervisor=%s mapper=%s recorder=%s\n' "$$" "$mapper_pid" "$recorder_pid" | tee "$records/pids"
echo "Save with /map_save while mapping is running. Stop both children with kill -TERM $$."
wait -n "$mapper_pid" "$recorder_pid"
