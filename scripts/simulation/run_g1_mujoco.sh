#!/usr/bin/env bash
set -e
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"
source "$REPO_ROOT/scripts/env/walking_env.sh"
exec 9>"$REPO_ROOT/.deps/g1-demo-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo "G1 demo is already running on ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >&2; exit 1; }
echo "$$" > "$REPO_ROOT/.deps/g1-demo-${ROS_DOMAIN_ID}.pid"
export OMP_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=1
"$REPO_ROOT/.deps/venv/bin/python" scripts/simulation/g1_mujoco_sensors.py --generate-only
exec ros2 launch "$REPO_ROOT/open3d_loc/launch/g1_mujoco.launch.py" repo:="$REPO_ROOT" "$@"
