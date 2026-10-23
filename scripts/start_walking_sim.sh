#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/walking_env.sh"
exec 9>"$G1_REPO/.deps/walking-sim-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'Walking simulation already running'; exit 1; }
exec "$G1_REPO/.deps/venv/bin/python" scripts/g1_walking_sim.py --domain "$ROS_DOMAIN_ID" "$@"
