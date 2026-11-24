#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/../env/walking_env.sh"
exec /usr/bin/python3 scripts/checks/validate_nav_goal.py "$@"
