#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/walking_env.sh"
exec /usr/bin/python3 scripts/validate_walking.py "$@"
