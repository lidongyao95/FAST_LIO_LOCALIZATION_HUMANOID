#!/usr/bin/env bash
# List by default. --check blocks a new stage; --stop explicitly ends selected processes.
set -e
exec /usr/bin/python3 "$(dirname -- "${BASH_SOURCE[0]}")/g1_processes.py" "$@"
