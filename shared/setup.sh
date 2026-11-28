#!/usr/bin/env bash
# Install local dependencies, build the simulated controller and the ROS packages, then self-check.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
case "${1:-}" in
    -h|--help) echo '用法：./g1_slam setup'; echo '安装本地依赖、编译仿真控制器和 ROS 包，最后自检。可以重复运行。'; exit 0 ;;
esac
jobs=${G1_BUILD_JOBS:-$(nproc)}
G1_BUILD_JOBS=$jobs "$repo/shared/setup_deps.sh"
/usr/bin/python3 "$repo/sim/build_controller.py"
G1_BUILD_JOBS=$jobs "$repo/shared/build_ros.sh"
"$repo/shared/self_check.sh" sim
