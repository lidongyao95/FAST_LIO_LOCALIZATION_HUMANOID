#!/usr/bin/env bash
# Read or switch the robot's locomotion mode. Navigation needs mode 501.
set -euo pipefail
case "${1:-}" in
    get) ;;
    set) [[ "${2:-}" =~ ^[0-9]+$ ]] || { echo '用法：./g1_slam robot fsm set 模式号'; exit 2; } ;;
    *) echo '用法：./g1_slam robot fsm get | set 模式号'; echo '网卡默认 eth0，可用 G1_ROBOT_INTERFACE 指定。'; exit 2 ;;
esac
tool="${G1_DATA:-$HOME/ldy/g1_real_data}/bin/g1_fsm"
[[ -x "$tool" ]] || { echo "缺少 $tool；运行 bash robot/sdk/build.sh"; exit 1; }
# The SDK ships its own DDS library; ROS's copy must not be on the library path.
exec env -u LD_LIBRARY_PATH "$tool" "${G1_ROBOT_INTERFACE:-eth0}" "$@"
