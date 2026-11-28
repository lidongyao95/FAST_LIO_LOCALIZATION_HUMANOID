#!/usr/bin/env bash
# Runs on PC2, started by `./g1_slam robot deploy build`. The build keeps going if SSH disconnects.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo"
verify_install() {
  export ROS_DISTRO=foxy ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
  bash shared/self_check.sh robot || return
  set +u
  source shared/env.sh || return
  set -u
  local binary dependencies
  for binary in install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node \
      install/fast_lio/lib/fast_lio/fastlio_mapping \
      install/open3d_loc/lib/open3d_loc/global_localization_node; do
    file "$binary" || return
    file -L "$binary" | grep -q 'ARM aarch64' || { echo "错误：不是 ARM64 程序：$binary"; return 1; }
    dependencies=$(ldd "$binary" 2>&1) || { echo "$dependencies"; return 1; }
    if [[ "$dependencies" == *'not found'* ]]; then
      echo "$dependencies"
      echo "错误：动态库缺失：$binary"
      return 1
    fi
    echo "动态库检查通过：$binary"
  done
}
if [[ "${1:-}" == --worker ]]; then
  run_dir=$2
  set +e
  ROS_DISTRO=foxy G1_BUILD_JOBS=2 bash shared/build_ros.sh
  result=$?
  if ((result == 0)); then
    bash robot/sdk/build.sh
    result=$?
  fi
  if ((result == 0)); then
    echo '构建通过，开始实机依赖与安装自检。'
    verify_install
    result=$?
  fi
  printf '%s\n' "$result" > "$run_dir/status.tmp"
  mv "$run_dir/status.tmp" "$run_dir/status"
  exit "$result"
fi
[[ $# == 0 ]] || { echo '不支持的构建参数。' >&2; exit 2; }
[[ $(uname -m) == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo '需要 PC2 ARM64/Foxy 环境。' >&2; exit 1;
}
mkdir -p "$HOME/ldy/g1_real_data/records/builds"
exec 9>"$HOME/ldy/g1_real_data/records/builds/pc2-build.lock"
flock -n 9 || { echo '已有 PC2 构建运行；请先查看 records/builds 下最新日志。' >&2; exit 1; }
/usr/bin/python3 shared/processes.py --check --scope all --domain 223
run_dir=$(mktemp -d "$HOME/ldy/g1_real_data/records/builds/pc2-$(date +%Y%m%d-%H%M%S)-XXXXXX")
nohup bash "$0" --worker "$run_dir" > "$run_dir/build.log" 2>&1 < /dev/null &
pid=$!
printf '%s\n' "$pid" > "$run_dir/pid"
echo "后台构建 PID=$pid，日志=$run_dir/build.log"
# Check completion every second; report process, byte growth and package state every 25s.
previous=0
next_report=0
while :; do
  if [[ -f "$run_dir/status" ]]; then
    cat "$run_dir/build.log"
    result=$(cat "$run_dir/status")
    echo "构建/自检终态：exit=$result，证据目录=$run_dir"
    if ((result != 0)); then
      grep -nE 'error:|CMake Error|No rule to make target|undefined reference|Failed|Missing|not found' "$run_dir/build.log" || true
    fi
    exit "$result"
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    echo '构建进程已退出但未写入终态；查看日志，禁止直接重试。' >&2
    tail -60 "$run_dir/build.log"
    exit 1
  fi
  if ((SECONDS >= next_report)); then
    size=$(stat -c %s "$run_dir/build.log")
    echo "进度：PID=$pid 存活，日志=$size bytes，较上次增加 $((size-previous)) bytes"
    tail -12 "$run_dir/build.log"
    grep -E 'Finished|Failed|Aborted|Summary:' "$run_dir/build.log" || true
    previous=$size
    next_report=$((SECONDS+25))
  fi
  sleep 1
done
