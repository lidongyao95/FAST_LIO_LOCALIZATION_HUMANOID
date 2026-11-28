#!/usr/bin/env bash
# Run on the deployment computer before copying sources or starting PC2 workloads.
set -euo pipefail
case "${1:-check}" in
  -h|--help)
    echo "用法：./g1 robot deploy [check|sync|build]"
    echo '在开发机通过有线 SSH 核对 PC2 身份并查询当次无线 IP；检查进程/服务、部署依赖/产物及反向源码差异；只读，不启动机器人程序。'
    echo 'sync：以本地源码覆盖 PC2，并删除同步范围内远端多余文件；保留排除的构建产物与数据。'
    echo 'build：构建已同步的 PC2 ARM64/Foxy 源码，后台记录日志并实时跟踪，成功后运行 robot 自检。'
    exit 0 ;;
  check|sync|build) ;;
  *) echo '错误：当前支持 check、sync、build；使用 --help 查看用法。' >&2; exit 2 ;;
esac
action=${1:-check}
[[ $# -le 1 ]] || { echo '错误：参数过多。' >&2; exit 2; }
command -v rsync >/dev/null || { echo "错误：需要 rsync。" >&2; exit 1; }
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v ssh >/dev/null || { echo '错误：需要 ssh。' >&2; exit 1; }
log_dir="${G1_SETUP_LOG_DIR:-${TMPDIR:-/tmp}/g1-pc2-setup}"
mkdir -p "$log_dir"
log_file=$(mktemp "$log_dir/check-$(date +%Y%m%d-%H%M%S)-XXXXXX")
echo "第一步：通过有线地址 unitree@192.168.123.164 检查 PC2。"
echo "日志：$log_file"
if ssh -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
  unitree@192.168.123.164 'bash -s' <<'REMOTE' 2>&1 | tee -a "$log_file"
set -euo pipefail
printf '主机：'; hostname
printf '架构：'; uname -m
[[ "$(uname -m)" == aarch64 ]] || { echo '错误：目标不是 ARM64 PC2。'; exit 1; }
ip -4 addr show eth0
ip -4 -o addr show eth0 | awk '$4 == "192.168.123.164/24" { found=1 } END { exit !found }' || {
  echo '错误：eth0 地址不符合 PC2 有线配置。'; exit 1;
}
. /etc/os-release
printf '系统：%s\n' "$PRETTY_NAME"
[[ "$ID" == ubuntu && "$VERSION_ID" == 20.04 ]] || { echo '错误：目标系统不是 Ubuntu 20.04。'; exit 1; }
[[ -f /opt/ros/foxy/setup.bash ]] || { echo '错误：缺少 ROS 2 Foxy。'; exit 1; }
df -h "$HOME"
echo '有线连接与 PC2 环境身份检查通过。'
echo '当次无线状态（DHCP 地址可能变化）：'
if command -v nmcli >/dev/null; then
  nmcli -f GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY device show wlan0 || true
  nmcli -f ipv4.method,ipv4.addresses connection show sic-guest || true
fi
ip -4 addr show wlan0 || true
echo '第二步：部署进程与自动重启服务（只读）'
ps -eo user,pid,ppid,args | grep -E '[r]os2|[f]astlio|[l]ivox|[g]lobal_localization|[n]av2|[g]1_|[g]amepad_combo|[r]obot/localize|[r]obot/nav' || true
systemctl show whu_rb.service -p ActiveState -p UnitFileState -p Restart -p ExecStart || true
systemctl list-units --type=service --state=running --no-pager || true
systemctl --user list-units --type=service --state=running --no-pager || true
echo '第三步：源码、依赖与安装产物（存在不代表版本一致）'
/usr/bin/python3 - <<'PYREMOTE'
import os
from pathlib import Path
repo = Path.home() / 'ldy/FAST_LIO_LOCALIZATION_HUMANOID'
checks = {
 '源码': ['g1', 'shared/self_check.sh', 'shared/processes.py',
          'packages/fast_lio/package.xml', 'packages/open3d_loc/package.xml',
          '.deps/src/livox_ros_driver2/package.xml', '.deps/src/Livox-SDK2/CMakeLists.txt'],
 '安装产物': ['install/local_setup.bash',
          'install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node',
          'install/fast_lio/lib/fast_lio/fastlio_mapping',
          'install/open3d_loc/lib/open3d_loc/global_localization_node'],
}
missing = []
for group, paths in checks.items():
 print(group + '：')
 for name in paths:
  path = repo / name
  ok = path.is_file()
  if '/lib/' in name:
   ok = ok and os.access(path, os.X_OK)
  print(('  OK ' if ok else '  缺失/不可执行 ') + name)
  if not ok: missing.append(name)
print('本机依赖候选（需结合构建配置核对版本）：')
for pattern in ('.deps/**/liblivox_lidar_sdk_shared.so', '.deps/**/Open3DConfig.cmake'):
 print(' ', pattern, ':', ', '.join(str(p.relative_to(repo)) for p in repo.glob(pattern)) or '未发现本地候选，检查系统安装')
if not repo.is_dir():
 print('待处理：首次部署，需要同步源码并准备 ARM64/Foxy 本机环境。')
elif missing:
 print('待处理：补齐上述源码或本机安装产物，再运行 robot 自检。')
else:
 print('文件完整性检查通过；仍需核对源码差异、动态库与运行自检。')
PYREMOTE
# ldd 只检查本部署二进制；不调用会启动 ROS daemon 或写工作流日志的自检入口。
if [[ -f "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID/install/local_setup.bash" ]]; then
  set +u
  source /opt/ros/foxy/setup.bash
  source "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID/install/local_setup.bash"
  set -u
  echo 'ROS 包安装位置（缺失项需补齐或构建）：'
  for package in livox_ros_driver2 fast_lio open3d_loc nav2_map_server nav2_planner; do
    ros2 pkg prefix "$package" || echo "待处理：缺少 ROS 包 $package"
  done
  /usr/bin/python3 -c 'import numpy; from scipy.spatial import cKDTree; import rclpy' || echo '待处理：补齐 Python NumPy/SciPy/rclpy 依赖'
  dep_repo="$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
  export LD_LIBRARY_PATH="$dep_repo/.deps/sdk/lib:$dep_repo/.deps/root/usr/lib/aarch64-linux-gnu:$dep_repo/.deps/arm-open3d/runtime/usr/lib/aarch64-linux-gnu:/usr/local/lib:${LD_LIBRARY_PATH:-}"
  for binary in "$dep_repo"/install/{livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node,fast_lio/lib/fast_lio/fastlio_mapping,open3d_loc/lib/open3d_loc/global_localization_node}; do
    if [[ -x "$binary" ]]; then
      echo "动态库：$binary"
      ldd "$binary" || true
    fi
  done
  echo 'Open3D 本地运行库解析（构建前预检）：'
  bash "$dep_repo/shared/open3d_probe.sh" ||
    echo '待处理：Open3D 传递依赖不完整；链接或运行会缺库，按上方提示补齐后再构建'
fi
REMOTE
then
  echo '有线与远端检查完成。' | tee -a "$log_file"
else
  echo '第一步失败：请检查网线、开发机 192.168.123.x/24 地址、SSH 认证及上述设备检查。' | tee -a "$log_file" >&2
  exit 1
fi

if [[ "$action" == build ]]; then
  echo '第四步：PC2 本机构建与自检；不自动同步或重启运行会话。' | tee -a "$log_file"
  ssh -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
    unitree@192.168.123.164 'cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID && bash robot/pc2_build.sh' \
    2>&1 | tee -a "$log_file"
  exit 0
fi

excludes=(
  --exclude='.git/' --exclude='build/' --exclude='install/' --exclude='log/'
  --exclude='.deps/sdk*' --exclude='.deps/root/' --exclude='vendor/'
  --exclude='doc/' --exclude='tmp/' --exclude='.deps/arm-open3d/'
  --exclude='sim/generated/' --exclude='__pycache__/' --exclude='*.pyc'
  --exclude='.deps/dufomap-venv/' --exclude='.deps/offline-venv/' --exclude='.deps/venv/' --exclude='.deps/controller-build/' --exclude='.deps/g1-controller/'
  --exclude='.deps/src/unitree_mujoco/' --exclude='.deps/src/unitree_rl_mjlab/'
)
if [[ "$action" == sync ]]; then
  echo '第四步：以本地源码为准同步 PC2（含删除远端多余源码）' | tee -a "$log_file"
  ssh -o ConnectTimeout=6 unitree@192.168.123.164 'mkdir -p ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID'
  # 排除项同时保护接收端；禁止 --delete-excluded，保留 PC2 本机构建与运行数据。
  rsync -aic --delete-delay "${excludes[@]}" \
    -e 'ssh -o LogLevel=ERROR -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2' \
    "$repo/" unitree@192.168.123.164:ldy/FAST_LIO_LOCALIZATION_HUMANOID/ \
    2>&1 | tee -a "$log_file"
  echo '第五步：校验同步结果' | tee -a "$log_file"
  rsync -ainc --delete "${excludes[@]}" \
    -e 'ssh -o LogLevel=ERROR -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2' \
    "$repo/" unitree@192.168.123.164:ldy/FAST_LIO_LOCALIZATION_HUMANOID/ \
    > "$log_file.sync-verify" 2>&1
  cat "$log_file.sync-verify" | tee -a "$log_file"
  # 旧源码目录可能仅剩受保护的 build/__pycache__，rsync 会报告无法删除空壳目录。
  # 这些目录保留；任何文件差异或其他诊断仍使校验失败。
  if ! awk '
    /^cannot delete non-empty directory: / { next }
    /^\*deleting .*\/$/ { next }
    NF { bad=1 }
    END { exit bad }
  ' "$log_file.sync-verify"; then
    echo '校验仍有差异或诊断输出，请检查上述清单。' >&2
    exit 1
  fi
  echo "源码同步与内容校验完成。日志：$log_file" | tee -a "$log_file"
  echo '未重建 install，也未停止或重启现有进程；运行中的程序可能仍使用旧版本。'
  exit 0
fi
echo '第四步：反向源码差异（PC2 → Mac，dry-run；不写入源码）' | tee -a "$log_file"
# -c 比较内容，避免只因跨机时间戳不同而误报；无 --delete。
if rsync -ainc -e 'ssh -o LogLevel=ERROR -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2' \
  "${excludes[@]}" unitree@192.168.123.164:ldy/FAST_LIO_LOCALIZATION_HUMANOID/ "$repo/" \
  2>&1 | tee "$log_file.reverse-diff" | tee -a "$log_file"; then
  if [[ -s "$log_file.reverse-diff" ]]; then
    echo '待处理：审阅反向差异，保留现场改动并合并后，再预览正向同步；不能据此自动判定哪端更新。'
  else
    echo '反向检查未发现差异；此检查不列出仅存在于 Mac 的文件，正向同步前仍需 dry-run。'
  fi
else
  echo '源码差异检查失败：若远端仓库尚不存在，按首次部署流程准备目录和同步；否则检查权限/传输错误。' >&2
  exit 1
fi
printf '检查完成。日志：%s\n差异清单：%s.reverse-diff\n' "$log_file" "$log_file"
echo '按报告处理进程冲突、依赖缺失与源码差异，再决定同步/构建；本次没有执行清理、同步或构建。'
