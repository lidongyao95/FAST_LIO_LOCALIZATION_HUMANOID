#!/usr/bin/env bash
# Run locally on the Ubuntu simulation server after source synchronization.
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo '用法：./scripts/simulation/03_server_check.sh'
  echo '在 Ubuntu 仿真服务器的仓库根目录执行，自动检查脚本所在仓库。'
  echo '仅检查服务器身份、代码目录及 GPU 余量；不编译、不启动仿真。'
  exit 0
fi
if (($#)); then echo '[失败] 本脚本不接受参数，请用 --help 查看用法。' >&2; exit 2; fi
if [[ "$(uname -s)" != Linux ]]; then
  echo '[失败] 请先 SSH 登录 Ubuntu 仿真服务器，再执行本脚本。' >&2
  exit 1
fi
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
printf '正在检查仿真服务器…\n'
printf '服务器名称：%s\n' "$(hostname)"
if ! cd "$repo"; then
  echo '[失败] 无法进入代码目录，请检查脚本所在仓库目录。' >&2
  exit 1
fi
printf '[通过] 代码目录：%s\n' "$PWD"
if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo '[失败] 未找到 nvidia-smi，请检查服务器 NVIDIA 驱动。' >&2
  exit 1
fi
if ! gpu_report=$(nvidia-smi --query-gpu=index,name,memory.free,utilization.gpu --format=csv,noheader,nounits); then
  echo '[失败] 无法读取 GPU 状态，请检查 NVIDIA 驱动及上方错误。' >&2
  exit 1
fi
printf '%s\n' "$gpu_report" | awk -F ',' '
{
  for (i=1; i<=NF; i++) gsub(/^[ \t]+|[ \t]+$/, "", $i)
  printf "GPU %s：%s，空闲显存 %s MiB，利用率 %s%%\n", $1, $2, $3, $4
  if ($3+0 >= 2048) available=1
}
END {
  if (available) print "[通过] 至少一张 GPU 空闲显存达到 2048 MiB，可继续检查构建依赖。"
  else {
    print "[失败] 没有 GPU 达到 2048 MiB 空闲显存，请切换服务器后重新核对并同步代码。"
    exit 1
  }
}'
echo '[通过] 服务器基本条件检查完成。'
