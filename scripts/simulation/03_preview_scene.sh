#!/usr/bin/env bash
# Static, GPU-backed MuJoCo scene inspection on the Ubuntu desktop.
set -euo pipefail
if [[ "${1:-}" == --help || $# != 1 ]]; then
  echo '用法：03_preview_scene.sh sim/generated/B1_run03'
  echo '在服务器远程桌面终端执行；只预览，不启动 ROS 或导航。'
  [[ "${1:-}" == --help ]] && exit 0 || exit 2
fi
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
[[ -n "${DISPLAY:-}" ]] || { echo '[失败] 请在服务器远程桌面终端检查 DISPLAY。' >&2; exit 1; }
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia MUJOCO_GL=glfw
if ! report=$(glxinfo -B 2>&1); then echo "$report"; exit 1; fi
renderer=$(printf '%s\n' "$report" | sed -n 's/^OpenGL renderer string: //p')
[[ "$renderer" == *NVIDIA* ]] || { echo "[失败] 当前渲染器不是 NVIDIA：$renderer" >&2; exit 1; }
echo "[通过] GPU 渲染器：$renderer"
exec .deps/venv/bin/python scripts/simulation/preview_plaster_scene.py "$1"
