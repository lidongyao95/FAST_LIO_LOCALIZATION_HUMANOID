#!/usr/bin/env bash
# Server-local scene generation with persistent evidence and progress monitoring.
set -euo pipefail
if [[ "${1:-}" == --help || $# == 0 ]]; then
  echo '用法：03_generate_scene.sh /服务器上的/地图.pcd [--rebuild] [生成参数]'
  echo '常用参数：--voxel M --origin-clear-radius M --radius M --plaster-grid M --plaster-ground-cut M --plaster-height M --max-faces N'
  echo '逐帧参数：--scans DIR --near-radius M --sample-period S --dufo --dufo-d-s M --dufo-d-p N'
  echo 'IMU 筛帧：--evidence FILE --imu-max-accel M；完整参数见 g1_scan_scene.py --help'
  echo '--rebuild 先归档同名旧场景；不修改源 PCD 或扫描。'
  echo '场景名称取 PCD 文件名；输出到本仓库 sim/generated/同名目录。'
  [[ "${1:-}" == --help ]] && exit 0 || exit 2
fi
[[ "$(uname -s)" == Linux ]] || { echo '[失败] 请在 Ubuntu 仿真服务器执行。' >&2; exit 1; }
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
pcd=$(realpath -- "$1")
shift
rebuild=false
generator_args=()
while (($#)); do
  case "$1" in
    --rebuild) rebuild=true; shift ;;
    --pcd|--out|--pcd=*|--out=*) echo '[失败] 此入口固定输入及输出路径，不接受 --pcd/--out。' >&2; exit 2 ;;
    *) generator_args+=("$1"); shift ;;
  esac
done
[[ -s "$pcd" ]] || { echo '[失败] PCD 不存在或为空。' >&2; exit 1; }
name=$(basename -- "$pcd" .pcd)
[[ "$name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || { echo '[失败] PCD 名称仅允许英数字、下划线、连字符。' >&2; exit 1; }
out="$repo/sim/generated/$name"
[[ -x .deps/venv/bin/python ]] || { echo '[失败] 缺少服务器 .deps/venv。' >&2; exit 1; }
# Validate every argument before archiving existing results.
.deps/venv/bin/python - "$pcd" "$out" "${generator_args[@]}" <<'PYARGS'
import sys
# Reuse the generator parser without starting reconstruction or MuJoCo compilation.
sys.path.insert(0, 'scripts/mapping')
from g1_scan_scene import parse_arguments
parse_arguments(['--pcd', sys.argv[1], '--out', sys.argv[2], '--auto-start', *sys.argv[3:]])
PYARGS
exec 9>"$repo/.deps/plaster-generation.lock"
flock -n 9 || { echo '[失败] 已有场景生成任务，请先检查它的日志和终态。' >&2; exit 1; }
if [[ -e "$out" && "$rebuild" != true ]]; then
  echo "[失败] 场景已存在：$out；需要调参重建时显式添加 --rebuild（旧结果自动归档）。" >&2
  exit 1
fi
if ! .deps/venv/bin/python -c 'import scipy, skimage, fast_simplification'; then
  echo '[失败] 石膏重建依赖缺失。请在服务器执行：' >&2
  echo '.deps/venv/bin/python -m pip install --only-binary=:all: scipy scikit-image fast-simplification' >&2
  exit 1
fi
run="${G1_DATA:-$HOME/ldy/g1_real_data}/records/preview"
mkdir -p "$run"
run=$(mktemp -d "$run/$name-XXXXXXXX")
if [[ -e "$out" ]]; then
  mv -- "$out" "$run/previous-scene"
  .deps/venv/bin/python - "$out" "$run/previous-scene" <<'PYARCHIVE'
import json, sys
from pathlib import Path
import xml.etree.ElementTree as ET
old, archived = map(Path, sys.argv[1:])
scene = archived / 'scene.xml'
if scene.is_file():
    tree = ET.parse(scene)
    for mesh in tree.getroot().iter('mesh'):
        value = mesh.get('file')
        if value and Path(value).is_absolute() and Path(value).parent == old:
            mesh.set('file', str(archived / Path(value).name))
    tree.write(scene)
meta = archived / 'metadata.json'
if meta.is_file():
    data = json.loads(meta.read_text())
    data['archived_from'] = str(old)
    for key in ('scene', 'filter_report', 'filtered_source'):
        if key in data and Path(data[key]).parent == old:
            data[key] = str(archived / Path(data[key]).name)
    meta.write_text(json.dumps(data, indent=2, ensure_ascii=False))
PYARCHIVE
  echo "[归档] 上一轮场景：$run/previous-scene"
fi
printf -v job '%q ' env PYTHONUNBUFFERED=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 "$repo/.deps/venv/bin/python" "$repo/scripts/mapping/g1_scan_scene.py" --pcd "$pcd" --out "$out" --auto-start "${generator_args[@]}"
printf -v status_path '%q' "$run/status"
nohup bash -c "$job; code=\$?; printf '%s\n' "\$code" > $status_path; exit "\$code"" >"$run/generate.log" 2>&1 < /dev/null &
pid=$!
echo "$pid" > "$run/pid"
echo "[生成] 输出目录：$out"
printf '[参数] %s\n' "${generator_args[*]:-使用默认参数}"
echo "[日志] $run（断开后先查 status、pid、generate.log，不重复生成）"
previous=0
next=0
while [[ ! -f "$run/status" ]]; do
  if ! kill -0 "$pid" 2>/dev/null; then
    [[ -f "$run/status" ]] && break
    echo '[失败] 生成进程退出但缺少终态。' >&2
    tail -20 "$run/generate.log"
    exit 1
  fi
  if ((SECONDS >= next)); then
    bytes=$(stat -c %s "$run/generate.log")
    echo "[进行中] PID=$pid，日志 $bytes 字节，新增 $((bytes-previous)) 字节"
    tail -3 "$run/generate.log"
    previous=$bytes
    next=$((SECONDS+25))
  fi
  sleep 1
done
code=$(cat "$run/status")
tail -20 "$run/generate.log"
[[ "$code" == 0 ]] || { echo "[失败] 生成退出码 $code，请检查上方日志。" >&2; exit 1; }
for file in scene.xml plaster.obj map.ply map.yaml map.pgm metadata.json filtered.pcd filter_report.json; do
  [[ -s "$out/$file" ]] || { echo "[失败] 缺少产物 $file" >&2; exit 1; }
done
echo "[通过] 石膏场景及配套文件已生成：$out"
echo '本步骤未启动仿真；下一步检查场景与地图后再启动。'
