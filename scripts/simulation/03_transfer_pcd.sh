#!/usr/bin/env bash
# Run on Mac: relay one PCD from PC2 to the selected server, without generating assets.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ "${1:-}" == --help ]]; then
  echo '用法：03_transfer_pcd.sh；读取 G1_PC2、G1_PCD、G1_SIM_HOST、G1_SIM_PORT、G1_SIM_DATA、G1_PREVIEW_NAME。仅传图并校验，不生成场景。'
  echo '可选参数：--scans PC2扫描目录 [--evidence PC2证据文件]；逐文件 SHA-256 校验，不生成场景。'
  exit 0
fi
exec python3 - "$repo" "$@" <<'PYTHON'
import argparse, os, shlex, subprocess, sys, uuid
sys.path.insert(0, sys.argv[1] + '/scripts/simulation')
from transfer_scan_inputs import transfer
parser = argparse.ArgumentParser()
parser.add_argument('--scans', help='PC2 上的同轮配对扫描绝对路径')
parser.add_argument('--evidence', help='PC2 上同轮配对证据的绝对路径')
args = parser.parse_args(sys.argv[2:])
if args.evidence and not args.scans:
    parser.error('--evidence 需要同时指定 --scans')
keys = ('G1_PC2','G1_PCD','G1_SIM_HOST','G1_SIM_PORT','G1_SIM_DATA','G1_PREVIEW_NAME')
v = {k: os.environ.get(k, '') for k in keys}
missing = [k for k in keys if not v[k]]
if missing:
    sys.exit('[失败] 请先 export：' + '、'.join(missing))
name = v['G1_PREVIEW_NAME']
if not name.isascii() or not name.replace('_','').replace('-','').isalnum():
    sys.exit('[失败] G1_PREVIEW_NAME 仅允许英数字、下划线、连字符。')
if not v['G1_SIM_PORT'].isdigit() or not 1 <= int(v['G1_SIM_PORT']) <= 65535:
    sys.exit('[失败] G1_SIM_PORT 必须是 1–65535。')
if not v['G1_PCD'].startswith('/') or not v['G1_SIM_DATA'].startswith('/'):
    sys.exit('[失败] G1_PCD 和 G1_SIM_DATA 必须是绝对路径。')
opts = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=15']
pc2 = opts + [v['G1_PC2']]
server = opts + ['-p',v['G1_SIM_PORT'],v['G1_SIM_HOST']]
q = shlex.quote
def run(conn, command):
    return subprocess.check_output(conn + [command], text=True).strip()
source = q(v['G1_PCD'])
dest = v['G1_SIM_DATA'].rstrip('/') + '/replay/' + name + '.pcd'
tmp = dest + '.partial-' + uuid.uuid4().hex
try:
    print('[检查] 读取 PC2 文件大小和 SHA-256…', flush=True)
    expected = run(pc2, 'test -s '+source+' && stat -c %s '+source+' && sha256sum '+source).splitlines()
    size, digest = expected[0], expected[1].split()[0]
    if run(server, 'if test -e '+q(dest)+'; then echo exists; fi'):
        actual = run(server, 'stat -c %s '+q(dest)+' && sha256sum '+q(dest)).splitlines()
        if actual[0] != size or actual[1].split()[0] != digest:
            sys.exit('[失败] 服务器已有不同内容的同名 PCD，请更换 G1_PREVIEW_NAME。')
        print('[通过] 服务器已有相同 PCD，无需重复传输。')
    else:
        run(server, 'mkdir -p '+q(os.path.dirname(dest)))
        print('[传输] PC2 → Mac 中转 → 服务器（不生成场景）', flush=True)
        with subprocess.Popen(pc2 + ['cat '+source], stdout=subprocess.PIPE) as reader:
            writer = subprocess.run(server + ['cat > '+q(tmp)], stdin=reader.stdout)
            reader.stdout.close()
            code = reader.wait()
        if code or writer.returncode:
            raise RuntimeError('传输中断')
        actual = run(server, 'stat -c %s '+q(tmp)+' && sha256sum '+q(tmp)).splitlines()
        if actual[0] != size or actual[1].split()[0] != digest:
            raise RuntimeError('大小或 SHA-256 不一致')
        run(server, 'mv -n '+q(tmp)+' '+q(dest))
        final = run(server, 'stat -c %s '+q(dest)+' && sha256sum '+q(dest)).splitlines()
        if final[0] != size or final[1].split()[0] != digest:
            raise RuntimeError('目标文件校验失败')
    print('[通过] PCD 大小：'+size+' 字节；SHA-256 校验一致。')
    print('[服务器 PCD] '+dest)
    transfer(args, pc2, server, v['G1_SIM_DATA'], name)
    print('下一步：在服务器终端使用此 PCD 生成场景。本步骤未生成或启动仿真。')
except (subprocess.SubprocessError, RuntimeError, OSError, IndexError, ValueError) as exc:
    sys.exit('[失败] '+str(exc)+'；若有未完成文件，仅检查本次临时路径：'+tmp)
PYTHON
