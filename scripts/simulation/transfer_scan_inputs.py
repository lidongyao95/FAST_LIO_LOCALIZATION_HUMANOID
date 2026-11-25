"""Stream optional exact-pair scan inputs through the operator host, with hashes."""
import json
from pathlib import PurePosixPath
import shlex
import subprocess
import tempfile
import uuid

Q = shlex.quote


def manifest(connection, path):
    code = '''import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]); result={}
for f in sorted(p.glob('*.npz')):
 if f.is_symlink() or not f.stem.isascii() or not f.stem.isdecimal():
  raise ValueError('Scan names must be numeric timestamps and cannot be symlinks')
 h=hashlib.sha256()
 with f.open('rb') as s:
  for block in iter(lambda:s.read(1048576),b''): h.update(block)
 result[f.name]=[f.stat().st_size,h.hexdigest()]
print(json.dumps(result))'''
    return json.loads(subprocess.check_output(connection + ['python3 -c ' + Q(code) + ' ' + Q(path)], text=True))


def relay(source_connection, source_command, target_connection, target_command, source_input=subprocess.DEVNULL):
    with subprocess.Popen(source_connection + [source_command], stdin=source_input, stdout=subprocess.PIPE) as reader:
        writer = subprocess.run(target_connection + [target_command], stdin=reader.stdout)
        reader.stdout.close()
        code = reader.wait()
    if code or writer.returncode:
        raise RuntimeError('逐帧输入传输中断，请检查上方连接错误')


def transfer(args, pc2, server, data, name):
    def run(conn, command):
        return subprocess.check_output(conn + [command], text=True).strip()
    if not args.scans:
        return
    if not args.scans.startswith('/') or args.evidence and not args.evidence.startswith('/'):
        raise ValueError('--scans / --evidence 必须是 PC2 上的绝对路径')
    expected = manifest(pc2, args.scans)
    if len(expected) < 2:
        raise ValueError('PC2 扫描目录至少需要两个以时间戳命名的 NPZ 文件')
    destination = data.rstrip('/') + '/replay/' + name + '/scans'
    if run(server, 'if test -e ' + Q(destination) + '; then echo exists; fi'):
        if manifest(server, destination) != expected:
            raise ValueError('服务器同名扫描目录与 PC2 不同，未覆盖；请核对本轮输入')
    else:
        temporary = destination + '.partial-' + uuid.uuid4().hex
        run(server, 'mkdir -p ' + Q(temporary))
        print(f'[传输扫描] {len(expected)} 帧，PC2 → Mac 中转 → 服务器', flush=True)
        with tempfile.TemporaryFile() as listing:
            listing.write(b''.join(item.encode('ascii') + b'\0' for item in expected))
            listing.seek(0)
            relay(pc2, 'tar --null -C ' + Q(args.scans) + ' -cf - -T -',
                  server, 'tar -C ' + Q(temporary) + ' -xf -', source_input=listing)
        if manifest(server, temporary) != expected:
            raise RuntimeError('扫描文件大小或 SHA-256 不一致，未采用临时目录：' + temporary)
        run(server, 'mv -T -n ' + Q(temporary) + ' ' + Q(destination))
        if manifest(server, destination) != expected:
            raise RuntimeError('服务器最终扫描目录校验失败')
    print(f'[通过] {len(expected)} 帧扫描逐文件大小及 SHA-256 一致', flush=True)
    print('[服务器扫描] ' + destination, flush=True)
    if args.evidence:
        target = str(PurePosixPath(destination).parent / 'evidence.npz')
        source_info = run(pc2, 'stat -c %s ' + Q(args.evidence) + ' && sha256sum ' + Q(args.evidence)).splitlines()
        expected_file = (source_info[0], source_info[1].split()[0])
        if run(server, 'if test -e ' + Q(target) + '; then echo exists; fi'):
            result = run(server, 'stat -c %s ' + Q(target) + ' && sha256sum ' + Q(target)).splitlines()
            if (result[0], result[1].split()[0]) != expected_file:
                raise ValueError('服务器已有不同的同名证据文件，未覆盖')
        else:
            temporary = target + '.partial-' + uuid.uuid4().hex
            relay(pc2, 'cat ' + Q(args.evidence), server, 'cat > ' + Q(temporary))
            result = run(server, 'stat -c %s ' + Q(temporary) + ' && sha256sum ' + Q(temporary)).splitlines()
            if (result[0], result[1].split()[0]) != expected_file:
                raise RuntimeError('证据文件传输校验失败：' + temporary)
            run(server, 'mv -T -n ' + Q(temporary) + ' ' + Q(target))
            result = run(server, 'stat -c %s ' + Q(target) + ' && sha256sum ' + Q(target)).splitlines()
            if (result[0], result[1].split()[0]) != expected_file:
                raise RuntimeError('服务器最终证据文件校验失败')
        print('[通过] 证据文件大小及 SHA-256 一致', flush=True)
        print('[服务器证据] ' + target, flush=True)
