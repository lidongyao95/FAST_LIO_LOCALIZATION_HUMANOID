#!/usr/bin/env python3
"""在 G1 二维地图副本上清掉机器人当前站位，绕开"建图时把机身扫进地图"。

建图当天如果机器人本体或现场人员停在激光高度带内，导出的 2D 投影会在机器人
站位处形成一圈占据格。Nav2 把机器人位姿栅格判定为 occupied，规划器直接拒绝
出自当前的任何路径（NavFn 起点不可通行），表现为网页点选目标返回 422
"未返回可行路径"。这个脚本不改原始地图，只把站位半径内的占据格改成自由，
产出一份可复现的副本。

用法：
  patch_g1_map.py MAP_DIR|MAP.yaml --pose X Y [--radius 0.8] --out OUT_DIR

判据：脚本结束打印清掉的格数、位姿格改前改后的值，以及半径内剩余的占据格数。
重复执行是幂等的（第二次清 0 格）。
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

FREE = 254


def load_pgm(path):
    data = path.read_bytes()
    fields = []
    index = 0
    while len(fields) < 4:
        start = index
        while data[index:index + 1].isspace():
            index += 1
        end = index
        while not data[end:end + 1].isspace():
            end += 1
        if data[start:index].startswith(b'#'):
            while data[index:index + 1] != b'\n':
                index += 1
            continue
        fields.append(data[start:end].decode('ascii'))
        index = end
    if fields[0] != 'P5':
        raise ValueError(f'只支持二进制 PGM（P5），当前是 {fields[0]}')
    width, height, maximum = int(fields[1]), int(fields[2]), int(fields[3])
    if maximum != 255:
        raise ValueError(f'只支持 8 位 PGM，当前最大值 {maximum}')
    pixels = bytearray(data[index + 1:])
    if len(pixels) != width * height:
        raise ValueError(f'像素数 {len(pixels)} 与 {width}x{height} 不匹配')
    return width, height, pixels


def write_pgm(path, width, height, pixels):
    path.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels))


def read_yaml(path):
    values = {}
    for line in path.read_text().splitlines():
        if ':' in line and not line.lstrip().startswith('#'):
            key, value = line.split(':', 1)
            values[key.strip()] = value.strip()
    origin = values['origin'].strip('[]').split(',')
    return {
        'image': values['image'],
        'resolution': float(values['resolution']),
        'origin': (float(origin[0]), float(origin[1])),
        'yaml': values,
    }


def patch(map_dir, pose, radius):
    meta_path = map_dir / 'metadata.json'
    meta = json.loads(meta_path.read_text()) if meta_path.is_file() else {}
    info = read_yaml(map_dir / 'map.yaml')
    width, height, pixels = load_pgm(map_dir / info['image'])
    resolution = info['resolution']
    ox, oy = info['origin']
    # PGM 首行是地图顶部，y 必须按 oy + (height - row - 0.5) * res 换算。
    start_col = int((pose[0] - radius - ox) / resolution)
    end_col = int((pose[0] + radius - ox) / resolution) + 1
    start_row = int(height - 0.5 - (pose[1] + radius - oy) / resolution)
    end_row = int(height - 0.5 - (pose[1] - radius - oy) / resolution) + 1
    pose_col = int((pose[0] - ox) / resolution)
    pose_row = int(height - 0.5 - (pose[1] - oy) / resolution)
    before = pixels[pose_row * width + pose_col]
    cleared = 0
    for row in range(max(0, start_row), min(height, end_row)):
        y = oy + (height - row - 0.5) * resolution
        for col in range(max(0, start_col), min(width, end_col)):
            x = ox + (col + 0.5) * resolution
            if (x - pose[0]) ** 2 + (y - pose[1]) ** 2 > radius ** 2:
                continue
            index = row * width + col
            if pixels[index] == 0:
                pixels[index] = FREE
                cleared += 1
    after = pixels[pose_row * width + pose_col]
    remaining = sum(
        1 for row in range(max(0, start_row), min(height, end_row))
        for col in range(max(0, start_col), min(width, end_col))
        if pixels[row * width + col] == 0
        and (ox + (col + 0.5) * resolution - pose[0]) ** 2
        + (oy + (height - row - 0.5) * resolution - pose[1]) ** 2 <= radius ** 2)
    return {
        'width': width, 'height': height, 'pixels': pixels,
        'info': info, 'meta': meta, 'cleared': cleared,
        'pose_cell': (pose_col, pose_row), 'before': before, 'after': after,
        'remaining': remaining, 'radius': radius, 'pose': pose,
    }


def main():
    parser = argparse.ArgumentParser(description='清掉 G1 地图上机器人当前站位的占据格')
    parser.add_argument('map', help='地图目录或 map.yaml')
    parser.add_argument('--pose', nargs=2, type=float, required=True, metavar=('X', 'Y'),
                        help='机器人当前位姿（地图坐标系，米）')
    parser.add_argument('--radius', type=float, default=0.8, help='清除半径，米（默认 0.8）')
    parser.add_argument('--out', required=True, help='输出目录（不能是原始地图目录）')
    args = parser.parse_args()
    source = Path(args.map).expanduser().resolve()
    if source.is_file():
        source = source.parent
    out = Path(args.out).expanduser().resolve()
    if out == source:
        sys.exit('输出目录不能与原始地图目录相同；请保留原始地图')
    if args.radius <= 0:
        sys.exit('--radius 需要正数')
    result = patch(source, args.pose, args.radius)
    out.mkdir(parents=True, exist_ok=True)
    write_pgm(out / result['info']['image'], result['width'], result['height'], result['pixels'])
    shutil.copy(source / 'map.yaml', out / 'map.yaml')
    meta = dict(result['meta'])
    meta['robot_clear'] = {
        'pose': list(args.pose), 'radius': args.radius,
        'cleared_cells': result['cleared'],
        'source_map': str(source),
    }
    (out / 'metadata.json').write_text(json.dumps(meta, indent=2))
    print(f'源地图 {source}')
    print(f'副本 {out}')
    print(f"位姿格 {result['pose_cell']} 值 {result['before']} -> {result['after']}")
    print(f"半径 {args.radius} m 内清掉占据格 {result['cleared']} 个，剩余 {result['remaining']} 个")
    if result['after'] == 0:
        sys.exit('位姿格仍是占据；检查 --pose 是否与地图坐标系一致')
    if result['remaining']:
        print('提示：半径内仍有占据格，多为真实墙面或家具，规划器会自行绕过', file=sys.stderr)


if __name__ == '__main__':
    main()
