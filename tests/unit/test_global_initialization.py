#!/usr/bin/env python3
"""Exercise global search on hidden transformed real geometry and rejection cases."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'shared/map'))
from pcd import read_pcd


def write(path, xyz):
    with path.open('wb') as stream:
        stream.write(('ply\nformat binary_little_endian 1.0\nelement vertex '
                      f'{len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode())
        stream.write(xyz.astype('<f4').tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--pcd', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    xyz = read_pcd(args.pcd)
    xyz = xyz[np.linalg.norm(xyz[:, :2], axis=1) < 25][::2]
    patch = xyz[np.linalg.norm(xyz[:, :2] - [2., -3.], axis=1) < 9]
    assert len(patch) > 1000
    rng = np.random.RandomState(103)
    yaw = np.deg2rad(117)
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    translation = np.array([11., -8., .35])
    source = (patch - translation) @ rotation
    expected = np.eye(4)
    expected[:3, :3] = rotation
    expected[:3, 3] = translation
    write(args.out / 'map.ply', xyz)
    for i in range(1, 4):
        write(args.out / f'scan{i}.ply', source + rng.normal(0, .008, source.shape))

    def run(name, map_name='map.ply', scans=None, success=False):
        inputs = scans or [f'scan{i}.ply' for i in range(1, 4)]
        command = [str(args.binary), str(args.out / map_name)]
        command += [str(args.out / item) for item in inputs]
        command += [str(args.out / f'{name}.json')]
        with (args.out / f'{name}.log').open('w') as log:
            result = subprocess.run(command, stdout=log, stderr=log, timeout=180)
        if success:
            assert result.returncode == 0, (name, (args.out / f'{name}.log').read_text())
            matrix = np.array(json.loads((args.out / f'{name}.json').read_text())['matrix'])
            assert np.linalg.norm(matrix[:3, 3] - expected[:3, 3]) < .15, matrix
            assert np.linalg.norm(matrix[:3, :3] - expected[:3, :3]) < .08, matrix
        else:
            assert result.returncode != 0, f'{name} unexpectedly accepted'
            assert not (args.out / f'{name}.json').exists(), 'failed search wrote accepted pose'
        print(name, 'PASS', flush=True)

    run('unknown_transform', success=True)
    # Two exact copies in distinct places: a global location cannot be unique.
    write(args.out / 'repeated-map.ply', np.vstack([patch, patch + [22, 0, 0]]))
    run('ambiguous', 'repeated-map.ply')
    write(args.out / 'unrelated.ply', rng.uniform([-9, -9, -4], [9, 9, 8], (3000, 3)))
    run('unrelated', scans=['unrelated.ply']*3)
    write(args.out / 'shifted.ply', source + [4, 3, 0])
    run('inconsistent_frames', scans=['scan1.ply', 'shifted.ply', 'scan3.ply'])


if __name__ == '__main__':
    main()
