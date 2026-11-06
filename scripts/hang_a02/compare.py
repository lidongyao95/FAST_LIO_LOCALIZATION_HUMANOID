"""Compare independent filters on exact-stamp FAST-LIO pairs from hang_A_02."""
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

home = Path.home()
root = home / 'ldy/dufomap-mask-review'
paths = sorted((root / 'runs/none/scans').glob('*.npz'))
bag = home / 'ldy/g1_a_replay/hang_A_02/bag/hang_A_02_0.db3'
floor = json.loads((root / 'metadata.json').read_text())['floor_fit']
normal = np.array(floor['normal'])
offset = floor['plane_offset_m']
lidar_in_body = np.array([-.011, .02329, -.04412])
stamps = np.array([int(path.stem) for path in paths])
t = (stamps - stamps[0]) * 1e-9
origin, quats = [], []
for path in paths:
    with np.load(path) as scan:
        origin.append(scan['pose'][:3])
        quats.append(scan['body_quaternion'][[1, 2, 3, 0]])
rot = Rotation.from_quat(quats)
body = np.array(origin) - rot.apply(lidar_in_body)
print('Paired frames:', len(t), 'duration:', t[-1], flush=True)
assert len(t) == 3357

imu_t, imu_a = [], []
with sqlite3.connect(f'file:{bag}?mode=ro', uri=True) as db:
    for (data,) in db.execute("select data from messages join topics on topics.id=messages.topic_id where topics.name='/livox/imu'"):
        sec, ns = struct.unpack_from('<iI', data, 4)
        imu_t.append((sec * 10**9 + ns - stamps[0]) * 1e-9)
        imu_a.append(struct.unpack_from('<3d', data, 228))
order = np.argsort(imu_t)
imu_t = np.array(imu_t)[order]
imu_a = np.array(imu_a)[order] * [9.80665, -9.80665, -9.80665]
# Interpolate orientation only. At the edges use the nearest available pose.
world_a = Slerp(t, rot)(np.clip(imu_t, t[0], t[-1])).apply(imu_a)
dynamic_a = np.linalg.norm(world_a - 9.80665 * normal, axis=1)
scores = np.full(len(t), np.inf)
imu_counts = np.zeros(len(t), dtype=int)
for i, end in enumerate(t):
    left, right = imu_t.searchsorted([end - .1, end], side='right')
    imu_counts[i] = right - left
    if right - left >= 15:
        scores[i] = np.quantile(dynamic_a[left:right], .95)
print('IMU windows:', np.quantile(imu_counts, [0, .5, 1]),
      'P95 acceleration:', np.quantile(scores[np.isfinite(scores)], [0, .1, .25, .5, .75, .9, 1]), flush=True)

periods = [.5, 1., 2.]
sampled = []
for period in periods:
    indices = np.unique(np.floor(t / period).astype(int), return_index=True)[1]
    sampled.append(set(indices))
heights = [(.2, 1.8), (.3, 1.5), (.45, 1.2)]
radii = [.75, 1., 1.5]
accels = [1.5, 1., .6]
names = (
    ['Baseline']
    + [f'{low:g}-{high:g} m above floor' for low, high in heights]
    + [f'Drop within {radius:g} m' for radius in radii]
    + [f'One scan per {period:g} s' for period in periods]
    + [f'IMU P95 <= {accel:g} m/s²' for accel in accels]
)
# Shared obstacle band and map window. Plot raw return counts at 10 cm.
xmin, xmax, ymin, ymax = -10, 13, -12, 12
resolution = .1
nx, ny = round((xmax-xmin)/resolution), round((ymax-ymin)/resolution)
shape = (len(names), ny, nx)
hits = np.zeros(shape, dtype=np.int64)
visits = np.zeros(shape, dtype=np.int32)
frame_counts = np.zeros(len(names), dtype=int)
selected = [0, 3, 6, 8, 12]
cloud_chunks = {i: [] for i in selected}
for i, path in enumerate(paths):
    with np.load(path) as scan:
        points = scan['points']
    height = np.einsum('ij,j->i', points, normal) + offset
    xy = np.floor((points[:, :2] - [xmin, ymin])/resolution).astype(int)
    valid = ((height >= .12) & (height <= 2.5) &
             (xy[:, 0] >= 0) & (xy[:, 0] < nx) &
             (xy[:, 1] >= 0) & (xy[:, 1] < ny))
    points, height, xy = points[valid], height[valid], xy[valid]
    ranges = np.linalg.norm(points[:, :2] - body[i, :2], axis=1)
    cells = xy[:, 1]*nx + xy[:, 0]
    all_points = np.ones(len(points), dtype=bool)
    masks = ([all_points]
             + [(height >= low) & (height <= high) for low, high in heights]
             + [ranges > radius for radius in radii]
             + [all_points if i in indices else None for indices in sampled]
             + [all_points if scores[i] <= limit else None for limit in accels])
    for j, mask in enumerate(masks):
        if mask is None:
            continue
        frame_counts[j] += 1
        ix, counts = np.unique(cells[mask], return_counts=True)
        hits[j].flat[ix] += counts
        visits[j].flat[ix] += 1
        if j in cloud_chunks:
            cloud = points[mask].copy()
            cloud[:, 2] = height[mask]
            cloud_chunks[j].append(np.floor(cloud / .08).astype(np.int16))
    if (i + 1) % 800 == 0:
        print('Processed', i + 1, flush=True)

np.savez_compressed('grids.npz', hits=hits, visits=visits, body=body, t=t, scores=scores,
                    names=names, frame_counts=frame_counts, bounds=[xmin,xmax,ymin,ymax])
for j, chunks in cloud_chunks.items():
    indices = np.concatenate(chunks)
    # Pack non-overlapping signed coordinate bins for a fast 1D unique.
    keys = (indices[:,0].astype(np.int64)+1024)*2048**2 + (indices[:,1].astype(np.int64)+1024)*2048 + indices[:,2]+1024
    assert np.abs(indices).max() < 1024
    keys = np.unique(keys)
    xyz = np.column_stack([keys//2048**2, keys//2048%2048, keys%2048])
    np.save(f'cloud-{j}.npy', ((xyz-1024+.5)*.08).astype(np.float32))
print('Done', dict(zip(names, frame_counts.tolist())), flush=True)
