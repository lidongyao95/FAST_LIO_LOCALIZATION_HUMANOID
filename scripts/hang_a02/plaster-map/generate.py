"""Build a candidate navigation map and source cloud from hang_A_02."""
from pathlib import Path
import argparse
import json
import math
import sys

import cv2
import numpy as np

root = Path.home() / 'ldy/dufomap-mask-review'
sys.path.insert(0, str(root))
from dufomap_clean import clean, read_scans, scan_paths, write_pcd

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scans', type=Path, default=root / 'runs/none/scans')
parser.add_argument('--evidence', type=Path, default=Path('../grids.npz'))
parser.add_argument('--floor-metadata', type=Path, default=root / 'metadata.json')
parser.add_argument('--height-min', type=float, default=-.15, help='Minimum signed height above fitted floor (m)')
parser.add_argument('--height-max', type=float, default=4.5, help='Maximum signed height above fitted floor (m)')
parser.add_argument('--near-radius', type=float, default=1.5, help='Drop returns within this robot-relative XY radius (m); 0 disables')
parser.add_argument('--keep-near-ground', action=argparse.BooleanOptionalAction, default=True,
                    help='Exempt fitted ground returns from the near-radius filter')
parser.add_argument('--sample-period', type=float, default=0.,
                    help='Keep first scan per period in seconds; 0 keeps all scans')
parser.add_argument('--imu-max-accel', type=float, default=.6,
                    help='Maximum gravity-compensated frame P95 (m/s²); omit the gate with --no-imu-filter')
parser.add_argument('--no-imu-filter', action='store_true')
parser.add_argument('--max-map-radius', type=float, default=12., help='Crop about map origin (m); 0 disables')
parser.add_argument('--dufo-d-s', type=float, default=.2, help='DUFOMap sensor noise margin (m)')
parser.add_argument('--dufo-d-p', type=int, default=1, help='DUFOMap pose uncertainty margin (voxels)')
args = parser.parse_args()
filter_params = {name: value for name, value in vars(args).items() if not isinstance(value, Path)}
if not all(math.isfinite(value) for value in filter_params.values()):
    parser.error('Filter thresholds must be finite')
if args.height_min >= args.height_max:
    parser.error('--height-min must be below --height-max')
if min(args.near_radius, args.sample_period, args.imu_max_accel,
       args.max_map_radius, args.dufo_d_s, args.dufo_d_p) < 0:
    parser.error('Radius, sampling period, acceleration and DUFOMap margins must be nonnegative')

paths = scan_paths(args.scans)
if len(paths) < 2:
    parser.error('At least two scan files are required')
with np.load(args.evidence) as evidence:
    times = evidence['t']
    body = evidence['body']
    scores = evidence['scores']
stamps = np.array([int(p.stem) for p in paths])
np.testing.assert_allclose((stamps - stamps[0])*1e-9, times, rtol=0, atol=1e-9)
if body.shape != (len(paths), 3) or scores.shape != times.shape:
    parser.error('Evidence must contain one body position and IMU score per scan')
keep_frames = np.ones(len(paths), dtype=bool)
if args.sample_period > 0:
    _, first = np.unique(np.floor(times / args.sample_period), return_index=True)
    keep_frames[:] = False
    keep_frames[first] = True
if not args.no_imu_filter:
    keep_frames &= np.isfinite(scores) & (scores <= args.imu_max_accel)
if not keep_frames.any():
    parser.error('No frames pass the time and IMU filters')
floor = json.loads(args.floor_metadata.read_text())['floor_fit']
normal = np.array(floor['normal'])
offset = floor['plane_offset_m']
resolution = .10
ox, oy = -10.6, -11.8
width, height = 231, 235
ground_hits = np.zeros((height, width), dtype=np.int32)
obstacle_hits = np.zeros_like(ground_hits)
chunks = []
print('Running DUFOMap on', len(paths), 'paired scans', flush=True)
for i, (points, static) in enumerate(clean(read_scans(paths), d_s=args.dufo_d_s, d_p=args.dufo_d_p)):
    if not keep_frames[i]:
        continue
    points = points[static]
    h = np.einsum('ij,j->i', points, normal) + offset
    ground = np.abs(h) < .15
    keep = (h >= args.height_min) & (h <= args.height_max)
    if args.near_radius > 0:
        far = np.linalg.norm(points[:, :2] - body[i, :2], axis=1) > args.near_radius
        keep &= far | (ground & args.keep_near_ground)
    if args.max_map_radius > 0:
        keep &= np.hypot(points[:,0], points[:,1]) <= args.max_map_radius
    points, h = points[keep], h[keep]
    chunks.append(points)
    xy = np.floor((points[:, :2] - [ox, oy])/resolution).astype(int)
    inside = ((xy[:,0] >= 0) & (xy[:,0] < width) & (xy[:,1] >= 0) & (xy[:,1] < height))
    cell = xy[:,1] * width + xy[:,0]
    ground_hits.flat[np.unique(cell[inside & (np.abs(h) < .15)])] += 1
    obstacle_hits.flat[np.unique(cell[inside & (h >= .20) & (h <= 1.8)])] += 1

points = np.concatenate(chunks)
if not len(points):
    parser.error('No static points pass the point filters')
voxel = np.floor(points / .035).astype(np.int32)
_, unique = np.unique(voxel, axis=0, return_index=True)
write_pcd(Path('clean.pcd'), points[unique])
# Require independent scan support. Close small holes in the observed floor,
# then expand that evidence by one cell. Sparse obstacle returns stay unknown.
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
free = cv2.morphologyEx((ground_hits >= 2).astype(np.uint8), cv2.MORPH_CLOSE, kernel)
free = cv2.dilate(free, np.ones((3,3), np.uint8)).astype(bool)
occupied = obstacle_hits >= 3
image = np.full_like(ground_hits, 205, dtype=np.uint8)
image[free & (obstacle_hits == 0)] = 254
image[occupied] = 0
assert cv2.imwrite('map.pgm', image[::-1])
Path('map.yaml').write_text(
    f'image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{ox}, {oy}, 0.0]\n'
    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
assert np.array_equal(cv2.imread('map.pgm', cv2.IMREAD_UNCHANGED), image[::-1])
np.savez_compressed('evidence.npz', ground=ground_hits, obstacle=obstacle_hits,
                    body=body, floor_normal=normal, floor_offset=offset)
metadata = dict(frames_total=len(paths), frames_kept=len(chunks),
                points_before_voxel=len(points), points=len(unique),
                d_s=args.dufo_d_s, d_p=args.dufo_d_p, filters=filter_params,
                floor_fit=floor, origin=[ox,oy,0.], frame='camera_init',
                resolution=resolution, width=width, height=height,
                origin_pixel=int(image[int(-oy/resolution),int(-ox/resolution)]),
                free_cells=int((image==254).sum()), occupied_cells=int((image==0).sum()),
                unknown_cells=int((image==205).sum()))
Path('metadata.json').write_text(json.dumps(metadata, indent=2))
print(metadata, flush=True)
