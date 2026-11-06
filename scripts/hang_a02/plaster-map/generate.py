"""Build a candidate navigation map and source cloud from hang_A_02."""
from pathlib import Path
import json
import sys

import cv2
import numpy as np

root = Path.home() / 'ldy/dufomap-mask-review'
sys.path.insert(0, str(root))
from dufomap_clean import clean, read_scans, scan_paths, write_pcd

paths = scan_paths(root / 'runs/none/scans')
prior = np.load('../grids.npz')
stamps = np.array([int(p.stem) for p in paths])
np.testing.assert_allclose((stamps - stamps[0])*1e-9, prior['t'], rtol=0, atol=1e-9)
floor = json.loads((root / 'metadata.json').read_text())['floor_fit']
normal = np.array(floor['normal'])
offset = floor['plane_offset_m']
resolution = .10
ox, oy = -10.6, -11.8
width, height = 231, 235
ground_hits = np.zeros((height, width), dtype=np.int32)
obstacle_hits = np.zeros_like(ground_hits)
chunks = []
print('Running DUFOMap on', len(paths), 'paired scans', flush=True)
for i, (points, static) in enumerate(clean(read_scans(paths), d_s=.2, d_p=1)):
    if prior['scores'][i] > .6:
        continue
    points = points[static]
    h = np.einsum('ij,j->i', points, normal) + offset
    ground = np.abs(h) < .15
    far = np.linalg.norm(points[:, :2] - prior['body'][i, :2], axis=1) > 1.5
    keep = ((h >= -.15) & (h <= 4.5) & (far | ground) &
            (np.hypot(points[:,0], points[:,1]) <= 12))
    points, h = points[keep], h[keep]
    chunks.append(points)
    xy = np.floor((points[:, :2] - [ox, oy])/resolution).astype(int)
    inside = ((xy[:,0] >= 0) & (xy[:,0] < width) & (xy[:,1] >= 0) & (xy[:,1] < height))
    cell = xy[:,1] * width + xy[:,0]
    ground_hits.flat[np.unique(cell[inside & (np.abs(h) < .15)])] += 1
    obstacle_hits.flat[np.unique(cell[inside & (h >= .20) & (h <= 1.8)])] += 1

points = np.concatenate(chunks)
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
                    body=prior['body'], floor_normal=normal, floor_offset=offset)
metadata = dict(frames_total=len(paths), frames_kept=len(chunks),
                points_before_voxel=len(points), points=len(unique),
                d_s=.2, d_p=1, floor_fit=floor, origin=[ox,oy,0.], frame='camera_init',
                resolution=resolution, width=width, height=height,
                origin_pixel=int(image[int(-oy/resolution),int(-ox/resolution)]),
                free_cells=int((image==254).sum()), occupied_cells=int((image==0).sum()),
                unknown_cells=int((image==205).sum()))
Path('metadata.json').write_text(json.dumps(metadata, indent=2))
print(metadata, flush=True)
