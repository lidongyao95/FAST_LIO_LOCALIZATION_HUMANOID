"""Check new scans already expressed in the map frame."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

raw = open('clean.pcd', 'rb').read().split(b'DATA binary\n', 1)[1]
target = np.frombuffer(raw, '<f4').reshape(-1, 3).astype(float)
target[:, 2] = target @ np.array([.078349821, .006797867, .996902764]) + 1.347228408
target = target[(target[:, 2] > .27) & (target[:, 2] < 2.35)]
alignment = json.loads(Path('alignment.json').read_text())
origin = [alignment['x'], alignment['y']]
scan = np.load('verified.npy')[:, 1:]
scan = scan[(scan[:, 2] > .32) & (scan[:, 2] < 2.2)
            & (np.linalg.norm(scan[:, :2] - origin, axis=1) < 8)]
scan = scan[np.unique(np.floor(scan / .12).astype(int), axis=0, return_index=True)[1]]
distances, _ = cKDTree(target).query(scan)
inliers = distances < .20
overlap = np.mean(inliers)
rms = np.sqrt(np.mean(distances[inliers] ** 2)) if inliers.any() else float('inf')
print(f'samples={len(scan)} overlap_20cm={overlap:.3f} inlier_rms_m={rms:.3f}')
if not (len(scan) >= 100 and overlap >= .85 and rms <= .10):
    raise SystemExit('Live alignment verification failed')
