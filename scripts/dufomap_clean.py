#!/usr/bin/env python3
"""Clean stamped, world-frame LiDAR scans with DUFOMap 1.1.1."""
import argparse
from pathlib import Path

import numpy as np


def clean(scans):
    from dufomap import dufomap

    mapper = dufomap(0.1, 0.2, 2, num_threads=4)
    frames = []
    for points, pose in scans:
        points = np.ascontiguousarray(points, dtype=np.float32)
        pose = np.asarray(pose, dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
                or pose.shape != (7,) or not np.isfinite(points).all()
                or not np.isfinite(pose).all()
                or not np.isclose(np.linalg.norm(pose[3:]), 1.0)):
            raise ValueError("Expected finite Nx3 points and a normalized xyz/qwxyz pose")
        pose = pose.tolist()
        mapper.run(points, pose, cloud_transform=False)
        frames.append((points, pose))
    if len(frames) < 2:
        raise ValueError("At least two scans are required")
    mapper.oncePropagateCluster(if_propagate=True, if_cluster=False)
    for points, pose in frames:
        labels = np.asarray(mapper.segment(points, pose, cloud_transform=False))
        if labels.shape != (len(points),):
            raise RuntimeError(f"Unexpected DUFOMap label shape {labels.shape}")
        yield points, labels == 0


def read_scans(directory):
    for path in sorted(directory.glob("*.npz"), key=lambda path: int(path.stem)):
        with np.load(path) as frame:
            yield frame["points"], frame["pose"]


def write_pcd(path, points):
    if not len(points):
        raise ValueError("Clean map is empty")
    header = (
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    with path.open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.asarray(points, dtype="<f4").tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scans", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    kept = []
    total = 0
    for points, mask in clean(read_scans(args.scans)):
        total += len(points)
        kept.append(points[mask])
    points = np.concatenate(kept)
    write_pcd(args.output, points)
    print(f"frames={len(kept)} input={total} kept={len(points)} removed={total-len(points)}")


if __name__ == "__main__":
    main()
