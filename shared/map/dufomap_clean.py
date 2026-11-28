#!/usr/bin/env python3
"""Clean stamped, world-frame LiDAR scans with DUFOMap 1.1.1."""
import argparse
import math
from pathlib import Path

import numpy as np


def clean(scans, *, d_s=0.2, d_p=2):
    from dufomap import dufomap

    if not math.isfinite(d_s) or d_s < 0 or d_p < 0:
        raise ValueError("DUFOMap requires nonnegative d_s and d_p")
    mapper = dufomap(0.1, d_s, d_p, num_threads=4)
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


def scan_paths(directory):
    return sorted(directory.glob("*.npz"), key=lambda path: int(path.stem))


def read_scans(paths):
    for path in paths:
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
    parser.add_argument("--revisit-after", type=int, metavar="STAMP_NS",
                        help="Require early points to be corroborated by a separate clear revisit")
    parser.add_argument("--d-s", type=float, default=0.2, help="DUFOMap sensor noise margin in metres")
    parser.add_argument("--d-p", type=int, default=2, help="DUFOMap pose uncertainty margin in voxels")
    args = parser.parse_args()
    if not math.isfinite(args.d_s) or args.d_s < 0 or args.d_p < 0:
        parser.error("--d-s and --d-p must be nonnegative and finite")
    paths = scan_paths(args.scans)
    corroborate = None
    if args.revisit_after is not None:
        from scipy.spatial import cKDTree

        revisit = [p for p in paths if int(p.stem) >= args.revisit_after]
        if len(revisit) < 2 or len(paths) - len(revisit) < 2:
            parser.error("At least two scans are required in each pass")
        corroborate = cKDTree(np.concatenate([p for p, _ in read_scans(revisit)]))

    kept = []
    total = 0
    for path, (points, mask) in zip(paths, clean(read_scans(paths), d_s=args.d_s, d_p=args.d_p)):
        total += len(points)
        if corroborate is not None and int(path.stem) < args.revisit_after:
            # Do not use the surveyed obstacle locations. Require independent
            # returns from the revisit within 1.5 map voxels of an earlier hit.
            distance, _ = corroborate.query(points[mask], distance_upper_bound=.15)
            mask[mask] = np.isfinite(distance)
        kept.append(points[mask])
    points = np.concatenate(kept)
    write_pcd(args.output, points)
    print(f"frames={len(kept)} input={total} kept={len(points)} removed={total-len(points)}")


if __name__ == "__main__":
    main()
