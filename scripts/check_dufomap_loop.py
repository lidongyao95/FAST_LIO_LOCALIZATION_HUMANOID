#!/usr/bin/env python3
"""Check seeded MuJoCo obstacles against raw and clean maps; truth is evaluation only."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from dufomap_clean import read_scans, scan_paths, write_pcd
from export_g1_map import export, read_pcd


def grid(directory):
    info = json.loads((directory / "metadata.json").read_text())
    image = cv2.imread(str(directory / "map.pgm"), cv2.IMREAD_GRAYSCALE)[::-1]
    y, x = np.indices(image.shape)
    origin_x, origin_y, _ = info["origin"]
    step = info["resolution"]
    xy = np.stack((origin_x + step * (x + .5), origin_y + step * (y + .5)), axis=-1)
    return info, image, xy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--revisit-after", type=int, required=True)
    args = parser.parse_args()
    run = args.run
    truth = json.loads((run / "truth.json").read_text())
    paths = scan_paths(run / "scans")
    frames = list(read_scans(paths))
    raw = np.concatenate([points for points, _ in frames])
    raw_pcd = run / "raw.pcd"
    if not raw_pcd.exists():
        write_pcd(raw_pcd, raw)
    export(raw_pcd, run / "raw-map", .1, 0.)
    clean_pcd = run / "clean.pcd"
    export(clean_pcd, run / "clean-map", .1, 0.)
    clear_pcd = run / "reference.pcd"
    if not clear_pcd.exists():
        reference = np.concatenate([points for path, (points, _) in zip(paths, frames)
                                    if int(path.stem) >= args.revisit_after])
        write_pcd(clear_pcd, reference)
    export(clear_pcd, run / "reference-map", .1, 0.)
    clean = read_pcd(clean_pcd)
    _, raw_map, raw_xy = grid(run / "raw-map")
    clean_info, clean_map, clean_xy = grid(run / "clean-map")
    _, ref_map, ref_xy = grid(run / "reference-map")
    # Compare occupied cells on the common 10 cm lattice. Near a permanent
    # fixture, the 3x3 dilation legitimately extends outside its physical box.
    reference_cells = set(map(tuple, np.rint(ref_xy[ref_map == 0] / .1 - .5).astype(int)))
    clean_cells = np.rint(clean_xy / .1 - .5).astype(int)
    ref_occupied = np.zeros_like(clean_map, dtype=bool)
    for y, x in zip(*np.nonzero(clean_map == 0)):
        ref_occupied[y, x] = tuple(clean_cells[y, x]) in reference_cells
    extra_clean_map = clean_map.copy()
    extra_clean_map[ref_occupied] = 205
    center_points = np.array([pose[:3] for _, pose in frames])
    path_length = np.linalg.norm(np.diff(center_points, axis=0), axis=1).sum()

    boxes = []
    for center in truth["centers"]:
        xy_half = np.asarray(truth["half_size"][:2])

        def point_count(points, low):
            xy_mask = (np.abs(points[:, :2] - center) <= xy_half + .11).all(axis=1)
            z = points[:, 2] - clean_info["floor_z"]
            return int((xy_mask & (z > low) & (z < 1.5)).sum())

        def cell_counts(image, xy):
            roi = (np.abs(xy - center) <= xy_half + .13).all(axis=2)
            return int((image[roi] == 0).sum()), int((image[roi] == 254).sum())

        raw_occupied, _ = cell_counts(raw_map, raw_xy)
        clean_occupied, clean_free = cell_counts(clean_map, clean_xy)
        extra_occupied, _ = cell_counts(extra_clean_map, clean_xy)
        boxes.append(dict(center=center, raw_points=point_count(raw, .2),
                          clean_points=point_count(clean, .2),
                          clean_low_returns=point_count(clean, .16) - point_count(clean, .2),
                          raw_occupied=raw_occupied, clean_occupied=clean_occupied,
                          clean_extra_occupied=extra_occupied, clean_free=clean_free))
    result = dict(seed=truth["seed"], scans=len(frames), path_length_m=round(path_length, 2),
                  raw_points=len(raw), clean_points=len(clean), boxes=boxes)
    print(json.dumps(result, indent=2))
    (run / "metrics.json").write_text(json.dumps(result, indent=2))
    if any(b["raw_points"] == 0 or b["raw_occupied"] == 0 or b["clean_extra_occupied"]
           or b["clean_points"] or b["clean_free"] == 0 for b in boxes):
        raise SystemExit("Failed to clear one or more seeded obstacle regions")


if __name__ == "__main__":
    main()
