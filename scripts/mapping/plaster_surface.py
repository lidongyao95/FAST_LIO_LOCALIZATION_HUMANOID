#!/usr/bin/env python3
"""Reconstruct a smoothed preview surface from returns above the fitted floor."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from scipy.ndimage import gaussian_filter
from skimage.measure import marching_cubes

from g1_scan_scene import fit_map_floor
from pcd import read_pcd


def surface(xyz, pitch, ground_cut, radius, height):
    _, world_from_map, floor = fit_map_floor(xyz)
    rotation = world_from_map[:3, :3]
    translation = world_from_map[:3, 3]
    level = xyz @ rotation.T + translation
    mask = ((level[:, 2] >= ground_cut) & (level[:, 2] <= height)
            & (np.hypot(level[:, 0], level[:, 1]) <= radius))
    above_floor = level[mask]
    if len(above_floor) < 100:
        raise ValueError("剔除地面后点数不足；检查地面拟合、点云或 ground-cut")
    cells = np.floor(above_floor / pitch).astype(np.int64)
    lower = cells.min(axis=0) - 4
    cells -= lower
    shape = cells.max(axis=0) + 5
    if np.prod(shape, dtype=np.float64) > 24_000_000:
        raise ValueError("石膏网格过大；增大 --plaster-grid 或裁剪点云")
    field = np.zeros(shape, dtype=np.float32)
    field[tuple(cells.T)] = 1
    field = gaussian_filter(field, .85)
    if field.max() <= .14:
        raise ValueError("点云过稀，无法生成连续表面；增大 --plaster-grid")
    vertices, faces, _, _ = marching_cubes(field, level=.14, spacing=(pitch,) * 3,
                                           allow_degenerate=False)
    vertices += (lower + .5) * pitch
    return vertices, faces[:, ::-1], (vertices - translation) @ rotation, floor, len(above_floor)


def write_mesh(path, vertices, faces):
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\n"
              f"element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
    data = np.empty(len(faces), dtype=[("count", "u1"), ("face", "<i4", (3,))])
    data["count"] = 3
    data["face"] = faces
    with path.open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(vertices.astype("<f4").tobytes())
        stream.write(data.tobytes())


def render(path, vertices, faces):
    azimuth, elevation = np.deg2rad([-35, 50])
    view = np.array([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
    right = np.cross([0, 0, 1], view)
    right /= np.linalg.norm(right)
    up = np.cross(view, right)
    screen = vertices @ np.column_stack([right, up])
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-10)
    light = .72 * view - .65 * right + .65 * up
    light /= np.linalg.norm(light)
    shades = .46 + .52 * np.maximum(normals @ light, 0)
    color = shades[:, None] * [1.0, .982, .949]
    order = np.argsort(triangles.mean(axis=1) @ view)
    background = "#26292c"
    fig, ax = plt.subplots(figsize=(12, 9), facecolor=background)
    ax.set_facecolor(background)
    ax.add_collection(PolyCollection(screen[faces[order]], facecolors=color[order],
                                     edgecolors="face", linewidths=.15))
    ax.autoscale()
    ax.set_aspect("equal")
    ax.axis("off")
    fig.savefig(path, dpi=140, bbox_inches="tight", facecolor=background)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--grid", type=float, default=.08)
    parser.add_argument("--ground-cut", type=float, default=.16)
    parser.add_argument("--radius", type=float, default=15.0)
    parser.add_argument("--height", type=float, default=2.8)
    args = parser.parse_args()
    if not (.02 <= args.grid <= .5 and .05 <= args.ground_cut <= .6):
        parser.error("网格允许 0.02–0.5 m，地面剔除高度允许 0.05–0.6 m")
    if not (1 <= args.radius <= 100 and args.ground_cut < args.height <= 10):
        parser.error("预览半径允许 1–100 m，最高高度须大于 ground-cut 且不超过 10 m")
    try:
        points = read_pcd(args.pcd)
        vertices, faces, map_vertices, floor, count = surface(
            points, args.grid, args.ground_cut, args.radius, args.height)
        write_mesh(args.out / "surface.ply", map_vertices, faces)
        render(args.out / "plaster.png", vertices, faces)
        meta = {"floor": floor, "ground_cut_m": args.ground_cut, "grid_m": args.grid,
                "radius_m": args.radius, "height_m": args.height, "source_points": len(points),
                "preview_points": count, "triangles": len(faces)}
        (args.out / "plaster.json").write_text(json.dumps(meta, indent=2))
        print(f"石膏预览保留 {count}/{len(points)} 点，生成 {len(faces)} 个三角面")
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
