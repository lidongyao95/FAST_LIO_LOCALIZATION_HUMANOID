#!/usr/bin/env python3
"""Generate a floor-aligned scan scene and paired maps for staged G1 navigation."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read_pcd(path):
    with path.open("rb") as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Missing PCD DATA header")
            parts = line.decode().strip().split()
            if parts and not parts[0].startswith("#"):
                header[parts[0]] = parts[1:]
            if parts and parts[0] == "DATA":
                break
        if header["DATA"] != ["binary"] or any(int(c) != 1 for c in header["COUNT"]):
            raise ValueError("Expected uncompressed binary scalar-field PCD")
        dtype = np.dtype(
            [
                (n, {"F": "f", "I": "i", "U": "u"}[t] + s)
                for n, t, s in zip(header["FIELDS"], header["TYPE"], header["SIZE"])
            ]
        )
        data = np.frombuffer(f.read(), dtype=dtype)
        if len(data) != int(header["POINTS"][0]):
            raise ValueError("PCD point count mismatch")
        xyz = np.column_stack([data[k] for k in ("x", "y", "z")]).astype(float)
        return xyz[np.isfinite(xyz).all(axis=1)]


def floor_transform(xyz):
    candidates = xyz[(np.linalg.norm(xyz[:, :2], axis=1) < 7) & (xyz[:, 2] < -0.5)]
    if len(candidates) < 100:
        raise ValueError("Insufficient floor candidates; need manual floor calibration")
    rng = np.random.default_rng(42)
    best = np.zeros(len(candidates), dtype=bool)
    for _ in range(600):
        a, b, c = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        if abs(normal[2]) < 0.96:
            continue
        mask = np.abs((candidates - a) @ normal) < 0.035
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < 100:
        raise ValueError("No supported floor plane")
    selected = candidates[best]
    center = selected.mean(0)
    _, _, vt = np.linalg.svd(selected - center, full_matrices=False)
    n = vt[-1]
    if n[2] < 0:
        n = -n
    v = np.cross(n, [0.0, 0.0, 1.0])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + K + K @ K / (1 + n[2])
    d = -n @ center
    T = np.eye(4)
    T[:3, :3] = R
    T[2, 3] = d
    return (
        xyz @ R.T + T[:3, 3],
        T,
        {
            "inliers": int(best.sum()),
            "normal": n.tolist(),
            "plane_offset_m": float(d),
            "rms_m": float(np.sqrt(np.mean(((selected - center) @ n) ** 2))),
        },
    )


def build(args):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    raw = read_pcd(args.pcd)
    points, T, floor = floor_transform(raw)
    points = points[
        (np.linalg.norm(points[:, :2], axis=1) < args.radius)
        & (points[:, 2] > -0.12)
        & (points[:, 2] < 2.5)
    ]
    # Small visual voxels preserve the scan without treating a convex hull as a room.
    cells = np.unique(np.floor(points / 0.09).astype(int), axis=0)
    visual = (cells + 0.5) * 0.09
    obs = points[(points[:, 2] > 0.18) & (points[:, 2] < 2.3)]
    cells = np.floor(obs / 0.25).astype(int)
    cells, counts = np.unique(cells, axis=0, return_counts=True)
    collision = (cells[counts >= 2] + 0.5) * 0.25
    if not len(collision):
        raise ValueError("No supported obstacle voxels; scene needs manual inspection")
    if np.any(
        (np.linalg.norm(collision[:, :2], axis=1) < 0.6) & (collision[:, 2] < 1.8)
    ):
        raise ValueError(
            "Scan obstacles overlap robot spawn; choose another spawn after inspection"
        )
    robot = ROOT / ".deps/src/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"
    scene = ET.Element("mujoco", model="G1 scan navigation scene")
    ET.SubElement(scene, "include", file=str(robot))
    ET.SubElement(
        scene, "compiler", angle="radian", meshdir=str(robot.parent / "meshes")
    )
    ET.SubElement(scene, "option", timestep=".002", gravity="0 0 -9.81")
    visual_cfg = ET.SubElement(scene, "visual")
    ET.SubElement(visual_cfg, "global", offwidth="1440", offheight="1000")
    ET.SubElement(visual_cfg, "headlight", ambient=".55 .55 .55", diffuse=".7 .7 .7")
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", pos="0 0 6", dir="0 0 -1", diffuse=".8 .8 .8")
    ET.SubElement(
        world,
        "geom",
        name="assumed_flat_floor",
        type="box",
        pos="0 0 -.05",
        size=f"{args.radius} {args.radius} .05",
        rgba=".20 .24 .29 1",
        group="0",
    )

    def fmt(p):
        return " ".join(f"{x:.6f}" for x in p)

    for i, p in enumerate(visual):
        color = [0.20 + 0.5 * min(max(p[2] / 2, 0), 1), 0.72, 0.84, 1]
        ET.SubElement(
            world,
            "geom",
            name=f"scan_visual_{i}",
            type="box",
            pos=fmt(p),
            size=".025 .025 .025",
            rgba=fmt(color),
            contype="0",
            conaffinity="0",
            group="2",
        )
    for i, p in enumerate(collision):
        ET.SubElement(
            world,
            "geom",
            name=f"scan_collision_{i}",
            type="box",
            pos=fmt(p),
            size=".125 .125 .125",
            rgba=".95 .45 .12 .25",
            group="3",
        )
    dynamic = ET.SubElement(
        world, "body", name="nav_dynamic_obstacle", mocap="true", pos="0 0 -5"
    )
    ET.SubElement(
        dynamic,
        "geom",
        name="nav_dynamic_obstacle_geom",
        type="box",
        size=".35 .35 .75",
        rgba=".9 .35 .2 1",
        group="0",
    )
    ET.SubElement(world, "body", name="support", mocap="true", pos="0 0 .793")
    eq = ET.SubElement(scene, "equality")
    ET.SubElement(eq, "weld", body1="support", body2="pelvis", solref=".004 1")
    path = out / "scene.xml"
    ET.ElementTree(scene).write(path)
    metadata = {
        "source": str(args.pcd.resolve()),
        "scene": str(path),
        "world_from_camera_init": T.tolist(),
        "floor_fit": floor,
        "visual_voxels": len(visual),
        "collision_voxels": len(collision),
        "crop_radius_m": args.radius,
        "floor_is_assumed_flat_extension": True,
        "collision_model": "0.25m occupied voxels with >=2 samples, not a watertight reconstruction",
        "startup": "scripts/start_walking_sim.sh --model-path scene.xml; controller/localization/Nav2 stages required",
    }
    export_navigation(out, collision, args.radius)
    metadata["navigation_map_source"] = (
        "same approximated collision geometry; artificial planar floor treated as free"
    )
    (out / "metadata.json").write_text(json.dumps(metadata, indent=2))
    return path


def export_navigation(out, collision, radius):
    # Match map and ray-cast geometry; do not register against a different raw scan.
    samples = []
    for pos in collision:
        for axis in range(3):
            a, b = [i for i in range(3) if i != axis]
            u, v = np.meshgrid(
                np.linspace(-0.125, 0.125, 4), np.linspace(-0.125, 0.125, 4)
            )
            for sign in (-1, 1):
                p = np.tile(pos, (u.size, 1))
                p[:, axis] += sign * 0.125
                p[:, a] += u.ravel()
                p[:, b] += v.ravel()
                samples.append(p)
    x, y = np.meshgrid(
        np.arange(-radius, radius, 0.15), np.arange(-radius, radius, 0.15)
    )
    samples.append(np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)]))
    xyz = np.concatenate(samples)
    with (out / "map.ply").open("w") as f:
        f.write(
            f"ply\nformat ascii 1.0\nelement vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
        )
        np.savetxt(f, xyz, fmt="%.4f")
    resolution = 0.05
    extent = radius + 0.3
    size = int(np.ceil(2 * extent / resolution))
    image = np.full((size, size), 205, np.uint8)
    lo = int(np.ceil((extent - radius) / resolution))
    hi = int(np.floor((extent + radius) / resolution))
    image[lo:hi, lo:hi] = 254
    # Boundary of the finite demonstration floor is blocked in the navigation map.
    image[lo : lo + 3, lo:hi] = 0
    image[hi - 3 : hi, lo:hi] = 0
    image[lo:hi, lo : lo + 3] = 0
    image[lo:hi, hi - 3 : hi] = 0
    for x, y, z in collision:
        if z - 0.125 >= 1.8:
            continue
        x0, y0 = np.floor((np.array([x, y]) - 0.125 + extent) / resolution).astype(int)
        x1, y1 = np.ceil((np.array([x, y]) + 0.125 + extent) / resolution).astype(int)
        image[max(0, y0) : min(size, y1), max(0, x0) : min(size, x1)] = 0
    with (out / "map.pgm").open("wb") as f:
        f.write(f"P5\n{size} {size}\n255\n".encode())
        f.write(image[::-1].tobytes())
    (out / "map.yaml").write_text(
        f"image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{-extent}, {-extent}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )


def main():
    import mujoco

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pcd",
        type=Path,
        required=True,
        help="Uncompressed binary PCD already corrected for inverted mounting",
    )
    parser.add_argument("--out", type=Path, default=ROOT / "sim/generated/real_scan")
    parser.add_argument("--radius", type=float, default=8.0)
    args = parser.parse_args()
    if not np.isfinite(args.radius) or not 1 <= args.radius <= 20:
        parser.error("radius must be finite and between 1 and 20 metres")
    scene = build(args)
    # Compile now so malformed assets cannot be mistaken for a ready scene.
    model = mujoco.MjModel.from_xml_path(str(scene))
    print(
        json.dumps(
            {
                "scene": str(scene),
                "geoms": model.ngeom,
                "map_ply": str(scene.with_name("map.ply")),
                "map_yaml": str(scene.with_name("map.yaml")),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
