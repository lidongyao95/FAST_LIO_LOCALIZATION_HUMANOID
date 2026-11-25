#!/usr/bin/env python3
"""Generate a floor-aligned scan scene and paired maps for staged G1 navigation."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

from pcd import read_pcd

ROOT = Path(__file__).resolve().parents[2]


def floor_transform(xyz, ground_z_max=-0.5):
    candidates = xyz[(np.linalg.norm(xyz[:, :2], axis=1) < 7) & (xyz[:, 2] < ground_z_max)]
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


def fit_map_floor(xyz):
    """Fit near the scan origin with a height window relative to the observed floor."""
    sample = xyz[::max(1, (len(xyz) + 119_999) // 120_000)]
    near = sample[np.linalg.norm(sample[:, :2], axis=1) < 7]
    if len(near) < 100:
        raise ValueError("Insufficient points within 7 m of the map origin for floor fitting")
    cutoff = np.percentile(near[:, 2], 10) + 0.55
    return floor_transform(sample, ground_z_max=cutoff)


def choose_spawn(collision, radius):
    """Return the free spawn nearest the scan origin, with 0.75 m clearance."""
    limit = radius - 0.6
    grid = np.arange(-limit, limit + 1e-9, 0.25)
    candidates = np.array([(x, y) for x in grid for y in grid])
    candidates = candidates[np.linalg.norm(candidates, axis=1) <= limit]
    distance = np.sqrt(((candidates[:, None, :] - collision[None, :, :2]) ** 2).sum(-1)).min(1)
    free = candidates[distance >= 0.75]
    clearance = distance[distance >= 0.75]
    if not free.size:
        raise ValueError("No free spawn with 0.75 m clearance inside the crop radius")
    order = np.lexsort((-clearance, np.linalg.norm(free, axis=1)))
    return free[order[0]], float(clearance[order[0]])


def build(args, start=(0.0, 0.0)):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    raw = read_pcd(args.pcd).astype(float)
    _, T, floor = fit_map_floor(raw)
    from scene_filters import prepare
    points, filter_report = prepare(raw, T, args, out)
    from plaster_geometry import reconstruct, sample_surface, write_obj
    vertices, faces = reconstruct(
        points, args.radius, pitch=args.plaster_grid, max_faces=args.max_faces,
        ground_cut=args.plaster_ground_cut, height=args.plaster_height)
    write_obj(out / "plaster.obj", vertices, faces)
    surface_points = sample_surface(vertices, faces)
    collision = surface_points[(surface_points[:, 2] > .12)
                               & (surface_points[:, 2] < 1.8)]
    if not len(collision):
        raise ValueError("No plaster surfaces within robot collision height")
    # Clearance checks need surface samples, not axis-aligned collision boxes.
    collision = np.unique(np.round(collision / .08).astype(int), axis=0) * .08
    print(f"plaster surface: {len(vertices)} vertices, {len(faces)} triangles", flush=True)
    if getattr(args, "auto_start", False):
        start, clearance = choose_spawn(collision, args.radius)
        print(
            f"auto spawn {start[0]:.2f} {start[1]:.2f} clearance {clearance:.2f} m",
            file=sys.stderr,
        )
    if np.any(
        (np.linalg.norm(collision[:, :2] - start, axis=1) < 0.6) & (collision[:, 2] < 1.8)
    ):
        raise ValueError(
            "Plaster surface overlaps robot spawn; choose another spawn after inspection"
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

    asset = ET.SubElement(scene, "asset")
    ET.SubElement(asset, "mesh", name="plaster_surface", file=str(out / "plaster.obj"))
    ET.SubElement(world, "geom", name="plaster_surface", type="mesh",
                  mesh="plaster_surface", rgba=".92 .90 .85 1", group="2",
                  contype="0", conaffinity="0")
    # MuJoCo convexifies mesh contacts. A single room mesh would seal concavities.
    # Use one 8-mm triangle prism per face; these preserve holes and passages.
    for i, tri in enumerate(vertices[faces]):
        center = tri.mean(axis=0)
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        normal /= np.linalg.norm(normal)
        prism = np.concatenate([tri - center + normal * .004,
                                tri - center - normal * .004])
        ET.SubElement(asset, "mesh", name=f"plaster_contact_{i}",
                      vertex=fmt(prism.ravel()))
        ET.SubElement(world, "geom", name=f"plaster_collision_{i}",
                      type="mesh", mesh=f"plaster_contact_{i}", pos=fmt(center),
                      rgba=".92 .90 .85 1", group="3")
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
    ET.SubElement(world, "body", name="support", mocap="true", pos=f"{start[0]} {start[1]} .793")
    eq = ET.SubElement(scene, "equality")
    # The robot qpos is moved to spawn by WalkingSim after MJCF compilation.
    # An implicit relpose would retain the pre-spawn offset and pull it back.
    ET.SubElement(eq, "weld", body1="support", body2="pelvis",
                  relpose="0 0 0 1 0 0 0", solref=".004 1")
    path = out / "scene.xml"
    ET.ElementTree(scene).write(path)
    metadata = {
        "source": str(args.pcd.resolve()),
        "scene": str(path),
        "world_from_camera_init": T.tolist(),
        "floor_fit": floor,
        "filters": filter_report["parameters"],
        "filter_report": str(out / "filter_report.json"),
        "filtered_source": str(out / "filtered.pcd"),
        "scene_representation": "continuous plaster mesh",
        "surface_vertices": len(vertices),
        "surface_triangles": len(faces),
        "collision_thickness_m": .008,
        "crop_radius_m": args.radius,
        "map_resolution_m": args.resolution,
        "spawn_xy_m": list(start),
        "floor_is_assumed_flat_extension": True,
        "collision_model": "8mm triangular prisms following the plaster surface; no voxel boxes",
        "startup": "scripts/simulation/start_walking_sim.sh --model-path scene.xml; controller/localization/Nav2 stages required",
    }
    export_navigation(out, surface_points, args.radius, args.resolution)
    metadata["navigation_map_source"] = (
        "sampled plaster surface; artificial planar floor treated as free"
    )
    (out / "metadata.json").write_text(json.dumps(metadata, indent=2))
    return path


def export_navigation(out, collision, radius, resolution=.05):
    # Same surface as visual geometry and thin triangle contacts.
    samples = [collision]
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
    from scipy.ndimage import binary_dilation
    obstacle = collision[(collision[:, 2] > .12) & (collision[:, 2] < 1.8)]
    pixels = np.floor((obstacle[:, :2] + extent) / resolution).astype(int)
    valid = np.all((pixels >= 0) & (pixels < size), axis=1)
    occupied = np.zeros_like(image, dtype=bool)
    occupied[pixels[valid, 1], pixels[valid, 0]] = True
    occupied = binary_dilation(occupied, iterations=1)
    image[occupied] = 0
    with (out / "map.pgm").open("wb") as f:
        f.write(f"P5\n{size} {size}\n255\n".encode())
        f.write(image[::-1].tobytes())
    (out / "map.yaml").write_text(
        f"image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{-extent}, {-extent}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pcd",
        type=Path,
        required=True,
        help="ASCII or uncompressed binary PCD corrected for inverted mounting",
    )
    parser.add_argument("--out", type=Path, default=ROOT / "sim/generated/real_scan")
    parser.add_argument("--radius", "--max-map-radius", "--plaster-radius", type=float, default=8.0)
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"), default=(0.0, 0.0))
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="spawn on the free point nearest the scan origin instead of (0, 0)",
    )
    from scene_filters import add_arguments, validate, check_sources
    add_arguments(parser)
    args = parser.parse_args(argv)
    try:
        validate(args)
        check_sources(args)
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    if not np.isfinite(args.radius) or not 1 <= args.radius <= 20:
        parser.error("radius must be finite and between 1 and 20 metres")
    if not np.isfinite(args.start).all() or np.linalg.norm(args.start) >= args.radius - 0.6:
        parser.error("start must be finite and at least 0.6 m inside the crop radius")
    return args


def main():
    import mujoco
    args = parse_arguments()
    scene = build(args, args.start)
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
