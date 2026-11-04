#!/usr/bin/env python3
"""Check supported G1 stability and drop a sphere onto a scan collision voxel."""
import argparse
import json
import struct
import zlib
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def step_supported(model, data):
    joints = model.actuator_trnid[:, 0]
    qpos = model.jnt_qposadr[joints]
    qvel = model.jnt_dofadr[joints]
    data.ctrl[:] = np.clip(
        -80 * (data.qpos[qpos] - model.qpos0[qpos]) - 3 * data.qvel[qvel],
        model.actuator_ctrlrange[:, 0],
        model.actuator_ctrlrange[:, 1],
    )
    mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all():
        raise RuntimeError("Nonfinite simulation state")


def render_preview(model, data, destination):
    renderer = mujoco.Renderer(model, height=1000, width=1440)
    try:
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, 0.9]
        camera.distance, camera.azimuth, camera.elevation = 8, 135, -28
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        renderer.update_scene(data, camera=camera, scene_option=option)
        pixels = renderer.render()
    finally:
        renderer.close()

    # Encode RGB PNG with the standard library; no extra imaging dependency.
    def chunk(kind, payload):
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    height, width, _ = pixels.shape
    raw = b"".join(b"\x00" + row.tobytes() for row in pixels)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    destination.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def check_stability(scene, out, render):
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial = data.qpos[:3].copy()
    displacement, minimum = 0.0, 0.0
    for _ in range(round(4 / model.opt.timestep)):
        step_supported(model, data)
        displacement = max(displacement, float(np.linalg.norm(data.qpos[:3] - initial)))
        if data.ncon:
            minimum = min(minimum, min(c.dist for c in data.contact))
    warnings = {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number}
    report = {
        "simulation_seconds": float(data.time),
        "ngeom": model.ngeom,
        "finite_qpos": bool(np.isfinite(data.qpos).all()),
        "max_base_displacement_m": displacement,
        "minimum_contact_distance_m": float(minimum),
        "warnings": warnings,
        "passed": bool(not warnings and displacement < 0.01 and minimum > -0.01),
    }
    (out / "validation.json").write_text(json.dumps(report, indent=2))
    if render:
        render_preview(model, data, out / "preview.png")
    return report


def check_collision(scene, out):
    tree = ET.parse(scene)
    world = tree.getroot().find("worldbody")
    voxels = [
        g
        for g in world.findall("geom")
        if g.get("name", "").startswith("scan_collision_")
    ]
    if not voxels:
        raise ValueError("Scene has no scan collision voxels")
    highest = max(voxels, key=lambda g: float(g.get("pos").split()[2]))
    pos = np.fromstring(highest.get("pos"), sep=" ")
    top = pos[2] + float(highest.get("size").split()[2])
    body = ET.SubElement(
        world, "body", name="collision_probe", pos=f"{pos[0]} {pos[1]} {top + .6}"
    )
    ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", name="probe_geom", type="sphere", size=".06", mass=".1")
    path = out / "collision_check.xml"
    tree.write(path)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    probe, hit = model.geom("probe_geom").id, False
    for _ in range(round(2 / model.opt.timestep)):
        step_supported(model, data)
        for contact in data.contact:
            if probe in (contact.geom1, contact.geom2):
                other = contact.geom2 if contact.geom1 == probe else contact.geom1
                hit |= model.geom(other).name.startswith("scan_collision_")
    z = float(data.xpos[model.body("collision_probe").id, 2])
    resting = bool(abs(z - (top + 0.06)) < 0.02)
    report = {
        "probe_hit_scan_collision": hit,
        "surface_top_m": float(top),
        "probe_final_z_m": z,
        "probe_resting_on_voxel": resting,
        "passed": bool(hit and resting and not any(w.number for w in data.warning)),
    }
    (out / "collision_validation.json").write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene-dir", type=Path, default=ROOT / "sim/generated/real_scan"
    )
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    out = args.scene_dir.resolve()
    stability = check_stability(out / "scene.xml", out, args.render)
    collision = check_collision(out / "scene.xml", out)
    print(json.dumps({"stability": stability, "collision": collision}, indent=2))
    if not (stability["passed"] and collision["passed"]):
        raise SystemExit("Scan scene checks failed; inspect validation files")


if __name__ == "__main__":
    main()
