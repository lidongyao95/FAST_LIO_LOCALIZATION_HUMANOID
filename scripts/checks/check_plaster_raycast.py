#!/usr/bin/env python3
"""Compare accelerated surface queries against the identical scene geometry."""
import json
from pathlib import Path
import sys
import time
import mujoco
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation'))
from plaster_raycast import PlasterRaycaster

scene = Path(sys.argv[1]).resolve()
model = mujoco.MjModel.from_xml_path(str(scene / 'scene.xml'))
data = mujoco.MjData(model)
meta = json.loads((scene / 'metadata.json').read_text())
spawn = meta['spawn_xy_m']
model.qpos0[:2] = spawn
mujoco.mj_resetData(model, data)
reference_groups = np.zeros(6, np.uint8)
reference_groups[[4, 5]] = 1
model.geom_group[model.geom('plaster_surface').id] = 4
model.geom_group[model.geom('assumed_flat_floor').id] = 5
model.geom_group[model.geom('nav_dynamic_obstacle_geom').id] = 5
caster = PlasterRaycaster(scene / 'scene.xml')
az, el = np.meshgrid(np.linspace(-np.pi, np.pi, 180, endpoint=False), np.deg2rad(np.linspace(-35, 35, 24)))
rays = np.ascontiguousarray(np.column_stack([(np.cos(el)*np.cos(az)).ravel(),
                                            (np.cos(el)*np.sin(az)).ravel(), np.sin(el).ravel()]))
ids = np.empty(len(rays), np.int32)
reference = np.empty(len(rays))
actual = np.empty(len(rays))
timings = []
errors = []
for enabled in (False, True, False):
    body = model.body('nav_dynamic_obstacle')
    data.mocap_pos[int(body.mocapid[0])] = [spawn[0]+1.5, spawn[1], .75 if enabled else -5.]
    mujoco.mj_forward(model, data)
    origin = np.array([*spawn, 1.2])
    mujoco.mj_multiRay(model, data, origin, rays.ravel(), reference_groups, 1, -1,
                      ids, reference, None, len(rays), 30.)
    before = time.perf_counter()
    caster.cast(model, data, origin, rays, ids, actual)
    timings.append(time.perf_counter() - before)
    assert np.array_equal(reference < 0, actual < 0), 'Ray hit/miss differs'
    hit = reference >= 0
    error = float(np.max(np.abs(reference[hit] - actual[hit])))
    assert error < 1e-6, error
    errors.append(error)
report = {'passed': True, 'rays_per_scan': len(rays), 'ray_model_geoms': caster.model.ngeom,
          'seconds_per_scan': timings, 'max_distance_error_m': max(errors),
          'dynamic_obstacle_sync': True}
(scene / 'raycast_validation.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
