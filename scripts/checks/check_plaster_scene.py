#!/usr/bin/env python3
"""Verify continuous visuals, matching LiDAR contacts and a free navigation spawn."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

scene = Path(sys.argv[1]).resolve()
meta = json.loads((scene / 'metadata.json').read_text())
assert meta['scene_representation'] == 'continuous plaster mesh'
root = ET.parse(scene / 'scene.xml').getroot()
names = [g.get('name', '') for g in root.iter('geom')]
assert not any(n.startswith(('scan_visual_', 'scan_collision_')) for n in names)
assert 'plaster_surface' in names
# Every physical triangle must be derived from the displayed OBJ face, not an
# independently reconstructed proxy obstacle layout.
vertices, faces = [], []
for line in (scene / 'plaster.obj').read_text().splitlines():
    fields = line.split()
    if fields and fields[0] == 'v':
        vertices.append(list(map(float, fields[1:4])))
    elif fields and fields[0] == 'f':
        faces.append([int(v.split('/')[0]) - 1 for v in fields[1:4]])
vertices, faces = np.asarray(vertices), np.asarray(faces)
meshes = {m.get('name'): m for m in root.find('asset').findall('mesh')}
contacts = [g for g in root.find('worldbody').findall('geom')
            if g.get('name', '').startswith('plaster_collision_')]
assert len(contacts) == len(faces)
maximum_face_error = 0.
for geom in contacts:
    assert geom.get('type') == 'mesh'
    index = int(geom.get('name').rsplit('_', 1)[1])
    prism = np.fromstring(meshes[geom.get('mesh')].get('vertex'), sep=' ').reshape(6, 3)
    middle = (prism[:3] + prism[3:]) / 2 + np.fromstring(geom.get('pos'), sep=' ')
    error = float(np.abs(middle - vertices[faces[index]]).max())
    maximum_face_error = max(maximum_face_error, error)
    assert error < 2e-6, 'Collision triangle does not follow the displayed plaster face'
model = mujoco.MjModel.from_xml_path(str(scene / 'scene.xml'))
model.qpos0[:2] = meta['spawn_xy_m']
model.body('support').pos[:2] = meta['spawn_xy_m']
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
origin = np.array([*meta['spawn_xy_m'], 1.2], dtype=float)
results = []
for angle in np.linspace(-np.pi, np.pi, 180, endpoint=False):
    direction = np.array([np.cos(angle), np.sin(angle), 0.])
    distances = []
    for group in (2, 3):
        mask = np.zeros(6, dtype=np.uint8)
        mask[group] = 1
        geom = np.array([-1], dtype=np.int32)
        distances.append(float(mujoco.mj_ray(model, data, origin, direction, mask,
                                             1, model.body('pelvis').id, geom)))
    results.append(distances)
rays = np.array(results)
hits = (rays[:, 0] > 0) & (rays[:, 1] > 0)
assert hits.sum() > 30, 'Insufficient paired visual/collision LiDAR hits'
agreement = np.mean((rays[:, 0] < 0) == (rays[:, 1] < 0))
errors = np.abs(rays[hits, 0] - rays[hits, 1])
assert agreement > .95, f'Visual/contact hit mismatch: {agreement}'
assert np.median(errors) < .02, f'Visual/contact ray error: {np.median(errors)}'
with (scene / 'map.pgm').open('rb') as stream:
    assert stream.readline().strip() == b'P5'
    width, height = map(int, stream.readline().split())
    assert stream.readline().strip() == b'255'
    image = np.frombuffer(stream.read(), np.uint8).reshape(height, width)[::-1]
extent = meta['crop_radius_m'] + .3
x, y = np.floor((np.array(meta['spawn_xy_m']) + extent) / meta.get('map_resolution_m', .05)).astype(int)
assert image[y, x] == 254, 'Navigation spawn is not free'
report = {'passed': True, 'voxel_geoms': 0,
          'contacts_follow_displayed_surface': True,
          'maximum_face_alignment_error_m': maximum_face_error,
          'triangles': meta['surface_triangles'], 'paired_ray_hits': int(hits.sum()),
          'ray_hit_agreement': float(agreement),
          'ray_median_error_m': float(np.median(errors)), 'spawn_is_free': True}
(scene / 'plaster_validation.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2), flush=True)
