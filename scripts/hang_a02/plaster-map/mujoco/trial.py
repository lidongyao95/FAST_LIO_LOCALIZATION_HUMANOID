"""Assemble and verify a MuJoCo scene from the cleaned hang_A_02 assets."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

repo = Path.home() / 'ldy/FAST_LIO_LOCALIZATION_HUMANOID'
sys.path.insert(0, str(repo / 'scripts'))
from check_g1_scan_scene import step_supported

out = Path.cwd()
assets = out.parent
old = Path('/tmp/hang-a02-clean')
transform = np.array(json.loads((old/'metadata.json').read_text())['world_from_camera_init'])
def world(points):
    return points @ transform[:3,:3].T + transform[:3,3]

with (assets/'surface.ply').open('rb') as stream:
    header = []
    while True:
        line = stream.readline().decode().strip()
        if line == 'end_header':
            break
        header.append(line)
    nvert = int(next(s.split()[-1] for s in header if s.startswith('element vertex')))
    nface = int(next(s.split()[-1] for s in header if s.startswith('element face')))
    vertices = world(np.fromfile(stream, '<f4', nvert*3).reshape(-1,3))
    faces = np.fromfile(stream, dtype=[('n','u1'),('ids','<i4',(3,))], count=nface)
assert (faces['n'] == 3).all()
with (out/'plaster.obj').open('w') as stream:
    np.savetxt(stream, vertices, fmt='v %.7f %.7f %.7f')
    np.savetxt(stream, faces['ids']+1, fmt='f %d %d %d')

tree = ET.parse(old/'scene-merged.xml')
root = tree.getroot()
body = root.find('worldbody')
for geom in body.findall('geom'):
    if geom.get('name','').startswith(('scan_visual_', 'scan_collision_')):
        body.remove(geom)
floor = body.find("geom[@name='assumed_flat_floor']")
floor.set('rgba', '.19 .20 .22 1')
# Keep the flat physics floor and hide its giant slab in the plaster preview.
floor.set('group', '3')
asset = root.find('asset')
if asset is None:
    asset = ET.Element('asset')
    root.insert(list(root).index(body), asset)
ET.SubElement(asset,'mesh',name='plaster',file=str(out/'plaster.obj'),smoothnormal='true')
ET.SubElement(asset,'material',name='matte_plaster',rgba='.89 .87 .82 1',specular='.06',shininess='.1')
ET.SubElement(body,'geom',name='plaster_visual',type='mesh',mesh='plaster',material='matte_plaster',
              contype='0',conaffinity='0',group='2')

with np.load(assets/'evidence.npz') as ev:
    support = ev['obstacle']
    route = world(ev['body'])
points = np.frombuffer((assets/'clean.pcd').read_bytes().split(b'DATA binary\n', 1)[1], '<f4').reshape(-1,3)
xy = np.floor((points[:,:2]-[-10.6,-11.8])/.1).astype(int)
inside = (xy[:,0]>=0)&(xy[:,0]<support.shape[1])&(xy[:,1]>=0)&(xy[:,1]<support.shape[0])
points, xy = points[inside], xy[inside]
points = world(points[support[xy[:,1],xy[:,0]]>=3])
points = points[(points[:,2]>=.18)&(points[:,2]<=2.5)]
pitch = .20
cells, counts = np.unique(np.floor(points/pitch).astype(int), axis=0, return_counts=True)
cells = cells[counts>=3]
for i, cell in enumerate(cells):
    xyz = ' '.join(f'{v:.3f}' for v in (cell+.5)*pitch)
    ET.SubElement(body,'geom',name=f'scan_collision_{i}',type='box',pos=xyz,
                  size='.10 .10 .10',rgba='.9 .7 .4 1',group='3')
tree.write('scene.xml')

model = mujoco.MjModel.from_xml_path(str(out/'scene.xml'))
data = mujoco.MjData(model)
mujoco.mj_forward(model,data)
def ray(origin, direction, groups):
    geom = np.array([-1],dtype=np.int32)
    distance = mujoco.mj_ray(model,data,np.asarray(origin,dtype=float),
                             np.asarray(direction,dtype=float),groups,1,-1,geom)
    return {'distance_m': float(distance), 'geom': model.geom(int(geom[0])).name if geom[0]>=0 else None}

colliders = np.array([0,0,0,1,0,0],dtype=np.uint8)
visual = np.array([0,0,1,0,0,0],dtype=np.uint8)
center = world(np.array([4.,4.,0.]))[:2]
angles = np.linspace(0,2*np.pi,16,endpoint=False)
rays = [ray([*center,.9],[np.cos(a),np.sin(a),0],colliders) for a in angles]
surface_rays = [ray([*center,.9],[np.cos(a),np.sin(a),0],visual) for a in angles]
floor_ray = ray([*center,1.],[0,0,-1],colliders)
assert floor_ray['geom'] == 'assumed_flat_floor'
assert sum(r['distance_m']>1 for r in rays)>=8
assert all(r['distance_m']<0 or r['distance_m']>.5 for r in rays)
center_positions = route[np.linalg.norm(route[:,:2]-center,axis=1)<2.,:2]
distances = np.array([np.linalg.norm((cells[:,:2]+.5)*pitch - pos, axis=1).min()
                      for pos in center_positions])
q0 = data.qpos[:3].copy()
max_drift = 0.
for _ in range(2000):
    step_supported(model,data)
    max_drift = max(max_drift,float(np.linalg.norm(data.qpos[:3]-q0)))
warnings = {str(i): int(w.number) for i,w in enumerate(data.warning) if w.number}
assert not warnings
from OpenGL.GL import glGetString, GL_RENDERER
renderer = mujoco.Renderer(model,height=1000,width=1440)
option = mujoco.MjvOption()
option.geomgroup[3]=0
for name, look, dist, az, elev in [
    ('overview',[2.5,2.5,.6],22,145,-61),
    ('inside',[2.5,2.,.85],9,135,-23),
]:
    camera = mujoco.MjvCamera()
    camera.lookat[:] = look
    camera.distance,camera.azimuth,camera.elevation = dist,az,elev
    renderer.update_scene(data,camera=camera,scene_option=option)
    pixels = renderer.render()
    import struct, zlib
    def chunk(kind, payload):
        return struct.pack('>I',len(payload))+kind+payload+struct.pack('>I',zlib.crc32(kind+payload))
    raw = b''.join(b'\0'+row.tobytes() for row in pixels)
    Path(f'{name}.png').write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',1440,1000,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
gpu = glGetString(GL_RENDERER).decode()
renderer.close()
report = dict(mujoco=mujoco.__version__, mesh_vertices=nvert, mesh_faces=nface,
              obstacle_geoms=len(cells), simulation_seconds=float(data.time),
              supported_base_max_drift_m=max_drift,warnings=warnings,
              center_xy=center.tolist(), floor_ray=floor_ray, collision_rays=rays,
              visual_mesh_rays=surface_rays, center_route_samples=len(distances),
              center_route_min_distance_to_collider_center_xy_m=float(distances.min()),
              renderer=gpu,world_from_camera_init=transform.tolist())
Path('trial.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
