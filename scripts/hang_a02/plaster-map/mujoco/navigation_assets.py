"""Reproject hang_A_02 navigation assets into the tested MuJoCo world."""
import json
from pathlib import Path

import cv2
import numpy as np

source = Path.cwd().parent
meta = json.loads((source / 'metadata.json').read_text())
trial = json.loads(Path('trial.json').read_text())
transform = np.array(trial['world_from_camera_init'])
rotation, translation = transform[:3, :3], transform[:3, 3]
floor = meta['floor_fit']
normal = np.asarray(floor['normal'])
xy_rotation = rotation[:2, :2] - np.outer(rotation[:2, 2], normal[:2]/normal[2])
xy_translation = translation[:2] - rotation[:2, 2]*floor['plane_offset_m']/normal[2]
origin = np.array(meta['origin'][:2])
resolution = meta['resolution']
image = cv2.imread(str(source/'map.pgm'), cv2.IMREAD_UNCHANGED)[::-1]
height, width = image.shape
corners = origin + resolution*np.array([[0,0],[width,0],[width,height],[0,height]])
corners = corners @ xy_rotation.T + xy_translation
low = np.floor(corners.min(axis=0)/resolution)*resolution
high = np.ceil(corners.max(axis=0)/resolution)*resolution
nx, ny = np.rint((high-low)/resolution).astype(int)
grid = np.stack(np.meshgrid(np.arange(nx), np.arange(ny)),axis=-1)
xy = ((grid+.5)*resolution+low-xy_translation) @ np.linalg.inv(xy_rotation).T
ix, iy = np.floor((xy-origin)/resolution).astype(int).transpose(2,0,1)
valid = (ix>=0)&(iy>=0)&(ix<width)&(iy<height)
mapped = np.full((ny,nx),205,dtype=np.uint8)
mapped[valid] = image[iy[valid],ix[valid]]
start = np.array(trial['center_xy'])
sx,sy = np.floor((start-low)/resolution).astype(int)
assert mapped[sy,sx] == 254, 'Spawn must be in a free grid cell'
assert cv2.imwrite('world-map.pgm',mapped[::-1])
Path('world-map.yaml').write_text(
    f'image: world-map.pgm\nmode: trinary\nresolution: {resolution}\n'
    f'origin: [{low[0]}, {low[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
payload = (source/'clean.pcd').read_bytes().split(b'DATA binary\n',1)[1]
xyz = np.frombuffer(payload,'<f4').reshape(-1,3) @ rotation.T + translation
header = ('ply\nformat binary_little_endian 1.0\n'
          f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
with Path('world-cloud.ply').open('wb') as stream:
    stream.write(header.encode('ascii'))
    stream.write(xyz.astype('<f4').tobytes())
print('Spawn in free cell:', *start, 'map dimensions:', nx, ny)
