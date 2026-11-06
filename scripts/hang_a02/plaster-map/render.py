"""Export an observation-derived mesh and render a matte plaster preview."""
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np
from scipy.ndimage import gaussian_filter
from PIL import Image
from skimage.measure import marching_cubes

sys.path.insert(0, str(Path.home() / 'ldy/FAST_LIO_LOCALIZATION_HUMANOID/scripts'))
from export_g1_map import read_pcd

source = read_pcd(Path('clean.pcd'))
with np.load('evidence.npz') as evidence:
    normal, offset = evidence['floor_normal'], float(evidence['floor_offset'])
    path = evidence['body']
up = normal
east = np.array([up[2], 0., -up[0]])
east /= np.linalg.norm(east)
basis = np.stack([east, np.cross(up,east), up], axis=1)
level = source @ basis
level[:,2] += offset
pitch = .085
bins = np.floor(level / pitch).astype(int)
lower = bins.min(axis=0) - 4
bins -= lower
field = np.zeros(bins.max(axis=0) + 5, dtype=np.float32)
field[tuple(bins.T)] = 1.
field = gaussian_filter(field, .78)
vertices, faces, _, _ = marching_cubes(field, level=.13, spacing=(pitch,)*3,
                                       allow_degenerate=False)
vertices += (lower+.5)*pitch
triangles = vertices[faces]
normals = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-10)
# marching_cubes returns inward-facing triangle winding for this field.
normals *= -1
export = vertices.copy()
export[:,2] -= offset
export = export @ basis.T
with open('surface.ply', 'wb') as stream:
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\n'
              f'element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n')
    stream.write(header.encode('ascii'))
    stream.write(export.astype('<f4').tobytes())
    records = np.empty(len(faces), dtype=[('size','u1'),('face','<i4',(3,))])
    records['size'], records['face'] = 3, faces[:, ::-1]
    stream.write(records.tobytes())
print('Mesh vertices:', len(vertices), 'triangles:', len(faces), flush=True)

def render(azimuth, elevation, filename):
    az, el = np.deg2rad([azimuth,elevation])
    view = np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
    right = np.cross([0,0,1], view)
    right /= np.linalg.norm(right)
    vertical = np.cross(view,right)
    screen = vertices @ np.column_stack([right,vertical])
    depth = np.einsum('ijk,k->ij', triangles, view).mean(axis=1)
    order = np.argsort(depth)
    light = .72*view - .65*right + .65*vertical
    light /= np.linalg.norm(light)
    shade = .53 + .45 * np.maximum(normals@light, 0)
    colors = shade[:,None] * np.array([1.0,.982,.949])
    fig, ax = plt.subplots(figsize=(13, 9), facecolor='#26292c')
    ax.set_facecolor('#26292c')
    ax.add_collection(PolyCollection(screen[faces[order]], facecolors=colors[order],
                                    edgecolors='face', linewidths=.20, antialiased=True))
    low, high = screen.min(axis=0), screen.max(axis=0)
    ax.set(xlim=(low[0]-.4,high[0]+.4), ylim=(low[1]-.4,high[1]+.4), aspect='equal')
    ax.axis('off')
    fig.subplots_adjust(left=.03,right=.97,bottom=.07,top=.90)
    fig.text(.06,.94,'HANG A 02',color='#f8f4ec',fontsize=25,weight='bold')
    fig.text(.06,.046,'DUFOMap static returns  /  LiDAR surface reconstruction',color='#d6d3ce',fontsize=12)
    fig.savefig(filename,dpi=160,facecolor=fig.get_facecolor())
    plt.close(fig)

render(-35, 52, 'plaster.png')
image = np.array(Image.open('map.pgm'))
colors = ListedColormap(['#14181f', '#959ca4', '#f7f4eb'])
norm = BoundaryNorm([-1,100,230,256], 3)
fig, ax = plt.subplots(figsize=(8,8),layout='constrained')
ax.imshow(image, cmap=colors, norm=norm, extent=[-10.6,12.5,-11.8,11.7],
          interpolation='nearest')
ax.plot(*path[:,:2].T,color='#22a6ad',lw=.9,label='Recorded trajectory')
ax.scatter([0],[0],s=20,marker='+',color='#d18915',label='Map origin')
ax.set(title='HANG A 02  /  Navigation occupancy', xlabel='Map x [m]', ylabel='Map y [m]')
ax.legend(loc='lower right')
fig.savefig('navigation.png',dpi=150)
