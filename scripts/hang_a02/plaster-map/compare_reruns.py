"""Compare three aligned Nav2 trinary maps and render changed cells."""
import argparse
from collections import Counter
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.lines import Line2D
from matplotlib.path import Path as PolygonPath
import numpy as np
from PIL import Image
import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('original', type=Path, help='Directory containing map.yaml')
parser.add_argument('legacy_rerun', type=Path)
parser.add_argument('parameterized', type=Path)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()

def load(directory):
    with (directory/'map.yaml').open() as stream:
        meta = yaml.safe_load(stream)
    with Image.open(directory/meta['image']) as image:
        pixels = np.array(image)[::-1]
    if pixels.ndim != 2 or not np.isin(pixels, [0,205,254]).all():
        raise ValueError(f'Expected grayscale trinary map in {directory}')
    keys = ('origin','resolution','mode','negate','free_thresh','occupied_thresh')
    return pixels, tuple(meta[key] for key in keys)

maps = [load(path) for path in (args.original,args.legacy_rerun,args.parameterized)]
base, grid = maps[0]
if any(image.shape != base.shape or spec != grid for image,spec in maps[1:]):
    parser.error('Map dimensions, origin, resolution and trinary settings must match')
(ox,oy,yaw), resolution, *_ = grid
if yaw != 0:
    parser.error('This diagnostic expects unrotated grid axes')
ny,nx = base.shape
extent = [ox,ox+nx*resolution,oy,oy+ny*resolution]
x,y = np.meshgrid(ox+(np.arange(nx)+.5)*resolution,oy+(np.arange(ny)+.5)*resolution)
interior = [(-.3,.45),(1.95,-1.75),(3.6,-.95),(8.8,4.85),(7.8,7.05),
            (6.6,8.35),(5.15,7.8),(3.9,5.65),(2.5,3.4)]
inside = PolygonPath(interior).contains_points(np.column_stack([x.ravel(),y.ravel()])).reshape(base.shape)
start = np.hypot(x,y) < .35
states = {0:'Occupied',205:'Unknown',254:'Free'}
new_colors = {0:'#e7426c',205:'#ffad2d',254:'#00a9ce'}
palette = ListedColormap(['#182031','#a9b2bd','#fbf8f0'])
norm = BoundaryNorm([-1,100,230,256],3)
fig, axes = plt.subplots(2,3,figsize=(14.5,10),layout='constrained')
fig.suptitle('Hang A 02 | Occupancy grid repeatability',fontsize=21,weight='bold')

def draw(ax, image, title):
    ax.imshow(image, origin='lower',extent=extent,cmap=palette,norm=norm,interpolation='nearest')
    ax.set(title=title, xlabel='Map x [m]', ylabel='Map y [m]')
    outline = np.array([*interior,interior[0]])
    ax.plot(*outline.T,ls='--',lw=.7,color='#546684')
    ax.add_patch(plt.Circle((0,0),.35,fill=False,lw=1,color='#8a56be'))

titles = ['Original delivery','Legacy script rerun','Parameterized rerun']
for ax,(image,_),title in zip(axes[0],maps,titles):
    draw(ax,image,title)

metrics = {'shape_xy':[nx,ny], 'origin':[ox,oy,yaw], 'resolution_m':resolution,
           'comparisons':[]}
for ax,(i,j) in zip(axes[1],[(0,1),(0,2),(1,2)]):
    old,new = maps[i][0],maps[j][0]
    changed = old!=new
    n = int(changed.sum())
    draw(ax,old,f'{titles[i]} → {titles[j]}\n{n} changed cells ({n/base.size:.2%})')
    ax.images[0].set_alpha(.48)
    for state,color in new_colors.items():
        mask = changed & (new==state)
        ax.scatter(x[mask],y[mask],s=13,marker='s',color=color,edgecolors='none')
    counts = Counter(f'{states[int(a)]} → {states[int(b)]}' for a,b in zip(old[changed],new[changed]))
    metrics['comparisons'].append(dict(before=titles[i],after=titles[j],changed=n,
                                       interior=int((changed&inside).sum()),
                                       start=int((changed&start).sum()),
                                       transitions=dict(sorted(counts.items()))))
legend = [Line2D([],[],color=color,marker='s',linestyle='',label=f'Changed to {states[state].lower()}')
          for state,color in new_colors.items()]
legend += [Line2D([],[],color='#546684',ls='--',label='Interior ROI'),
           Line2D([],[],color='#8a56be',marker='o',mfc='none',linestyle='',label='Start ROI')]
fig.legend(handles=legend,loc='outside lower center',ncols=5)
args.out.mkdir(parents=True,exist_ok=True)
fig.savefig(args.out/'hang_a02_grid_rerun.png',dpi=125)
(args.out/'hang_a02_grid_rerun.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(metrics,indent=2,ensure_ascii=False))
