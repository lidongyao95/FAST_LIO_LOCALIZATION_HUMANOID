import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib.patches import Circle, Polygon
from matplotlib.path import Path
import numpy as np

d = np.load('grids.npz')
hits, body, names = d['hits'], d['body'], d['names']
xmin, xmax, ymin, ymax = d['bounds']
ny, nx = hits.shape[1:]
x, y = np.meshgrid(xmin + (np.arange(nx)+.5)*.1, ymin + (np.arange(ny)+.5)*.1)
start = np.hypot(x, y) < .35
# Visual ROI inside the surrounding structures. Vertices are in map meters.
interior = [(-.3,.45), (1.95,-1.75), (3.6,-.95), (8.8,4.85), (7.8,7.05),
            (6.6,8.35), (5.15,7.8), (3.9,5.65), (2.5,3.4)]
inside = Path(interior).contains_points(np.column_stack([x.ravel(),y.ravel()])).reshape(x.shape)
fig, axes = plt.subplots(4, 4, figsize=(16, 15), layout='constrained')
fig.suptitle('Hang A 02  |  Filter sweep on 4322', fontsize=21, weight='bold')
row_titles = ['Height above floor', 'Robot-relative XY range', 'Time subsampling', 'Gravity-compensated IMU']
for row, axes_row in enumerate(axes):
    for col, ax in enumerate(axes_row):
        j = 0 if col == 0 else 3*row+col
        density = np.ma.masked_equal(hits[j], 0)
        im = ax.imshow(density, origin='lower', extent=d['bounds'], interpolation='nearest',
                       cmap='magma', norm=LogNorm(vmin=1, vmax=500))
        ax.set_facecolor('#111a29')
        ax.plot(*body[:,:2].T, lw=.65, color='#54d4e0', alpha=.85)
        ax.add_patch(Circle((0,0), .35, fill=False, color='#f1c64b', lw=1))
        ax.add_patch(Polygon(interior, fill=False, color='#a6bbd4', ls='--', lw=.65, alpha=.8))
        ax.set_xlim(-7, 11)
        ax.set_ylim(-8, 10)
        ax.set_xticks([-5,0,5,10])
        ax.set_yticks([-5,0,5,10])
        ax.set_title(f'{names[j]}  |  {d["frame_counts"][j]:,} scans', fontsize=10)
        if col == 0:
            ax.set_ylabel(row_titles[row] + '\nMap y [m]', fontsize=11)
        if row == 3:
            ax.set_xlabel('Map x [m]')
fig.colorbar(im, ax=axes, shrink=.8, label='Returns per 10 cm XY cell (shared log scale)')
fig.supxlabel('Cyan: robot trajectory   |   Gold: start ROI   |   Dashed: interior sample ROI   |   Common obstacle band: 0.12–2.50 m above fitted floor', fontsize=11)
fig.savefig('comparison.png', dpi=135)

metrics = []
for j, name in enumerate(names):
    metrics.append(dict(name=str(name), frames=int(d['frame_counts'][j]),
                        hits=int(hits[j].sum()), cells=int((hits[j]>0).sum()),
                        interior_hits=int(hits[j][inside].sum()), interior_cells=int((hits[j][inside]>0).sum()),
                        start_hits=int(hits[j][start].sum()), start_cells=int((hits[j][start]>0).sum())))
with open('metrics.json', 'w') as f:
    json.dump(metrics, f, indent=2, ensure_ascii=False)
