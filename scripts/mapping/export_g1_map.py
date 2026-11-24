#!/usr/bin/env python3
"""Convert uncompressed PCD into paired Open3D and Nav2 map assets."""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np

from pcd import read_pcd

def export(source, out, resolution, known_free_radius, *, floor_band=.15,
           obstacle_min=.2, obstacle_max=1.8, max_range=None, floor_model='percentile',
           voxel_size=0.0):
    if floor_model not in ('percentile', 'plane'):
        raise ValueError('Floor model must be percentile or plane')
    values = (resolution, known_free_radius, floor_band, obstacle_min, obstacle_max, voxel_size)
    if not all(map(math.isfinite, values)):
        raise ValueError('Projection parameters must be finite')
    if resolution <= 0 or floor_band <= 0 or known_free_radius < 0:
        raise ValueError('Resolution and floor band must be positive, and free radius nonnegative')
    if obstacle_min >= obstacle_max:
        raise ValueError('Obstacle minimum must be below obstacle maximum')
    if voxel_size < 0:
        raise ValueError('3D voxel size must be nonnegative')
    if max_range is not None and (not math.isfinite(max_range) or max_range <= 0):
        raise ValueError('Maximum range must be finite and positive')
    xyz=read_pcd(source)
    if len(xyz)<100:raise ValueError('Map has too few finite points')
    # Range is measured in the map plane about the mapping start. Keep
    # the paired 3D localization map intact.
    projected = xyz if max_range is None else xyz[np.hypot(xyz[:, 0], xyz[:, 1]) <= max_range]
    if len(projected) < 100:
        raise ValueError('Too few points remain for occupancy projection')
    z_percentiles = dict(zip(('5', '50', '95'), map(float, np.percentile(projected[:, 2], [5, 50, 95]))))
    floor_z=z_percentiles['5']
    floor_fit = None
    if floor_model == 'plane':
        from g1_scan_scene import fit_map_floor

        # Fit a bounded sample, then classify the full projection by signed distance.
        _, _, floor_fit = fit_map_floor(projected)
        normal = np.asarray(floor_fit['normal'])
        floor_z = float(-floor_fit['plane_offset_m'] / normal[2])
        above_floor = projected @ normal + floor_fit['plane_offset_m']
    else:
        above_floor = projected[:, 2] - floor_z
    ox=float(np.floor((projected[:,0].min()-.5)/resolution)*resolution)
    oy=float(np.floor((projected[:,1].min()-.5)/resolution)*resolution)
    width=int(np.ceil((projected[:,0].max()+.5-ox)/resolution))
    height=int(np.ceil((projected[:,1].max()+.5-oy)/resolution))
    if width*height>8_000_000:raise ValueError('Projected occupancy map is too large')
    ix=np.floor((projected[:,0]-ox)/resolution).astype(int)
    iy=np.floor((projected[:,1]-oy)/resolution).astype(int)
    floor=np.zeros((height,width),np.uint8)
    obstacle=np.zeros_like(floor)
    floor_mask=np.abs(above_floor)<floor_band
    obstacle_band=(above_floor>obstacle_min)&(above_floor<obstacle_max)
    floor[iy[floor_mask],ix[floor_mask]]=1
    obstacle[iy[obstacle_band],ix[obstacle_band]]=1
    cover=cv2.dilate(floor,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(int(.8/resolution)//2*2+1,)*2))
    if known_free_radius>0:
        cx=int((0.-ox)/resolution);cy=int((0.-oy)/resolution)
        cv2.circle(cover,(cx,cy),round(known_free_radius/resolution),1,-1)
    occupied=cv2.dilate(obstacle,np.ones((3,3),np.uint8))
    image=np.full((height,width),205,np.uint8)
    image[cover>0]=254
    image[occupied>0]=0
    origin_x = math.floor(-ox / resolution)
    origin_y = math.floor(-oy / resolution)
    origin_cell = int(image[origin_y, origin_x]) if 0 <= origin_x < width and 0 <= origin_y < height else None
    source_points = len(xyz)
    if voxel_size:
        cells = np.floor(xyz / voxel_size).astype(np.int64)
        _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
        inverse = inverse.reshape(-1)
        xyz = np.column_stack([np.bincount(inverse, weights=axis) / counts for axis in xyz.T])
        if len(xyz) < 100:
            raise ValueError('Too few points after 3D downsampling; reduce --voxel')
    out.mkdir(parents=True,exist_ok=True)
    ply=out/'map.ply'
    with ply.open('w') as f:
        f.write(f'ply\nformat ascii 1.0\nelement vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(f,xyz,fmt='%.4f')
    with (out/'map.pgm').open('wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode());f.write(image[::-1].tobytes())
    (out/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {resolution}\n'
                                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    meta={'source_pcd':str(source.resolve()),'frame':'camera_init','map_from_camera_init':[0,0,0,0,0,0],
          'points':len(xyz),'source_points':source_points,'voxel_size_m':voxel_size,
          'floor_z':floor_z,'resolution':resolution,'origin':[ox,oy,0],
          'size':[width,height],'known_free_radius':known_free_radius,
          'projected_points': len(projected), 'max_range': max_range,
          'z_percentiles': z_percentiles, 'floor_band': floor_band,
          'obstacle_min': obstacle_min, 'obstacle_max': obstacle_max,
          'origin_cell': origin_cell,
          'free_cells':int((image==254).sum()),
          'occupied_cells':int((image==0).sum()),'unknown_cells':int((image==205).sum())}
    if floor_fit is not None:
        meta.update(floor_model=floor_model, floor_fit=floor_fit)
    (out/'metadata.json').write_text(json.dumps(meta,indent=2))
    print(json.dumps(meta,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('pcd',type=Path);p.add_argument('--out',type=Path,default=Path('sim/generated/mapping'))
    p.add_argument('--resolution',type=float,default=.1)
    p.add_argument('--voxel', type=float, default=0., help='3D PLY voxel size in metres, independent of 2D projection')
    p.add_argument('--known-free-radius',type=float,default=1.5,
                   help='Verified clear radius around initial camera_init origin; obstacles still take precedence')
    p.add_argument('--floor-band', type=float, default=.15, help='Ground band half-width in metres')
    p.add_argument('--obstacle-min', type=float, default=.2, help='Minimum obstacle height above estimated ground')
    p.add_argument('--obstacle-max', type=float, default=1.8, help='Maximum obstacle height above estimated ground')
    p.add_argument('--max-range', type=float, help='Limit 2D projection to this radius in metres around the mapping start')
    p.add_argument('--floor-model', choices=('percentile', 'plane'), default='percentile',
                   help='Use plane for a tilted, predominantly planar indoor floor')
    args=p.parse_args()
    try:
        export(args.pcd, args.out, args.resolution, args.known_free_radius,
               floor_band=args.floor_band, obstacle_min=args.obstacle_min,
               obstacle_max=args.obstacle_max, max_range=args.max_range,
               floor_model=args.floor_model, voxel_size=args.voxel)
    except (ValueError, OSError, KeyError) as error:
        p.error(str(error))
if __name__=='__main__':main()
