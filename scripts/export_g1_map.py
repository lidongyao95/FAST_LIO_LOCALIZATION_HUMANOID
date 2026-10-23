#!/usr/bin/env python3
"""Convert FAST-LIO binary PCD into paired Open3D and Nav2 map assets."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

TYPE={'F':{4:'<f4',8:'<f8'},'I':{1:'i1',2:'<i2',4:'<i4'},'U':{1:'u1',2:'<u2',4:'<u4'}}

def read_pcd(path):
    with path.open('rb') as f:
        header={}
        while True:
            line=f.readline()
            if not line:raise ValueError('PCD header ended before DATA')
            if line.startswith(b'#') or not line.strip():continue
            parts=line.decode('ascii').strip().split()
            header[parts[0].upper()]=parts[1:]
            if parts[0].upper()=='DATA':break
        fields=header['FIELDS'];sizes=list(map(int,header['SIZE']))
        types=header['TYPE'];counts=list(map(int,header.get('COUNT',['1']*len(fields))))
        if header['DATA'][0].lower()!='binary':raise ValueError('Only uncompressed binary PCD is supported')
        formats=[];offsets=[];offset=0
        for field,size,kind,count in zip(fields,sizes,types,counts):
            formats.append((TYPE[kind][size],(count,)) if count!=1 else TYPE[kind][size])
            offsets.append(offset);offset+=size*count
        dtype=np.dtype({'names':fields,'formats':formats,'offsets':offsets,'itemsize':offset})
        n=int(header.get('POINTS',[header['WIDTH'][0]])[0])
        points=np.frombuffer(f.read(n*offset),dtype=dtype,count=n)
        xyz=np.column_stack([points[k].astype(np.float32) for k in ('x','y','z')])
        return xyz[np.isfinite(xyz).all(axis=1)]

def export(source,out,resolution,known_free_radius):
    xyz=read_pcd(source)
    if len(xyz)<100:raise ValueError('Map has too few finite points')
    out.mkdir(parents=True,exist_ok=True)
    ply=out/'map.ply'
    with ply.open('w') as f:
        f.write(f'ply\nformat ascii 1.0\nelement vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(f,xyz,fmt='%.4f')
    floor_z=float(np.percentile(xyz[:,2],5))
    ox=float(np.floor((xyz[:,0].min()-.5)/resolution)*resolution)
    oy=float(np.floor((xyz[:,1].min()-.5)/resolution)*resolution)
    width=int(np.ceil((xyz[:,0].max()+.5-ox)/resolution))
    height=int(np.ceil((xyz[:,1].max()+.5-oy)/resolution))
    if width*height>8_000_000:raise ValueError('Projected occupancy map is too large')
    ix=np.floor((xyz[:,0]-ox)/resolution).astype(int)
    iy=np.floor((xyz[:,1]-oy)/resolution).astype(int)
    floor=np.zeros((height,width),np.uint8)
    obstacle=np.zeros_like(floor)
    floor_band=np.abs(xyz[:,2]-floor_z)<.15
    obstacle_band=(xyz[:,2]>floor_z+.2)&(xyz[:,2]<floor_z+1.8)
    floor[iy[floor_band],ix[floor_band]]=1
    obstacle[iy[obstacle_band],ix[obstacle_band]]=1
    cover=cv2.dilate(floor,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(int(.8/resolution)//2*2+1,)*2))
    if known_free_radius>0:
        cx=int((0.-ox)/resolution);cy=int((0.-oy)/resolution)
        cv2.circle(cover,(cx,cy),round(known_free_radius/resolution),1,-1)
    occupied=cv2.dilate(obstacle,np.ones((3,3),np.uint8))
    image=np.full((height,width),205,np.uint8)
    image[cover>0]=254
    image[occupied>0]=0
    with (out/'map.pgm').open('wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode());f.write(image[::-1].tobytes())
    (out/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {resolution}\n'
                                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    meta={'source_pcd':str(source.resolve()),'frame':'camera_init','map_from_camera_init':[0,0,0,0,0,0],
          'points':len(xyz),'floor_z':floor_z,'resolution':resolution,'origin':[ox,oy,0],
          'size':[width,height],'known_free_radius':known_free_radius,
          'free_cells':int((image==254).sum()),
          'occupied_cells':int((image==0).sum()),'unknown_cells':int((image==205).sum())}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2))
    print(json.dumps(meta,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('pcd',type=Path);p.add_argument('--out',type=Path,default=Path('sim/generated/mapping'))
    p.add_argument('--resolution',type=float,default=.1)
    p.add_argument('--known-free-radius',type=float,default=1.5,
                   help='Verified clear radius around initial camera_init origin; obstacles still take precedence')
    args=p.parse_args()
    export(args.pcd,args.out,args.resolution,args.known_free_radius)
if __name__=='__main__':main()
