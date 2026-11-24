import json
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree
from pathlib import Path

Path('alignment.json').unlink(missing_ok=True)
raw=open('clean.pcd','rb').read().split(b'DATA binary\n',1)[1]
target=np.frombuffer(raw,'<f4').reshape(-1,3).astype(float)
target[:,2]=target@np.array([.078349821,.006797867,.996902764])+1.347228408
scans=np.load('scans.npy')
def vox(p,r):
 return p[np.unique(np.floor(p/r).astype(int),axis=0,return_index=True)[1]]
def scan(rows):
 p=rows[:,1:]
 return vox(p[(p[:,2]>.32)&(p[:,2]<2.2)&(np.linalg.norm(p[:,:2],axis=1)<8)],.12)
p=scan(scans)
t=vox(target[(target[:,2]>.27)&(target[:,2]<2.35)],.08)
print('points',len(p),len(t),flush=True)
tree=cKDTree(t)
step=.15
low=np.floor(t[:,:2].min(axis=0)/step).astype(int)-8
high=np.ceil(t[:,:2].max(axis=0)/step).astype(int)+8
grid=np.zeros(high-low+1,bool)
grid[tuple((np.rint(t[:,:2]/step).astype(int)-low).T)]=True
dist=distance_transform_edt(~grid)*step
field=np.exp(-.5*(dist/.20)**2)
def rot(a):
 c,s=np.cos(a),np.sin(a)
 return np.array([[c,-s],[s,c]])
cand=[]
for ang in np.deg2rad(np.arange(-180,180,4)):
 q=np.unique(np.rint((p[:,:2]@rot(ang).T)/step).astype(int),axis=0)
 radius=int(np.max(np.abs(q)))
 kernel=np.zeros((radius*2+1,)*2);kernel[tuple((q+radius).T)]=1
 corr=fftconvolve(field,kernel[::-1,::-1],mode='same')/len(q)
 for _ in range(3):
  i,j=np.unravel_index(np.argmax(corr),corr.shape)
  cand.append((corr[i,j],ang,step*(low+[i,j])))
  corr[max(0,i-5):i+6,max(0,j-5):j+6]=0
def refine(p,a,txy):
 txy=txy.copy()
 for _ in range(24):
  q=p.copy();q[:,:2]=q[:,:2]@rot(a).T+txy
  d,idx=tree.query(q)
  good=d<.36
  if sum(good)<25:break
  x,y=q[good,:2],t[idx[good],:2]
  cx,cy=x.mean(0),y.mean(0)
  x,y=x-cx,y-cy
  da=np.arctan2(np.sum(x[:,0]*y[:,1]-x[:,1]*y[:,0]),np.sum(x*y))
  txy=rot(da)@(txy-cx)+cy;a+=da
 q=p.copy();q[:,:2]=q[:,:2]@rot(a).T+txy
 d,_=tree.query(q)
 return np.mean(np.exp(-.5*(d/.17)**2)),a,txy,np.mean(d<.20),np.sqrt(np.mean(d[d<.20]**2))
seeds=[]
for _,a,xy in sorted(cand,key=lambda c:-c[0]):
 if any(abs(np.angle(np.exp(1j*(a-b))))<.16 and np.linalg.norm(xy-y)<.65 for b,y in seeds):continue
 seeds.append((a,xy))
 if len(seeds)==20:break
fits=sorted((refine(p,a,xy) for a,xy in seeds),key=lambda x:-x[0])
unique=[]
for f in fits:
 score,a,xy,overlap,rms=f
 if any(abs(np.angle(np.exp(1j*(a-b))))<.20 and np.linalg.norm(xy-y)<.5 for _,b,y,_,_ in unique):continue
 unique.append(f)
 print('candidate',round(score,3),np.round(xy,3),round(np.rad2deg(a),2),round(overlap,3),round(rms,3))
batch_fits=[]
for batch in np.array_split(np.unique(scans[:,0]),3):
 results=[refine(scan(scans[np.isin(scans[:,0],batch)]),a,xy) for _,a,xy,_,_ in unique[:4]]
 print('batch',[(round(s,3),np.round(xy,2).tolist(),round(np.rad2deg(a),1),round(o,3)) for s,a,xy,o,_ in results])

 batch_fits.append(results[0])

score, angle, xy, overlap, rms = unique[0]
assert len(unique) > 1 and score - unique[1][0] >= .15, 'Ambiguous pose'
assert overlap >= .85 and rms <= .10, 'Insufficient fit'
assert all(o >= .85 and np.linalg.norm(pos - xy) <= .20
           and abs(np.angle(np.exp(1j * (heading - angle)))) <= np.deg2rad(5)
           for _, heading, pos, o, _ in batch_fits), 'Inconsistent scan batches'
z, pitch, roll = map(float, Path('floor.txt').read_text().split()[5:8])
Path('alignment.json').write_text(json.dumps({
 'x': float(xy[0]), 'y': float(xy[1]), 'z': z,
 'yaw': float(angle), 'pitch': pitch, 'roll': roll,
 'overlap_20cm': float(overlap), 'inlier_rms_m': float(rms),
}, indent=2) + '\n')
