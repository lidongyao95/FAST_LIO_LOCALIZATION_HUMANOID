import numpy as np
from scipy.spatial.transform import Rotation
rows=np.load('raw.npy'); a=rows[:,1:][::6]
r=np.linalg.norm(a[:,:2],axis=1); a=a[(r>.8)&(r<6)&(a[:,2]<-.3)]
rng=np.random.default_rng(23);best=(0,None,None)
for _ in range(400):
 p,q,r=a[rng.choice(len(a),3,replace=False)];v=np.cross(q-p,r-p); norm=np.linalg.norm(v)
 if norm<1e-5:continue
 v/=norm
 if abs(v[2])<.9:continue
 if v[2]<0:v=-v
 d=float(v@p); count=np.sum(np.abs(a@v-d)<.04)
 if count>best[0]:best=(count,v,d)
count,v,d=best
assert count>len(a)*.25
p=a[np.abs(a@v-d)<.04];c=p.mean(0);_,_,vt=np.linalg.svd(p-c,full_matrices=False);v=vt[-1];v*=np.sign(v[2]);d=float(v@c)
roll=np.arctan2(v[1],v[2]);pitch=-np.arcsin(v[0])
rows[:,1:]=Rotation.from_euler('xyz',[roll,pitch,0]).apply(rows[:,1:])+[0,0,-d]
np.save('scans.npy',rows)
print('floor z pitch roll inliers',-d,pitch,roll,len(p),flush=True)
