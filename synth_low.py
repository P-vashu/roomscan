import numpy as np, json
from pathlib import Path
from floorplan import plan_from_points, rot_xz
rng=np.random.default_rng(1)
def wall(x0,z0,x1,z1,h0=0,h1=1.3,n=40000):
    t=rng.uniform(0,1,n); y=rng.uniform(h0,h1,n); return np.stack([x0+(x1-x0)*t,y,z0+(z1-z0)*t],1)
def plane(x0,x1,z0,z1,y,n=150000):
    return np.stack([rng.uniform(x0,x1,n),np.full(n,y),rng.uniform(z0,z1,n)],1)
P=[]
for (x0,x1) in [(0,4),(4.1,7.1)]:
    P+=[wall(x0,0,x1,0),wall(x0,3,x1,3),plane(x0,x1,0,3,0)]
P+=[wall(0,0,0,3),wall(7.1,0,7.1,3)]
for x in (4.0,4.1): P+=[wall(x,0,x,1.0),wall(x,1.9,x,3)]
# sofa against wall, 0-0.9 tall, and bed
for y in np.linspace(0,0.9,10): P+=[wall(5,2.6,6.5,2.6,y,y+0.02,3000)]
P+=[plane(0.5,2.5,0.5,2.0,0.6,40000)]
pts=np.concatenate(P); pts+=rng.normal(0,0.008,pts.shape)
pts[:,1]+=0.06*pts[:,0]/7.1   # 6 cm vertical drift across apartment
pts[:,1]-=1.45; pts=rot_xz(pts,np.deg2rad(-12))
r=plan_from_points(pts.astype(np.float32),np.array([-0.1,0.1]),Path("out/synth_low"))
print(r["floor_flatness"]); [print(x["id"],x["floor_area_m2"],x["bbox_m"]) for x in r["rooms"]]; print(r["adjacency"])
