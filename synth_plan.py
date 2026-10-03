# Synthetic apartment: room A 4.0x3.0, room B 3.0x3.0, 0.9 m door, 0.1 m walls, 2.5 m ceiling, a bed in A
import numpy as np
from pathlib import Path
from floorplan import plan_from_points, rot_xz
rng=np.random.default_rng(0)
def wall(x0,z0,x1,z1,h0=0,h1=2.5,n=60000):
    t=rng.uniform(0,1,n); y=rng.uniform(h0,h1,n)
    return np.stack([x0+(x1-x0)*t,y,z0+(z1-z0)*t],1)
def plane(x0,x1,z0,z1,y,n=150000):
    return np.stack([rng.uniform(x0,x1,n),np.full(n,y),rng.uniform(z0,z1,n)],1)
P=[]
# room A interior x 0..4, z 0..3 ; room B x 4.1..7.1, z 0..3
for (x0,x1) in [(0,4),(4.1,7.1)]:
    P+= [wall(x0,0,x1,0), wall(x0,3,x1,3), plane(x0,x1,0,3,0), plane(x0,x1,0,3,2.5)]
P+= [wall(0,0,0,3), wall(7.1,0,7.1,3)]
# shared wall faces with door z 1.0..1.9
for x in (4.0,4.1):
    P+= [wall(x,0,x,1.0), wall(x,1.9,x,3), wall(x,1.0,x,1.9,2.05,2.5,8000)]
# bed (should not become a wall)
P+= [plane(0.5,2.5,0.5,2.0,0.6,40000)]
pts=np.concatenate(P)+rng.normal(0,0.008,(sum(len(p) for p in P),3))
pts[:,1]-=1.45                      # phone at y=0, floor 1.45 below
pts=rot_xz(pts,np.deg2rad(23))       # not axis-aligned
cam_y=np.array([-0.1,0.1])
r=plan_from_points(pts.astype(np.float32),cam_y,Path("out/synth_plan"))
import json; print(json.dumps(r,indent=1)[:1500])
