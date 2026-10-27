"""Independent sampled-sphere reference for the capsule/AABB clearance check."""
import importlib.util
from pathlib import Path
import numpy as np

p=Path(__file__).resolve().parent/'verify_clearance.py'
s=importlib.util.spec_from_file_location('clearance',p);c=importlib.util.module_from_spec(s);s.loader.exec_module(c)
rng=np.random.default_rng(123)
for xy in [np.array([1.5,0.]),np.array([1.16,0.]),np.array([1.5,-1.3]),*rng.uniform([0,-1.6],[3,1.6],size=(30,2))]:
    reference=float('inf')
    for _,center,half in c.BOXES:
        lo=np.array(center)-half;hi=np.array(center)+half
        axis=np.column_stack([np.full(2001,xy[0]),np.full(2001,xy[1]),np.linspace(.22,1.32,2001)])
        distances=np.linalg.norm(np.maximum(np.maximum(lo-axis,axis-hi),0),axis=1)-.22
        head=np.r_[xy,1.65]
        head_distance=np.linalg.norm(np.maximum(np.maximum(lo-head,head-hi),0))-.15
        reference=min(reference,float(distances.min()),float(head_distance))
    exact,_=c.person_clearance(xy)
    assert abs(exact-reference)<.00056,(xy,exact,reference)
for mode in ['robot_only','person_aware']:
    route,rects=c.plan(mode,.5)
    assert all(not c.segment_blocked(a,b,r) for a,b in zip(route[:-1],route[1:]) for r in rects)
assert c.person_clearance(np.array([1.5,0.]))[0]<0
assert c.person_clearance(np.array([1.5,-1.3]))[0]>0
print('Passed: 33 capsule/sphere clearance cases against dense independent reference; both planned paths collision-free in their respective projected maps.')
