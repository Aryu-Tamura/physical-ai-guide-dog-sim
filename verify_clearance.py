"""Known-map path planning: robot clearance versus robot+person clearance.

The low doorway is passable by Go2 but intersects the kinematic person's torso.
Robot motion is torque-driven; the person follows recorded robot path arclength.
Person/environment clearance is checked geometrically, not as a force response.
"""
import argparse
import bisect
import heapq
import importlib.util
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

BOXES=[('left_post',[1.5,-.65,.625],[.12,.10,.625]),
       ('right_post',[1.5,.65,.625],[.12,.10,.625]),
       ('low_lintel',[1.5,0,1.25],[.12,.75,.12])]
PERSON_RADIUS=.22

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def segment_blocked(a,b,rect):
    lo=np.array(rect[:2]);hi=np.array(rect[2:]);a=np.asarray(a);v=np.asarray(b)-a
    t0,t1=0.,1.
    for k in range(2):
        if abs(v[k])<1e-12:
            if a[k]<lo[k] or a[k]>hi[k]:return False
        else:
            s,t=sorted(((lo[k]-a[k])/v[k],(hi[k]-a[k])/v[k]));t0=max(t0,s);t1=min(t1,t)
            if t0>t1:return False
    return True

def plan(mode,margin):
    # Same map, same footprint margin, different required vertical clearance.
    height=.6 if mode=='robot_only' else 1.8
    rects=[]
    for _,center,half in BOXES:
        c=np.array(center);h=np.array(half)
        if c[2]-h[2]<=height:
            rects.append([*(c[:2]-h[:2]-margin),*(c[:2]+h[:2]+margin)])
    start=(0,0);goal=(38,0)
    point=lambda q:np.array(q,dtype=float)*.1
    clear=lambda a,b:not any(segment_blocked(a,b,r) for r in rects)
    queue=[(0.,0.,start)];cost={start:0.};prev={}
    while queue:
        _,g,u=heapq.heappop(queue)
        if g!=cost[u]:continue
        if u==goal:break
        for dx,dy in [(1,0),(0,1),(0,-1),(-1,0)]:
            v=(u[0]+dx,u[1]+dy)
            if not (-5<=v[0]<=45 and -17<=v[1]<=17):continue
            if not clear(point(u),point(v)):continue
            ng=g+.1
            if ng<cost.get(v,float('inf')):
                cost[v]=ng;prev[v]=u
                heapq.heappush(queue,(ng+float(np.linalg.norm(point(v)-point(goal))),ng,v))
    if goal not in cost:raise RuntimeError('No route in known map')
    nodes=[goal]
    while nodes[-1]!=start:nodes.append(prev[nodes[-1]])
    nodes=[point(q) for q in nodes[::-1]]
    smooth=[nodes[0]];i=0
    while i<len(nodes)-1:
        j=len(nodes)-1
        while j>i+1 and not clear(nodes[i],nodes[j]):j-=1
        smooth.append(nodes[j]);i=j
    return smooth,rects

def scene(project):
    root=ET.parse(project/'models/go2/go2.xml').getroot()
    root.find('compiler').set('meshdir',str(project/'models/go2/assets'))
    world=root.find('worldbody')
    ET.SubElement(world,'light',pos='1 0 5',dir='0 0 -1',directional='true')
    assets=root.find('asset')
    ET.SubElement(assets,'texture',name='floor_texture',type='2d',builtin='checker',rgb1='.25 .30 .34',rgb2='.18 .23 .27',width='256',height='256')
    ET.SubElement(assets,'material',name='floor_mat',texture='floor_texture',texrepeat='8 8',texuniform='true')
    ET.SubElement(world,'geom',name='floor',type='plane',size='0 0 .1',material='floor_mat')
    for name,center,half in BOXES:
        ET.SubElement(world,'geom',name=name,type='box',pos=' '.join(map(str,center)),size=' '.join(map(str,half)),rgba='.78 .35 .20 .90')
    # Non-colliding visual marker of the destination region.
    ET.SubElement(world,'geom',name='destination',type='box',pos='3.5 0 .006',size='1 .85 .005',rgba='.18 .55 .28 .45',contype='0',conaffinity='0')
    body=ET.SubElement(world,'body',name='person_proxy',mocap='true',pos='-.95 0 0')
    ET.SubElement(body,'geom',name='person_torso',type='capsule',size='.22 .55',pos='0 0 .77',rgba='.10 .65 .75 1')
    ET.SubElement(body,'geom',name='person_head',type='sphere',size='.15',pos='0 0 1.65',rgba='.75 .80 .82 1')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));m.opt.timestep=.002
    return m

def person_clearance(xy):
    # Exact distance between a vertical capsule centre segment and an AABB,
    # and between the head sphere centre and the AABB, minus their radii.
    best=(float('inf'),None)
    for name,center,half in BOXES:
        c=np.array(center);h=np.array(half);lo=c-h;hi=c+h
        gap_xy=np.maximum(np.maximum(lo[:2]-xy,xy-hi[:2]),0.)
        gap_z=max(lo[2]-1.32,.22-hi[2],0.)
        torso=float(np.sqrt(np.dot(gap_xy,gap_xy)+gap_z*gap_z)-.22)
        head_gaps=np.maximum(np.maximum(lo-np.r_[xy,1.65],np.r_[xy,1.65]-hi),0.)
        head=float(np.linalg.norm(head_gaps)-.15)
        clearance=min(torso,head)
        if clearance<best[0]:best=(clearance,name)
    return best

def along(points,lengths,s):
    i=max(0,min(bisect.bisect_right(lengths,s)-1,len(points)-2))
    size=lengths[i+1]-lengths[i]
    a=0 if size<1e-12 else np.clip((s-lengths[i])/size,0,1)
    return (1-a)*points[i]+a*points[i+1]

def run(args,seed):
    walk=module('walk',args.project/'verify_go2.py');nav=module('nav',args.project/'verify_navigation.py')
    route,rects=plan(args.mode,args.margin)
    m=scene(args.project);d=mujoco.MjData(m)
    ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in walk.JOINTS]
    qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
    amap={int(m.actuator_trnid[i,0]):i for i in range(m.nu)};act=np.array([amap[i] for i in ids])
    rng=np.random.default_rng(seed);yaw0=0 if seed==0 else rng.uniform(-.08,.08)
    d.qpos[:7]=[0,0,.34,np.cos(yaw0/2),0,0,np.sin(yaw0/2)]
    d.qpos[qa]=walk.DEFAULT+(0 if seed==0 else rng.normal(0,.015,12))
    person=np.array([-.95,0. if seed==0 else rng.uniform(-.02,.02)])
    d.mocap_pos[0]=np.r_[person,0.];mujoco.mj_forward(m,d)
    points=[person.copy(),np.zeros(2)];lengths=[0.,float(np.linalg.norm(person))]
    robot_filtered=np.zeros(2);person_s=0.;person_speed=0.
    policy=walk.Policy(.25,.4);feedback=nav.Feedback();target=walk.DEFAULT.copy()
    index=1;goal=route[index].copy();heading=float(np.arctan2(*(goal-route[0])[::-1]));state='settle';stable=0.;finish_start=None
    samples=[];events=[];fallen=False;human_hit=None;robot_hit=None;min_clearance=float('inf');max_gap=0.
    obstacle_ids={mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n,_,_ in BOXES}
    human_ids={mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ['person_torso','person_head']}
    robot_ids={i for i in range(m.ngeom) if m.geom_bodyid[i]>0 and i not in human_ids}
    robot_contact_steps=0;human_robot_contact_steps=0
    writer=None;renderer=None;frames=0
    name=f'{args.output}_{args.mode}_seed{seed}';out=args.project/'outputs';out.mkdir(exist_ok=True)
    if args.render:
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw
        renderer=mujoco.Renderer(m,height=480,width=640);writer=imageio.get_writer(str(out/f'{name}.mp4'),fps=25)
        cam=mujoco.MjvCamera();cam.lookat[:]=[1.7,-.2,.65];cam.distance=6.4;cam.azimuth=115;cam.elevation=-35
    start=time.perf_counter()
    try:
        for step in range(round(args.seconds/.002)):
            xy,yaw,upright=nav.pose(d)
            if step%10==0:
                robot_filtered=.82*robot_filtered+.18*xy
                segment=float(np.linalg.norm(robot_filtered-points[-1]))
                if segment>.02:points.append(robot_filtered.copy());lengths.append(lengths[-1]+segment)
                desired_s=max(0.,lengths[-1]-.9)
                desired_speed=float(np.clip(1.4*(desired_s-person_s),0,.22)) if d.time>=2 else 0.
                person_speed+=float(np.clip(desired_speed-person_speed,-.01,.01))
                person_s=min(desired_s,person_s+person_speed*.02)
                person=along(points,lengths,person_s)
                gap=float(np.linalg.norm(xy-person));max_gap=max(max_gap,gap)
                old=state
                if state=='settle' and d.time>=2:state='navigate'
                if state=='navigate':
                    err=np.linalg.norm(xy-goal)
                    stable=stable+.02 if err<.065 else 0.
                    if stable>.4:
                        if index<len(route)-1:
                            index+=1;goal=route[index].copy()
                            heading=float(np.arctan2(*(goal-route[index-1])[::-1]));stable=0.
                            events.append({'t':float(d.time),'event':'next_waypoint','index':index,'goal':goal.tolist()})
                        else:state='finish';finish_start=float(d.time)
                if old!=state:events.append({'t':float(d.time),'event':state})
                cmd=feedback.step(d,goal,heading,settle=state=='settle')
                if state=='settle':cmd=np.zeros(3)
                target=policy.step(m,d,cmd,qa,va)
            d.mocap_pos[0]=np.r_[person,0.]
            d.ctrl[act]=20*(target-d.qpos[qa])-.5*d.qvel[va]
            mujoco.mj_step(m,d)
            if not np.isfinite(d.qpos).all():raise RuntimeError('Non-finite state')
            clearance,obstacle=person_clearance(person);min_clearance=min(min_clearance,clearance)
            if clearance<0:human_hit={'t':float(d.time),'obstacle':obstacle,'clearance_m':clearance,'person_xy':person.tolist(),'robot_xy':d.qpos[:2].tolist()}
            for c in d.contact:
                pair={int(c.geom1),int(c.geom2)}
                if pair&obstacle_ids and pair&robot_ids:
                    robot_contact_steps+=1;robot_hit={'t':float(d.time),'geom1':int(c.geom1),'geom2':int(c.geom2)}
                if pair&human_ids and pair&robot_ids:human_robot_contact_steps+=1
            xy,yaw,upright=nav.pose(d)
            if upright<.5 or d.qpos[2]<.15:fallen=True
            terminal=human_hit is not None or fallen
            if step%20==0 or terminal:
                samples.append({'t':float(d.time),'state':state,'robot_x':float(xy[0]),'robot_y':float(xy[1]),'yaw':yaw,'person_x':float(person[0]),'person_y':float(person[1]),'gap_m':float(np.linalg.norm(xy-person)),'person_clearance_m':clearance,'upright':upright,'waypoint_index':index,'goal_error_m':float(np.linalg.norm(xy-route[-1]))})
                if writer:
                    renderer.update_scene(d,camera=cam);sc=renderer.scene
                    for a,b in zip(route[:-1],route[1:]):
                        mujoco.mjv_initGeom(sc.geoms[sc.ngeom],mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.85,.85,.15,1.]));mujoco.mjv_connector(sc.geoms[sc.ngeom],mujoco.mjtGeom.mjGEOM_CAPSULE,.012,np.r_[a,.02],np.r_[b,.02]);sc.ngeom+=1
                    frame=renderer.render();im=Image.fromarray(frame);draw=ImageDraw.Draw(im)
                    draw.rectangle((0,0,640,68),fill=(20,25,30))
                    status='PERSON COLLISION' if human_hit else state.upper()
                    draw.text((12,8),f'CLEARANCE CHECK | {args.mode.upper()} | {status} | {d.time:.1f}s',fill='white')
                    draw.text((12,28),f'Person clearance: {clearance*100:.1f} cm | Same doorway, different height constraint',fill='white')
                    draw.text((12,47),'Known map + ideal pose | Kinematic person follows recorded robot path',fill=(190,200,210))
                    frame=np.asarray(im);writer.append_data(frame);frames+=1
                    if terminal or state=='finish':imageio.imwrite(out/f'{name}.png',frame)
                    if terminal:
                        for _ in range(50):writer.append_data(frame)
            if terminal:break
            if finish_start is not None and d.time-finish_start>=5:break
    finally:
        if writer:writer.close()
        if renderer:renderer.close()
    # Both agent centres, including the person's circular extent, must be in
    # the broad marked destination region; this is not point arrival for a person.
    person_in_zone=bool(person[0]-.22>=2.5 and person[0]+.22<=4.5 and abs(person[1])+.22<=.85)
    robot_arrived=bool(np.linalg.norm(d.qpos[:2]-route[-1])<.08)
    result={'mode':args.mode,'seed':seed,'route':[p.tolist() for p in route],'planner_inflated_rectangles':rects,'planner_grid_m':.1,'planner_margin_m':args.margin,'planner_required_height_m':.6 if args.mode=='robot_only' else 1.8,'person_model':'kinematic recorded-path follower','physical_leash':False,'uses_ground_truth_pose':True,'person_collision_detector':'vertical capsule / sphere versus axis-aligned box geometry, not contact force','person_collision':human_hit,'robot_obstacle_contact_steps':robot_contact_steps,'human_robot_contact_steps':human_robot_contact_steps,'fallen':fallen,'min_person_clearance_m':min_clearance,'max_agent_center_gap_m':max_gap,'robot_arrived':robot_arrived,'person_in_destination_zone':person_in_zone,'final_person_xy':person.tolist(),'final_robot_xy':d.qpos[:2].tolist(),'simulation_seconds':float(d.time),'wall_seconds':time.perf_counter()-start,'frames':frames,'events':events,'samples':samples}
    result['passed']=bool(robot_arrived and person_in_zone and human_hit is None and not fallen and robot_contact_steps==0 and human_robot_contact_steps==0)
    (out/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,default=Path(__file__).resolve().parent);p.add_argument('--mode',choices=['robot_only','person_aware'],default='person_aware');p.add_argument('--seeds',type=int,nargs='+',default=[0]);p.add_argument('--seconds',type=float,default=180);p.add_argument('--output',default='clearance');p.add_argument('--render',action='store_true');p.add_argument('--margin',type=float,default=.50);a=p.parse_args()
    for seed in a.seeds:run(a,seed)
