"""Integrated guide experiment: height-aware planning and observed-person waiting.

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

from verify_clearance import BOXES, module, plan, scene, person_clearance, along
from guide_supervisor import GuideSupervisor

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
    supervisor=GuideSupervisor(); measured_speed=0.; previous_person=person.copy()
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
                paused=args.pause_start<=d.time<args.pause_start+args.pause_duration
                if paused:desired_speed=0.
                person_speed+=float(np.clip(desired_speed-person_speed,-.01,.01))
                person_s=min(desired_s,person_s+person_speed*.02)
                person=along(points,lengths,person_s)
                observed_speed=float(np.linalg.norm(person-previous_person))/.02
                measured_speed=np.exp(-.02/.25)*measured_speed+(1-np.exp(-.02/.25))*observed_speed
                previous_person=person.copy()
                gap=float(np.linalg.norm(xy-person));max_gap=max(max_gap,gap)
                old=state
                if state=='settle' and d.time>=2:state='navigate'
                if state=='navigate' and not supervisor.waiting:
                    err=np.linalg.norm(xy-goal)
                    stable=stable+.02 if err<.065 else 0.
                    if stable>.4:
                        if index<len(route)-1:
                            index+=1;goal=route[index].copy()
                            heading=float(np.arctan2(*(goal-route[index-1])[::-1]));stable=0.
                            events.append({'t':float(d.time),'event':'next_waypoint','index':index,'goal':goal.tolist()})
                        else:state='finish';finish_start=float(d.time)
                if old!=state:events.append({'t':float(d.time),'event':state})
                # Pause schedule is deliberately NOT an input to the supervisor.
                if args.waiting=='enabled' and state=='navigate':
                    transition=supervisor.update(xy,yaw,measured_speed,gap,.02)
                    if transition:events.append({'t':float(d.time),'event':transition,'gap_m':gap,'measured_person_speed_m_s':measured_speed})
                active_goal=supervisor.hold_xy if supervisor.waiting else goal
                active_heading=supervisor.hold_yaw if supervisor.waiting else heading
                cmd=feedback.step(d,active_goal,active_heading,settle=state=='settle')
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
                samples.append({'t':float(d.time),'state':'wait' if supervisor.waiting else state,'person_pause_schedule':paused,'measured_person_speed_m_s':measured_speed,'robot_x':float(xy[0]),'robot_y':float(xy[1]),'yaw':yaw,'person_x':float(person[0]),'person_y':float(person[1]),'gap_m':float(np.linalg.norm(xy-person)),'person_clearance_m':clearance,'upright':upright,'waypoint_index':index,'goal_error_m':float(np.linalg.norm(xy-route[-1]))})
                if writer:
                    renderer.update_scene(d,camera=cam);sc=renderer.scene
                    for a,b in zip(route[:-1],route[1:]):
                        mujoco.mjv_initGeom(sc.geoms[sc.ngeom],mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.85,.85,.15,1.]));mujoco.mjv_connector(sc.geoms[sc.ngeom],mujoco.mjtGeom.mjGEOM_CAPSULE,.012,np.r_[a,.02],np.r_[b,.02]);sc.ngeom+=1
                    frame=renderer.render();im=Image.fromarray(frame);draw=ImageDraw.Draw(im)
                    draw.rectangle((0,0,640,68),fill=(20,25,30))
                    status='PERSON COLLISION' if human_hit else ('WAIT' if supervisor.waiting else state.upper())
                    draw.text((12,8),f'GUIDE | waiting={args.waiting} | {args.mode.upper()} | {status} | {d.time:.1f}s',fill='white')
                    draw.text((12,28),f'Person clearance: {clearance*100:.1f} cm | Same doorway, different height constraint',fill='white')
                    draw.text((12,47),'Ideal pose | Kinematic person | Pause: '+str(args.pause_start)+'-'+str(args.pause_start+args.pause_duration)+'s',fill=(190,200,210))
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
    result={'waiting':args.waiting,'pause_start_s':args.pause_start,'pause_duration_s':args.pause_duration,'controller_reads_pause_schedule':False,'mode':args.mode,'seed':seed,'route':[p.tolist() for p in route],'planner_inflated_rectangles':rects,'planner_grid_m':.1,'planner_margin_m':args.margin,'planner_required_height_m':.6 if args.mode=='robot_only' else 1.8,'person_model':'kinematic recorded-path follower','physical_leash':False,'uses_ground_truth_pose':True,'person_collision_detector':'vertical capsule / sphere versus axis-aligned box geometry, not contact force','person_collision':human_hit,'robot_obstacle_contact_steps':robot_contact_steps,'human_robot_contact_steps':human_robot_contact_steps,'fallen':fallen,'min_person_clearance_m':min_clearance,'max_agent_center_gap_m':max_gap,'robot_arrived':robot_arrived,'person_in_destination_zone':person_in_zone,'final_person_xy':person.tolist(),'final_robot_xy':d.qpos[:2].tolist(),'simulation_seconds':float(d.time),'wall_seconds':time.perf_counter()-start,'frames':frames,'events':events,'samples':samples}
    result['passed']=bool(robot_arrived and person_in_zone and human_hit is None and not fallen and robot_contact_steps==0 and human_robot_contact_steps==0)
    wait_samples=[r for r in samples if r['state']=='wait' and r['person_pause_schedule']]
    result['wait_motion_during_pause_m']=None
    if wait_samples:
        origin=np.array([wait_samples[0]['robot_x'],wait_samples[0]['robot_y']])
        result['wait_motion_during_pause_m']=max(float(np.linalg.norm(np.array([r['robot_x'],r['robot_y']])-origin)) for r in wait_samples)
    result['wait_count']=sum(e['event']=='wait' for e in events)
    result['resume_count']=sum(e['event']=='resume' for e in events)
    result['mission_passed']=result['passed']
    result['resumed_after_pause']=any(e['event']=='resume' and e['t']>=args.pause_start+args.pause_duration for e in events)
    result['guidance_passed']=bool(result['passed'] and max_gap<1.45 and ((args.pause_duration==0 and result['wait_count']==0) or (bool(wait_samples) and result['resumed_after_pause'] and result['wait_motion_during_pause_m']<.10)))
    (out/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,default=Path(__file__).resolve().parent);p.add_argument('--mode',choices=['robot_only','person_aware'],default='person_aware');p.add_argument('--seeds',type=int,nargs='+',default=[0]);p.add_argument('--seconds',type=float,default=180);p.add_argument('--output',default='clearance');p.add_argument('--render',action='store_true');p.add_argument('--margin',type=float,default=.50);p.add_argument('--waiting',choices=['enabled','disabled'],default='enabled');p.add_argument('--pause-start',type=float,default=8.);p.add_argument('--pause-duration',type=float,default=6.);a=p.parse_args()
    for seed in a.seeds:run(a,seed)
