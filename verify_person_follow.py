"""Straight-line guide test with a kinematic human proxy and learned Go2 walking.

Robot: torque-driven physics, ground-truth pose feedback.
Person: externally driven mocap body, bounded-speed delayed following model.
There is no physical leash, human balance model, or learned human behavior.
"""
import argparse
from collections import deque
import importlib.util
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod

def make_model(project):
    root=ET.parse(project/'models/go2/go2.xml').getroot()
    root.find('compiler').set('meshdir',str(project/'models/go2/assets'))
    world=root.find('worldbody')
    ET.SubElement(world,'light',pos='1 0 4',dir='0 0 -1',directional='true')
    assets=root.find('asset')
    ET.SubElement(assets,'texture',name='floor_texture',type='2d',builtin='checker',rgb1='.23 .29 .34',rgb2='.18 .23 .28',width='256',height='256')
    ET.SubElement(assets,'material',name='floor_mat',texture='floor_texture',texrepeat='8 8',texuniform='true')
    ET.SubElement(world,'geom',name='floor',type='plane',size='0 0 .1',material='floor_mat')
    human=ET.SubElement(world,'body',name='person_proxy',mocap='true',pos='-.95 0 0')
    ET.SubElement(human,'geom',name='person_torso',type='capsule',size='.22 .55',pos='0 0 .77',rgba='.10 .65 .75 1')
    ET.SubElement(human,'geom',name='person_head',type='sphere',size='.15',pos='0 0 1.65',rgba='.75 .8 .82 1')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));model.opt.timestep=.002
    return model

def run(args,seed):
    walking=load_module('walking',args.project/'verify_go2.py')
    nav=load_module('nav',args.project/'verify_navigation.py')
    m=make_model(args.project);d=mujoco.MjData(m)
    ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in walking.JOINTS]
    qadr=m.jnt_qposadr[ids];vadr=m.jnt_dofadr[ids]
    amap={int(m.actuator_trnid[i,0]):i for i in range(m.nu)};act=np.array([amap[i] for i in ids])
    rng=np.random.default_rng(seed)
    yaw0=0 if seed==0 else rng.uniform(-.08,.08)
    d.qpos[:7]=[0,0,.34,np.cos(yaw0/2),0,0,np.sin(yaw0/2)]
    d.qpos[qadr]=walking.DEFAULT+(0 if seed==0 else rng.normal(0,.015,12))
    person=np.array([-.95,0. if seed==0 else rng.uniform(-.04,.04)])
    person_v=np.zeros(2);measured_v=np.zeros(2);history=deque(maxlen=21)
    mujoco.mj_forward(m,d)
    policy=walking.Policy(.25,.4);feedback=nav.Feedback();target=walking.DEFAULT.copy()
    final_goal=np.array([2.,0.]);wait_goal=np.zeros(2);state='settle';events=[];samples=[]
    stopped_time=0.;has_walked=False;arrived_at=None;settled=0.;fallen=False
    commanded_human_pause=False;pause_start=args.pause_start;pause_end=pause_start+args.pause_duration
    human_geom={mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ['person_torso','person_head']}
    robot_geom={i for i in range(m.ngeom) if m.geom_bodyid[i]>0 and i not in human_geom}
    contact_steps=0;max_gap=0.;min_gap=float('inf')
    out=args.project/'outputs';out.mkdir(exist_ok=True)
    name=f'{args.output}_{args.mode}_seed{seed}'
    renderer=None;writer=None;frames=0
    if args.render:
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw
        renderer=mujoco.Renderer(m,height=480,width=640)
        writer=imageio.get_writer(str(out/f'{name}.mp4'),fps=25)
        cam=mujoco.MjvCamera();cam.lookat[:]=[.7,0,.6];cam.distance=4.9;cam.azimuth=110;cam.elevation=-25
    start=time.perf_counter()
    try:
        for step in range(round(args.seconds/.002)):
            xy,yaw,upright=nav.pose(d)
            if step%10==0:
                history.append(xy.copy());delayed=history[0]
                commanded_human_pause=pause_start<=d.time<pause_end
                if commanded_human_pause or d.time<2:desired_human_v=np.zeros(2)
                else:
                    target_human=delayed-np.array([.90,0.])
                    desired_human_v=1.3*(target_human-person)
                    norm=np.linalg.norm(desired_human_v)
                    if norm>.22:desired_human_v*=.22/norm
                dv=desired_human_v-person_v
                if np.linalg.norm(dv)>.5*.02:dv*=.5*.02/np.linalg.norm(dv)
                person_v+=dv
                alpha=np.exp(-.02/.25);measured_v=alpha*measured_v+(1-alpha)*person_v
                human_speed=np.linalg.norm(measured_v);gap=np.linalg.norm(xy-person)
                if human_speed>.045:has_walked=True
                old=state
                if state=='settle' and d.time>=2:state='guide'
                elif state=='guide':
                    stopped_time=stopped_time+.02 if has_walked and human_speed<.018 else 0.
                    if args.mode=='aware' and (stopped_time>.3 or gap>1.35):state='wait';wait_goal=xy.copy()
                    elif np.linalg.norm(xy-final_goal)<.06:
                        settled+=.02
                        if settled>.5:state='arrived';arrived_at=float(d.time)
                    else:settled=0.
                elif state=='wait':
                    if human_speed>.03 and gap<1.25:state='guide';stopped_time=0.
                if state!=old:events.append({'t':float(d.time),'from':old,'to':state,'robot_xy':xy.tolist(),'person_xy':person.tolist(),'gap_m':float(gap)})
                goal=wait_goal if state=='wait' else final_goal
                cmd=feedback.step(d,goal,0.,settle=state=='settle')
                if state=='settle':cmd=np.zeros(3)
                target=policy.step(m,d,cmd,qadr,vadr)
            person+=person_v*.002
            d.mocap_pos[0]=[person[0],person[1],0.]
            d.ctrl[act]=20*(target-d.qpos[qadr])-.5*d.qvel[vadr]
            mujoco.mj_step(m,d)
            for contact in d.contact:
                g1,g2=int(contact.geom1),int(contact.geom2)
                if (g1 in human_geom and g2 in robot_geom) or (g2 in human_geom and g1 in robot_geom):contact_steps+=1;break
            if not np.isfinite(d.qpos).all():raise RuntimeError('Non-finite state')
            xy,yaw,upright=nav.pose(d)
            if upright<.5 or d.qpos[2]<.15:fallen=True;break
            if step%20==0:
                gap=float(np.linalg.norm(xy-person));max_gap=max(max_gap,gap);min_gap=min(min_gap,gap)
                samples.append({'t':float(d.time),'state':state,'robot_x':float(xy[0]),'robot_y':float(xy[1]),'person_x':float(person[0]),'person_y':float(person[1]),'person_speed_m_s':float(np.linalg.norm(person_v)),'measured_person_speed_m_s':float(human_speed),'person_pause_schedule':commanded_human_pause,'gap_m':gap,'upright':upright,'goal_error_m':float(np.linalg.norm(xy-final_goal))})
                if writer:
                    renderer.update_scene(d,camera=cam);scene=renderer.scene
                    # Visual reference line only: it transmits no force.
                    idx=scene.ngeom
                    mujoco.mjv_initGeom(scene.geoms[idx],mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.9,.7,.2,1.]));scene.ngeom+=1
                    mujoco.mjv_connector(scene.geoms[idx],mujoco.mjtGeom.mjGEOM_CAPSULE,.007,np.array([person[0]+.15,person[1],.9]),np.array([xy[0],xy[1],.35]))
                    frame=renderer.render();im=Image.fromarray(frame);draw=ImageDraw.Draw(im)
                    draw.rectangle((0,0,640,67),fill=(20,25,30))
                    draw.text((12,8),f'GUIDE CHECK | {state.upper()} | t={d.time:.1f}s | gap={gap:.2f} m',fill='white')
                    draw.text((12,27),f'Person pauses at {pause_start:g}-{pause_end:g}s | {args.mode.upper()} controller',fill='white')
                    draw.text((12,46),'Kinematic person + ideal sensing | Line is not a physical leash',fill=(190,200,210))
                    frame=np.asarray(im);writer.append_data(frame)
                    if state=='wait' and not (out/f'{name}.png').exists():imageio.imwrite(out/f'{name}.png',frame)
                    frames+=1
            if arrived_at is not None and d.time-arrived_at>5:break
    finally:
        if writer:writer.close()
        if renderer:renderer.close()
    wait=[r for r in samples if r['state']=='wait' and r['person_pause_schedule']]
    wait_motion=None
    if wait:
        ref=np.array([wait[0]['robot_x'],wait[0]['robot_y']])
        wait_motion=max(float(np.linalg.norm(np.array([r['robot_x'],r['robot_y']])-ref)) for r in wait)
    wait_events=[e for e in events if e['to']=='wait']
    resumed=any(e['from']=='wait' and e['to']=='guide' and e['t']>pause_end for e in events)
    result={'mode':args.mode,'seed':seed,'person_model':'kinematic_mocap_capsule','person_height_m':1.8,'person_width_m':.44,'person_follow_delay_s':.4,'person_speed_limit_m_s':.22,'pause_start_s':pause_start,'pause_end_s':pause_end,'controller_reads_pause_schedule':False,'uses_ground_truth_pose':True,'physical_leash':False,'fallen':fallen,'contact_steps':contact_steps,'min_gap_m':min_gap,'max_gap_m':max_gap,'first_wait_delay_from_pause_s':wait_events[0]['t']-pause_start if wait_events else None,'wait_motion_during_pause_m':wait_motion,'resumed_after_pause':resumed,'arrived':arrived_at is not None,'final_goal_error_m':float(np.linalg.norm(d.qpos[:2]-final_goal)),'simulation_seconds':float(d.time),'wall_seconds':time.perf_counter()-start,'frames':frames,'events':events,'samples':samples}
    result['passed']=bool(not fallen and contact_steps==0 and arrived_at is not None and wait_motion is not None and wait_motion<.10 and resumed and max_gap<1.45)
    (out/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,default=Path(__file__).resolve().parent);p.add_argument('--mode',choices=['aware','unaware'],default='aware');p.add_argument('--seeds',type=int,nargs='+',default=[0]);p.add_argument('--seconds',type=float,default=80);p.add_argument('--output',default='person_follow');p.add_argument('--render',action='store_true');p.add_argument('--pause-start',type=float,default=8.);p.add_argument('--pause-duration',type=float,default=6.);a=p.parse_args()
    for seed in a.seeds:run(a,seed)
