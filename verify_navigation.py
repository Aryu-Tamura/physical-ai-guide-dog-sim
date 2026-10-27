"""Closed-loop navigation feasibility test, using learned torque-driven walking.

Perfect simulator pose is used for feedback; localization is not evaluated.
Only initial state is assigned. During the rollout, movement is entirely physics
and learned-policy joint targets, without position/heading overrides.
"""
import argparse
import importlib.util
import json
import time
from pathlib import Path
import numpy as np
import mujoco

DEFAULT_PROJECT=Path(__file__).resolve().parent
def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi
def pose(d):
    w,x,y,z=d.qpos[3:7]
    return np.array(d.qpos[:2]),float(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))),float(1-2*(x*x+y*y))

class Feedback:
    def __init__(self):
        self.velocity_i=np.zeros(2)
        self.yaw_i=0.
    def step(self,d,goal,heading,settle=False):
        xy,yaw,_=pose(d)
        rot=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
        error_body=rot@(goal-xy)
        desired=np.clip(.8*error_body,[-.15,-.12],[.20,.12])
        yaw_error=wrap(heading-yaw)
        if abs(yaw_error)>.45:desired[:]=0.
        actual=rot@d.qvel[:2]
        if not settle:
            self.velocity_i=np.clip(self.velocity_i+.9*(desired-actual)*.02,-.5,.5)
            if abs(yaw_error)<.4:self.yaw_i=np.clip(self.yaw_i+.25*yaw_error*.02,-.4,.4)
        cmd_xy=2*desired+2*(desired-actual)+self.velocity_i
        cmd_yaw=5*yaw_error-.7*d.qvel[5]+self.yaw_i
        return np.r_[np.clip(cmd_xy,[-.7,-.5],[.9,.5]),np.clip(cmd_yaw,-1.,1.)]

def run(args,seed):
    spec=importlib.util.spec_from_file_location('walking',args.project/'verify_go2.py')
    walking=importlib.util.module_from_spec(spec); spec.loader.exec_module(walking)
    m=mujoco.MjModel.from_xml_path(str(args.project/'models/go2/scene.xml')); m.opt.timestep=.002
    d=mujoco.MjData(m)
    ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in walking.JOINTS]
    qadr=m.jnt_qposadr[ids]; vadr=m.jnt_dofadr[ids]
    act=np.array([{int(m.actuator_trnid[i,0]):i for i in range(m.nu)}[i] for i in ids])
    rng=np.random.default_rng(seed)
    initial_yaw=0. if seed==0 else rng.uniform(-.1,.1)
    d.qpos[:7]=[0,0,.34,np.cos(initial_yaw/2),0,0,np.sin(initial_yaw/2)]
    d.qpos[qadr]=walking.DEFAULT+(0 if seed==0 else rng.normal(0,.015,12))
    mujoco.mj_forward(m,d)
    policy=walking.Policy(.25,.4); feedback=Feedback(); target=walking.DEFAULT.copy()
    phase='settle'; samples=[]; events=[]; stable=0.; hold_start=None
    goal=np.array([0.,0.]); heading=0.; cmd=np.zeros(3)
    phase_start=0.; timeouts=[]; fallen=False; filtered_velocity=np.zeros(2)
    writer=None; renderer=None; cam=None
    out=args.project/'outputs'; out.mkdir(exist_ok=True)
    name=f'{args.output}_seed{seed}'
    if args.render:
        import imageio.v2 as imageio
        from PIL import Image, ImageDraw
        renderer=mujoco.Renderer(m,height=480,width=640)
        writer=imageio.get_writer(str(out/f'{name}.mp4'),fps=25)
        cam=mujoco.MjvCamera();cam.lookat[:]=[.5,.5,.15];cam.distance=3.8;cam.azimuth=115;cam.elevation=-40
    start=time.perf_counter()
    try:
        for step in range(round(args.seconds/.002)):
            xy,yaw,upright=pose(d)
            if step%10==0:
                old_phase=phase
                # A trotting body has periodic velocity even at a fixed location.
                # Use a documented 0.5 s low-pass velocity for arrival detection;
                # evaluate the entire 10 s position envelope independently below.
                alpha=np.exp(-.02/.5)
                filtered_velocity=alpha*filtered_velocity+(1-alpha)*d.qvel[:2]
                speed=float(np.linalg.norm(filtered_velocity))
                if phase=='settle':
                    if d.time>=2:
                        phase='straight';goal=np.array([1.,0.]);heading=0.
                elif phase=='straight':
                    stable=stable+.02 if np.linalg.norm(xy-goal)<.06 and speed<.06 else 0.
                    if stable>.5:phase='turn_90';heading=np.pi/2;stable=0.
                elif phase=='turn_90':
                    stable=stable+.02 if abs(wrap(heading-yaw))<.08 and np.linalg.norm(xy-goal)<.10 else 0.
                    if stable>.5:phase='second_leg';goal=np.array([1.,1.]);stable=0.
                elif phase=='second_leg':
                    stable=stable+.02 if np.linalg.norm(xy-goal)<.06 and speed<.06 else 0.
                    if stable>.5:phase='hold';hold_start=float(d.time);stable=0.
                if phase!=old_phase:
                    events.append({'t':float(d.time),'from':old_phase,'to':phase,'xy':xy.tolist(),'yaw':yaw})
                    phase_start=float(d.time)
                cmd=feedback.step(d,goal,heading,settle=phase=='settle')
                if phase=='settle':cmd=np.zeros(3)
                target=policy.step(m,d,cmd,qadr,vadr)
            d.ctrl[act]=20*(target-d.qpos[qadr])-.5*d.qvel[vadr]
            mujoco.mj_step(m,d)
            if not np.isfinite(d.qpos).all():raise RuntimeError('Non-finite state')
            if step%20==0:
                xy,yaw,upright=pose(d)
                samples.append({'t':float(d.time),'phase':phase,'x':float(xy[0]),'y':float(xy[1]),'yaw':yaw,'height':float(d.qpos[2]),'upright':upright,'goal_error_m':float(np.linalg.norm(xy-goal)),'heading_error_rad':float(wrap(heading-yaw)),'speed_m_s':float(np.linalg.norm(d.qvel[:2])),'filtered_speed_m_s':speed,'command':cmd.tolist()})
                if writer:
                    renderer.update_scene(d,camera=cam)
                    scene=renderer.scene
                    # Ground markers distinguish corner and goal from the robot.
                    for pos,color in [([1,0,.015],[.9,.6,.1,1]),([1,1,.015],[.1,.8,.3,1])]:
                        if scene.ngeom<scene.maxgeom:
                            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_CYLINDER,np.array([.07,.01,0.]),np.array(pos,dtype=float),np.eye(3).ravel(),np.array(color,dtype=float));scene.ngeom+=1
                    frame=renderer.render()
                    image=Image.fromarray(frame);draw=ImageDraw.Draw(image)
                    draw.rectangle((0,0,640,64),fill=(20,25,30))
                    draw.text((14,9),f'GO2 CLOSED-LOOP CHECK | {phase.upper()} | t={d.time:.1f}s',fill='white')
                    draw.text((14,28),f'Position error: {np.linalg.norm(xy-goal)*100:.1f} cm | Heading error: {np.degrees(wrap(heading-yaw)):.1f} deg',fill='white')
                    draw.text((14,46),'Simulator ground-truth feedback | No person model yet',fill=(190,200,210))
                    frame=np.asarray(image);writer.append_data(frame)
                    if phase=='hold' and not (out/f'{name}.png').exists():imageio.imwrite(out/f'{name}.png',frame)
                if upright<.5 or d.qpos[2]<.15:fallen=True;break
            if hold_start is not None and d.time-hold_start>=10:break
    finally:
        if writer:writer.close()
        if renderer:renderer.close()
    elapsed=time.perf_counter()-start
    hold=[r for r in samples if r['phase']=='hold']
    phases={}
    for ph in dict.fromkeys(r['phase'] for r in samples):
        a=[r for r in samples if r['phase']==ph]
        phases[ph]={'seconds':a[-1]['t']-a[0]['t'],'final_goal_error_m':a[-1]['goal_error_m'],'final_heading_error_deg':float(np.degrees(a[-1]['heading_error_rad'])),'min_upright':min(r['upright'] for r in a)}
    hold_duration=hold[-1]['t']-hold[0]['t'] if hold else 0
    max_hold_error=max(r['goal_error_m'] for r in hold) if hold else None
    result={'seed':seed,'initial_yaw_rad':float(initial_yaw),'uses_ground_truth_pose':True,'arrival_velocity_filter_seconds':.5,'fallen':fallen,'reached_hold':bool(hold),'hold_duration_s':hold_duration,'max_hold_goal_error_m':max_hold_error,'passed':bool(not fallen and hold_duration>=9.9 and max_hold_error<.06),'final_goal_error_m':samples[-1]['goal_error_m'],'simulation_seconds':float(d.time),'wall_seconds':elapsed,'real_time_factor':d.time/elapsed,'events':events,'phases':phases,'samples':samples}
    (out/f'{name}.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2),flush=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,default=DEFAULT_PROJECT);p.add_argument('--seconds',type=float,default=120);p.add_argument('--seeds',type=int,nargs='+',default=[0]);p.add_argument('--output',default='navigation');p.add_argument('--render',action='store_true');a=p.parse_args()
    for seed in a.seeds:run(a,seed)
