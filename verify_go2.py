"""Mac feasibility probe. WTW observation conventions follow art-e-fact/unitree_mujoco.

See assets/controller_LICENSE and assets/WTW_LICENSE for upstream licenses.
No SDK, ROS, training, or real robot connection is used.
"""
import argparse
import json
import time
from pathlib import Path
import mujoco
import numpy as np
import torch

ROOT = Path(__file__).parent
DEFAULT = np.array([.1,.8,-1.5, -.1,.8,-1.5, .1,1.,-1.5, -.1,1.,-1.5], dtype=np.float32)
JOINTS = [f"{leg}_{part}_joint" for leg in ["FL","FR","RL","RR"] for part in ["hip","thigh","calf"]]

class Policy:
    def __init__(self, stance_width=0., stance_length=0.):
        cfg=json.loads((ROOT/"assets/policy_config.json").read_text())
        self.cfg=cfg
        self.scale=cfg["obs_scales"]
        self.history=torch.zeros((1,70*30))
        self.action=np.zeros(12,dtype=np.float32)
        self.last=np.zeros(12,dtype=np.float32)
        self.phase=0.
        self.stance_width=stance_width
        self.stance_length=stance_length
        torch.set_num_threads(1)
        self.body=torch.jit.load(str(ROOT/"assets/body_latest.jit"),map_location="cpu").eval()
        self.adapt=torch.jit.load(str(ROOT/"assets/adaptation_module_latest.jit"),map_location="cpu").eval()
    def step(self,m,d,cmd,qadr,vadr):
        s=self.scale
        c=np.zeros(15,dtype=np.float32)
        c[:3]=cmd
        c[4:6]=[3.,.5]; c[8:10]=[.5,.08]
        c[12:14]=[self.stance_width,self.stance_length]
        scale=np.array([s["lin_vel"],s["lin_vel"],s["ang_vel"],s["body_height_cmd"],1,1,1,1,1,s["footswing_height_cmd"],s["body_pitch_cmd"],s["body_roll_cmd"],s["stance_width_cmd"],s["stance_length_cmd"],s["aux_reward_cmd"]])
        rot=np.empty(9); mujoco.mju_quat2Mat(rot,d.qpos[3:7])
        gravity=rot.reshape(3,3).T@np.array([0,0,-1.])
        clocks=np.sin(2*np.pi*(self.phase+np.array([c[5]+c[6]+c[7],c[6],c[7],c[5]])))
        obs=np.concatenate([gravity,c*scale,(d.qpos[qadr]-DEFAULT)*s["dof_pos"],d.qvel[vadr]*s["dof_vel"],np.clip(self.action,-10,10),self.last,clocks]).astype(np.float32)
        self.history=torch.roll(self.history,-70,1)
        self.history[0,-70:]=torch.from_numpy(obs)
        with torch.inference_mode():
            latent=self.adapt(self.history)
            a=self.body(torch.cat([self.history,latent],dim=1))[0].numpy().copy()
        self.last=self.action.copy(); self.action=a
        a=a*self.cfg["control"]["action_scale"]
        a[[0,3,6,9]]*=self.cfg["control"]["hip_scale_reduction"]
        self.phase=(self.phase+.02*c[4])%1
        return DEFAULT+a

def command(t):
    if t<2:return "settle",[0.,0.,0.]
    if t<8:return "forward",[.35,0.,0.]
    if t<10:return "stop",[0.,0.,0.]
    if t<14:return "turn",[0.,0.,.35]
    if t<18:return "forward_after_turn",[.3,0.,0.]
    return "final_stop",[0.,0.,0.]

def main():
    p=argparse.ArgumentParser(); p.add_argument("--seconds",type=float,default=20); p.add_argument("--render",action="store_true"); p.add_argument("--viewer",action="store_true"); p.add_argument("--output",default="probe"); p.add_argument("--model",default=str(ROOT/"models/go2/scene.xml")); p.add_argument("--turn-rate",type=float,default=.35); p.add_argument("--turn-forward",type=float,default=0.); p.add_argument("--stance-width",type=float,default=0.); p.add_argument("--stance-length",type=float,default=0.); args=p.parse_args()
    out=ROOT/"outputs"; out.mkdir(exist_ok=True)
    m=mujoco.MjModel.from_xml_path(args.model)
    m.opt.timestep=.002
    d=mujoco.MjData(m)
    ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in JOINTS]
    qadr=m.jnt_qposadr[ids]; vadr=m.jnt_dofadr[ids]
    actuator_for_joint={int(m.actuator_trnid[i,0]):i for i in range(m.nu)}
    act=np.array([actuator_for_joint[i] for i in ids])
    d.qpos[:7]=[0,0,.34,1,0,0,0]; d.qpos[qadr]=DEFAULT
    mujoco.mj_forward(m,d)
    policy=Policy(args.stance_width,args.stance_length); target=DEFAULT.copy()
    renderer=None; writer=None; viewer=None
    if args.render:
        import imageio.v2 as imageio
        renderer=mujoco.Renderer(m,height=480,width=640)
        writer=imageio.get_writer(str(out/f"{args.output}.mp4"),fps=25)
    if args.viewer:
        import mujoco.viewer as viewer_module
        viewer=viewer_module.launch_passive(m,d)
    cam=mujoco.MjvCamera(); cam.distance=2.4; cam.azimuth=135; cam.elevation=-22
    frames=0; samples=[]; inference=[]; start=time.perf_counter()
    try:
        for step in range(round(args.seconds/m.opt.timestep)):
            phase,cmd=command(d.time)
            if phase=="turn":cmd=[args.turn_forward,0.,args.turn_rate]
            if step%10==0:
                st=time.perf_counter(); target=policy.step(m,d,cmd,qadr,vadr); inference.append(time.perf_counter()-st)
            d.ctrl[act]=20*(target-d.qpos[qadr])-.5*d.qvel[vadr]
            mujoco.mj_step(m,d)
            if step%20==0:
                w,x,y,z=d.qpos[3:7]; upright=1-2*(x*x+y*y)
                yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
                samples.append({"t":float(d.time),"phase":phase,"x":float(d.qpos[0]),"y":float(d.qpos[1]),"z":float(d.qpos[2]),"yaw":float(yaw),"upright":float(upright)})
                if writer:
                    cam.lookat[:]=d.qpos[:3]
                    renderer.update_scene(d,camera=cam)
                    frame=renderer.render(); writer.append_data(frame)
                    if frames==0:imageio.imwrite(out/f"{args.output}.png",frame)
                    frames+=1
                if viewer:
                    viewer.cam.lookat[:]=d.qpos[:3]; viewer.sync()
            if viewer:
                if not viewer.is_running():break
                delay=d.time-(time.perf_counter()-start)
                if delay>0:time.sleep(delay)
            if not np.isfinite(d.qpos).all():raise RuntimeError("Non-finite state")
    finally:
        if writer:writer.close()
        if renderer:renderer.close()
        if viewer:viewer.close()
    elapsed=time.perf_counter()-start
    phases={}
    for name in dict.fromkeys(r["phase"] for r in samples):
        a=[r for r in samples if r["phase"]==name]; first,last=a[0],a[-1]
        phases[name]={"displacement_m":float(np.hypot(last["x"]-first["x"],last["y"]-first["y"])),"yaw_change_rad":float(np.unwrap([r["yaw"] for r in a])[-1]-first["yaw"]),"min_base_height_m":min(r["z"] for r in a),"min_upright":min(r["upright"] for r in a)}
    result={"arguments":vars(args),"simulation_seconds":float(d.time),"wall_seconds":elapsed,"real_time_factor":d.time/elapsed,"policy_median_ms":float(np.median(inference)*1000),"policy_p95_ms":float(np.percentile(inference,95)*1000),"render":args.render,"viewer":args.viewer,"frames":frames,"phases":phases,"samples":samples}
    (out/f"{args.output}.json").write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!="samples"},indent=2))

if __name__=="__main__":main()
