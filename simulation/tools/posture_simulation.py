"""Actual CPU dynamics: crouch, rise, level feedback, crawl, input loss and stop.

--ros runs real rclpy nodes/messages with the same controller; no ROS substitutes.
"""
from pathlib import Path
import sys
import argparse
import json
import time
import csv

ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':
    sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
import numpy as np
import mujoco
from local_physics import mjcf
from bigspot_sim.posture import PostureController


def episode(parameters, seconds=20., render=False, ros=False, friction=.8, mass_scale=1.,
            disturbance=1.5, speed=.01, torque_limit=6., case=None, interactive=False):
    if interactive and not ros:
        raise ValueError('Interactive input requires --ros')
    if seconds < 12:
        raise ValueError('Use at least 12 seconds to observe the startup sequence')
    path,_=mjcf(True,300,8)
    assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')}
    model=mujoco.MjModel.from_xml_string(path.read_text(encoding='utf-8'),assets=assets)
    model.actuator_forcerange[:]=[-torque_limit,torque_limit]
    model.geom_friction[:,0]=friction
    model.body_mass[:]*=mass_scale
    model.body_inertia[:]*=mass_scale
    mujoco.mj_setConst(model,mujoco.MjData(model))
    data=mujoco.MjData(model)
    definition=json.loads((ROOT/'simulation/ros2_ws/src/bigspot_sim/config/model.json').read_text())
    controller=PostureController(definition,**parameters)
    names=[n for leg in controller.gait.legs for n in leg.names]
    jids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names]
    qadr=model.jnt_qposadr[jids]
    vadr=model.jnt_dofadr[jids]
    aids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,n+'_drive') for n in names]
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    foot_sites=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,leg.name.lower()+'_foot') for leg in controller.gait.legs]
    foot_geoms=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,leg.name+'F_1_collision') for leg in controller.gait.legs]
    foot_rows=[]
    data.qpos[2]=controller.crouch_height+.008
    data.qpos[qadr]=controller.gait.q.ravel()
    command=controller.gait.q.ravel().copy()
    data.ctrl[aids]=command
    mujoco.mj_forward(model,data)
    frames=[]
    renderer=mujoco.Renderer(model,height=480,width=640) if render else None
    camera=mujoco.MjvCamera()
    camera.distance=1.2
    camera.azimuth=130
    camera.elevation=-18
    bridge=None
    if ros:
        from ros_physics_bridge import Bridge
        bridge=Bridge(names,parameters)
    states=[]
    rows=[]
    fault=''
    started=time.monotonic()
    dt=model.opt.timestep
    for step in range(round(seconds/dt)):
        t=data.time
        matrix=data.xmat[base].reshape(3,3)
        rp=np.array([np.arctan2(matrix[2,1],matrix[2,2]),np.arcsin(np.clip(-matrix[2,0],-1,1))])
        gyro=data.qvel[3:5].copy()
        error=float(np.max(np.abs(data.qpos[qadr]-command)))
        if step%8==0:
            velocity=np.array([speed,0.,0.]) if 8<=t<seconds-3 else np.zeros(3)
            fresh=t<seconds-3
            if bridge:
                command,state=bridge.update(data.time,data.qpos[3:7],gyro,data.qpos[qadr],velocity,
                                            start=t>=1. and not interactive,
                                            send_command=fresh and not interactive)
                if command is None:
                    command=data.ctrl[aids].copy()
                fault=bridge.fault
            else:
                if t>=1.:
                    controller.start()
                try:
                    command=np.array(controller.step(.016,velocity,rp,gyro,
                          command_fresh=fresh,measured_height=float(data.xpos[base,2]),joint_error=error))
                except ValueError as exc:
                    controller.state,controller.fault='FAULT',str(exc)
                state=controller.state
                fault=controller.fault
            data.ctrl[aids]=command
            if not states or states[-1]['state']!=state:
                states.append({'time_s':float(t),'state':state})
            rows.append([t,*data.xpos[base].tolist(),*rp.tolist(),error,float(np.max(np.abs(data.actuator_force))),
                         float(np.max(np.abs(data.qvel[vadr]))),float(np.mean(np.abs(data.actuator_force)>=torque_limit*.99)),state])
            contact_geoms={int(g) for c in data.contact for g in c.geom}
            contact=[int(g in contact_geoms) for g in foot_geoms]
            foot_rows.append([t,*data.site_xpos[foot_sites].ravel().tolist(),*contact,state])
        # Small reproducible pitch torque pulse after standing. No pose teleports.
        data.xfrc_applied[base,:]=0.
        if not interactive and 6<=t<6.2:
            data.xfrc_applied[base,4]=disturbance
        mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('Non-finite physical state')
        if renderer and step%25==0:
            camera.lookat[:]=data.xpos[base]
            renderer.update_scene(data,camera=camera)
            frame=renderer.render().copy()
            from PIL import Image,ImageDraw,ImageFont
            pil=Image.fromarray(frame)
            font_path=Path('C:/Windows/Fonts/arial.ttf') if sys.platform=='win32' else Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
            font=ImageFont.truetype(str(font_path),16) if font_path.exists() else ImageFont.load_default()
            draw=ImageDraw.Draw(pil)
            draw.rectangle((0,0,640,68),fill=(15,18,22))
            label='Slow wave: RL -> FL -> RR -> FR' if parameters.get('static_crawl') else ('Diagonal trot: FR+RL / FL+RR' if parameters.get('gait_mode')=='trot' else 'Continuous crawl')
            draw.text((12,8),f'BigSpot | {state} | t={data.time:.1f}s',font=font,fill='white')
            draw.text((12,31),label+(' | ROS 2 Humble' if ros else ' | MuJoCo CPU'),font=font,fill='white')
            draw.rectangle((0,447,640,480),fill=(15,18,22))
            active_geoms={int(g) for c in data.contact for g in c.geom}
            colors=[(235,136,70),(80,176,237),(230,207,73),(90,197,110)]
            for i,leg in enumerate(controller.gait.legs):
                label=leg.name+(': ON' if foot_geoms[i] in active_geoms else ': AIR')
                draw.text((15+i*155,453),label,font=font,fill=colors[i])
            frames.append(pil)
        if bridge:
            # Real-time wall clock: ROS watchdogs represent actual device timeouts.
            delay=started+data.time-time.monotonic()
            if delay>0:
                time.sleep(delay)
    if bridge:
        transport=bridge.close()
    else:
        transport={'ros_executed':False}
    a=np.array([r[:8] for r in rows],dtype=float)
    stood=[s for s in states if s['state'] in ('READY','WALKING')]
    moving=a[(a[:,0]>=8)&(a[:,0]<seconds-3)]
    settled=a[a[:,0]>=5]
    distance=float(moving[-1,1]-moving[0,1]) if len(moving)>1 else 0.
    target=speed*max(0,seconds-11) if not interactive else 0.
    fallen=bool(np.any(a[:,3]<.12) or np.any(np.abs(a[:,4:6])>np.pi/4))
    report={'engine':f'MuJoCo {mujoco.__version__}',**transport,'parameters':parameters,
            'seconds':seconds,'mass_kg':float(model.body_mass.sum()),'friction':friction,
            'torque_limit_nm_unmeasured':torque_limit,'disturbance_torque_nm':disturbance if not interactive else 0.,
            'input_mode':'external_ros' if interactive else 'automatic_scenario',
            'speed_command_m_s':speed if not interactive else None,'walk_target_m':target if not interactive else None,
            'walk_distance_m':distance if not interactive else float(a[-1,1]-a[0,1]),
            'speed_error_fraction':abs(distance-target)/max(target,.01) if not interactive else None,
            'net_distance_after_stop_m':float(a[-1,1]-a[0,1]),
            'net_distance_error_fraction':abs(float(a[-1,1]-a[0,1])-target)/max(target,.01) if not interactive else None,
            'max_tilt_after_rise_deg':float(np.max(np.abs(settled[:,4:6]))*180/np.pi),
            'max_joint_error_rad':float(a[:,6].max()),'peak_actuator_torque_nm':float(a[:,7].max()),
            'peak_joint_speed_rad_s':float(max(row[8] for row in rows)),
            'ready_reached':bool(stood),'fallen':fallen,'states':states,'fault':fault,
            'finite_states':True,'parameters_unmeasured':True,'self_collision_enabled':False,
            'end_state':states[-1]['state']}
    # Episode objective for derivative-free policy parameter learning.
    report['reward']=None if interactive else (-report['speed_error_fraction']-report['max_tilt_after_rise_deg']*.04
                      -float(fallen)*100-float(not stood)*50-float(bool(fault))*100)
    feet=np.array([r[:-1] for r in foot_rows],dtype=float)
    walk_mask=(feet[:,0]>=9)&(feet[:,0]<seconds-3)
    if np.count_nonzero(walk_mask)>2:
        f=feet[walk_mask]
        contacts=f[:,13:17]
        body=a[(a[:,0]>=9)&(a[:,0]<seconds-3)]
        forward_velocity=np.diff(body[:,1])/np.diff(body[:,0])
        report['foot_clearance_m']={leg.name:float(np.percentile(f[:,3+3*i],95)-np.percentile(f[:,3+3*i],5))
                                   for i,leg in enumerate(controller.gait.legs)}
        report['foot_ground_contact_fraction']={leg.name:float(contacts[:,i].mean()) for i,leg in enumerate(controller.gait.legs)}
        report['fewer_than_three_support_fraction']=float((contacts.sum(1)<3).mean())
        report['actuator_saturation_fraction_walking']=float(np.mean([r[9] for r in rows if 9<=r[0]<seconds-3]))
        report['fewer_than_two_support_fraction']=float((contacts.sum(1)<2).mean())
        report['mean_walking_speed_m_s']=float(np.mean(forward_velocity))
        report['measured_speed_ratio_to_slow_reference']=float(np.mean(forward_velocity)/.01)
        report['backward_motion_fraction']=float((forward_velocity<-.002).mean())
        report['forward_velocity_rms_error_m_s']=float(np.sqrt(np.mean((forward_velocity-speed)**2)))
    if case:
        if not case.replace('_','').replace('-','').isalnum():
            raise ValueError('Invalid case name')
        output=ROOT/'simulation/generated/posture'
        output.mkdir(parents=True,exist_ok=True)
        with (output/(case+'.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream)
            writer.writerow(['time_s','base_x_m','base_y_m','base_z_m','roll_rad','pitch_rad','max_joint_error_rad','peak_torque_nm','max_joint_speed_rad_s','actuator_saturation_fraction','state'])
            writer.writerows(rows)
        with (output/(case+'_feet.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream)
            writer.writerow(['time_s']+[leg.name+'_'+axis+'_m' for leg in controller.gait.legs for axis in ['x','y','z']]+
                            [leg.name+'_contact' for leg in controller.gait.legs]+['state'])
            writer.writerows(foot_rows)
        (output/(case+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        if frames:
            frames[0].save(output/(case+'.gif'),save_all=True,append_images=frames[1:],duration=50,loop=0)
            frames[-1].save(output/(case+'.png'))
    if renderer:
        renderer.close()
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=20.)
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--interactive',action='store_true',help='Disable automatic start, cmd_vel and disturbance; use external ROS inputs')
    parser.add_argument('--case',default='startup_baseline')
    parser.add_argument('--policy',type=Path)
    parser.add_argument('--torque',type=float,default=6.)
    parser.add_argument('--speed',type=float,default=.01)
    args=parser.parse_args()
    parameters=json.loads(args.policy.read_text())['parameters'] if args.policy else {}
    print(json.dumps(episode(parameters,args.seconds,args.render,args.ros,case=args.case,
                             torque_limit=args.torque,interactive=args.interactive,speed=args.speed),indent=2),flush=True)
