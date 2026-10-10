"""Free-base dynamics. Never set base/joint poses after the initial condition."""
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from .design import LEGS, motor_names, mass_and_cost
from .gait import Gait
from .model import make_mjcf
from .kinematics import smoothstep
from .safety import Supervisor


def rollout(d, speed=0.25, seconds=12.0, friction=0.8, mass_scale=1.0, startup=False, seed=0, render=False, output=None, navigator=None, fault=None,on_step=None,command_source=None,test_obstacles=False):
    obstacles = ((1.1,0.0,0.18,0.25,0.5),(2.1,0.65,0.2,0.18,0.5)) if navigator or test_obstacles else ()
    model = mujoco.MjModel.from_xml_string(make_mjcf(d, obstacles))
    model.geom_friction[:,0] = friction
    model.body_mass[:] *= mass_scale
    model.body_inertia[:] *= mass_scale
    data = mujoco.MjData(model)
    mujoco.mj_setConst(model, data)
    gait = Gait(d)
    joint_ids = np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,f"{leg}_{suffix}") for leg,_,_ in LEGS for suffix in ("abad","hip","knee")])
    qadr = model.jnt_qposadr[joint_ids]
    vadr = model.jnt_dofadr[joint_ids]
    base_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"base_link")
    names = motor_names(d)
    peak = np.array([d["actuators"][name]["peak_torque_nm"] for name in names])
    hold = np.array([d["actuators"][name]["stall_rated_torque_nm"] for name in names])*d["control"]["thermal_derating"]
    no_load = np.array([d["actuators"][name]["no_load_rpm"] for name in names])*math.pi/30
    stand_q = gait.stand()
    data.qpos[qadr] = np.tile(d.get('startup_joint_angles_rad',[0.,-1.43,2.86]),4) if startup else stand_q
    if startup:
        data.qpos[2] = 0.12
        mujoco.mj_forward(model,data)
        floor_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"floor")
        lowest = min(mujoco.mj_geomDistance(model,data,g,floor_id,1.,None) for g in range(model.ngeom) if g!=floor_id and model.geom_bodyid[g]!=0 and model.geom_contype[g])
        data.qpos[2] += 0.002-lowest
    mujoco.mj_forward(model,data)
    initial = data.qpos[:3].copy()
    captured = None
    state = "OFF" if startup else "STAND"
    estimate = np.zeros(3)
    supervisor=Supervisor(d['control']['sensor_timeout_s'],d['control']['command_timeout_s'])
    rng = np.random.default_rng(seed)
    rows, targets, torques, frames, joint_states, joint_speeds = [], [], [], [], [], []
    errors, contacts = [], []
    self_impulses = 0.0
    obstacle_impulses = 0.0
    budget_scale_min = 1.0
    off_torque_max = 0.0
    off_belly_contact_steps=0
    fault_started = False
    fault_torque_max = 0.0
    fault_detection_time = None
    camera_stop_time=None;motion_reason='OK'
    renderer = mujoco.Renderer(model,height=480,width=640) if render else None
    view = mujoco.MjvCamera()
    view.distance=1.25; view.azimuth=135; view.elevation=-18
    dt = model.opt.timestep
    gyro_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SENSOR,"imu_gyro")
    accel_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SENSOR,"imu_accel")
    gyro_adr = model.sensor_adr[gyro_id]; accel_adr = model.sensor_adr[accel_id]
    startup_end = 5.0 if startup else 1.5
    for step in range(round(seconds/dt)):
        t = float(data.time)
        gyro = data.sensordata[gyro_adr:gyro_adr+3] + rng.normal(0,0.002,3)
        accel = data.sensordata[accel_adr:accel_adr+3] + rng.normal(0,0.02,3)
        estimate += gyro*dt
        accel_tilt=np.array([math.atan2(accel[1],accel[2]),math.atan2(-accel[0],math.hypot(accel[1],accel[2]))])
        if 8.0<np.linalg.norm(accel)<11.5:estimate[:2]=0.995*estimate[:2]+0.005*accel_tilt
        for source in ('imu','joints','command'):
            if fault!=source or t<6.0:supervisor.update(source,t)
        velocity=np.zeros(3)
        if startup and t<1.0:
            desired=data.qpos[qadr].copy(); state="OFF"
        elif startup and t<startup_end:
            if captured is None:captured=data.qpos[qadr].copy()
            desired=captured+(stand_q-captured)*smoothstep((t-1.0)/(startup_end-1.0)); state="RISE"
        else:
            elapsed=t-startup_end
            if elapsed<seconds-startup_end-2.0:
                velocity[0]=speed*smoothstep(elapsed/1.5)
                if speed:velocity[2]=np.clip(-1.5*estimate[2],-.3,.3)
                state="WALK" if speed else "STAND"
                if navigator:
                    velocity=navigator(model,data,t)
                    if math.isfinite(navigator.last_capture):supervisor.update('camera',navigator.last_capture)
                if command_source:velocity=np.asarray(command_source(t),dtype=float)
            else:state="STOP"
            decision=supervisor.evaluate(t,autonomous=bool(navigator),tilt=max(abs(estimate[0]),abs(estimate[1])))
            if not decision.motion_enabled:
                velocity[:]=0;motion_reason=decision.reason
                if decision.reason=='STALE_CAMERA' and camera_stop_time is None:camera_stop_time=t
            desired=gait.targets(dt,velocity,roll=estimate[0]*0.5,pitch=estimate[1]*0.5)
        decision=supervisor.evaluate(t,tilt=0 if startup and t<startup_end else max(abs(estimate[0]),abs(estimate[1])))
        if not decision.torque_enabled:
            if not fault_started:fault_detection_time=t-6.0 if fault else t
            fault_started=True
        torque=d["control"]["kp"]*(desired-data.qpos[qadr])-d["control"]["kd"]*data.qvel[vadr]
        # A linear speed envelope is an explicitly unmeasured conservative approximation.
        available=peak*np.clip(1-np.abs(data.qvel[vadr])/no_load,0,1)
        torque=np.clip(torque,-available,available)
        positive_power=np.maximum(torque*data.qvel[vadr],0).sum()/0.8
        budget_scale=min(1.0,d["battery"]["maximum_motor_bus_power_w"]/max(positive_power,1.))
        torque*=budget_scale;budget_scale_min=min(budget_scale_min,budget_scale)
        if state=="OFF":torque[:]=0;off_torque_max=max(off_torque_max,float(np.max(np.abs(torque))))
        if fault_started:
            torque[:]=0;state="FAULT_OFF"
            fault_torque_max=max(fault_torque_max,float(np.max(np.abs(torque))))
        data.ctrl[:]=torque
        positive_power=float(np.maximum(torque*data.qvel[vadr],0).sum()/0.8)
        mujoco.mj_step(model,data)
        if on_step:on_step(model,data,desired,torque,estimate)
        mat=data.xmat[base_id].reshape(3,3)
        roll=math.atan2(mat[2,1],mat[2,2]);pitch=math.atan2(-mat[2,0],math.hypot(mat[2,1],mat[2,2]))
        yaw=math.atan2(mat[1,0],mat[0,0])
        foot_count=0
        for ci,c in enumerate(data.contact):
            impulse=np.zeros(6);mujoco.mj_contactForce(model,data,ci,impulse)
            g1,g2=int(c.geom1),int(c.geom2)
            n1=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g1) or ""
            n2=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g2) or ""
            if impulse[0]>1 and ((n1.endswith("_foot") and n2=="floor") or (n2.endswith("_foot") and n1=="floor")):foot_count+=1
            if model.geom_bodyid[g1]!=0 and model.geom_bodyid[g2]!=0:self_impulses+=impulse[0]*dt
            if n1.startswith("obstacle_") or n2.startswith("obstacle_"):obstacle_impulses+=impulse[0]*dt
            if state=='OFF' and {n1,n2}=={'floor','belly_skid'} and impulse[0]>1:off_belly_contact_steps+=1
        rows.append([data.time,*data.qpos[:3],roll,pitch,yaw,foot_count,positive_power])
        targets.append(desired.copy());torques.append(torque.copy())
        joint_states.append(data.qpos[qadr].copy());joint_speeds.append(data.qvel[vadr].copy())
        errors.append(float(np.max(np.abs(desired-data.qpos[qadr]))))
        contacts.append(foot_count)
        if renderer and step%round(1/(20*dt))==0:
            if navigator:
                view.lookat[:]=[1.5,0,.1];view.distance=4.8;view.azimuth=90;view.elevation=-62
            else:view.lookat[:]=data.qpos[:3]
            renderer.update_scene(data,camera=view)
            frame=renderer.render().copy()
            if navigator:
                from PIL import Image,ImageDraw
                im=Image.fromarray(frame);depth=np.nan_to_num(navigator.camera.depth,nan=4.)
                shade=np.uint8(np.clip(1-depth/4,0,1)*255)
                inset=Image.fromarray(np.stack((shade,shade,shade),axis=-1)).resize((144,84))
                im.paste(inset,(484,36));ImageDraw.Draw(im).text((484,122),'Depth camera',fill='white')
                frame=np.asarray(im)
            frames.append(frame)
        if not np.isfinite(data.qpos).all():raise RuntimeError("Non-finite physics state")
    if renderer:renderer.close()
    rows=np.array(rows);torques=np.array(torques)
    window=rows[:,0]>startup_end+1.0
    rms=np.sqrt(np.mean(torques[window]**2,axis=0))
    final=rows[-1]
    tilt=float(np.rad2deg(np.abs(rows[window,4:6])).max())
    min_height=float(rows[window,3].min())
    fall=bool(min_height<d["standing_leg_height_m"]*0.55 or tilt>35)
    report={**mass_and_cost(d),"engine":f"MuJoCo {mujoco.__version__}","floating_base":True,"external_lifting_forces":False,"pose_writes_after_initialization":False,"hardware_tested":False,"isaac_tested":False,"seconds":seconds,"mass_scale":mass_scale,"friction":friction,"requested_speed_m_s":speed,"net_x_m":float(final[1]-initial[0]),"net_y_m":float(final[2]-initial[1]),"max_tilt_deg":tilt,"minimum_height_m":min_height,"fallen":fall,"joint_torque_rms_nm":rms.tolist(),"derated_hold_limits_nm":hold.tolist(),"thermal_margin_ratio":float(np.max(rms/hold)),"peak_joint_torque_nm":float(np.abs(torques).max()),"motor_mechanical_power_peak_w":float(rows[:,8].max()*0.8),"power_budget_scale_min":budget_scale_min,"self_collision_impulse_ns":float(self_impulses),"obstacle_collision_impulse_ns":float(obstacle_impulses),"max_joint_tracking_error_rad":max(errors),"off_torque_max_nm":off_torque_max,"fault":fault,"fault_detection_time_s":fault_detection_time,"fault_torque_max_nm":fault_torque_max,"motor_speed_envelope":"unmeasured linear envelope","continuous_torque_derating_assumed":d["control"]["thermal_derating"],"navigation_odometry":"simulator ground truth; VSLAM not executed" if navigator else None}
    report["passed"]=not fall and report["thermal_margin_ratio"]<=1.0 and self_impulses<0.5 and obstacle_impulses<0.5 and (speed==0 or report["net_x_m"]>0.3)
    report['passed']=bool(report['passed'])
    report['last_safety_reason']=supervisor.latched or 'OK'
    report['off_belly_contact_steps']=off_belly_contact_steps
    if startup:report['passed']=report['passed'] and off_belly_contact_steps>0 and off_torque_max==0
    report['motion_interlock_reason']=motion_reason
    report['camera_motion_stop_time_s']=camera_stop_time
    if navigator and navigator.camera_dropout_at is not None:
        report['camera_dropout_stop_passed']=bool(camera_stop_time is not None and camera_stop_time-navigator.camera_dropout_at<=.31 and not fall)
    if navigator:
        report['navigation']=navigator.report(data)
        report['passed']=report['passed'] and navigator.reached
    if fault:
        expected_limit=d['control']['sensor_timeout_s'] if fault in ('imu','joints') else d['control']['command_timeout_s']
        report['expected_shutdown_passed']=bool(fault_started and fault_detection_time<=expected_limit+.003 and fault_torque_max==0) if fault in ('imu','joints') else bool(not fault_started)
    if output:
        output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
        output.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        np.savez_compressed(output.with_suffix(".npz"),state=rows,targets=np.array(targets),torques=torques,joints=np.array(joint_states),joint_speeds=np.array(joint_speeds))
        if navigator:output.with_name(output.name+'_navigation.json').write_text(json.dumps(navigator.log)+'\n',encoding='utf-8')
        if frames:
            from PIL import Image, ImageDraw
            import imageio_ffmpeg
            video=imageio_ffmpeg.write_frames(str(output.with_suffix(".mp4")),(640,480),fps=20,codec="libx264",quality=7)
            video.send(None)
            for i,frame in enumerate(frames):
                image=Image.fromarray(frame);draw=ImageDraw.Draw(image)
                draw.rectangle((0,0,640,28),fill=(10,15,20))
                draw.text((10,8),f"BigSpot QDD | CPU PHYSICS | {i/20:.2f}s | 1x",fill="white")
                video.send(np.asarray(image))
            video.close()
            Image.fromarray(frames[len(frames)//2]).save(output.with_suffix(".png"))
    return report
