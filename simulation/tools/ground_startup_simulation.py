"""Actual dynamics of torque-off body rest, joint capture and supported startup.

The initial condition is folded at floor level. There are no base pose writes,
lifting forces, or joint position writes after initialization. Non-adjacent
self-collision is enabled for the complete episode.
"""
import argparse
import csv
import json
import time
from motion_simulation import ROOT, mjcf, mujoco, MotionController, StateEstimator, foot_forces
from bigspot_sim.hardware_reference import pulse_angles
import numpy as np


def run(ros=False,holdout=False,fault='',case=None,legacy=False):
    cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
    if legacy:
        params=json.loads((cfg/'control_policy_adaptive.json').read_text())['parameters']
        params={**params,'ground_rest':True,'ground_prepare_seconds':7.,'ground_arm_seconds':.8}
    else:params=json.loads((cfg/'control_policy_ground_startup.json').read_text())['parameters']
    path,_=mjcf(True,300,8.)
    model=mujoco.MjModel.from_xml_string(path.read_text(),assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')})
    model.geom_conaffinity[:]=3
    if holdout:
        model.body_mass[:]*=1.15;model.body_inertia[:]*=1.15;model.geom_friction[:,0]=.55
        mujoco.mj_setConst(model,mujoco.MjData(model))
    data=mujoco.MjData(model)
    control=MotionController(json.loads((cfg/'model.json').read_text()),**params)
    names=[n for leg in control.gait.legs for n in leg.names]
    joints=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names]
    qadr=model.jnt_qposadr[joints];vadr=model.jnt_dofadr[joints]
    aids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,n+'_drive') for n in names]
    reference=control.hardware_reference
    if reference:
        model.jnt_range[joints]=np.array([[j['lower'],j['upper']] for leg in control.gait.legs for j in leg.joints])
        model.actuator_ctrlrange[aids]=model.jnt_range[joints]
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'base_link_collision')
    ground=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'ground')
    feet=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,l.name+'F_1_collision') for l in control.gait.legs]
    # One-time initial condition: folded, 2 mm above measured mesh floor height.
    data.qpos[2]=.055;data.qpos[qadr]=control.ground_startup.folded_q
    data.ctrl[aids]=data.qpos[qadr]
    gains=model.actuator_gainprm.copy();bias=model.actuator_biasprm.copy()
    model.actuator_gainprm[:]=0.;model.actuator_biasprm[:]=0.
    mujoco.mj_forward(model,data)
    # Place the whole folded mesh above the floor before free settling. A body
    # at 55 mm alone can leave its folded paws penetrating the initial plane.
    lowest=min(mujoco.mj_geomDistance(model,data,i,ground,1.,None) for i in range(model.ngeom) if i!=ground)
    data.qpos[2]+=max(0.,.002-lowest)
    mujoco.mj_forward(model,data)
    initial_clearance=min(mujoco.mj_geomDistance(model,data,i,ground,1.,None) for i in range(model.ngeom) if i!=ground)
    # Physically let the torque-off robot settle before recording its resting
    # startup. Only time is offset; no base or joint pose is rewritten.
    stable=0.
    for _ in range(round(15./model.opt.timestep)):
        mujoco.mj_step(model,data)
        support=0.
        for i,contact in enumerate(data.contact):
            if set(contact.geom)=={ground,body}:
                f=np.zeros(6);mujoco.mj_contactForce(model,data,i,f);support+=max(0.,float(f[0]))
        w,x,y,z=data.qpos[3:7]
        rp=np.array([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1))])
        touching=abs(mujoco.mj_geomDistance(model,data,body,ground,1.,None))<.002
        calm=support>1. and touching and max(abs(rp))<np.deg2rad(5.) and max(abs(data.sensor('imu_gyro').data))<.10
        stable=stable+model.opt.timestep if calm else 0.
        if stable>=.4:break
    else:raise RuntimeError('Unpowered folded model did not settle onto body-ground support')
    preroll=data.time
    bridge=None
    if ros:
        from ros_physics_bridge import Bridge
        bridge=Bridge(names,params)
    estimator=StateEstimator();rng=np.random.default_rng(912)
    rows=[];sensors=[];states=[];currents=[];pulses=[];self_force=0.;min_self_distance=0.
    command=data.qpos[qadr].copy();status={};start=time.monotonic();step_s=model.opt.timestep
    for step in range(round(23./step_s)):
        t=data.time-preroll
        if step%8==0:
            wxyz=data.sensor('imu_orientation').data.copy()
            gyro=data.sensor('imu_gyro').data.copy();accel=data.sensor('imu_accel').data.copy()
            if holdout:gyro+=rng.normal(0,.015,3);accel+=rng.normal(0,.12,3)
            forces=foot_forces(model,data,feet);body_force=0.
            for contact_i,contact in enumerate(data.contact):
                f=np.zeros(6);mujoco.mj_contactForce(model,data,contact_i,f)
                if set(contact.geom)=={ground,body}:body_force+=max(0.,float(f[0]))
                if ground not in contact.geom:
                    self_force=max(self_force,float(abs(f[0])));min_self_distance=min(min_self_distance,float(contact.dist))
            clearance=mujoco.mj_geomDistance(model,data,body,ground,1.,None)
            gf=[body_force,clearance,float(data.xpos[base,2])]
            estimator.update([*wxyz[1:],wxyz[0]],gyro,accel,t);estimator.contacts(forces,t)
            obs=estimator.snapshot(t)
            obs.update(joint_feedback_fresh=not(fault=='joints' and t>=6.),measured_joint_angles_rad=data.qpos[qadr].tolist(),
                ultrasonic={'fresh':[True,True],'both_fresh':True,'distance_m':[4.,4.]},
                ground_rest_feedback={'fresh':not(fault=='ground' and t>=6.),'body_force_n':body_force,
                    'body_clearance_m':clearance,'base_height_m':float(data.xpos[base,2])})
            if body_force>1.:obs['motion_state']='BODY_SUPPORTED';obs['airborne_seconds']=0.
            if bridge:
                command,state=bridge.update(t,wxyz,gyro,data.qpos[qadr],np.zeros(3),start=t>=3.,send_command=True,
                    accel=accel,forces=forces,ranges=[4.,4.],ground_feedback=None if fault=='ground' and t>=6. else gf,
                    send_imu=not(fault=='imu' and t>=6.),send_joints=not(fault=='joints' and t>=6.))
                if command is None:command=data.qpos[qadr].copy()
                status=bridge.latest_status;drive=status.get('startup_drive',{'servo_enabled':False,'strength':0.})
                failure=bridge.fault
            else:
                control.observe(obs)
                if t>=3.:control.start()
                command=np.array(control.step(.016,np.zeros(3),estimator.rpy[:2],gyro[:2],
                    imu_fresh=not(fault=='imu' and t>=6.),measured_height=float(data.xpos[base,2]),
                    joint_error=float(np.max(abs(data.qpos[qadr]-command)))))
                state=control.state;drive=control.ground_startup.snapshot();failure=control.fault
            data.ctrl[aids]=command
            pulse=pulse_angles(command,reference).tolist() if reference else None
            if pulse is not None:pulses.append(pulse)
            strength=drive['strength'] if drive['servo_enabled'] else 0.
            model.actuator_gainprm[:]=gains*strength;model.actuator_biasprm[:]=bias*strength
            if not states or states[-1]['state']!=state:states.append({'time_s':float(t),'state':state})
            sensors.append({'time_s':float(t),'measured_joints_rad':data.qpos[qadr].tolist(),'commanded_joints_rad':command.tolist(),
                'requested_velocity':[0.,0.,0.],'startup_drive':drive,'ground_rest_feedback':obs['ground_rest_feedback'],
                'contact_forces_n':forces.tolist(),'servo_pulse_angles_deg':pulse,
                'balance_stable_s':control.stable_time if not bridge else None})
            rows.append([float(t),*data.xpos[base].tolist(),*estimator.rpy.tolist(),state,
                float(np.max(np.abs(data.actuator_force[aids]))),body_force,clearance,float(forces.min()),
                bool(drive['servo_enabled']),float(strength)])
        # Same unmeasured speed/torque/current approximation as the action suite.
        servo=json.loads((cfg/'servo_rds5160.json').read_text()) if step==0 else servo
        cap=servo['stall_torque_nm_at_assumed_supply'];no_load=servo['no_load_speed_rad_s_at_assumed_supply']
        limits=cap*np.clip(1.-np.abs(data.qvel[vadr])/no_load,0.,1.)
        strength=drive['strength'] if drive['servo_enabled'] else 0.
        candidate=np.clip(strength*(300.*(data.ctrl[aids]-data.qpos[qadr])-8.*data.qvel[vadr]),-limits,limits)
        amps=float(np.abs(candidate).sum()/cap*5.4)
        scale=min(1.,29.4/max(amps,1e-9));limits=np.minimum(limits,np.abs(candidate)*scale) if scale<1. else limits
        model.actuator_forcerange[aids,0]=-limits;model.actuator_forcerange[aids,1]=limits
        currents.append(0. if not drive['servo_enabled'] else .6+amps*scale)
        mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all():raise RuntimeError('Nonfinite ground dynamics')
        if bridge:
            delay=start+data.time-preroll-time.monotonic()
            if delay>0:time.sleep(delay)
    transport=bridge.close() if bridge else {'ros_executed':False}
    physical=np.array([r[:7] for r in rows]);ready=[s['time_s'] for s in states if s['state']=='READY']
    off=[r for r in rows if r[7]=='GROUND_OFF' and r[0]>=1.]
    success=(bool(ready) and not failure and state=='READY' and max(abs(physical[:,4:6].ravel()))<np.deg2rad(20.) and
        self_force<1. and abs(min_self_distance)<.002 and max(r[8] for r in off)<1e-9 and
        min(r[9] for r in off)>1. and max(abs(r[10]) for r in off)<.002)
    frozen=None
    if fault:
        index=next(i for i,r in enumerate(rows) if r[7]=='FAULT')
        q=np.array([s['commanded_joints_rad'] for s in sensors]);frozen=float(abs(q[index:]-q[index]).max())
        success=state=='FAULT' and bool(failure) and frozen==0. and max(abs(physical[:,4:6].ravel()))<np.deg2rad(20.)
    report={'scenario':'ground_startup','startup_mode':'ground_contact_power_off','engine':'MuJoCo '+mujoco.__version__,
        **transport,'parameters':params,'seconds':23.,'passed':bool(success),'end_state':state,'fault':failure,
        'fallen':bool(max(abs(physical[:,4:6].ravel()))>np.pi/4),'states':states,'action_events':[],
        'servo_off_peak_torque_nm':max(r[8] for r in off),'body_force_while_off_min_n':min(r[9] for r in off),
        'body_clearance_while_off_max_abs_m':max(abs(r[10]) for r in off),
        'off_base_height_mean_m':float(np.mean([r[3] for r in off])),
        'peak_tilt_deg':float(np.rad2deg(abs(physical[:,4:6]).max())),
        'peak_self_contact_force_n':self_force,'minimum_self_contact_distance_m':min_self_distance,
        'self_collision_enabled':True,'adjacent_link_self_collision_filtered':True,
        'ready_at_s':ready[0] if ready else None,'peak_current_a_estimated':max(currents),
        'startup_displacement_xy_m':(physical[-1,1:3]-physical[0,1:3]).tolist(),
        'startup_position_hold_validated':False,
        'fault_command_delta_rad':frozen,'hardware_validated':False,'physical_rest_calibrated':False,
        'motor_model_measured':False,'ideal_orientation_supplied':True,'holdout':holdout,
        'base_pose_writes_after_initialization':False,'external_lifting_forces':False}
    report['minimum_floor_clearance_at_initialization_m']=float(initial_clearance)
    report['torque_off_preroll_seconds']=float(preroll)
    report.update(hardware_reference_used=bool(reference),standing_height_target_m=control.height,
        legacy_servo_input_correspondence_restored=bool(reference),cad_motor_axis_calibrated=False,
        servo_pulse_min_deg=float(np.min(pulses)) if pulses else None,
        servo_pulse_max_deg=float(np.max(pulses)) if pulses else None,
        all_servo_commands_in_range=bool(pulses and np.min(pulses)>=0 and np.max(pulses)<=180),
        hardware_reference=reference)
    folder=ROOT/('simulation/generated/ground_startup' if legacy else 'simulation/generated/hardware_startup');folder.mkdir(parents=True,exist_ok=True)
    case=case or ('ros_' if ros else 'local_')+('holdout_' if holdout else '')+('fault_'+fault if fault else 'ground_startup')
    with (folder/(case+'.csv')).open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['time_s','x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad','state',
            'peak_torque_Nm','body_force_N','body_clearance_m','minimum_foot_force_N','servo_enabled','servo_strength']);writer.writerows(rows)
    (folder/(case+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    (folder/(case+'_sensors.json')).write_text(json.dumps(sensors,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('passed','ros_executed','end_state','fault','servo_off_peak_torque_nm',
        'body_clearance_while_off_max_abs_m','ready_at_s','peak_tilt_deg','peak_self_contact_force_n','peak_current_a_estimated')},indent=2),flush=True)
    if not success:raise RuntimeError('Ground startup acceptance failed: '+case)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ros',action='store_true');p.add_argument('--holdout',action='store_true')
    p.add_argument('--fault',default='',choices=['','imu','joints','ground']);p.add_argument('--legacy',action='store_true')
    a=p.parse_args();run(**vars(a))
