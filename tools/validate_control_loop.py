"""Exercise the actual external controller/effort path against Pi4 physics."""
import argparse
import json
import math
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from bigspot_autonomy.design import load_design,motor_names
from bigspot_autonomy.control_core import ControlCore,JOINT_NAMES
from bigspot_autonomy.sim_backend import CpuPlant
from bigspot_autonomy.navigation import Navigator


def run(case,seconds=18.):
    d=load_design(ROOT/'config/pi4_design.json')
    holdout=case=='holdout'
    nav=Navigator(camera_dropout_at=9. if case=='camera_dropout' else None) if case in ('navigation','camera_dropout') else None
    plant=CpuPlant(d,friction=.55 if holdout else .8,mass_scale=1.15 if holdout else 1.,obstacles=bool(nav))
    core=ControlCore(d,autonomous=bool(nav))
    dt=plant.dt;armed=False;ready=False;fault_at=9.;fault_time=None
    tilt=[];torques=[];states=[];collision=0.;zero_after_fault=True;camera_stopped=False
    for i in range(round(seconds/dt)):
        t=float(plant.data.time)
        if i%5==0:
            q,v,quat,gyro=plant.measurements()
            try:
                if case!='joint_dropout' or t<fault_at:core.joints(list(JOINT_NAMES),q,v,t)
                if case!='imu_dropout' or t<fault_at:core.imu(quat,gyro,t)
            except ValueError:pass
            if not armed and t>=1.:armed=core.enable(True,t)
        if i%3==0:
            velocity=np.array([.24 if t>7. and t<seconds-2. and case!='startup' else 0.,0.,0.])
            if nav:
                velocity=nav(plant.model,plant.data,t)
                if math.isfinite(nav.last_capture):core.supervisor.update('camera',nav.last_capture)
                core.supervisor.update('odom',t)
            if case!='command_dropout' or t<fault_at:core.command(velocity,t)
            result=core.step(dt*3,t)
            ready=ready or result.state=='READY'
            if case!='controller_dropout' or t<fault_at:
                plant.watchdog.receive_target(list(JOINT_NAMES),result.positions.tolist(),result.efforts.tolist() if result.torque_enabled else [],t,t,t)
            if result.state=='FAULT' and fault_time is None:fault_time=t
            if nav and t>9.31 and result.reason=='STALE_CAMERA':camera_stopped=True
        plant.step(t)
        if t>fault_at+.16 and case in ('imu_dropout','joint_dropout','controller_dropout'):
            zero_after_fault=zero_after_fault and max(abs(plant.data.ctrl))==0.
        if t>6. and case not in ('imu_dropout','joint_dropout','controller_dropout'):
            tilt.append(float(np.rad2deg(max(abs(core.rp)))))
        torques.append(plant.data.ctrl.copy());states.append(core.state)
        for ci,c in enumerate(plant.data.contact):
            n=[plant.mj.mj_id2name(plant.model,plant.mj.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in (c.geom1,c.geom2)]
            if any(x.startswith('obstacle_') for x in n):
                force=np.zeros(6);plant.mj.mj_contactForce(plant.model,plant.data,ci,force);collision+=force[0]*dt
    faults=case in ('imu_dropout','joint_dropout','controller_dropout')
    rms=np.sqrt(np.mean(np.asarray(torques)[int(6/dt):]**2,axis=0))
    hold=np.array([d['actuators'][n]['stall_rated_torque_nm'] for n in motor_names(d)])*d['control']['thermal_derating']
    report={'case':case,'engine':'MuJoCo 3.15.0','design':'pi4','controller':'ControlCore targets with local motor PD',
            'ready_reached':ready,'final_state':core.state,'final_xyz_m':plant.data.qpos[:3].tolist(),
            'max_tilt_deg':max(tilt,default=0.),'thermal_margin_ratio':float(max(rms/hold)),
            'fault_detection_s':None if fault_time is None else fault_time-fault_at,
            'fault_efforts_zero':zero_after_fault,'actuator_watchdog_latched':plant.watchdog.expired,
            'obstacle_collision_impulse_ns':collision,'hardware_tested':False,'ros_executed':False,'isaac_executed':False}
    if nav:report['navigation']=nav.report(plant.data)
    if faults:
        report['passed']=ready and zero_after_fault and (plant.watchdog.expired if case=='controller_dropout' else fault_time is not None and fault_time-fault_at<=.153)
    else:
        report['passed']=ready and core.state!='FAULT' and report['max_tilt_deg']<35 and plant.data.qpos[2]>.16 and collision<.5 and report['thermal_margin_ratio']<=1.
        if case in ('walk','holdout'):report['passed'] &= plant.data.qpos[0]>.3
        if case=='navigation':report['passed'] &= nav.reached
        if case=='camera_dropout':report['passed'] &= camera_stopped and np.linalg.norm(core.gait.velocity)<.006
        if case=='command_dropout':report['passed'] &= np.linalg.norm(core.gait.velocity)<.006
    report['passed']=bool(report['passed'])
    report['fault_efforts_zero']=bool(report['fault_efforts_zero'])
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--cases',nargs='+',default=['startup','walk','holdout','imu_dropout','joint_dropout','controller_dropout','command_dropout','navigation','camera_dropout'])
    args=parser.parse_args();reports=[]
    for case in args.cases:
        r=run(case,seconds=45. if case=='navigation' else 18.);reports.append(r);print(json.dumps(r),flush=True)
    output=ROOT/'validation/control_loop.json';output.write_text(json.dumps({'reports':reports,'passed':all(r['passed'] for r in reports)},indent=2)+'\n')
    raise SystemExit(0 if all(r['passed'] for r in reports) else 2)
