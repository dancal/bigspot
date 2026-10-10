"""Physical multi-action tests with real simulated IMU/contact signals and optional ROS.

No external lift forces, motion-capture bodies, base teleportation or playback speedup.
The optional linear motor envelope is an explicit approximation, not identification.
"""
from pathlib import Path
import argparse
import csv
import json
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
if sys.platform=='win32':
    sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
sys.path.insert(0,str(ROOT/'simulation/ros2_ws/src/bigspot_sim'))
import numpy as np
import mujoco
from local_physics import mjcf
from bigspot_sim.motion import MotionController
from bigspot_sim.state_estimator import StateEstimator
from bigspot_sim.awareness import RangePair
from bigspot_sim.poses import POSES
from bigspot_sim.dance import DANCES
from bigspot_sim.expressions import CUTES
from bigspot_sim.clap import CLAPS
from bigspot_sim.clap import SeatedClapTrajectory
from bigspot_sim.poses import support_margin

SCENARIOS={'forward':[.1,0.,0.],'backward':[-.08,0.,0.],
           'left':[0.,.05,0.],'right':[0.,-.05,0.],
           'turn_left':[0.,0.,.4],'turn_right':[0.,0.,-.4],
           'run':[.18,0.,0.], 'accelerate':[.06,0.,0.]}
ADAPTIVE={'smooth_start_stop','smooth_reverse','obstacle_wall','sonar_dropout','sonar_invalid',
          'gyro_fallback','imu_dropout','joint_dropout','pose_gallery','pose_cancel','balance_push'}
DANCE_CHECKS={'dance_cancel','dance_sonar_dropout','dance_imu_dropout','dance_contact_dropout',
              'dance_joint_dropout','dance_buttons','dance_deadman'}
EXPRESSION_CHECKS={'big_cancel','cute_cancel','cute_sonar_dropout','cute_joint_dropout',
                   'cute_imu_dropout','cute_contact_dropout','cute_buttons'}
TEMPO_CHECKS={'rhythm_cancel','rhythm_sonar_dropout','rhythm_joint_dropout',
              'rhythm_imu_dropout','rhythm_contact_dropout','rhythm_buttons','rhythm_deadman','rhythm_joy_dropout',
              'selector_buttons','selector_deadman'}
JOY_CASES={'buttons','dance_buttons','dance_deadman','cute_buttons',
           'rhythm_buttons','rhythm_deadman','rhythm_joy_dropout','selector_buttons','selector_deadman',
           'clap_buttons','clap_deadman','show_buttons','show_deadman','steps_buttons','steps_deadman'}
CLAP_CHECKS={'clap_cancel','clap_sonar_dropout','clap_joint_dropout','clap_imu_dropout','clap_seat_dropout','clap_contact_dropout','clap_buttons','clap_deadman'}
SHOW_CHECKS={'show_buttons','show_deadman'}
STEP_CHECKS={'steps_buttons','steps_deadman','steps_cancel','steps_sonar_dropout',
             'steps_imu_dropout','steps_joint_dropout','steps_contact_dropout'}
STEP_INTERRUPT_S=13.4  # First paw is fully airborne, already at its tap offset.
FUNK_CHECKS={'funk_buttons','funk_cancel','funk_deadman','funk_sonar_dropout','funk_imu_dropout','funk_joint_dropout','funk_contact_dropout'}
JOY_CASES|={'funk_buttons','funk_deadman'}
FUNK_INTERRUPT_S=15.2  # Full hip wiggle, with the diagonal step still in motion.
RECORDED_PERFORMANCES=DANCES|CUTES|DANCE_CHECKS|EXPRESSION_CHECKS|TEMPO_CHECKS|CLAPS|CLAP_CHECKS|SHOW_CHECKS|STEP_CHECKS|FUNK_CHECKS


def seat_telemetry(model,data,base,geoms,sites):
    """Actual convex-mesh contacts; front paw contact pair enabled separately."""
    base_geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'base_link_collision')
    ground=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'ground')
    seat_force=rear_force=paw_force=other=0.;offset=1.;points=[]
    tips=data.site_xpos[sites[:2]].copy()
    for i,c in enumerate(data.contact):
        force=np.zeros(6);mujoco.mj_contactForce(model,data,i,force)
        n=max(0.,float(force[0]));pair=set(c.geom)
        if pair==set(geoms[:2]):
            paw_force+=n;offset=min(offset,float(np.linalg.norm(c.pos-tips.mean(axis=0))))
        elif ground in pair:
            rear=bool(pair.intersection(geoms[2:]))
            if base_geom in pair:
                local=data.xmat[base].reshape(3,3).T@(c.pos-data.xpos[base])
                if local[0]<-.10:seat_force+=n;rear=True
            if rear:
                rear_force+=n
                if n>.1:points.append(c.pos[:2].copy())
        else:other+=n
    # Monotone convex hull of measured rear-leg and rear-body contact points.
    xy=sorted(set(tuple(p) for p in points))
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    hull=[]
    for p in xy:
        while len(hull)>1 and cross(hull[-2],hull[-1],p)<=0:hull.pop()
        hull.append(p)
    lower=len(hull)
    for p in xy[-2::-1]:
        while len(hull)>lower and cross(hull[-2],hull[-1],p)<=0:hull.pop()
        hull.append(p)
    hull=hull[:-1]
    margin=support_margin(data.subtree_com[base,:2],hull) if len(hull)>=3 else -1.
    seat_telemetry.debug={'com_world':data.subtree_com[base].tolist(),'base_world':data.xpos[base].tolist(),
        'rear_contact_hull':hull}
    return [rear_force,paw_force,float(np.linalg.norm(tips[0]-tips[1])),offset,float(margin),
            float(tips[0,2]),float(tips[1,2]),other,seat_force]


def simulated_ranges(model,data,base):
    """Five rays per assumed cone. This does not model acoustic reflections."""
    body_rotation=data.xmat[base].reshape(3,3)
    output=[]
    for side in [1.,-1.]:
        origin=data.xpos[base]+body_rotation@np.array([.24,side*.055,.04])
        distances=[]
        for offset in np.linspace(-.13,.13,5):
            angle=side*.18+offset
            ray=body_rotation@np.array([np.cos(angle),np.sin(angle),0.])
            geomid=np.array([-1],dtype=np.int32)
            distance=mujoco.mj_ray(model,data,origin,ray,np.array([1,0,0,0,0,0],dtype=np.uint8),1,base,geomid)
            if distance>=0:
                distances.append(distance)
        closest=min(distances) if distances else 4.
        output.append(float(np.clip(closest,.02,4.)))
    return output


def foot_forces(model,data,geoms):
    forces=np.zeros(4)
    for i,c in enumerate(data.contact):
        if not any(model.geom_type[g]==mujoco.mjtGeom.mjGEOM_PLANE for g in c.geom):continue
        force=np.zeros(6)
        mujoco.mj_contactForce(model,data,i,force)
        for j,g in enumerate(geoms):
            if g in c.geom:
                forces[j]+=max(0.,float(force[0]))
    return forces


def episode(scenario,parameters=None,seconds=18.,ros=False,render=False,case=None,
            motor_envelope=True,torque=None,friction=.8,mass_scale=1.,jump_settings=None,
            imu_noise=False,current_limit=30.,stall_current=5.4,drive_kv=8.,profile='motion',
            output_folder='motion',enable_experiments=True,joystick_settings=None):
    folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
    if parameters is None:
        parameters=json.loads((folder/f'control_policy_{profile}.json').read_text())['parameters']
    servo=json.loads((folder/'servo_rds5160.json').read_text(encoding='utf-8'))
    if seconds<15 or (scenario.endswith('_experimental') and seconds<27):
        raise ValueError('Allow at least 15 s for ordinary motion, 27 s for acrobatics recovery')
    if case and not case.replace('_','').replace('-','').isalnum():
        raise ValueError('Case names must contain only letters, numbers, underscores or hyphens')
    if not output_folder.replace('_','').replace('-','').isalnum():
        raise ValueError('Output folder must be a single directory name')
    cap=torque if torque is not None else servo['stall_torque_nm_at_assumed_supply']
    no_load=servo['no_load_speed_rad_s_at_assumed_supply']
    if scenario not in {*SCENARIOS,*MotionController.JUMPS,'bow','crouch','rear_stand_probe','handstand_probe',
                         'turn_left_90','turn_right_90','buttons','rear_stand_experimental','handstand_experimental',
                         *POSES,*ADAPTIVE,*RECORDED_PERFORMANCES,'salute_cancel'}:
        raise ValueError('Unknown scenario')
    path,_=mjcf(True,300,drive_kv)
    assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')}
    xml=ET.fromstring(path.read_text(encoding='utf-8'))
    clap_case=scenario in CLAPS|CLAP_CHECKS
    if clap_case:
        contact=ET.SubElement(xml,'contact')
        ET.SubElement(contact,'pair',geom1='FRF_1_collision',geom2='FLF_1_collision',
            friction='.8 .8 .005 .0001 .0001',solref='.02 1')
    for geom in xml.findall('.//geom'):
        if geom.get('name','').endswith('_collision'):
            geom.set('group','1')
    wall_x=None
    if scenario=='obstacle_wall':
        wall_x=.90
        ET.SubElement(xml.find('worldbody'),'geom',name='front_wall',type='box',size='.05 1 .35',
                      pos='0.95 0 .35',contype='2',conaffinity='1',rgba='.70 .25 .20 1')
    model=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'),assets=assets)
    model.geom_friction[:,0]=friction
    model.body_mass[:]*=mass_scale
    model.body_inertia[:]*=mass_scale
    mujoco.mj_setConst(model,mujoco.MjData(model))
    data=mujoco.MjData(model)
    definition=json.loads((folder/'model.json').read_text())
    control=MotionController(definition,enable_experiments=enable_experiments,**parameters)
    for key,value in (jump_settings or {}).items():
        if key not in {'jump_extension','jump_crouch','jump_push_seconds','jump_shift','jump_joint_rate','jump_retract_height'}:
            raise ValueError('Invalid jump setting')
        setattr(control,key,value)
    estimator=StateEstimator()
    range_pair=RangePair()
    names=[n for leg in control.gait.legs for n in leg.names]
    jids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names]
    qadr=model.jnt_qposadr[jids]
    vadr=model.jnt_dofadr[jids]
    aids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,n+'_drive') for n in names]
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    geoms=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,leg.name+'F_1_collision') for leg in control.gait.legs]
    sites=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,leg.name.lower()+'_foot') for leg in control.gait.legs]
    # Initial pose only. There are no base qpos/qvel writes after this initialization.
    data.qpos[2]=control.crouch_height+.008
    data.qpos[qadr]=control.gait.q.ravel()
    command=control.gait.q.ravel().copy()
    data.ctrl[aids]=command
    mujoco.mj_forward(model,data)
    bridge=None
    if ros:
        from ros_physics_bridge import Bridge
        if scenario in TEMPO_CHECKS and scenario.startswith('rhythm_'):
            joystick_settings={'dance_action':'dance_rhythm',**(joystick_settings or {})}
        elif scenario in SHOW_CHECKS:
            joystick_settings={'dance_action':'dance_show',**(joystick_settings or {})}
        elif scenario in ('dance_buttons','dance_deadman'):
            joystick_settings={'dance_action':'dance',**(joystick_settings or {})}
        elif scenario in STEP_CHECKS:
            joystick_settings={'dance_action':'dance_steps',**(joystick_settings or {})}
        bridge=Bridge(names,parameters,enable_experiments=enable_experiments,
                      use_joystick=scenario in JOY_CASES,joystick_settings=joystick_settings)
    camera=mujoco.MjvCamera()
    camera.distance=1.25
    camera.azimuth=130
    if scenario=='obstacle_wall':
        camera.azimuth=310  # Keep the viewing camera on the robot side of the wall.
    camera.elevation=-18
    if scenario in ('dance_show','dance_steps') or clap_case:
        camera.lookat[:]=[-.10 if clap_case else 0.,0.,.22]
        camera.azimuth=215 if clap_case else 130
    renderer=mujoco.Renderer(model,height=480,width=640) if render else None
    frames=[]
    rows=[]
    states=[]
    event_sent=False
    gallery_index=0
    adaptive_rows=[]
    wall_contact_steps=0
    pose_tracking=[]
    sent_events=[]
    next_range=0.
    started=time.monotonic()
    rng=np.random.default_rng(61)
    current_rows=[]
    current_limited_steps=0
    funk_case=scenario=='dance_funk' or scenario in FUNK_CHECKS
    floor_air_s=floor_air_max_s=0.
    dt=model.opt.timestep
    for step in range(round(seconds/dt)):
        t=data.time
        velocity=np.array(SCENARIOS.get(scenario,[0.,0.,0.])) if 8<=t<seconds-4 else np.zeros(3)
        if scenario=='accelerate' and 11<=t<seconds-4:
            velocity[0]=.15
        if scenario in ADAPTIVE:
            moving=scenario not in ('pose_gallery','pose_cancel')
            velocity=np.array([.15,0.,0.]) if moving and 8<=t<seconds-4 else np.zeros(3)
            if scenario=='smooth_reverse' and 11<=t<15:
                velocity[0]=-.10
        if step%8==0:
            joy=None
            if scenario=='buttons':
                if not bridge:
                    raise ValueError('Button transport test requires actual ROS')
                axes=[0.]*4
                buttons=[0]*10
                buttons[0]=int(1<=t<1.1)
                if 8<=t<14:
                    axes[1]=1.
                    buttons[5]=1
                buttons[4]=int(10<=t<12)
                buttons[3]=int(12<=t<12.1)
                buttons[1]=int(14<=t<14.1)
                joy=axes,buttons
            elif scenario in ('dance_buttons','dance_deadman','cute_buttons','rhythm_buttons','rhythm_deadman','rhythm_joy_dropout','show_buttons','show_deadman','clap_buttons','clap_deadman','steps_buttons','steps_deadman','funk_buttons','funk_deadman'):
                if not bridge:
                    raise ValueError('Dance button test requires actual ROS')
                buttons=[0]*14
                buttons[0]=int(1<=t<1.1)
                end=19. if scenario=='clap_deadman' else FUNK_INTERRUPT_S if scenario=='funk_deadman' else STEP_INTERRUPT_S if scenario=='steps_deadman' else seconds-1 if scenario in ('clap_buttons','steps_buttons','funk_buttons') else 12.
                buttons[5]=int(8<=t<end)
                buttons[8 if scenario.startswith('clap_') else 13 if scenario=='cute_buttons' else 12]=int(8.5<=t<11.5)
                buttons[1]=int(scenario in ('dance_buttons','cute_buttons','rhythm_buttons','show_buttons') and 12<=t<12.1)
                joy=[0.]*4,buttons
                if scenario=='rhythm_joy_dropout' and t>=12.:
                    joy=None  # No release message: the wall-clock Joy watchdog must cancel.
            elif scenario in ('selector_buttons','selector_deadman'):
                from bigspot_sim.joy_mapping import SELECTABLE
                buttons=[0]*14
                buttons[0]=int(1<=t<1.1)
                buttons[5]=int(8<=t<(16. if scenario=='selector_deadman' else 27.))
                if 8<=t<10.4:
                    cycle=int((t-8.)/.24)
                    buttons[4]=1
                    buttons[13]=int(cycle<SELECTABLE.index('cute_wave') and (t-8.)% .24<.1)
                if 10.8<=t<11.3:
                    buttons[4]=buttons[6]=1  # Hold execute: one action.
                joy=[0.]*4,buttons
            w,x,y,z=data.sensor('imu_orientation').data.copy()
            gyro=data.sensor('imu_gyro').data.copy()
            accel=data.sensor('imu_accel').data.copy()
            if imu_noise:
                gyro+=rng.normal(0,.015,3)
                accel+=rng.normal(0,.12,3)
            forces=foot_forces(model,data,geoms)
            all_ground_force=0.
            for contact_i,contact in enumerate(data.contact):
                if not any(model.geom_type[g]==mujoco.mjtGeom.mjGEOM_PLANE for g in contact.geom):continue
                contact_force=np.zeros(6)
                mujoco.mj_contactForce(model,data,contact_i,contact_force)
                all_ground_force+=max(0.,float(contact_force[0]))
            estimator.contacts(forces,t)
            rp=estimator.update([x,y,z,w],gyro,accel,t)[:2]
            observation=estimator.snapshot(t)
            observation['joint_feedback_fresh']=True
            observation['measured_joint_angles_rad']=data.qpos[qadr].tolist()
            seat_data=seat_telemetry(model,data,base,geoms,sites) if clap_case else None
            if seat_data is not None:
                observation['seat_feedback']={'fresh':True,'rear_support_force_n':seat_data[0],'rear_body_force_n':seat_data[8],
                    'paw_contact_force_n':seat_data[1],'paw_gap_m':seat_data[2],
                    'paw_contact_tip_offset_m':seat_data[3],'rear_support_margin_m':seat_data[4],
                    'front_paw_height_m':seat_data[5:7],'other_self_contact_force_n':seat_data[7]}
            ranges=None
            if t>=next_range:
                next_range=t+.128
                ranges=simulated_ranges(model,data,base)
                if (scenario in ('sonar_dropout','dance_sonar_dropout','rhythm_sonar_dropout') and 10<=t<seconds) or (scenario=='steps_sonar_dropout' and t>=STEP_INTERRUPT_S) or (scenario=='funk_sonar_dropout' and t>=FUNK_INTERRUPT_S):
                    ranges=[None,ranges[1]]
                if scenario=='cute_sonar_dropout' and t>=14.:
                    ranges=[None,ranges[1]]
                if scenario=='clap_sonar_dropout' and t>=19.:
                    ranges=[None,ranges[1]]
                if scenario=='sonar_invalid' and 10<=t<seconds:
                    ranges=[float('nan'),ranges[1]]
                for side,distance in enumerate(ranges):
                    if distance is not None:
                        range_pair.update(side,distance,t,t)
            observation['ultrasonic']=range_pair.snapshot(t)
            action=None
            if scenario=='pose_gallery' and gallery_index<len(POSES) and t>=8.5+7.5*gallery_index:
                action=list(POSES)[gallery_index]
                gallery_index+=1
            elif scenario=='pose_cancel' and 8.5<=t and not event_sent:
                action='look_down'
                event_sent=True
            elif scenario=='pose_cancel' and 10.5<=t and gallery_index==0:
                action='stop'
                gallery_index+=1
            elif scenario=='salute_cancel' and t>=11. and gallery_index==0:
                action='stop';gallery_index+=1
            elif scenario in ('dance_cancel','rhythm_cancel') and 12<=t and gallery_index==0:
                action='stop'
                gallery_index+=1
            elif scenario in ('big_cancel','cute_cancel') and t>=(12. if scenario=='big_cancel' else 14.) and gallery_index==0:
                action='stop'
                gallery_index+=1
            elif scenario=='clap_cancel' and t>=19. and gallery_index==0:
                action='stop';gallery_index+=1
            elif scenario=='steps_cancel' and t>=STEP_INTERRUPT_S and gallery_index==0:
                action='stop';gallery_index+=1
            elif scenario=='funk_cancel' and t>=FUNK_INTERRUPT_S and gallery_index==0:
                action='stop';gallery_index+=1
            elif t>=8.5 and not event_sent and scenario not in SCENARIOS and scenario not in JOY_CASES and scenario not in ADAPTIVE:
                action=('salute_right' if scenario=='salute_cancel' else 'dance_big' if scenario=='big_cancel' else 'cute_wave' if scenario in EXPRESSION_CHECKS else
                        'dance_funk' if scenario in FUNK_CHECKS else 'dance_steps' if scenario in STEP_CHECKS else 'seated_clap' if scenario in CLAP_CHECKS else 'dance_rhythm' if scenario in TEMPO_CHECKS else 'dance' if scenario in DANCE_CHECKS else scenario.replace('_90',''))
                event_sent=True
            elif t>=8. and not event_sent and scenario=='run':
                action='run'
                event_sent=True
            if bridge:
                command,state=bridge.update(t,[w,x,y,z],gyro,data.qpos[qadr],velocity,
                    start=t>=1. and scenario not in JOY_CASES,
                    send_command=scenario not in JOY_CASES,
                    accel=accel,forces=None if (scenario in ('dance_contact_dropout','rhythm_contact_dropout') and t>=10) or (scenario=='cute_contact_dropout' and t>=14) or (scenario=='clap_contact_dropout' and t>=19) or (scenario=='steps_contact_dropout' and t>=STEP_INTERRUPT_S) or (scenario=='funk_contact_dropout' and t>=FUNK_INTERRUPT_S) else forces,
                    action=action,joy=joy,ranges=ranges,
                    orientation_available=scenario!='gyro_fallback',
                    send_imu=not ((scenario in ('imu_dropout','dance_imu_dropout','rhythm_imu_dropout') and t>=10.) or
                                  (scenario=='cute_imu_dropout' and t>=14.) or (scenario=='clap_imu_dropout' and t>=19.) or (scenario=='steps_imu_dropout' and t>=STEP_INTERRUPT_S) or (scenario=='funk_imu_dropout' and t>=FUNK_INTERRUPT_S)),
                    send_joints=not ((scenario in ('joint_dropout','dance_joint_dropout','rhythm_joint_dropout') and t>=10.) or
                                     (scenario=='cute_joint_dropout' and t>=14.) or (scenario=='clap_joint_dropout' and t>=19.) or (scenario=='steps_joint_dropout' and t>=STEP_INTERRUPT_S) or (scenario=='funk_joint_dropout' and t>=FUNK_INTERRUPT_S)),
                    seat_feedback=None if scenario=='clap_seat_dropout' and t>=19. else seat_data)
                if command is None:
                    command=data.ctrl[aids].copy()
                status=bridge.latest_status
                if scenario in JOY_CASES:
                    velocity=bridge.controller.velocity.copy()
            else:
                control.observe(observation)
                if t>=1.:
                    control.start()
                if action:
                    control.request(action)
                try:
                    command=np.array(control.step(.016,velocity,rp,gyro[:2],
                         measured_height=float(data.xpos[base,2]),
                         joint_error=float(np.max(np.abs(data.qpos[qadr]-command)))))
                except ValueError as exc:
                    control.fault,control.state=str(exc),'FAULT'
                    control.action=''
                state=control.state
                status={'state':state,'fault':control.fault,'action_result':control.action_result,
                        'action_events':control.action_events,'estimator':observation,
                        'sensor_constraints':control.constraints.status,
                        'velocity_profile':{'velocity':control.gait.speed.tolist(),
                            'acceleration':control.gait.speed_acceleration.tolist(),'jerk':control.gait.speed_jerk.tolist()},
                        'pose_target_rpy_rad':getattr(control,'pose_last_rpy',np.zeros(3)).tolist()}
                if clap_case:status['clap']={'count':getattr(control,'clap_count',0),'elapsed_s':control.action_time}
                status['dance']=control.dance_status()
            if action:
                sent_events.append({'time_s':t,'action':action})
            vp=status.get('velocity_profile',{})
            awareness=status.get('sensor_constraints',{})
            target=status.get('pose_target_rpy_rad',[0,0,0])
            adaptive_rows.append({'time_s':t,'requested_velocity':velocity.tolist(),
                'control_time_s':status.get('control_time_s',t),
                'velocity':vp.get('velocity',[0,0,0]),'acceleration':vp.get('acceleration',[0,0,0]),
                'jerk':vp.get('jerk',[0,0,0]),'sonar':status.get('estimator',observation).get('ultrasonic',{}),
                'constraints':awareness,'target_rpy_rad':target})
            adaptive_rows[-1]['estimated_rpy_rad']=status.get('estimator',observation).get('rpy_rad',estimator.rpy.tolist())
            adaptive_rows[-1]['orientation_source']=status.get('estimator',observation).get('orientation_source','unknown')
            adaptive_rows[-1]['commanded_joints_rad']=np.asarray(command).tolist()
            adaptive_rows[-1]['measured_joints_rad']=data.qpos[qadr].tolist()
            if scenario in RECORDED_PERFORMANCES:
                adaptive_rows[-1].update({'dance':status.get('dance'),
                    'measured_foot_positions_world_m':data.site_xpos[sites].tolist()})
            if clap_case:
                adaptive_rows[-1]['seat']=seat_data
                adaptive_rows[-1]['clap']=status.get('clap')
                adaptive_rows[-1]['seat_geometry']=seat_telemetry.debug
            if state in ('POSE','DANCE','CUTE'):
                pose_tracking.append({'time_s':t,'target':target,'actual':estimator.rpy.tolist(),
                                      'action':bridge.controller.posture.action if bridge else control.action})
            data.ctrl[aids]=command
            if not states or states[-1]['state']!=state:
                states.append({'time_s':float(t),'state':state})
            rows.append([t,*data.xpos[base].tolist(),*estimator.rpy.tolist(),
                         *gyro.tolist(),*accel.tolist(),*forces.tolist(),
                         float(np.max(np.abs(data.actuator_force))),float(np.max(np.abs(data.qvel[vadr]))),
                         float(np.max(np.abs(data.qpos[qadr]-command))),
                         *data.site_xpos[sites,2].tolist(),*velocity.tolist(),all_ground_force,
                         int(data.ncon),float(data.qvel[2]),state,
                         status.get('action_result',''),observation['motion_state']])
        if motor_envelope:
            # Conservative symmetric speed-dependent force ceiling; braking and
            # electronics are not identified. Battery droop/current/temperature omitted.
            limits=cap*np.clip(1-np.abs(data.qvel[vadr])/no_load,0.,1.)
        else:
            limits=np.full(12,cap)
        requested=300*(data.ctrl[aids]-data.qpos[qadr])-drive_kv*data.qvel[vadr]
        candidate=np.clip(requested,-limits,limits)
        # Unmeasured torque/current proportional model, with 50mA allowance per
        # servo. 5.4A stall is a sensitivity assumption, not verified unit data.
        motor_current=float(np.sum(np.abs(candidate))/cap*stall_current)
        scale=min(1.,max(0.,current_limit-.6)/max(motor_current,1e-9)) if current_limit else 1.
        if scale<1.:
            current_limited_steps+=1
            limits=np.minimum(limits,np.abs(candidate)*scale)
        current_rows.append(.6+motor_current*scale)
        model.actuator_forcerange[aids,0]=-limits
        model.actuator_forcerange[aids,1]=limits
        data.xfrc_applied[:]=0.
        if scenario=='balance_push' and 10.5<=t<10.65:
            data.xfrc_applied[base,1]=20.
        mujoco.mj_step(model,data)
        if funk_case and t>=8.5:
            floor_force=0.
            for contact_i,contact in enumerate(data.contact):
                if not any(model.geom_type[g]==mujoco.mjtGeom.mjGEOM_PLANE for g in contact.geom):continue
                contact_force=np.zeros(6);mujoco.mj_contactForce(model,data,contact_i,contact_force)
                floor_force+=max(0.,float(contact_force[0]))
            floor_air_s=floor_air_s+dt if floor_force<1. else 0.
            floor_air_max_s=max(floor_air_max_s,floor_air_s)
        if wall_x is not None:
            wallid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'front_wall')
            wall_contact_steps+=int(any(wallid in contact.geom for contact in data.contact))
        if not np.isfinite(data.qpos).all():
            raise RuntimeError('Nonfinite physical state')
        if renderer and step%(50 if profile=='adaptive' else 25)==0:
            if scenario not in ('dance_show','dance_steps') and not clap_case:camera.lookat[:]=data.xpos[base]
            renderer.update_scene(data,camera=camera)
            from PIL import Image,ImageDraw,ImageFont
            frame=Image.fromarray(renderer.render().copy())
            draw=ImageDraw.Draw(frame)
            font_path=Path('C:/Windows/Fonts/arial.ttf') if sys.platform=='win32' else Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
            font=ImageFont.truetype(str(font_path),16) if font_path.exists() else ImageFont.load_default()
            draw.rectangle((0,0,640,75),fill=(15,18,22))
            draw.text((12,8),f'BigSpot | {scenario.upper()} | {state} | {t:.2f}s',font=font,fill='white')
            draw.text((12,31),f'RDS5160 x12 | 7.4V / 30A | '+('ROS 2 + MuJoCo' if ros else 'MuJoCo CPU'),font=font,fill='white')
            if adaptive_rows and profile=='adaptive':
                last=adaptive_rows[-1]
                distances=last['sonar'].get('distance_m',[None,None])
                label=' / '.join(f'{v:.2f}m' if v is not None else 'LOST' for v in distances)
                draw.text((12,53),f'Sonar L/R: {label} | cmd {last["velocity"][0]:+.3f}m/s | normal time',font=font,fill=(190,220,190))
            else:
                draw.text((12,53),'Unmeasured torque-speed approximation | normal time',font=font,fill=(190,190,190))
            draw.rectangle((0,423,640,480),fill=(15,18,22))
            draw.text((12,429),f'IMU R/P: {rp[0]*180/np.pi:+.1f}/{rp[1]*180/np.pi:+.1f} deg | '+observation['motion_state'],font=font,fill='white')
            for i,leg in enumerate(control.gait.legs):
                draw.text((12+i*155,453),f'{leg.name}: {forces[i]:.1f} N',font=font,fill=(185,210,235))
            frames.append(frame)
        if bridge:
            delay=started+data.time-time.monotonic()
            if delay>0:
                time.sleep(delay)
    transport=bridge.close() if bridge else {'ros_executed':False}
    a=np.array([row[:30] for row in rows],dtype=float)
    # Columns t xyz rpy gyro accel forces(4) torque qspeed error feetz(4) cmd(3).
    after=a[a[:,0]>=8]
    active=a[(a[:,0]>=9)&(a[:,0]<seconds-4)]
    contact_count=(after[:,13:17]>1.).sum(1)
    longest=0
    current=0
    for airborne in contact_count==0:
        current=current+1 if airborne else 0
        longest=max(longest,current)
    delta=after[-1,1:4]-after[0,1:4]
    yaw=np.unwrap(after[:,6])
    acrobatics=scenario.endswith('_experimental')
    fallen=bool(np.any(after[:,3]<.12) or np.any(np.abs(after[:,4])>np.pi/4)
                or (not acrobatics and np.any(np.abs(after[:,5])>np.pi/4)))
    report={'scenario':scenario,**transport,'engine':f'MuJoCo {mujoco.__version__}',
            'seconds':seconds,'parameters':parameters,'servo_model':'RDS5160','servo_count':12,
            'assumed_voltage_v':7.4,'torque_stall_cap_nm':cap,'no_load_speed_rad_s':no_load,
            'linear_motor_envelope':motor_envelope,'hardware_validated':False,'drive_kv_unmeasured':drive_kv,
            'supply_current_limit_a':current_limit,'stall_current_a_assumed':stall_current,
            'peak_current_a_estimated':float(max(current_rows)),
            'current_limit_active_fraction':current_limited_steps/len(current_rows),
            'mass_kg_assumed':float(model.body_mass.sum()),'friction_assumed':friction,
            'self_collision_enabled':False,'imu_noise':imu_noise,
            'displacement_after_action_m':delta.tolist(),'yaw_change_deg':float((yaw[-1]-yaw[0])*180/np.pi),
            'max_tilt_deg':float(np.max(np.abs(after[:,4:6]))*180/np.pi),
            'max_airborne_interval_s':longest*.016,
            'peak_base_height_m':float(after[:,3].max()),
            'lowest_airborne_sole_height_m':float(np.min(after[contact_count==0,20:24])) if np.any(contact_count==0) else None,
            'peak_torque_nm':float(after[:,17].max()),'peak_joint_speed_rad_s':float(after[:,18].max()),
            'mean_velocity_m_s':((active[-1,1:3]-active[0,1:3])/(active[-1,0]-active[0,0])).tolist(),
            'end_state':state,'fallen':fallen,'fault':status.get('fault',''),
            'action_result':status.get('action_result',''),'action_events':status.get('action_events',[]),
            'states':states,'jump_settings':jump_settings or {},
            'imu_model':'MuJoCo local-frame gyroscope and specific-force accelerometer; ideal orientation supplied',
            'contact_model':'MuJoCo foot-ground normal force >1 N; simulation-only contact feedback'}
    report.update({'profile':profile,'sent_action_events':sent_events,
        'ultrasonic_model':'5 rays per front cone, assumed mounts [.24,+/- .055,.04], yaw +/- .18 rad; no acoustic model',
        'ultrasonic_mount_measured':False,'wall_contact_steps':wall_contact_steps,
        'sensor_constraints_final':adaptive_rows[-1]['constraints'],
        'peak_command_acceleration':np.max(np.abs([r['acceleration'] for r in adaptive_rows]),axis=0).tolist(),
        'peak_command_jerk':np.max(np.abs([r['jerk'] for r in adaptive_rows]),axis=0).tolist(),
        'sensor_constraint_reasons':sorted({reason for r in adaptive_rows for reason in r['constraints'].get('reasons',[])})})
    report['sensor_constraint_reasons_after_start']=sorted({reason for r in adaptive_rows if r['time_s']>=8 for reason in r['constraints'].get('reasons',[])})
    attitude_errors=[np.asarray(r['estimated_rpy_rad'])[:2]-row[4:6] for r,row in zip(adaptive_rows,a) if r['time_s']>=8]
    report['estimated_attitude_rms_deg']=float(np.sqrt(np.mean(np.square(attitude_errors)))*180/np.pi)
    report['orientation_provided_to_ros']=scenario!='gyro_fallback'
    report['external_disturbance']='20 N lateral force, t=10.5..10.65 s' if scenario=='balance_push' else None
    if funk_case:
        report['max_zero_ground_force_interval_s']=floor_air_max_s
        if scenario in FUNK_CHECKS and scenario!='funk_buttons':report['funk_interrupt_at_s']=FUNK_INTERRUPT_S
    if scenario in RECORDED_PERFORMANCES:
        danced=a[np.array([row[30] in ('DANCE','CUTE') for row in rows])]
        report['dance_bpm']=control.tempo(scenario if scenario in DANCES else 'dance_funk' if scenario in FUNK_CHECKS else 'dance_steps' if scenario in STEP_CHECKS else 'dance_rhythm' if scenario in TEMPO_CHECKS else 'dance')
        report['dance_support_min_count']=int(np.min(np.sum(danced[:,13:17]>1.,axis=1))) if len(danced) else None
        report['dance_measured_height_range_m']=float(np.ptp(danced[:,3])) if len(danced) else 0.
        report['measured_body_xy_range_m']=np.ptp(danced[:,1:3],axis=0).tolist() if len(danced) else [0.,0.]
        report['dance_measured_roll_pitch_range_deg']=(np.ptp(danced[:,4:6],axis=0)*180/np.pi).tolist() if len(danced) else [0.,0.]
        report['dance_success']=bool((scenario in DANCES or scenario=='steps_buttons') and not fallen and not report['fault'] and state=='READY'
            and report['action_events'] and report['action_events'][-1]['result']=='Completed sensor-settled dance'
            and report['dance_support_min_count']>=3
            and (report['dance_measured_height_range_m']>.008 or max(report['dance_measured_roll_pitch_range_deg'])>1.))
        if scenario in CUTES:
            foot_peak=float(np.max(danced[:,20])) if len(danced) else 0.
            wave_seconds=float(np.sum((danced[:,20]>.04)&(np.min(danced[:,14:17],axis=1)>1.))*.016) if len(danced) else 0.
            report.update({'front_paw_peak_clearance_m':foot_peak,'three_stance_feet_paw_wave_seconds':wave_seconds,
                'cute_success':bool(not fallen and not report['fault'] and state=='READY' and report['action_events']
                    and report['action_events'][-1]['result']=='Completed sensor-settled gesture'
                    and report['dance_support_min_count']>=3
                    and (scenario not in ('cute','cute_wave') or (foot_peak>.04 and wave_seconds>.5)))})
        if scenario=='dance_steps' or scenario in STEP_CHECKS:
            measured=[]
            for foot in range(4):
                indices=[i for i,s in enumerate(adaptive_rows) if
                         (s.get('dance') or {}).get('planned_lift_foot')==foot and rows[i][30]=='DANCE']
                xyz=np.array([adaptive_rows[i]['measured_foot_positions_world_m'][foot] for i in indices])
                f=a[indices,13:17]
                clear=(xyz[:,2]>.025)&(f[:,foot]<.2)&(np.min(np.delete(f,foot,axis=1),axis=1)>1.) if len(xyz) else []
                measured.append({'foot':['FR','FL','RR','RL'][foot],
                    'peak_clearance_m':float(xyz[:,2].max()) if len(xyz) else 0.,
                    'xy_travel_range_m':np.ptp(xyz[:,:2],axis=0).tolist() if len(xyz) else [0.,0.],
                    'clear_swing_with_three_supports_s':float(np.sum(clear))*.016})
            landings=max(((s.get('dance') or {}).get('measured_landings',[]) for s in adaptive_rows),key=len,default=[])
            report.update({'measured_dance_steps':measured,'measured_step_landings':landings,
                'step_dance_success':bool(report['dance_success'] and len(landings)==4 and
                    all(s['clear_swing_with_three_supports_s']>.25 and max(s['xy_travel_range_m'])>.025 for s in measured))})
        if funk_case:
            from funk_metrics import measured_funk
            report.update(measured_funk(adaptive_rows,rows))
            interval=report['median_pair_step_interval_s']
            report['funk_dance_success']=bool(not fallen and not report['fault'] and state=='READY'
                and report['action_events'] and report['action_events'][-1]['result']=='Completed sensor-settled funk dance'
                and report['measured_diagonal_pair_steps']>=30 and interval is not None and .42<=interval<=.58
                and report['dynamic_two_or_more_support_fraction']>=.95 and floor_air_max_s<=.032
                and report['max_tilt_deg']<12)
            if scenario=='dance_funk':report['dance_success']=report['funk_dance_success']
    if clap_case:
        contact_count=max((row.get('clap') or {}).get('count',0) for row in adaptive_rows)
        clap_rows=[row for row,physical in zip(adaptive_rows,rows) if row.get('clap') and physical[30]=='SEATED_CLAP'
            and SeatedClapTrajectory.clap_start<=row['clap']['elapsed_s']<SeatedClapTrajectory.landing_start]
        report['measured_paw_clap_count']=contact_count
        report['paw_contact_pair_enabled']=True
        report['other_self_collision_enabled']=False
        report['rear_support_peak_force_n']=max(row['seat'][0] for row in adaptive_rows)
        report['rear_body_contact_peak_force_n']=max(row['seat'][8] for row in adaptive_rows)
        report['paw_contact_peak_force_n']=max(row['seat'][1] for row in adaptive_rows)
        report['front_paw_min_gap_m']=min(row['seat'][2] for row in adaptive_rows)
        report['rear_support_min_margin_during_clap_m']=min((row['seat'][4] for row in clap_rows),default=None)
        report['rear_support_min_force_during_clap_n']=min((row['seat'][0] for row in clap_rows),default=None)
        report['front_paw_min_height_during_clap_m']=min((min(row['seat'][5:7]) for row in clap_rows),default=None)
        report['seated_clap_success']=bool(scenario in CLAPS|{'clap_buttons'} and not fallen and not report['fault'] and state=='READY'
            and contact_count>=3 and any(event.get('action')=='seated_clap' and
                event.get('result')=='Completed seated clap; three measured paw contacts' for event in report['action_events']))
    if pose_tracking:
        report['pose_tracking_rms_deg']=float(np.sqrt(np.mean([(np.asarray(p['actual'])[:2]-np.asarray(p['target'])[:2])**2 for p in pose_tracking]))*180/np.pi)
        report['pose_max_tracking_error_deg']=float(np.max(np.abs([(np.asarray(p['actual'])[:2]-np.asarray(p['target'])[:2]) for p in pose_tracking]))*180/np.pi)
    if wall_x is not None:
        report['final_front_sonar_clearance_m']=adaptive_rows[-1]['sonar'].get('distance_m')
        stopped_while_requested=[float(row[0]) for row in rows if 12<=row[0]<seconds-4 and row[30]=='READY']
        report['obstacle_stopped_while_forward_requested']=bool(stopped_while_requested)
        report['first_ready_before_operator_stop_s']=min(stopped_while_requested) if stopped_while_requested else None
        report['obstacle_stop_success']=bool(not wall_contact_steps and not fallen and not report['fault']
                                              and state=='READY' and stopped_while_requested)
    if scenario in MotionController.JUMPS:
        direction=MotionController.JUMPS[scenario]
        report['directional_displacement_m']=float(delta[:2]@direction)
        report['peak_minimum_sole_clearance_m']=float(np.max(np.min(after[:,20:24],axis=1)))
        physical_flight=(after[:,27]<.1)&(np.min(after[:,20:24],axis=1)>.01)
        run=longest_physical=0
        for airborne in physical_flight:
            run=run+1 if airborne else 0
            longest_physical=max(run,longest_physical)
        report['strict_airborne_interval_s']=longest_physical*.016
        report['jump_success']=bool(longest_physical>=4
                                    and not fallen and not report['fault'] and state=='READY'
                                    and (np.linalg.norm(direction)==0 or delta[:2]@direction>.01)
                                    and 'Landed after flight' in report['action_result'])
    if acrobatics:
        stance=[2,3] if scenario=='rear_stand_experimental' else [0,1]
        lifted=[i for i in range(4) if i not in stance]
        mask=(np.abs(after[:,5])>np.deg2rad(65))&(np.min(after[:,13+np.array(stance)],axis=1)>1.)&(np.max(after[:,13+np.array(lifted)],axis=1)<1.)
        report['measured_two_leg_near_vertical_seconds']=float(mask.sum()*.016)
        report['acrobatics_success']=bool(mask.sum()>=32 and not fallen and not report['fault'] and state=='READY')
    if case:
        output=ROOT/'simulation/generated'/output_folder
        output.mkdir(parents=True,exist_ok=True)
        header=['time_s','x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad',
                'gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z',
                'FR_force_N','FL_force_N','RR_force_N','RL_force_N',
                'peak_torque_Nm','peak_qspeed_rad_s','joint_error_rad',
                'FR_sole_z_m','FL_sole_z_m','RR_sole_z_m','RL_sole_z_m',
                'cmd_x','cmd_y','cmd_yaw','total_ground_force_N','engine_contact_count','vertical_velocity_m_s',
                'state','action_result','contact_state']
        with (output/(case+'.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)
        (output/(case+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        (output/(case+'_sensors.json')).write_text(json.dumps(adaptive_rows,indent=2)+'\n',encoding='utf-8')
        if frames:
            frames[0].save(output/(case+'.gif'),save_all=True,append_images=frames[1:],duration=100 if profile=='adaptive' else 50,loop=0)
            frames[-1].save(output/(case+'.png'))
    if renderer:
        renderer.close()
    return report


def default_episode_seconds(scenario,dance_bpm=60.):
    """Include startup, the complete routine and its sensor-settling allowance."""
    if scenario in ('dance_funk','funk_buttons'):return 50.
    if scenario in ('dance_steps','steps_buttons'):return 49.
    if scenario in ('seated_clap','clap_buttons'):return 59.
    if scenario=='cute':return 47.
    if scenario in CUTES:return 28.
    if scenario=='dance_show':return 37.
    if scenario=='dance_rhythm':return 32.
    if scenario=='dance_big':return max(53.,float(np.ceil(21.+1920./dance_bpm)))
    if scenario in DANCES:return max(36.,float(np.ceil(19.+960./dance_bpm)))
    return 30. if scenario.endswith('_experimental') else 24.


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',default='forward')
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--case')
    parser.add_argument('--constant-torque',action='store_true')
    parser.add_argument('--torque',type=float)
    parser.add_argument('--noise',action='store_true')
    parser.add_argument('--dance-bpm',type=float)
    parser.add_argument('--profile',choices=['motion','adaptive'],default='motion')
    parser.add_argument('--output-folder',default='motion')
    args=parser.parse_args()
    parameters=None
    if args.dance_bpm is not None:
        parameters=json.loads((ROOT/f'simulation/ros2_ws/src/bigspot_sim/config/control_policy_{args.profile}.json').read_text())['parameters']
        parameters['dance_bpm']=args.dance_bpm
    seconds=args.seconds if args.seconds is not None else default_episode_seconds(args.scenario,args.dance_bpm or 60.)
    print(json.dumps(episode(args.scenario,parameters=parameters,seconds=seconds,ros=args.ros,render=args.render,
            case=args.case,motor_envelope=not args.constant_torque,torque=args.torque,imu_noise=args.noise,
            profile=args.profile,output_folder=args.output_folder),indent=2),flush=True)
