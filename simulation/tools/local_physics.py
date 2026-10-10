"""Local MuJoCo CPU physics using the same geometry, joints and inertia as ROS URDF.

AMD-compatible local alternative; this does NOT execute Isaac ROS or Isaac Sim.
Mesh collisions use convex hulls per link and self collision is disabled, as in the
initial Isaac scene. Masses, drive gains, friction and servo force limits are unmeasured.
"""
from pathlib import Path
import argparse
import csv
import json
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
if sys.platform == 'win32':
    sys.path.insert(0,str(ROOT/'simulation/generated/python_deps'))
PKG=ROOT/'simulation/ros2_ws/src/bigspot_sim'
sys.path.insert(0,str(PKG))
import numpy as np
import mujoco
from bigspot_sim.kinematics import Gait


def xyz(element, key='xyz'):
    return element.get(key,'0 0 0') if element is not None else '0 0 0'


def mjcf(floating=True,kp=40,kv=2):
    robot=ET.parse(PKG/'urdf/bigspot.urdf').getroot()
    model=json.loads((PKG/'config/model.json').read_text())
    gait=Gait(model)
    root=ET.Element('mujoco',model='BigSpot CPU validation')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',autolimits='true')
    ET.SubElement(root,'option',timestep='0.002',gravity='0 0 -9.81',integrator='implicitfast',iterations='100')
    asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'texture',name='floor_checker',type='2d',builtin='checker',
                  rgb1='.22 .24 .27',rgb2='.30 .32 .35',width='512',height='512')
    ET.SubElement(asset,'material',name='floor_material',texture='floor_checker',texrepeat='12 12',texuniform='true')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',diffuse='.8 .8 .8')
    ET.SubElement(world,'light',pos='-1 1 2',dir='1 -1 -2',diffuse='.5 .5 .5')
    ET.SubElement(world,'geom',name='ground',type='plane',size='3 3 .1',rgba='.22 .24 .27 1',
                  contype='2',conaffinity='1',friction='.8 .005 .0001',material='floor_material')
    base_z=-float(gait.rest[:,2].min())+.008
    bodies={'base_link':ET.SubElement(world,'body',name='base_link',pos=f'0 0 {base_z}')}
    if floating:
        ET.SubElement(bodies['base_link'],'freejoint',name='floating_base')
    ET.SubElement(bodies['base_link'],'site',name='imu_site',pos='0 0 0',size='.002')
    sensors=ET.SubElement(root,'sensor')
    ET.SubElement(sensors,'gyro',name='imu_gyro',site='imu_site')
    ET.SubElement(sensors,'accelerometer',name='imu_accel',site='imu_site')
    ET.SubElement(sensors,'framequat',name='imu_orientation',objtype='site',objname='imu_site')
    actuators=ET.SubElement(root,'actuator')
    pending=list(robot.findall('joint'))
    while pending:
        ready=[j for j in pending if j.find('parent').get('link') in bodies]
        if not ready:
            raise ValueError('Disconnected URDF tree')
        for j in ready:
            origin=j.find('origin')
            if not np.allclose(np.fromstring(xyz(origin,'rpy'),sep=' '),0):
                # The camera-only fixed optical rotation is omitted from physics bodies.
                if j.get('name')=='camera_optical_joint':
                    pending.remove(j)
                    continue
                raise ValueError('Nonzero link rotation requires explicit transform conversion')
            p,c=j.find('parent').get('link'),j.find('child').get('link')
            body=ET.SubElement(bodies[p],'body',name=c,pos=xyz(origin))
            bodies[c]=body
            if j.get('type')=='revolute':
                limit=j.find('limit')
                ET.SubElement(body,'joint',name=j.get('name'),type='hinge',axis=xyz(j.find('axis')),
                              range=f"{limit.get('lower')} {limit.get('upper')}",damping='.1',armature='.005')
                effort=limit.get('effort')
                ET.SubElement(actuators,'position',name=j.get('name')+'_drive',joint=j.get('name'),
                              kp=str(kp),kv=str(kv),ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
                              forcerange=f'-{effort} {effort}')
            pending.remove(j)
    for link in robot.findall('link'):
        name=link.get('name')
        if name not in bodies:
            continue
        body=bodies[name]
        inertial=link.find('inertial')
        if inertial is not None:
            if not np.allclose(np.fromstring(xyz(inertial.find('origin'),'rpy'),sep=' '),0):
                raise ValueError('Rotated inertia needs tensor transformation')
            inertia=inertial.find('inertia')
            full=' '.join(inertia.get(k) for k in ['ixx','iyy','izz','ixy','ixz','iyz'])
            ET.SubElement(body,'inertial',pos=xyz(inertial.find('origin')),
                          mass=inertial.find('mass').get('value'),fullinertia=full)
        visual=link.find('visual')
        if visual is not None and visual.find('geometry/mesh') is not None:
            mesh=visual.find('geometry/mesh')
            path=ROOT/'src/bigspot_description/meshes'/Path(mesh.get('filename')).name
            ET.SubElement(asset,'mesh',name=name+'_mesh',file=path.name,scale='.001 .001 .001')
            colors={'FR':'.80 .38 .16 1','FL':'.16 .55 .78 1','RR':'.8 .64 .18 1','RL':'.3 .67 .36 1'}
            ET.SubElement(body,'geom',name=name+'_collision',type='mesh',mesh=name+'_mesh',
                          pos=xyz(visual.find('origin')),rgba=colors.get(name[:2],'.52 .55 .60 1'),
                          contype='1',conaffinity='2',friction='.8 .005 .0001')
        if name.endswith('_foot'):
            ET.SubElement(body,'site',name=name,size='.003',rgba='1 .2 .2 1')
    target=ROOT/'simulation/generated/local'
    target.mkdir(parents=True,exist_ok=True)
    path=target/('bigspot_free.xml' if floating else 'bigspot_fixed.xml')
    ET.indent(root)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return path,gait


def run(args):
    path,gait=mjcf(not args.fixed,args.kp,args.kv)
    gait.mode=args.gait if args.gait!='stand' else 'crawl'
    gait.support_shift=args.support_shift
    # MuJoCo's Windows native path reader cannot open this Korean workspace path.
    # Load XML and mesh bytes through Python; filenames stay portable in the XML.
    assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')}
    model=mujoco.MjModel.from_xml_string(path.read_text(encoding='utf-8'),assets=assets)
    data=mujoco.MjData(model)
    names=[name for leg in gait.legs for name in leg.names]
    joint_ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name) for name in names]
    qadr=model.jnt_qposadr[joint_ids]
    actuator_ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,name+'_drive') for name in names]
    base_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    data.qpos[qadr]=gait.q.ravel()
    data.ctrl[actuator_ids]=gait.q.ravel()
    mujoco.mj_forward(model,data)
    # Confirm engine forward kinematics matches controller before any integration.
    for leg,q in zip(gait.legs,gait.q):
        site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,leg.name.lower()+'_foot')
        computed=data.site_xpos[site]-data.xpos[base_id]
        if not np.allclose(computed,leg.fk(q),atol=1e-6):
            raise RuntimeError(f'MuJoCo vs URDF FK differs: {leg.name}: {computed} vs {leg.fk(q)}')
    frames=[]
    renderer=None
    if args.render:
        renderer=mujoco.Renderer(model,height=480,width=640)
        camera=mujoco.MjvCamera()
        camera.distance=1.2
        camera.azimuth=130
        camera.elevation=-20
    tag=args.case or f"{'fixed' if args.fixed else 'free'}_{args.gait}"
    if not tag.replace('_','').replace('-','').isalnum():
        raise ValueError('Case name must contain only letters, numbers, hyphens or underscores')
    output=ROOT/'simulation/generated/local'
    log_path=output/f'{tag}.csv'
    rows=[]
    dt=float(model.opt.timestep)
    command=gait.q.ravel()
    started=time.monotonic()
    for step in range(round(args.seconds/dt)):
        t=step*dt
        velocity=np.zeros(3)
        if args.gait!='stand' and 2<=t<args.seconds-2:
            velocity[0]=args.speed
        if step%8==0:
            command=np.array(gait.step(8*dt,velocity))
            data.ctrl[actuator_ids]=command
        mujoco.mj_step(model,data)
        matrix=data.xmat[base_id].reshape(3,3)
        roll=np.arctan2(matrix[2,1],matrix[2,2])
        pitch=np.arctan2(-matrix[2,0],np.hypot(matrix[2,1],matrix[2,2]))
        error=float(np.max(np.abs(data.qpos[qadr]-command)))
        rows.append([float(data.time),*data.xpos[base_id].tolist(),float(roll),float(pitch),int(data.ncon),error])
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('Non-finite physics state')
        if renderer and step%25==0:
            camera.lookat[:]=data.xpos[base_id]
            renderer.update_scene(data,camera=camera)
            frames.append(renderer.render().copy())
    array=np.array(rows)
    with log_path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream)
        writer.writerow(['time_s','base_x_m','base_y_m','base_z_m','roll_rad','pitch_rad','contacts','max_joint_error_rad'])
        writer.writerows(rows)
    settled=array[array[:,0]>=2]
    report={'engine':f'MuJoCo {mujoco.__version__}', 'floating_base':not args.fixed,'gait':args.gait,
            'seconds':args.seconds,'command_speed_m_s':args.speed if args.gait!='stand' else 0,
            'physics_timestep_s':dt,'actuated_joints':model.nu,
            'fk_consistency_passed':True,'finite_states':True,
            'min_base_height_after_settle_m':float(settled[:,3].min()),
            'max_body_tilt_after_settle_deg':float(np.max(np.abs(settled[:,4:6]))*180/np.pi),
            'max_joint_error_after_settle_rad':float(settled[:,7].max()),
            'net_x_displacement_m':float(array[-1,1]-array[0,1]),
            'net_x_after_settle_m':float(settled[-1,1]-settled[0,1]),
            'wall_seconds':time.monotonic()-started,
            'collision_model':'one convex hull per STL link; robot self collisions disabled',
            'parameters_unmeasured':True,'kp':args.kp,'kv':args.kv,
            'support_shift':args.support_shift,
            'fallen':bool(np.any(settled[:,3]<.15) or np.any(np.abs(settled[:,4:6])>np.pi/4))}
    (output/f'{tag}.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if renderer:
        renderer.close()
        from PIL import Image
        images=[Image.fromarray(frame) for frame in frames]
        images[0].save(output/f'{tag}.gif',save_all=True,append_images=images[1:],duration=50,loop=0,optimize=True)
        images[-1].save(output/f'{tag}.png')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--fixed',action='store_true')
    parser.add_argument('--gait',choices=['stand','crawl','trot'],default='stand')
    parser.add_argument('--seconds',type=float,default=12)
    parser.add_argument('--speed',type=float,default=.005)
    parser.add_argument('--kp',type=float,default=40)
    parser.add_argument('--kv',type=float,default=2)
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--case',help='Keep separately named results for controller tuning')
    parser.add_argument('--support-shift',type=float,default=0.0,help='Crawl feedforward torso shift, 0..1')
    args=parser.parse_args()
    if args.seconds<4:
        parser.error('Use at least four seconds for settling and stopping')
    if not 0<=args.support_shift<=1:
        parser.error('Support shift must be between 0 and 1')
    run(args)
