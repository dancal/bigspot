"""Render one recorded ROS physical state from four camera angles for review."""
from motion_simulation import ROOT,mjcf,mujoco,ET,MotionController
import csv,json,numpy as np
from bigspot_sim.kinematics import rotation
from PIL import Image,ImageDraw
f=ROOT/'simulation/generated/show';cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
c=MotionController(json.loads((cfg/'model.json').read_text()),**json.loads((cfg/'control_policy_adaptive.json').read_text())['parameters'])
rows=list(csv.DictReader((f/'ros_seated_clap.csv').open()));sensors=json.loads((f/'ros_seated_clap_sensors.json').read_text())
row=min(rows,key=lambda r:abs(float(r['time_s'])-19.6));s=min(sensors,key=lambda r:abs(r['time_s']-19.6))
path,_=mjcf(True,300,8.);model=mujoco.MjModel.from_xml_string(path.read_text(),assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')});data=mujoco.MjData(model)
data.qpos[:3]=[float(row[k]) for k in ['x_m','y_m','z_m']]
r,p,y=[float(row[k]) for k in ['roll_rad','pitch_rad','yaw_rad']]
R=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
mujoco.mju_mat2Quat(data.qpos[3:7],R.ravel())
for name,q in zip([n for leg in c.gait.legs for n in leg.names],s['measured_joints_rad']):
 j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name);data.qpos[model.jnt_qposadr[j]]=q
mujoco.mj_forward(model,data)
render=mujoco.Renderer(model,height=480,width=640);camera=mujoco.MjvCamera();camera.lookat[:]=[-.12,0,.20]
camera.distance=.95;camera.elevation=-15
sheet=Image.new('RGB',(1280,960))
for i,az in enumerate([35,130,215,310]):
 camera.azimuth=az;render.update_scene(data,camera=camera);im=Image.fromarray(render.render().copy())
 ImageDraw.Draw(im).text((10,10),'Recorded ROS clap state 19.6 s | azimuth '+str(az),fill='white')
 sheet.paste(im,((i%2)*640,(i//2)*480))
sheet.save(f/'clap_views.png');render.close()
