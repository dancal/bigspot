"""Front view of recorded ROS physics states; no pose changes or speedup."""
from motion_simulation import ROOT,mjcf,mujoco,MotionController
from bigspot_sim.kinematics import rotation
import csv,json,numpy as np
from PIL import Image,ImageDraw,ImageFont
f=ROOT/'simulation/generated/show';cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
c=MotionController(json.loads((cfg/'model.json').read_text()),**json.loads((cfg/'control_policy_adaptive.json').read_text())['parameters'])
rows=list(csv.DictReader((f/'ros_seated_clap.csv').open()));sensors=json.loads((f/'ros_seated_clap_sensors.json').read_text())
time=np.array([float(r['time_s']) for r in rows]);sensor_time=np.array([s['time_s'] for s in sensors])
path,_=mjcf(True,300,8.);m=mujoco.MjModel.from_xml_string(path.read_text(),assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')})
d=mujoco.MjData(m);ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for leg in c.gait.legs for n in leg.names]
render=mujoco.Renderer(m,height=480,width=640);camera=mujoco.MjvCamera()
camera.lookat[:]=[-.10,0,.22];camera.distance=1.05;camera.elevation=-15;camera.azimuth=215
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',16);frames=[];recorded_count=0
for t in np.arange(0.,60.,.1):
 row=rows[int(np.argmin(abs(time-t)))];s=sensors[int(np.argmin(abs(sensor_time-t)))]
 d.qpos[:3]=[float(row[k]) for k in ['x_m','y_m','z_m']]
 r,p,y=[float(row[k]) for k in ['roll_rad','pitch_rad','yaw_rad']]
 R=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
 mujoco.mju_mat2Quat(d.qpos[3:7],R.ravel());d.qpos[m.jnt_qposadr[ids]]=s['measured_joints_rad']
 mujoco.mj_forward(m,d);render.update_scene(d,camera=camera);im=Image.fromarray(render.render().copy());draw=ImageDraw.Draw(im)
 draw.rectangle((0,0,640,73),fill=(15,18,22));draw.rectangle((0,423,640,480),fill=(15,18,22))
 draw.text((10,7),f'BigSpot | SEATED CLAP | {row["state"]} | {float(row["time_s"]):.2f}s',font=font,fill='white')
 draw.text((10,30),'Recorded ROS 2 + MuJoCo states | front view | normal time',font=font,fill='white')
 recorded_count=(s.get('clap') or {}).get('count',recorded_count)
 count=recorded_count
 draw.text((10,52),f'Measured paw claps: {count}/3 | paw contact: {s["seat"][1]:.2f} N',font=font,fill=(185,225,210))
 draw.text((10,429),f'IMU roll/pitch: {r*180/np.pi:+.1f}/{p*180/np.pi:+.1f} deg',font=font,fill='white')
 for i,name in enumerate(['FR','FL','RR','RL']):draw.text((10+i*155,453),f'{name}: {float(row[name+"_force_N"]):.1f} N',font=font,fill=(185,210,235))
 frames.append(im)
render.close();frames[0].save(f/'ros_seated_clap_front.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
print('Rendered 600 recorded physical states; camera changed, motion unchanged')
