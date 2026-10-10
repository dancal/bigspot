"""CAD paw separation at joint limits; geometry probe, not a dynamics test."""
from motion_simulation import ROOT,mjcf,mujoco,ET,MotionController,seat_telemetry
from bigspot_sim.clap import SeatedClapTrajectory
import numpy as np,json
folder=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
c=MotionController(json.loads((folder/'model.json').read_text()),**json.loads((folder/'control_policy_adaptive.json').read_text())['parameters'])
plan=SeatedClapTrajectory(c.standing,c.standing,c.gait.legs)
path,_=mjcf(True,300,8.);xml=ET.fromstring(path.read_text());pair=ET.SubElement(xml,'contact')
ET.SubElement(pair,'pair',geom1='FRF_1_collision',geom2='FLF_1_collision')
m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'),assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')})
d=mujoco.MjData(m);geoms=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n+'F_1_collision') for n in ['FR','FL']]
ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name) for leg in c.gait.legs for name in leg.names]
d.qpos[:3]=[0,0,plan.seat_height];d.qpos[3:7]=[np.cos(plan.pitch/2),0,np.sin(plan.pitch/2),0]
found=[]
for knee in np.linspace(.08,.60,27):
 for thigh in np.linspace(.12,.50,20):
  q=c.gait.q.copy();q[0]=[c.gait.legs[0].lower[0]+.00005,thigh,-knee];q[1]=[c.gait.legs[1].upper[0]-.00005,thigh,knee]
  for j,v in zip(ids,q.ravel()):d.qpos[m.jnt_qposadr[j]]=v
  mujoco.mj_forward(m,d);segment=np.zeros(6)
  distance=mujoco.mj_geomDistance(m,d,*geoms,.10,segment)
  tips=np.array([c.gait.legs[i].fk(q[i]) for i in [0,1]])
  R=np.array([[np.cos(plan.pitch),0,np.sin(plan.pitch)],[0,1,0],[-np.sin(plan.pitch),0,np.cos(plan.pitch)]])
  tips=tips@R.T+[0,0,plan.seat_height]
  offset=np.linalg.norm(segment.reshape(2,3).mean(axis=0)-tips.mean(axis=0))
  if min(tips[:,2])>.050 and offset<.065:found.append({'distance':distance,'offset':offset,'q':q[:2].tolist(),'tips':tips.tolist()})
found.sort(key=lambda x:x['distance'])
print(json.dumps(found[:5],indent=2))
