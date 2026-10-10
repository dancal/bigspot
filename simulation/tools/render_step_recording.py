"""Two fixed views of measured ROS physics states, replayed at normal time."""
from motion_simulation import ROOT,mjcf,mujoco,MotionController
from bigspot_sim.kinematics import rotation
import csv,json,numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg

f=ROOT/'simulation/generated/steps';cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
report=json.loads((f/'ros_dance_steps.json').read_text())
if not report.get('ros_executed') or not report.get('step_dance_success'):
    raise RuntimeError('A successful ROS recording is required before rendering')
c=MotionController(json.loads((cfg/'model.json').read_text()),**json.loads((cfg/'control_policy_adaptive.json').read_text())['parameters'])
rows=list(csv.DictReader((f/'ros_dance_steps.csv').open()));sensors=json.loads((f/'ros_dance_steps_sensors.json').read_text())
time=np.array([float(r['time_s']) for r in rows]);sensor_time=np.array([s['time_s'] for s in sensors])
path,_=mjcf(True,300,8.)
m=mujoco.MjModel.from_xml_string(path.read_text(),assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')})
d=mujoco.MjData(m);ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for leg in c.gait.legs for n in leg.names]
render=mujoco.Renderer(m,height=336,width=480);camera=mujoco.MjvCamera()
camera.lookat[:]=[0.,0.,.20];camera.distance=1.05;camera.elevation=-22
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
frames=[];preview=[];count=0
writer=imageio_ffmpeg.write_frames(str(f/'dance_steps.mp4'),(960,480),fps=10,codec='libx264',
    pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
writer.send(None)
for t in np.arange(8.5,42.,.1):
    row=rows[int(np.argmin(abs(time-t)))];s=sensors[int(np.argmin(abs(sensor_time-t)))]
    d.qpos[:3]=[float(row[k]) for k in ['x_m','y_m','z_m']]
    r,p,y=[float(row[k]) for k in ['roll_rad','pitch_rad','yaw_rad']]
    R=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
    mujoco.mju_mat2Quat(d.qpos[3:7],R.ravel());d.qpos[m.jnt_qposadr[ids]]=s['measured_joints_rad']
    mujoco.mj_forward(m,d)
    im=Image.new('RGB',(960,480),(15,18,22))
    for i,azimuth in enumerate([215,35]):
        camera.azimuth=azimuth;render.update_scene(d,camera=camera)
        im.paste(Image.fromarray(render.render().copy()),(i*480,64))
    draw=ImageDraw.Draw(im);dance=s.get('dance') or {};count=max(count,len(dance.get('measured_landings',[])))
    foot=dance.get('planned_lift_foot');name=['FR','FL','RR','RL'][foot] if foot is not None else 'ALL FOUR'
    draw.text((12,8),f'BigSpot | PAW STEP DANCE | {row["state"]} | {float(row["time_s"]):.2f}s',font=font,fill='white')
    draw.text((12,35),f'ROS 2 + MuJoCo | 90 BPM | normal time | step foot: {name} | measured landings: {count}/4',font=font,fill=(185,225,210))
    draw.text((12,405),'Fixed front / rear views of the same recorded physical state',font=font,fill='white')
    for i,name in enumerate(['FR','FL','RR','RL']):
        h=float(row[name+'_sole_z_m'])*1000;force=float(row[name+'_force_N'])
        draw.text((12+i*238,438),f'{name}: {h:4.0f} mm | {force:4.1f} N',font=font,fill=(185,210,235))
    writer.send(np.asarray(im).tobytes());frames.append(im)
    if 11.5-.001<=t<35.5-.001:preview.append(im)
writer.close();render.close()
palette=frames[50].quantize(colors=128)
compact=[im.resize((720,360)).quantize(palette=palette,dither=Image.Dither.NONE) for im in preview]
compact[0].save(f/'steps_preview.gif',save_all=True,append_images=compact[1:],duration=100,loop=0,optimize=True)
sheet=Image.new('RGB',(960,720))
for i,j in enumerate([50,102,163,223,285,329]):
    sheet.paste(frames[j].resize((480,240)),((i%2)*480,(i//2)*240))
sheet.save(f/'steps_review.png')
print('33.5 s video; 24 s excerpt; two fixed cameras; measured joints/root; 1x playback')
