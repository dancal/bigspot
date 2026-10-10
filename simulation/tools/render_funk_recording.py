"""Actual ROS states at 1x speed, with a newly synthesized 120 BPM drum loop."""
from motion_simulation import ROOT,mjcf,mujoco,MotionController
from bigspot_sim.kinematics import rotation
import argparse,csv,json,subprocess,wave
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg

p=argparse.ArgumentParser();p.add_argument('--output-folder',default='groove')
p.add_argument('--input-folder');p.add_argument('--model-file');a=p.parse_args()
if not a.output_folder.replace('_','').replace('-','').isalnum():raise ValueError('Single result directory required')
f=ROOT/'simulation/generated'/a.output_folder;cfg=ROOT/'simulation/ros2_ws/src/bigspot_sim/config'
if a.input_folder and not a.input_folder.replace('_','').replace('-','').isalnum():raise ValueError('Single input directory required')
source=ROOT/'simulation/generated'/(a.input_folder or a.output_folder)
f.mkdir(parents=True,exist_ok=True)
report=json.loads((source/'ros_dance_funk.json').read_text())
if not report.get('ros_executed') or not report.get('funk_dance_success'):
    raise RuntimeError('Successful actual ROS recording required')
c=MotionController(json.loads((cfg/'model.json').read_text()),**json.loads((cfg/'control_policy_adaptive.json').read_text())['parameters'])
rows=list(csv.DictReader((source/'ros_dance_funk.csv').open()));sensors=json.loads((source/'ros_dance_funk_sensors.json').read_text())
times=np.array([float(r['time_s']) for r in rows]);sensor_times=np.array([s['time_s'] for s in sensors])
if a.model_file:path=Path(a.model_file)
else:path,_=mjcf(True,300,8.)
m=mujoco.MjModel.from_xml_string(path.read_text(),assets={p.name:p.read_bytes() for p in (ROOT/'src/bigspot_description/meshes').glob('*.stl')})
m.vis.global_.offwidth=960
d=mujoco.MjData(m);ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for leg in c.gait.legs for n in leg.names]
renderer=mujoco.Renderer(m,height=408,width=480);cameras=[]
for azimuth in (215,35):
    camera=mujoco.MjvCamera();camera.lookat[:]=[.060,0.,.16]
    camera.distance=1.0;camera.elevation=-22;camera.azimuth=azimuth;cameras.append(camera)
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19)
large=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',24)
writer=imageio_ffmpeg.write_frames(str(f/'dance_funk_silent.mp4'),(960,540),fps=20,codec='libx264',
    pix_fmt_out='yuv420p',quality=7,macro_block_size=1,output_params=['-movflags','+faststart'])
writer.send(None);frames=[];preview=[]
duration=float(np.ceil(report['action_events'][0]['duration_s']+1.5))
for frame_index,t in enumerate(np.arange(8.5,8.5+duration,.05)):
    row=rows[int(np.argmin(abs(times-t)))];s=sensors[int(np.argmin(abs(sensor_times-t)))]
    d.qpos[:3]=[float(row[k]) for k in ['x_m','y_m','z_m']]
    r,p,y=[float(row[k]) for k in ['roll_rad','pitch_rad','yaw_rad']]
    R=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
    mujoco.mju_mat2Quat(d.qpos[3:7],R.ravel());d.qpos[m.jnt_qposadr[ids]]=s['measured_joints_rad']
    mujoco.mj_forward(m,d);im=Image.new('RGB',(960,540),(15,18,22))
    for i,camera in enumerate(cameras):
        renderer.update_scene(d,camera=camera)
        im.paste(Image.fromarray(renderer.render().copy()),(480*i,66))
    draw=ImageDraw.Draw(im);dance=s.get('dance') or {};elapsed=dance.get('elapsed_s',t-8.5)
    phrase=dance.get('phrase','SETTLED').replace('_',' ').upper()
    draw.text((14,8),f'BIGSPOT | 120 BPM | {phrase}',font=large,fill='white')
    draw.text((14,39),'Recorded ROS physics | two fixed views | normal playback | 120 BPM beat',font=font,fill=(185,225,210))
    draw.line((480,66,480,474),fill=(85,95,105),width=2)
    beat=int((t-8.5)/.5)%4
    for b in range(4):
        x=756+b*48;draw.rounded_rectangle((x,10,x+36,35),radius=5,fill=(50,200,140) if b==beat else (45,52,62))
        draw.text((x+10,11),str(b+1),font=font,fill='white')
    for i,name in enumerate(['FR','FL','RR','RL']):
        h=float(row[name+'_sole_z_m'])*1000;force=float(row[name+'_force_N'])
        draw.text((14+i*238,478),f'{name}: {h:4.0f} mm | {force:4.1f} N',font=font,fill=(185,210,235))
    draw.text((14,508),f'IMU roll/pitch: {r*180/np.pi:+.1f}/{p*180/np.pi:+.1f} deg | yaw: {y*180/np.pi:+.1f} deg | {row["state"]}',font=font,fill='white')
    writer.send(np.asarray(im).tobytes())
    if frame_index%2==0:frames.append(im)
    if 13.5-.001<=t<25.5-.001:preview.append(im)
writer.close();renderer.close()
palette=frames[80].quantize(colors=128)
compact=[im.resize((720,405)).quantize(palette=palette,dither=Image.Dither.NONE) for im in preview]
compact[0].save(f/'funk_preview.gif',save_all=True,append_images=compact[1:],duration=50,loop=0,optimize=True)
sheet=Image.new('RGB',(960,810))
for i,j in enumerate([63,103,143,183,263,343]):sheet.paste(frames[j].resize((480,270)),((i%2)*480,(i//2)*270))
sheet.save(f/'funk_review.png')

# Original percussion/bass synthesis. No sampled or licensed recording used.
rate=48000;sound=np.zeros(round(rate*duration));rng=np.random.default_rng(120)
def add(start,signal,gain):
    a=round(start*rate);b=min(len(sound),a+len(signal))
    if a<b:sound[a:b]+=gain*signal[:b-a]
for step in range(round(duration/.25)):
    start=step*.25;t=np.arange(round(.065*rate))/rate
    add(start,rng.normal(size=len(t))*np.exp(-t*75),.025 if step%2 else .035)
for beat in range(round(duration/.5)):
    start=beat*.5;t=np.arange(round(.22*rate))/rate
    if beat%2==0:
        frequency=48+75*np.exp(-t*35);phase=2*np.pi*np.cumsum(frequency)/rate
        add(start,np.sin(phase)*np.exp(-t*20),.25)
    else:
        add(start,(rng.normal(size=len(t))+.25*np.sin(2*np.pi*180*t))*np.exp(-t*28),.08)
    freq=[65.406,65.406,77.782,87.307,65.406,97.999,87.307,77.782][beat%8]
    t=np.arange(round(.28*rate))/rate
    envelope=np.minimum(1.,t/.012)*np.exp(-t*13)
    add(start,(np.sin(2*np.pi*freq*t)+.2*np.sin(4*np.pi*freq*t))*envelope,.075)
sound*=np.minimum(1.,np.maximum(0.,(duration-np.arange(len(sound))/rate)/1.0))
sound=np.clip(sound,-.85,.85)
with wave.open(str(f/'original_120bpm.wav'),'wb') as wav:
    wav.setnchannels(1);wav.setsampwidth(2);wav.setframerate(rate);wav.writeframes((sound*32767).astype('<i2').tobytes())
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-v','error','-i',str(f/'dance_funk_silent.mp4'),
    '-i',str(f/'original_120bpm.wav'),'-c:v','copy','-c:a','aac','-b:a','128k','-shortest','-movflags','+faststart',str(f/'dance_funk.mp4')],check=True)
(f/'funk_media.json').write_text(json.dumps({'source':str(source.relative_to(ROOT))+'/ros_dance_funk.csv + measured joint/sole sensor JSON',
    'playback_multiplier':1.,'video_seconds':duration,'fps':20,'preview_seconds':12.,
    'audio':'Original numpy-synthesized 120 BPM percussion/bass, aligned to dance start',
    'camera':'Two fixed views; root and joints are recorded physical values',
    'head_motion':'Rigid-body front emphasis; no independent neck joint'},indent=2)+'\n')
print(f'{duration:g} s physical ROS video, 20 fps, 1x time, original 120 BPM beat; 12 s GIF')
