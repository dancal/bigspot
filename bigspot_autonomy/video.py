"""Stream real physics frames to H.264; never stages robot poses for the camera."""
import math
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg

def motion_folder(root):
    import json
    revision=json.loads((root/'config/pi4_assembly.json').read_text(encoding='utf-8'))['revision']
    return root/'generated'/('motion_videos_'+revision)

TITLES={'startup':'낮은 자세에서 일어서기','walk':'전진 보행','holdout':'무게 +15% · 낮은 마찰',
    'lateral':'옆으로 이동','turn':'제자리 회전',
    'navigation':'깊이 카메라 장애물 회피','imu_dropout':'IMU 단절 · 출력 차단',
    'joint_dropout':'관절 피드백 단절 · 출력 차단','controller_dropout':'제어기 단절 · 모터 watchdog',
    'command_dropout':'이동 명령 단절 · 서기 유지','camera_dropout':'카메라 단절 · 이동 중지'}

class VideoRecorder:
    def __init__(self,plant,case,output,fps=30):
        self.plant,self.case,self.fps=plant,case,fps
        self.output=Path(output);self.output.parent.mkdir(parents=True,exist_ok=True)
        mj=plant.mj;self.renderer=mj.Renderer(plant.model,height=720,width=1280)
        self.view=mj.MjvCamera();self.options=mj.MjvOption();self.options.geomgroup[1]=0
        self.options.geomgroup[3]=1;self.options.sitegroup[:]=0
        self.options.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT]=False
        self.stream=imageio_ffmpeg.write_frames(str(self.output.with_suffix('.mp4')),(1280,720),
            fps=fps,codec='libx264',quality=7,pix_fmt_out='yuv420p',output_params=['-profile:v','baseline','-level:v','3.1','-movflags','+faststart'])
        self.stream.send(None);self.next_frame=0.;self.frames=0
        font=Path('C:/Windows/Fonts/malgun.ttf')
        self.font=ImageFont.truetype(str(font),23) if font.exists() else ImageFont.load_default(size=23)
        self.small=ImageFont.truetype(str(font),17) if font.exists() else ImageFont.load_default(size=17)

    def capture(self,core):
        p=self.plant;t=float(p.data.time)
        if t+1e-9<self.next_frame:return
        self.next_frame+=1/self.fps
        if self.case in ('navigation','camera_dropout'):
            self.view.lookat[:]=[1.45,0,.1];self.view.distance=4.3;self.view.azimuth=110;self.view.elevation=-48
        else:
            self.view.lookat[:]=[p.data.qpos[0],p.data.qpos[1],.15]
            self.view.distance=1.05;self.view.azimuth=40;self.view.elevation=-22
        self.renderer.update_scene(p.data,camera=self.view,scene_option=self.options)
        im=Image.fromarray(self.renderer.render())
        if self.case in ('navigation','camera_dropout'):
            close=p.mj.MjvCamera();close.lookat[:]=[p.data.qpos[0],p.data.qpos[1],.15]
            close.distance=1.05;close.azimuth=135;close.elevation=-22
            self.renderer.update_scene(p.data,camera=close,scene_option=self.options)
            inset=Image.fromarray(self.renderer.render()).resize((440,247))
            im.paste(inset,(816,110))
            draw=ImageDraw.Draw(im);draw.rectangle((816,86,1256,110),fill=(13,23,34));draw.text((828,88),'같은 시점 · 근접 화면',font=self.small,fill=(113,218,204))
        draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,1280,84),fill=(13,23,34))
        draw.text((24,12),'BIGSPOT / '+TITLES[self.case],font=self.font,fill=(226,238,243))
        draw.text((24,49),f'{t:05.2f}s  |  1x  |  {core.state}  |  {core.step_reason if hasattr(core,"step_reason") else ""}',font=self.small,fill=(113,218,204))
        draw.rectangle((0,664,1280,720),fill=(13,23,34))
        draw.text((24,674),'CAD 외형 + CPU 물리 시뮬레이션 · 충돌 형상은 단순화 모델 · 실물 동작 아님',font=self.small,fill=(226,238,243))
        torque=max(abs(p.data.ctrl));tilt=math.degrees(max(abs(core.rp)))
        draw.text((875,684),f'Tilt {tilt:.1f}° | Torque {torque:.2f} Nm',font=self.small,fill=(113,218,204))
        if self.case.endswith('dropout'):
            color=(245,156,74) if t>=9 else (115,159,183)
            draw.text((24,105),'9초: 신호 단절 발생' if t>=9 else '9초에 신호 단절 시험',font=self.font,fill=color)
        self.stream.send(np.asarray(im));self.frames+=1
        if self.frames==self.fps*6:im.save(self.output.with_suffix('.png'))

    def close(self):
        self.stream.close();self.renderer.close()
        count,seconds=imageio_ffmpeg.count_frames_and_secs(str(self.output.with_suffix('.mp4')))
        path=self.output.with_suffix('.mp4').relative_to(Path(__file__).resolve().parents[1]).as_posix()
        return {'path':path,'frames':count,'duration_s':seconds,'fps':self.fps,'cad_visuals':True,'playback_speed':1.}
