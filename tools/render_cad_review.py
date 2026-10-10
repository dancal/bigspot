"""Orbit a static CAD assembly. This is a design view, not a physics motion test."""
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
from bigspot_autonomy.design import load_design
from bigspot_autonomy.gait import Gait
from bigspot_autonomy.sim_backend import CpuPlant
from bigspot_autonomy.cad_visual import CadVisuals
from bigspot_autonomy.video import motion_folder

def main():
    folder=motion_folder(ROOT);d=load_design(ROOT/'config/pi4_design.json')
    visual=CadVisuals(ROOT/'design/pi4/cad_meshes.json',folder/'meshes')
    p=CpuPlant(d,obstacles=False,model_transform=visual.decorate)
    p.data.qpos[p.qadr]=Gait(d).stand();p.data.qpos[:3]=[0,0,d['standing_leg_height_m']+d['foot_radius_m']]
    p.mj.mj_forward(p.model,p.data)
    renderer=p.mj.Renderer(p.model,720,1280);options=p.mj.MjvOption();options.geomgroup[1]=0;options.geomgroup[3]=1;options.sitegroup[:]=0
    camera=p.mj.MjvCamera();camera.lookat[:]=[0,0,.16];camera.distance=1.05;camera.elevation=-24
    font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',24) if Path('C:/Windows/Fonts/malgun.ttf').exists() else ImageFont.load_default(size=24)
    stream=imageio_ffmpeg.write_frames(str(folder/'cad_orbit.mp4'),(1280,720),fps=30,codec='libx264',quality=7,pix_fmt_out='yuv420p',output_params=['-profile:v','baseline','-level:v','3.1','-movflags','+faststart']);stream.send(None)
    try:
        for frame in range(360):
            camera.azimuth=40+frame
            renderer.update_scene(p.data,camera=camera,scene_option=options)
            im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,1280,64),fill=(13,23,34));draw.text((24,17),'BIGSPOT / '+d['mechanical_revision'].upper()+' · CAD 외형 검토',font=font,fill=(113,218,204))
            draw.rectangle((0,664,1280,720),fill=(13,23,34));draw.text((24,678),'고정된 CAD 조립 모델 · 카메라만 회전 · 동작 검증 영상 아님',font=font,fill=(226,238,243))
            stream.send(np.asarray(im))
            if frame==0:im.save(folder/'cad_orbit.png')
            if frame in (90,180,270):im.save(folder/f'cad_view_{frame}.png')
    finally:stream.close();renderer.close()
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(folder/'cad_orbit.mp4'))
    print(json.dumps({'frames':frames,'seconds':seconds,'physics_motion':False}))

if __name__=='__main__':main()
