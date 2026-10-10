"""Decode every frame and verify ranged HTTP delivery for every gallery video."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from bigspot_autonomy.video import motion_folder


def verify(path, base_url):
    frames, seconds=imageio_ffmpeg.count_frames_and_secs(str(path))
    decoded=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-xerror','-i',str(path),'-f','null','-'],capture_output=True,text=True)
    if decoded.returncode or decoded.stderr.strip():
        raise RuntimeError(f'{path.name}: decode failure: {decoded.stderr}')
    header=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-i',str(path)],capture_output=True,text=True).stderr
    if 'Constrained Baseline' not in header or 'yuv420p' not in header:
        raise RuntimeError(f'{path.name}: unexpected browser video encoding')
    raw=path.read_bytes()
    if raw.find(b'moov')>raw.find(b'mdat'):
        raise RuntimeError(f'{path.name}: metadata must precede frame data')
    request=urllib.request.Request(base_url+path.name,headers={'Range':'bytes=1024-2047'})
    with urllib.request.urlopen(request,timeout=10) as response:
        if response.status!=206 or response.headers['Content-Type']!='video/mp4' or response.read()!=raw[1024:2048]:
            raise RuntimeError(f'{path.name}: HTTP media seek failed')
    return {'file':path.name,'frames':frames,'duration_s':seconds,'full_decode_passed':True,
            'range_delivery_passed':True,'encoding':'H.264 Constrained Baseline / yuv420p / faststart',
            'sha256':hashlib.sha256(raw).hexdigest()}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--url',default='http://127.0.0.1:59355/')
    args=parser.parse_args();folder=motion_folder(ROOT)
    paths=sorted(folder.glob('*.mp4'))
    if len(paths)!=12:raise RuntimeError('Expected exactly twelve videos')
    reports=[verify(path,args.url) for path in paths]
    revision=json.loads((ROOT/'config/pi4_assembly.json').read_text(encoding='utf-8'))['revision']
    report={'revision':revision,'passed':True,'videos':reports,'browser_playback_tested':False,'gallery_url':args.url,
            'cad_mesh_sha256':hashlib.sha256((ROOT/'design/pi4/cad_meshes.json').read_bytes()).hexdigest()}
    (ROOT/'validation/motion_media.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'full_decode_and_range_passed':len(reports)}))
