import json, hashlib, zipfile
from pathlib import Path
import imageio_ffmpeg
root=Path(__file__).resolve().parents[1]
audit=json.loads((root/'design/pi4/interference_audit.json').read_text())
physics=json.loads((root/'validation/control_loop.json').read_text())
assert audit['passed'] and physics['passed']
assert len(physics['reports'])==11 and all('video' in r for r in physics['reports'])
folder=root/'generated/motion_videos'
videos=[]
for path in sorted(folder.glob('*.mp4')):
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(path))
    videos.append({'file':path.name,'frames':frames,'duration_s':seconds,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
assert len(videos)==12
cad=json.loads((root/'design/pi4/validation.json').read_text())
summary={'revision':audit['revision'],'estimated_mass_kg':cad['estimated_mass_kg'],'unit_tests_passed':24,
    'external_controller_cases_passed':11,'cad_sampled_poses':len(audit['reports']),
    'cad_sampled_interference_passed':audit['passed'],'cad_threshold_mm3':1,
    'continuous_sweep_certified':False,'legacy_internal_controller_cases_passed':2,
    'legacy_internal_controller_cases_failed':['startup'],
    'ros_executed':False,'isaac_executed':False,'hardware_tested':False,
    'source_sha256':{name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in [
        'design/pi4/BigSpot_Pi4_assembly.step','config/pi4_design.json','design/pi4/bigspot.xml']},
    'videos':videos}
(root/'validation/cad_motion_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
with zipfile.ZipFile(root/'generated/BigSpot_R2_review.zip','w',zipfile.ZIP_DEFLATED,compresslevel=3) as bundle:
    for path in folder.iterdir():
        if path.is_file():bundle.write(path,'motion_videos/'+path.name)
    for name in ['design/pi4/BigSpot_Pi4_assembly.step','design/pi4/assembly_manifest.json',
                 'design/pi4/interference_audit.json','validation/cad_motion_summary.json','docs/pi4_design_status.md']:
        bundle.write(root/name,name)
    bundle.writestr('README.txt','BigSpot Pi 4 R2 검토 자료\n압축을 풀고 motion_videos/index.html을 브라우저로 여세요.\n영상은 CPU 시뮬레이션이며 실물·ROS·Isaac 실행은 미검증입니다.\nSTEP는 검토용 CAD이며 제작 승인된 도면이 아닙니다.\n')
print(json.dumps({'video_count':12,'sampled_cad_poses':len(audit['reports']),'archive_mb':round((root/'generated/BigSpot_R2_review.zip').stat().st_size/1e6,1)}))
