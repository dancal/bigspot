"""Build a local video gallery and check each exported video's frame count."""
import hashlib
import html
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import imageio_ffmpeg
from bigspot_autonomy.video import TITLES

def main():
    folder=ROOT/'generated/motion_videos';data=json.loads((ROOT/'validation/control_loop.json').read_text())
    cards=[];manifest=[]
    if (folder/'cad_orbit.mp4').exists():
        cards.append('<article><div class="label">CAD 외형 · 360° 검토</div><video controls playsinline preload="metadata" poster="cad_orbit.png" src="cad_orbit.mp4"></video><p>12초 · 카메라만 회전 · 동작 검증 영상 아님</p><a href="cad_orbit.mp4" download>MP4 저장</a></article>')
    for r in data['reports']:
        name=r['case'];path=folder/f'{name}.mp4'
        frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(path))
        if frames!=r['video']['frames'] or frames<30 or abs(seconds-r['video']['duration_s'])>.05:raise RuntimeError('Invalid video '+name)
        record={'case':name,'title':TITLES[name],'file':path.name,'frames':frames,'duration_s':seconds,
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'physics_passed':r['passed']}
        manifest.append(record);(folder/f'{name}.json').write_text(json.dumps(r,indent=2)+'\n')
        summary=f"{seconds:.1f}초 · 30fps · 1배속 · {'검증 통과' if r['passed'] else '검토 필요'}"
        cards.append(f'<article><div class="label">{html.escape(TITLES[name])}</div><video controls playsinline preload="metadata" poster="{name}.png" src="{name}.mp4"></video><p>{summary}</p><a href="{name}.mp4" download>MP4 저장</a> · <a href="{name}.json">검증 수치</a></article>')
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    cad=json.loads((ROOT/'design/pi4/validation.json').read_text())
    audit=json.loads((ROOT/'design/pi4/interference_audit.json').read_text())
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BigSpot CAD · 동작 영상</title><style>
    *{box-sizing:border-box}body{margin:0;background:#0d1722;color:#dfedf0;font:16px/1.65 system-ui,"Malgun Gothic",sans-serif}main{max-width:1440px;margin:auto;padding:44px 28px}.eyebrow{color:#5bd0bf;letter-spacing:.18em;font-size:13px}h1{font-size:clamp(30px,4vw,54px);margin:12px 0}header{max-width:1000px;margin-bottom:32px}.stats{display:flex;gap:12px;flex-wrap:wrap;margin:22px 0}.stats span{border:1px solid #29414f;padding:8px 16px;border-radius:24px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,440px),1fr));gap:24px}article{border:1px solid #29414f;border-radius:14px;overflow:hidden;background:#152330;padding-bottom:20px}.label{padding:18px 20px;font-size:20px;font-weight:650}video{width:100%;display:block;background:#000}article p,article a{margin-left:20px}article p{color:#9db6c3;font-size:14px}a{color:#74dacb;text-decoration:none}.note{color:#9db6c3;font-size:15px}footer{margin-top:40px;border-top:1px solid #29414f;padding-top:24px}</style><main><header><div class="eyebrow">BIGSPOT / PI 4 QDD / R2</div><h1>설계부터 움직임까지.</h1><p>그라파이트 통풍 커버, 청록색 프레임, 12° 센서 장착과 실제 CAD 링크 외형을 반영했습니다. 각 영상에서 시작 자세부터 움직임과 신호 단절 대응까지 직접 확인할 수 있습니다.</p>'''
    audit_status='통과' if audit['passed'] else '간섭 검토 필요'
    page+=f'<div class="stats"><span>추정 {cad["estimated_mass_kg"]:.3f}kg</span><span>12축</span><span>CAD {len(audit["reports"])}개 자세 · {audit_status}</span><span>{len(manifest)}개 동작·오류 영상</span></div>'
    page+='<p class="note">영상은 1배속 CPU 물리 시뮬레이션입니다. CAD 외형은 실제 모델이며, 충돌 계산은 단순화 형상입니다. 정답 위치를 사용하는 CPU 장애물 회피 시험으로, 실제 Isaac VSLAM 성능을 뜻하지 않습니다.</p></header><section class="grid">'+''.join(cards)+'</section><footer class="note">검사한 CAD 자세에서 1mm³를 넘는 겹침을 확인합니다. 연속 운동 전체의 간섭·하중·피로·베어링·체결·실물 센서 검증은 별도입니다. 모터 출력 차단 시험에서 몸체가 내려앉는 동작은 정상 서기 정지와 구분합니다.</footer></main><script>document.querySelectorAll("video").forEach(v=>v.addEventListener("play",()=>document.querySelectorAll("video").forEach(o=>{if(o!==v)o.pause()})));</script></html>'
    (folder/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps({'videos':len(manifest),'all_physics_passed':all(r['physics_passed'] for r in manifest),'gallery':str(folder/'index.html')}))

if __name__=='__main__':main()
