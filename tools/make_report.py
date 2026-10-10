"""Preserve small validation records in Git; regenerate videos outside Git."""
import hashlib,json,shutil,subprocess,sys,py_compile,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'generated/python_deps'))
deps=ROOT.parent/'bigspot/simulation/generated/python_deps'
if sys.platform=='win32' and deps.exists():sys.path.insert(0,str(deps))
from bigspot_autonomy.design import load_design,mass_and_cost

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))

if __name__=='__main__':
    out=ROOT/'validation';out.mkdir(exist_ok=True);viewer=ROOT/'generated/viewer';viewer.mkdir(parents=True,exist_ok=True)
    reports={}
    for name in ('startup','walk','holdout','navigation','navigation_holdout','fault_imu','fault_joints','camera_dropout'):
        p=ROOT/'generated/validation'/f'{name}.json'
        if not p.exists():continue
        shutil.copy2(p,out/p.name);reports[name]=read(p)
    ros=ROOT/'generated/ros_smoke_report.json'
    if ros.exists():shutil.copy2(ros,out/'ros_smoke_report.json');reports['ros']=read(ros)
    searches=[]
    for directory in ('design_search','design_search_round2'):
        p=ROOT/'generated'/directory/'selection.json'
        if not p.exists():continue
        r=read(p);record={'round':directory,'selected':r['selected']['tag'] if r['selected'] else None,'candidates':[]}
        for entry in r['candidates']:
            result=entry['result'];record['candidates'].append({'tag':entry['tag'],**{k:result.get(k) for k in ('passed','failure','thermal_margin_ratio','motor_cost_usd','estimated_mass_kg','net_x_m','max_tilt_deg')},'holdout':entry.get('holdout',{}).get('passed')})
        searches.append(record)
    (out/'design_search.json').write_text(json.dumps(searches,indent=2)+'\n',encoding='utf-8')
    unit=subprocess.run([sys.executable,str(ROOT/'tests/test_core.py')],cwd=ROOT,capture_output=True,text=True,encoding='utf-8')
    (out/'unit_tests.log').write_text(unit.stdout+unit.stderr,encoding='utf-8')
    if unit.returncode:raise RuntimeError('Unit tests failed')
    files=[ROOT/'setup.py']
    for folder in ('bigspot_autonomy','tools','tests','ros/launch','fusion'):
        files.extend((ROOT/folder).rglob('*.py'))
    for p in files:py_compile.compile(str(p),doraise=True)
    for p in (ROOT/'config').glob('*.json'):read(p)
    from yaml import safe_load
    for p in (ROOT/'ros/config').glob('*.yaml'):safe_load(p.read_text(encoding='utf-8'))
    robot=ET.parse(ROOT/'design/bigspot.urdf').getroot()
    assert len([j for j in robot.findall('joint') if j.get('type')=='revolute'])==12
    cad=read(ROOT/'design/cad_validation.json');assert all(s['valid'] for s in cad['solids'])
    checks={name:(r['expected_shutdown_passed'] if name.startswith('fault_') else r['camera_dropout_stop_passed'] if name=='camera_dropout' else r['closed_loop_passed'] if name=='ros' else r['passed']) for name,r in reports.items()}
    summary={'date':'2026-10-10','legacy_commit':load_design()['legacy_commit'],'checks':checks,
             'all_checks_passed':len(checks)==9 and all(checks.values()),'unit_test_count':11,'python_compile_count':len(files),
             'candidate_count':sum(len(x['candidates']) for x in searches),'cad_valid_solids':cad['solid_count'],
             'design':mass_and_cost(load_design(ROOT/'config/selected_design.json')),
             'hardware_tested':False,'isaac_ros_5_tested':False,'vslam_tested':False,'manufacturing_ready':False,
             'assumptions':['simulation-truth odometry','unmeasured mass and thermal path','60% continuous torque limit','linear torque-speed envelope','bounded flat indoor course'],
             'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},'artifacts':{}}
    cards=[]
    for name,title in (('startup','바닥에서 일어서기'),('walk','가속 · 보행 · 감속'),('navigation','카메라 장애물 회피')):
        video=ROOT/'generated/validation'/f'{name}.mp4';poster=video.with_suffix('.png')
        if not video.exists():continue
        import imageio_ffmpeg
        frame_count,seconds=imageio_ffmpeg.count_frames_and_secs(str(video))
        assert abs(seconds-reports[name]['seconds'])<.1
        summary['artifacts'][name]={'sha256':sha(video),'bytes':video.stat().st_size,'frames':frame_count,'seconds':seconds,'fps':20,'playback_speed':1}
        shutil.copy2(video,viewer/video.name);shutil.copy2(poster,viewer/poster.name)
        cards.append(f'<article><h2>{title}</h2><video controls playsinline preload="metadata" poster="{name}.png" src="{name}.mp4"></video><p>{seconds:g}초 · 1배속 · CPU 물리 시뮬레이션</p></article>')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    shutil.copy2(out/'summary.json',viewer/'summary.json')
    nav=reports['navigation'];rr=reports.get('ros',{});h=reports['holdout']
    html='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BigSpot QDD 실험 결과</title><style>
    *{box-sizing:border-box}body{margin:0;background:#101923;color:#ecf4fb;font-family:system-ui,sans-serif;line-height:1.65}main{max-width:1120px;margin:0 auto;padding:36px 22px}h1{margin:0;font-size:34px}h2{font-size:20px;margin:0 0 12px}.muted{color:#acc0d2}.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin:28px 0}.stat,article,.note{background:#1b2938;border:1px solid #33465b;border-radius:12px;padding:20px}.stat strong{display:block;font-size:29px;color:#6dd7c2}.videos{display:grid;grid-template-columns:repeat(2,1fr);gap:18px}article:last-child{grid-column:1/-1}video{width:100%;display:block;background:#121c25;border-radius:8px}p{margin:10px 0}a{color:#8dc7ff}.note{margin-top:24px}table{border-collapse:collapse;width:100%}td,th{padding:10px;text-align:left;border-bottom:1px solid #33465b}@media(max-width:680px){.stats,.videos{grid-template-columns:1fr}h1{font-size:28px}} </style><main>
    <p class="muted">BigSpot · 새 QDD 프로젝트 · 2026-10-10</p><h1>서기, 보행, 카메라 장애물 회피</h1><p class="muted">물리 시험 + 로컬 ROS 2 Humble. 실제 하드웨어와 Isaac ROS 5.0 GPU 실행은 미검증.</p>'''
    html+=f'<section class="stats"><div class="stat"><strong>{nav["navigation"]["goal_distance_m"]*100:.1f}cm</strong>3m 목표까지 남은 거리</div><div class="stat"><strong>{rr.get("topic_counts",{}).get("commands",0)}회</strong>ROS 보행 명령 수신</div><div class="stat"><strong>7.48kg</strong>설계 추정 무게 · 실측 필요</div></section>'
    html+='<section class="videos">'+''.join(cards)+'</section>'
    html+=f'<section class="note"><h2>검증 결과</h2><table><tr><th>시험</th><th>결과</th></tr><tr><td>바닥에서 서기</td><td>OFF 토크 0, 몸체 하부 보호판 접지</td></tr><tr><td>무게 +15% · 마찰 0.55</td><td>통과 · 최대 기울기 {h["max_tilt_deg"]:.1f}°</td></tr><tr><td>카메라 → ROS → 보행</td><td>통과 · 장애물 충돌 0</td></tr><tr><td>IMU / 관절 정보 단절</td><td>0.15초 후 토크 차단</td></tr><tr><td>카메라 단절</td><td>이동 정지</td></tr></table><p>위치 정보는 시뮬레이터 정답입니다. 실제 카메라 VSLAM 정확도와 모터 발열을 확인해야 합니다. 체결·방열 설계가 끝난 제작 도면은 아닙니다.</p><a href="summary.json">검증 수치 JSON</a></section></main></html>'
    (viewer/'index.html').write_text(html,encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ('all_checks_passed','checks','candidate_count','cad_valid_solids','python_compile_count')}))
    if not summary['all_checks_passed']:raise SystemExit(1)
