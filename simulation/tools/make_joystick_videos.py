"""Create a provenance-labelled joystick video library from measured physics traces.

Rendering replays measured base/joint states; it never runs the controller or
changes playback speed. ROS transport is identified separately for each clip.
"""
import argparse
import csv
import html
import json
import shutil
import subprocess
import zipfile
from pathlib import Path

from motion_simulation import ROOT, mujoco, MotionController
from bigspot_sim.joy_mapping import SELECTABLE
from bigspot_sim.kinematics import rotation
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio_ffmpeg

GENERATED = ROOT / 'simulation/generated'
OUTPUT = GENERATED / 'joystick_videos'
FPS = 15
LABELS = {
    'dance_funk': '엉덩이 흔들기·고개 끄덕임·대각선 스텝 댄스',
    'dance_steps': '발 스텝 댄스', 'dance_show': '큰 몸동작 쇼 댄스',
    'dance_rhythm': '리듬 댄스', 'dance_big': '큰 진폭 댄스',
    'dance': '기본 댄스', 'dance_sway': '좌우 흔들기 댄스',
    'dance_bounce': '위아래 바운스 댄스', 'cute': '애교 모음',
    'cute_tilt': '몸 기울이기 애교', 'cute_wave': '앞발 흔들기 애교',
    'cute_bow': '놀이 인사 애교', 'low': '낮게 서기', 'high': '높게 서기',
    'sit': '뒤로 낮춰 앉는 자세', 'lean_left': '왼쪽으로 몸 이동',
    'lean_right': '오른쪽으로 몸 이동', 'look_up': '몸을 들어 위 보기',
    'look_down': '몸을 숙여 아래 보기', 'roll_left': '왼쪽으로 몸 기울이기',
    'roll_right': '오른쪽으로 몸 기울이기', 'stretch': '스트레칭',
    'body_circle': '몸 원 그리기', 'salute_right': '오른 앞발 들기',
    'salute_left': '왼 앞발 들기', 'bow': '인사', 'crouch': '낮은 자세 (서보 ON)',
    'seated_clap': '뒤로 앉아 앞발 박수', 'turn_left': '왼쪽 90도 회전',
    'turn_right': '오른쪽 90도 회전', 'forward': '앞으로 걷기',
    'backward': '뒤로 걷기', 'left': '왼쪽 옆걸음', 'right': '오른쪽 옆걸음',
    'run': '달리기 모드', 'accelerate': '걷기에서 가속',
    'yaw_left': '스틱으로 왼쪽 연속 회전', 'yaw_right': '스틱으로 오른쪽 연속 회전',
    'startup': '쪼그림 → 기존 20cm 서기 (하드웨어 참조)', 'stop': '감속 후 정지',
    'smooth_reverse': '전진 후 후진 전환', 'salute_cancel': '앞발 든 상태에서 취소·착지',
}
ROUTES = {
    'startup': 'B0', 'stop': 'B1 / B5 해제',
    'forward': 'B5 + 축1 양수', 'backward': 'B5 + 축1 음수',
    'left': 'B5 + 축0 양수', 'right': 'B5 + 축0 음수',
    'yaw_left': 'B5 + 축3 양수', 'yaw_right': 'B5 + 축3 음수',
    'run': 'B5 + B3으로 모드 전환 후 스틱', 'accelerate': 'B5 + B4 + 스틱',
    'dance_funk': 'B5 + B12', 'cute': 'B5 + B13', 'bow': 'B5 + B6',
    'crouch': 'B5 + B7', 'seated_clap': 'B5 + B8',
    'turn_left': 'B5 + B10', 'turn_right': 'B5 + B11',
    'smooth_reverse': 'B5 + 축1 방향 전환', 'salute_cancel': '실행 중 B1 / B5 해제',
}


def load_source(folder, case):
    source = GENERATED / folder
    report = json.loads((source / (case + '.json')).read_text(encoding='utf-8'))
    if report['fallen'] or report['fault'] or report['end_state'] != 'READY':
        raise RuntimeError('Unsuccessful source: ' + case)
    with (source / (case + '.csv')).open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    sensors = json.loads((source / (case + '_sensors.json')).read_text(encoding='utf-8'))
    if len(rows) != len(sensors) or not all('measured_joints_rad' in s for s in sensors):
        raise RuntimeError('Complete measured joint trace required: ' + case)
    times = np.array([float(r['time_s']) for r in rows])
    if not np.allclose(times, [s['time_s'] for s in sensors], atol=1e-9):
        raise RuntimeError('Mismatched base/joint timestamps: ' + case)
    return report, rows, sensors, times


def interval(name, report, times):
    if name == 'startup':
        ready=[s['time_s'] for s in report['states'] if s['state']=='READY']
        return 0., min(float(times[-1]),(ready[0]+1. if ready else float(times[-1])))
    if name == 'stop':
        start = next(s['time_s'] for s in report['states'] if s['state'] == 'STOPPING') - .6
        return start, float(times[-1])
    if report['action_events']:
        start = 8.3
        # Include recovery to READY, not just planned routine duration.
        ready = [s['time_s'] for s in report['states'] if s['state'] == 'READY' and s['time_s'] > 8.5]
        return start, min(float(times[-1]), max(ready) + .6)
    return 7.5, float(times[-1])


def startup_source():
    for case in ('ros_ground_startup','local_ground_startup'):
        path=GENERATED/'hardware_startup'/(case+'.json')
        if path.exists():
            report=json.loads(path.read_text(encoding='utf-8'))
            if (report.get('passed') and report.get('hardware_reference_used') and
                    report.get('minimum_floor_clearance_at_initialization_m',-1.)>=.002-1e-9):
                return 'hardware_startup',case
    for case in ('ros_ground_startup','local_ground_startup'):
        path=GENERATED/'ground_startup'/(case+'.json')
        if path.exists():
            report=json.loads(path.read_text(encoding='utf-8'))
            if report.get('passed') and report.get('startup_mode')=='ground_contact_power_off':
                return 'ground_startup',case
    raise RuntimeError('A successful torque-off, body-on-floor startup record is required')


def render(name, folder, case, model, data, joint_ids, renderer, fonts):
    report, rows, sensors, times = load_source(folder, case)
    start, end = interval(name, report, times)
    target = OUTPUT / (name + '.mp4')
    camera = mujoco.MjvCamera()
    camera.distance = 1.04
    camera.elevation = -24
    camera.azimuth = 215
    writer = imageio_ffmpeg.write_frames(str(target), (640, 480), fps=FPS,
        codec='libx264', pix_fmt_out='yuv420p', quality=7, macro_block_size=1,
        output_params=['-movflags', '+faststart'])
    writer.send(None)
    sample_times = np.arange(start, end, 1. / FPS)
    origin = np.array([float(rows[0]['x_m']), float(rows[0]['y_m'])])
    review = None
    startup_previews={}
    preview_times={'off':1.,'arming':3.5,'preparing':8.,'ready':end-.5}
    try:
        for frame_index, t in enumerate(sample_times):
            j = min(int(np.searchsorted(times, t)), len(times) - 1)
            if j and abs(times[j-1]-t) < abs(times[j]-t):
                j -= 1
            row, sensor = rows[j], sensors[j]
            xyz = np.array([float(row[k]) for k in ('x_m', 'y_m', 'z_m')])
            r, p, y = [float(row[k]) for k in ('roll_rad', 'pitch_rad', 'yaw_rad')]
            data.qpos[:3] = xyz
            R = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
            mujoco.mju_mat2Quat(data.qpos[3:7], R.ravel())
            data.qpos[model.jnt_qposadr[joint_ids]] = sensor['measured_joints_rad']
            mujoco.mj_forward(model, data)
            # Following the measured root is a camera move, not a robot teleport.
            camera.lookat[:] = [xyz[0], xyz[1], .17]
            renderer.update_scene(data, camera=camera)
            frame = Image.new('RGB', (640, 480), (15, 18, 22))
            frame.paste(Image.fromarray(renderer.render().copy()), (0, 54))
            draw = ImageDraw.Draw(frame)
            transport = 'ROS 2 Humble' if report['ros_executed'] else 'LOCAL CONTROLLER'
            draw.text((12, 5), 'BIGSPOT | ' + name.upper(), font=fonts[1], fill='white')
            reference=' | 20 cm target' if report.get('hardware_reference_used') else ''
            draw.text((12, 31), transport + ' + MuJoCo | measured states | 1x'+reference, font=fonts[0], fill=(180, 225, 210))
            draw.text((12, 433), f'{t-start:5.1f}s | {row["state"]} | roll/pitch {r*180/np.pi:+.1f}/{p*180/np.pi:+.1f} deg', font=fonts[0], fill='white')
            delta = xyz[:2] - origin
            cmd = sensor.get('requested_velocity', [0., 0., 0.])
            drive=sensor.get('startup_drive')
            if drive:
                power='ON' if drive['servo_enabled'] else 'OFF / zero torque'
                contact=sensor['ground_rest_feedback']
                pulse=sensor.get('servo_pulse_angles_deg')
                bounds=f' | MAP {min(pulse):.0f}..{max(pulse):.0f} deg' if pulse else ''
                draw.text((12,457),f'SERVO {power} | floor {contact["body_clearance_m"]*1000:.1f}mm | x {delta[0]:+.2f}m'+bounds,font=fonts[0],fill=(185,210,235))
            else:
                draw.text((12, 457), f'x/y {delta[0]:+.2f}/{delta[1]:+.2f} m | command {cmd[0]:+.2f}/{cmd[1]:+.2f} m/s, {cmd[2]:+.2f} rad/s', font=fonts[0], fill=(185, 210, 235))
            writer.send(np.asarray(frame).tobytes())
            if name=='startup':
                for key,at in preview_times.items():
                    if key not in startup_previews and t>=at:
                        startup_previews[key]=frame.copy()
                if review is None and t>=1.:review=frame.copy()
            elif frame_index == len(sample_times) // 2:
                review = frame.copy()
    finally:
        writer.close()
    commands = np.array([s['requested_velocity'] for s in sensors])
    mask = (times >= start) & (times < end)
    item={
        'id': name, 'label': LABELS[name], 'file': target.name,
        'source_folder': folder, 'source_case': case,
        'source_start_s': start, 'source_end_s': end,
        'video_seconds': len(sample_times) / FPS, 'fps': FPS,
        'ros_transport_executed': report['ros_executed'],
        'command_velocity_max_abs': np.abs(commands[mask]).max(axis=0).tolist(),
        'playback_multiplier': 1., 'hardware_validated': False,
    }
    if name=='startup':
        item.update(startup_mode=report['startup_mode'],
            servo_off_peak_torque_nm=report['servo_off_peak_torque_nm'],
            body_clearance_while_off_max_abs_m=report['body_clearance_while_off_max_abs_m'],
            startup_displacement_xy_m=report['startup_displacement_xy_m'],
            startup_position_hold_validated=False,physical_rest_calibrated=False)
        item.update(hardware_reference_used=report.get('hardware_reference_used',False),
            standing_height_target_m=report.get('standing_height_target_m'),
            servo_pulse_min_deg=report.get('servo_pulse_min_deg'),servo_pulse_max_deg=report.get('servo_pulse_max_deg'),
            all_servo_commands_in_range=report.get('all_servo_commands_in_range',False),
            torque_off_preroll_seconds=report.get('torque_off_preroll_seconds'))
        sheet=Image.new('RGB',(1280,960))
        for i,frame in enumerate(startup_previews.values()):
            sheet.paste(frame,((i%2)*640,(i//2)*480))
        sheet.save(OUTPUT/'startup_review.png')
        shutil.copyfile(GENERATED/folder/(case+'.json'),OUTPUT/'startup_validation.json')
    return item,review


def write_library(items):
    catalog = {a: i + 1 for i, a in enumerate(SELECTABLE)}
    for item in items:
        item['catalog_number'] = catalog.get(item['id'])
        item['label']=LABELS[item['id']]
        item['route'] = ROUTES.get(item['id'], '동작 선택 후 B5 + B4 + B6')
        item['video_version']=(OUTPUT/item['file']).stat().st_mtime_ns
    earlier_path = OUTPUT / 'experimental/manifest.json'
    earlier = json.loads(earlier_path.read_text(encoding='utf-8')) if earlier_path.exists() else {'clips': []}
    payload = {
        'date': '2026-10-10', 'video_count': len(items), 'selectable_count': len(SELECTABLE),
        'mapping_source': 'simulation/ros2_ws/src/bigspot_sim/config/joystick.yaml + bigspot_sim/joy_mapping.py',
        'physical_button_mapping_verified': False, 'isaac_ros_5_executed': False,
        'hardware_validated': False, 'playback_multiplier': 1.,
        'note': '현재 제어기 검증 기록의 실측 상태 재생. 일부는 ROS 전송 실행, 나머지는 같은 제어기의 로컬 물리 검증. 보행 검증 속도는 기본 스틱 설정과 다를 수 있으며 영상 하단에 표시.',
        'disabled': [
            {'action': 'jump / jump_forward / jump_back / jump_left / jump_right', 'route': 'B5 + B2 (+ 스틱)', 'status': '기본 설정 비활성, 검증된 성공 영상 없음'},
            {'action': 'handstand_experimental', 'route': 'B5 + B9', 'status': '기본 설정 비활성, 검증된 성공 영상 없음'},
            {'action': 'rear_stand_experimental', 'route': '버튼 미할당 (-1)', 'status': '기본 설정 비활성, 검증된 성공 영상 없음'},
        ], 'items': items, 'earlier_unsuccessful_attempts': earlier['clips'],
    }
    (OUTPUT / 'manifest.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    cards = []
    startup=next(i for i in items if i['id']=='startup')
    shift=np.linalg.norm(startup['startup_displacement_xy_m'])*100
    startup_note=(f'쪼그림에서 기존 20cm 서기 목표로 올라갑니다. 원본 서보 보정값·입력 배열 대응과 0~180° 범위를 반영했습니다. '
        f'몸통 바닥 접촉·서보 OFF → 현재 각도 포착 → 다리 펴기 → 서기 순서입니다. 현재 기립 시험의 이동량은 약 {shift:.1f}cm입니다. '
        '실물 OFF 관절각과 CAD 축/모터 축 대응은 미측정입니다. 다른 영상은 이전 28cm 설정의 검증 기록이며 새 20cm 설정의 재검증 영상이 아닙니다. '
        'B1은 움직임 정지이며 전원 차단이 아닙니다. crouch 영상은 서보 ON의 낮은 자세입니다.')
    for i in items:
        tag = 'ROS' if i['ros_transport_executed'] else '로컬 물리'
        order = str(i['catalog_number']) if i['catalog_number'] else '—'
        cards.append(f'<tr data-id="{i["id"]}" data-search="{html.escape(i["label"] + " " + i["id"] + " " + i["route"])}"><td>{order}</td><td><button data-file="{i["file"]}?v={i["video_version"]}" data-title="{html.escape(i["label"])}">▶ {html.escape(i["label"])}</button><small>{i["id"]}</small></td><td>{html.escape(i["route"])}</td><td>{i["video_seconds"]:.1f}초 · {tag}</td><td><a href="{i["file"]}" download>MP4</a></td></tr>')
    failures = ''.join('<li><a href="' + html.escape(c['file']) + '">' + {
        'jump_left': '왼쪽 점프 — 비행 확인 실패',
        'jump_right': '오른쪽 점프 — 비행 확인 실패',
        'jump_back': '뒤쪽 점프 — 비행 확인 실패',
        'rear_stand_experimental': '두 발 서기 — 도달 불가능한 자세로 요청 거부',
        'handstand_experimental': '물구나무 — 성공 기준 미달',
    }[c['id']] + '</a></li>' for c in earlier['clips'])
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BigSpot 조이스틱 동작 영상</title>
<style>body{margin:0;background:#11161d;color:#e8eff7;font:16px/1.6 system-ui,sans-serif}main{max-width:1080px;margin:auto;padding:24px}h1{font-size:26px;margin:0}p{margin:12px 0}.note{color:#b1c7dd}.controls{background:#202c3a;padding:14px;border-radius:10px}video{width:100%;max-height:550px;background:#090c10;border-radius:12px}#player-title{font-size:20px;margin:12px 0}input{box-sizing:border-box;width:100%;padding:13px;margin:14px 0;background:#202c3a;color:white;border:1px solid #5b7186;border-radius:8px}table{width:100%;border-collapse:collapse}th,td{text-align:left;border-bottom:1px solid #354353;padding:9px}td:first-child{width:35px}button{border:0;background:transparent;color:#73dbc1;cursor:pointer;padding:0;font:inherit;text-align:left}small{display:block;color:#98adbf}a{color:#73bdf6}tr.active{background:#253f4c}.muted{color:#b6c6d7}.scroll{overflow:auto}nav{display:flex;gap:24px;margin:10px 0}nav button{background:#26384b;border-radius:6px;padding:5px 14px}</style>
<main><h1>BigSpot 조이스틱 동작 영상</h1><p class="note">2026-10-10 · 현재 동작 영상 42개: 선택 30개 + 기본 조작 10개 + 전환·취소 예시 2개. 모두 정상 속도(1배). <a href="bigspot-joystick-videos.zip" download>전체 ZIP 다운로드</a></p>
<p class="note">실제 로봇 촬영이 아닙니다. MuJoCo CPU 물리 기록입니다. ROS 표시는 ROS 2 Humble 전송 실행을 뜻합니다. Isaac ROS 5는 실행하지 않았습니다. 머리 전용 관절이 없어 고개 동작은 몸통 자세로 표현합니다.</p>
<p class="note"><b>하드웨어 참조 시작:</b> __STARTUP_NOTE__</p>
<div class="controls"><b>버튼 표기:</b> B0~B13은 ROS Joy의 0부터 시작하는 배열 번호입니다. 실제 패드의 A/B/X/Y 등과 대응은 미확인입니다.<br><b>이동·동작:</b> B5를 누른 채 조작. B0 시작, B1 정지, B3 달리기 전환, B4 가속.<br><b>30개 동작 선택:</b> B5+B4+B13 다음, B5+B4+B12 이전, B5+B4+B6 실행. 시작 선택은 1번. 선택만 하면 움직이지 않습니다.<br><b>속도:</b> 기본 전후진 0.06 m/s, 옆걸음 0.05 m/s, 회전 0.4 rad/s. 가속 전후진 0.15 m/s, 달리기 요청 0.18 m/s. 일부 검증 영상은 별도 시험 속도이며 하단 수치에 표시됩니다.</div>
<h2 id="player-title"></h2><video id="player" controls playsinline preload="metadata"></video><nav><button id="previous">← 이전 영상</button><button id="next">다음 영상 →</button><a id="download" download>현재 MP4 다운로드</a></nav>
<input id="search" type="search" placeholder="동작, 버튼, 코드 이름 검색"><div class="scroll"><table><thead><tr><th>선택 번호</th><th>동작 / 재생</th><th>조작</th><th>길이 / 기록</th><th>파일</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<p class="muted">정지 영상은 속도 명령을 0으로 내린 감속 예시입니다. 취소 영상은 stop 명령 후 앞발 착지를 확인한 예시입니다. 실제 패드 버튼 입력을 촬영한 영상은 아닙니다.</p>
<p class="muted">점프(제자리·앞·뒤·좌·우), 물구나무, 두 발 서기는 기본 설정에서 비활성입니다. 검증된 성공 영상이 없습니다.</p><details><summary>과거 실패·요청 거부 영상 (현재 정상 동작과 구분)</summary><p>이전 버전의 참고 영상입니다. 앞쪽·제자리 점프의 개별 기록은 없습니다.</p><ul>__FAILURES__</ul></details><p><a href="manifest.json">각 영상의 원본 기록·시각·재생 속도 확인</a></p></main>
<script>const rows=[...document.querySelectorAll('tbody tr')],v=document.querySelector('#player'),title=document.querySelector('#player-title'),dl=document.querySelector('#download');let selected=0;function choose(n,play=false){selected=(n+rows.length)%rows.length;const r=rows[selected],b=r.querySelector('button');rows.forEach(x=>x.classList.toggle('active',x===r));v.src=b.dataset.file;title.textContent=b.dataset.title;dl.href=b.dataset.file;location.hash=r.dataset.id;if(play)v.play().catch(()=>{});}rows.forEach((r,n)=>r.querySelector('button').onclick=()=>choose(n,true));document.querySelector('#previous').onclick=()=>choose(selected-1,true);document.querySelector('#next').onclick=()=>choose(selected+1,true);document.querySelector('#search').oninput=e=>{const q=e.target.value.toLowerCase();rows.forEach(r=>r.hidden=!r.dataset.search.toLowerCase().includes(q));};let first=rows.findIndex(r=>r.dataset.id===location.hash.slice(1));choose(first<0?0:first);</script></html>'''.replace('__ROWS__', ''.join(cards)).replace('__FAILURES__', failures).replace('__STARTUP_NOTE__',html.escape(startup_note))
    (OUTPUT / 'index.html').write_text(page, encoding='utf-8')


def main(refresh_startup=False):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    summary = json.loads((GENERATED / 'quality/all_motion_validation.json').read_text(encoding='utf-8'))
    if not summary['passed']:
        raise RuntimeError('Current motion validation must pass')
    config = ROOT / 'simulation/ros2_ws/src/bigspot_sim/config'
    controller = MotionController(json.loads((config / 'model.json').read_text()),
        **json.loads((config / 'control_policy_adaptive.json').read_text())['parameters'])
    model = mujoco.MjModel.from_xml_string((GENERATED / 'quality/replay_model.xml').read_text(),
        assets={p.name: p.read_bytes() for p in (ROOT / 'src/bigspot_description/meshes').glob('*.stl')})
    data = mujoco.MjData(model)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for leg in controller.gait.legs for n in leg.names]
    if min(ids) < 0:
        raise RuntimeError('Missing joint in replay model')
    renderer = mujoco.Renderer(model, height=372, width=640)
    fonts = [ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 15),
             ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 21)]
    controls = ['startup', 'forward', 'backward', 'left', 'right', 'yaw_left', 'yaw_right', 'run', 'accelerate', 'stop']
    names = ['startup'] if refresh_startup else controls + list(SELECTABLE) + ['smooth_reverse', 'salute_cancel']
    items, snapshots = [], []
    for name in names:
        print('VIDEO ' + name, flush=True)
        if name == 'dance_funk':
            shutil.copyfile(GENERATED / 'quality/dance_funk.mp4', OUTPUT / 'dance_funk.mp4')
            item = {'id': name, 'label': LABELS[name], 'file': name+'.mp4', 'video_seconds': 37., 'fps': 20,
                'source_folder': 'audit_ros', 'source_case': 'ros_dance_funk',
                'source_start_s': 8.5, 'source_end_s': 45.5,
                'ros_transport_executed': True, 'playback_multiplier': 1., 'hardware_validated': False,
                'audio': 'Original synthesized 120 BPM beat'}
        else:
            source_name = 'run' if name == 'stop' else name
            if name=='startup':
                folder,case=startup_source()
            elif source_name in ('yaw_left', 'yaw_right'):
                folder, case = 'joystick_video_recordings', 'local_' + source_name
            elif (GENERATED / 'audit_ros' / ('ros_' + source_name + '.json')).exists():
                folder, case = 'audit_ros', 'ros_' + source_name
            else:
                source = summary['local_final_cases'][source_name]
                folder, case = source['folder'], source['case']
            item, snapshot = render(name, folder, case, model, data, ids, renderer, fonts)
            if name in ('startup', 'forward', 'seated_clap', 'cute_wave', 'low', 'salute_left'):
                snapshots.append((name, snapshot))
        items.append(item)
    renderer.close()
    if refresh_startup:
        replacement=items[0]
        items=json.loads((OUTPUT/'manifest.json').read_text(encoding='utf-8'))['items']
        items=[replacement if i['id']=='startup' else i for i in items]
    write_library(items)
    if refresh_startup:
        sheet=Image.open(OUTPUT/'review.png').convert('RGB')
        sheet.paste(snapshots[0][1],(0,0))
    else:
        sheet = Image.new('RGB', (1280, 1440))
        for i, (_, frame) in enumerate(snapshots):
            sheet.paste(frame, ((i % 2) * 640, (i // 2) * 480))
    sheet.save(OUTPUT / 'review.png')
    # Decode every complete stream, including the reused audio/video dance.
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    videos=sorted(OUTPUT.rglob('*.mp4'))
    for path in videos:
        subprocess.run([ffmpeg, '-v', 'error', '-xerror', '-i', str(path),
                        '-f', 'null', '-'], check=True, capture_output=True)
    (OUTPUT / 'media_validation.json').write_text(json.dumps({
        'all_streams_decoded': True, 'video_count': len(items), 'selectable_count': len(SELECTABLE),
        'total_decoded_including_earlier_attempts':len(videos),
        'unique_ids': len({i['id'] for i in items}) == len(items),
        'all_selectable_actions_present': set(SELECTABLE) <= {i['id'] for i in items},
        'normal_playback': all(i['playback_multiplier'] == 1. for i in items),
    }, indent=2) + '\n', encoding='utf-8')
    paths=videos+[OUTPUT/n for n in ('index.html','manifest.json','media_validation.json',
        'review.png','startup_review.png','startup_validation.json','experimental/manifest.json')]
    temporary=OUTPUT/'bigspot-joystick-videos.zip.tmp'
    with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=1) as archive:
        for path in paths:
            archive.write(path,path.relative_to(OUTPUT).as_posix())
    with zipfile.ZipFile(temporary) as archive:
        if archive.testzip() is not None:raise RuntimeError('Video ZIP CRC validation failed')
    temporary.replace(OUTPUT/'bigspot-joystick-videos.zip')
    print('COMPLETE ' + str(len(items)) + ' measured-state videos, all streams decoded', flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--refresh-startup',action='store_true')
    main(**vars(parser.parse_args()))
