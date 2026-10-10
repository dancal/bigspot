"""Package reproducible source and selected verified local results, without dependencies/account data."""
from pathlib import Path
import zipfile
import os

repo = Path(__file__).resolve().parents[2]
output = repo.parent / 'bigspot-local-results.zip'
excluded = {'.git', '__pycache__', 'generated', 'build', 'install', 'log', '.venv'}
files = []
def source_paths(root):
    if root.is_file():
        yield root
        return
    for folder, directories, filenames in os.walk(root, followlinks=False):
        directories[:] = [name for name in directories if name not in excluded]
        for name in filenames:
            yield Path(folder)/name

for name in ['src', 'simulation', 'README.md', 'LICENSE', '.gitignore']:
    root = repo / name
    paths = source_paths(root)
    for path in paths:
        relative = path.relative_to(repo)
        if not excluded.intersection(relative.parts) and path.suffix not in {'.exe', '.pyc', '.zip'}:
            files.append(path)
for name in ['BigSpot_reconstructed.step', 'BigSpot_reconstructed.json']:
    files.append(repo / 'simulation/generated/cad' / name)
for case in ['free_stand', 'crawl_kp300_noshift', 'crawl_validated', 'crawl_support06_60s']:
    for suffix in ['.json', '.csv']:
        files.append(repo / 'simulation/generated/local' / (case + suffix))
for name in ['crawl_validated.gif', 'crawl_validated.png', 'validation.png']:
    files.append(repo / 'simulation/generated/local' / name)
posture=repo/'simulation/generated/posture'
for name in ['learned_policy.json','training_history.json','startup_baseline.json','startup_baseline.csv',
             'learned_validation.json','learned_validation.csv','ros_learned_validation.json',
             'ros_learned_validation.csv','ros_learned_validation.mp4','posture_validation.png',
             'ros_endurance.json','ros_endurance.csv',
             'holdout_0.json','holdout_1.json','holdout_2.json']:
    path=posture/name
    if path.exists():
        files.append(path)
for name in ['wave_policy.json','wave_training_history.json','ros_wave_validation.json',
             'ros_wave_validation.csv','ros_wave_validation_feet.csv','ros_wave_validation.mp4',
             'ros_wave_walk.mp4','wave_gait_validation.png','gait_before_24s.json',
             'gait_before_24s.csv','gait_before_24s_feet.csv',
             'wave_holdout_0.json','wave_holdout_1.json','wave_holdout_2.json']:
    path=posture/name
    if path.exists():
        files.append(path)
for name in ['ros_fast_validation.json','ros_fast_validation.csv','ros_fast_validation_feet.csv',
             'ros_fast_validation.mp4','ros_fast_walk.mp4','fast_speed_validation.png',
             'fast_speed_comparison.json','fast_endurance.json','fast_endurance.csv',
             'fast_holdout_0.json','fast_holdout_1.json','fast_holdout_2.json','fast_torque_3nm.json']:
    path=posture/name
    if path.exists():
        files.append(path)
motion=repo/'simulation/generated/motion'
for case in ['forward','backward','left','right','turn_left_90','turn_right_90','run','accelerate',
             'buttons','bow','crouch','jump_left','jump_right','jump_back',
             'rear_stand_experimental','handstand_experimental']:
    for suffix in ['.json','.csv']:
        path=motion/('ros_'+case+suffix)
        if path.exists():
            files.append(path)
for name in ['motion_validation.json','motion_validation.png','motion_review.png',
             'motion_demo.mp4','experimental_attempts.mp4','buttons_measured_speeds.json']:
    path=motion/name
    if path.exists():
        files.append(path)
for path in motion.glob('holdout_*.json'):
    files.append(path)
adaptive=repo/'simulation/generated/adaptive'
for prefix,cases in [('ros_',['smooth_start_stop','smooth_reverse','obstacle_wall','sonar_dropout','sonar_invalid',
                            'gyro_fallback','joint_dropout','imu_dropout','balance_push','turn_left_90',
                            'turn_right_90','buttons','pose_gallery','pose_cancel','bow','crouch']),
                      ('holdout_',['salute_left','look_up','smooth_start_stop'])]:
    for case in cases:
        for suffix in ['.json','.csv','_sensors.json']:
            path=adaptive/(prefix+case+suffix)
            if path.exists():
                files.append(path)
for name in ['adaptive_validation.json','adaptive_validation.png','adaptive_review.png',
             'poses_showcase.mp4','adaptive_demo.mp4']:
    path=adaptive/name
    if path.exists():
        files.append(path)
dance=repo/'simulation/generated/dance'
for case in ['ros_dance','ros_dance_sway','ros_dance_bounce','ros_dance_cancel',
             'ros_dance_sonar_dropout','ros_dance_imu_dropout','ros_dance_contact_dropout',
             'ros_dance_joint_dropout','ros_dance_buttons','ros_dance_deadman','holdout_dance']:
    for suffix in ['.json','.csv','_sensors.json']:
        path=dance/(case+suffix)
        if path.exists():
            files.append(path)
for name in ['dance_validation.json','dance_validation.png','dance_review.png','dance_full.mp4','dance_demo.mp4']:
    path=dance/name
    if path.exists():
        files.append(path)
expressive=repo/'simulation/generated/expressive'
for case in ['ros_dance_big','ros_cute_tilt','ros_cute_wave','ros_cute_bow','ros_cute',
             'ros_big_cancel','ros_cute_cancel','ros_cute_sonar_dropout','ros_cute_joint_dropout',
             'ros_cute_imu_dropout','ros_cute_contact_dropout','ros_cute_buttons',
             'holdout_dance_big','holdout_cute_wave']:
    for suffix in ['.json','.csv','_sensors.json']:
        path=expressive/(case+suffix)
        if path.exists():
            files.append(path)
for name in ['expressive_validation.json','expressive_validation.png','expressive_review.png',
             'dance_big.mp4','cute_demo.mp4','expressive_demo.mp4']:
    path=expressive/name
    if path.exists():
        files.append(path)
tempo=repo/'simulation/generated/tempo'
for path in tempo.glob('ros_*'):
    if path.suffix in {'.json','.csv'}:files.append(path)
for name in ['tempo_validation.json','tempo_validation.png','rhythm_review.png','rhythm_demo.mp4','rhythm_preview.gif']:
    path=tempo/name
    if path.exists():files.append(path)
show=repo/'simulation/generated/show'
for path in show.glob('ros_*'):
    if path.suffix in {'.json','.csv'}:files.append(path)
for name in ['show_validation.json','show_validation.png','dance_show.mp4','seated_clap.mp4',
             'show_preview.gif','dance_show_review.png','seated_clap_review.png']:
    path=show/name
    if path.exists():files.append(path)
steps=repo/'simulation/generated/steps'
for path in steps.glob('ros_*'):
    if path.suffix in {'.json','.csv'}:files.append(path)
for name in ['step_validation.json','step_validation.png','dance_steps.mp4','steps_preview.gif','steps_review.png']:
    path=steps/name
    if path.exists():files.append(path)
funk=repo/'simulation/generated/funk'
for path in funk.glob('ros_*'):
    if path.suffix in {'.json','.csv'}:files.append(path)
for name in ['funk_validation.json','funk_validation.png','dance_funk.mp4','funk_preview.gif',
             'funk_review.png','funk_media.json','original_120bpm.wav']:
    path=funk/name
    if path.exists():files.append(path)
groove=repo/'simulation/generated/groove'
for path in groove.glob('ros_*'):
    if path.suffix in {'.json','.csv'}:files.append(path)
for name in ['funk_validation.json','funk_validation.png','dance_funk.mp4','funk_preview.gif',
             'funk_review.png','funk_media.json','original_120bpm.wav']:
    path=groove/name
    if path.exists():files.append(path)
quality=repo/'simulation/generated/quality'
validation=quality/'all_motion_validation.json'
if validation.exists():
    import json
    audit=json.loads(validation.read_text(encoding='utf-8'))
    if not audit.get('passed'):raise RuntimeError('Do not package an unsuccessful final audit')
    for path in quality.iterdir():
        if path.suffix in {'.json','.png','.gif','.mp4','.wav'} and not path.name.endswith('_silent.mp4'):
            files.append(path)
    for source in audit['local_final_cases'].values():
        folder=repo/'simulation/generated'/source['folder']
        for suffix in ['.json','.csv','_sensors.json']:files.append(folder/(source['case']+suffix))
    for folder in ['audit_current','audit_final','audit_baseline']:
        files.append(repo/'simulation/generated'/folder/'summary.json')
    for name in audit['comparisons']:
        for suffix in ['.json','.csv','_sensors.json']:
            files.append(repo/'simulation/generated/audit_baseline'/('local_'+name+suffix))
    ros_audit=repo/'simulation/generated/audit_ros'
    for path in ros_audit.iterdir():
        if path.suffix in {'.json','.csv'}:files.append(path)
    for name in ['audit_unit_tests.log']:
        files.append(repo/'simulation/generated'/name)
files=list(dict.fromkeys(files))
with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
    for path in files:
        archive.write(path, Path('bigspot') / path.relative_to(repo))
with zipfile.ZipFile(output) as archive:
    bad = archive.testzip()
    if bad:
        raise RuntimeError(f'Corrupt archive member: {bad}')
print(f'{output}: {output.stat().st_size / 1024 / 1024:.1f} MiB, {len(files)} files, CRC checked')
