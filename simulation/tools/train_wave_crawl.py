"""Learn a slow static wave crawl; assess feet and body motion, not distance alone."""
import json
import numpy as np
from posture_simulation import episode, ROOT


def quality(result):
    if result['fallen'] or result['fault'] or not result['ready_reached']:
        return -100.
    clearance=min(result['foot_clearance_m'].values())
    return -(2*result['net_distance_error_fraction']
             +8*result['forward_velocity_rms_error_m_s']
             +.5*result['backward_motion_fraction']
             +.8*result['fewer_than_three_support_fraction']
             +.1*result['max_tilt_after_rise_deg']
             +100*max(0.,.012-clearance))


keys=['balance_kp','support_margin','transfer_fraction','lift','speed_scale','period']
lower=np.array([.3,.015,.25,.018,.85,4.5])
upper=np.array([1.5,.04,.60,.035,1.15,7.])
mean=np.array([.8,.025,.40,.025,1.,6.])
sigma=np.array([.2,.007,.07,.004,.1,.5])
fixed={'balance_kd':0.,'center_stance':True,'stance_center_x':-.020,
       'com_x':-.020,'static_crawl':True,'wave_sequence':True}
rng=np.random.default_rng(43)
history=[]
for generation in range(3):
    candidates=np.clip(rng.normal(mean,sigma,size=(10,len(keys))),lower,upper)
    candidates[0]=mean
    scores=[]
    for index,values in enumerate(candidates):
        parameters={**fixed,**dict(zip(keys,values.tolist()))}
        result=episode(parameters,seconds=24.)
        score=quality(result)
        record={'generation':generation,'candidate':index,'parameters':parameters,
                'quality_score':score,'result':result}
        history.append(record)
        scores.append(score)
        print(f"Wave {generation+1}/3, {index+1}/10: score={score:.3f}; "
              f"net={result['net_distance_after_stop_m']:.3f} m; "
              f"clearance={min(result['foot_clearance_m'].values())*1000:.1f} mm; "
              f"backwards={result['backward_motion_fraction']:.1%}",flush=True)
    elite=candidates[np.argsort(scores)[-3:]]
    mean=elite.mean(0)
    sigma=np.maximum(elite.std(0),[.07,.002,.015,.0015,.025,.15])
best=max(history,key=lambda record:record['quality_score'])
folder=ROOT/'simulation/generated/posture'
folder.mkdir(parents=True,exist_ok=True)
(folder/'wave_training_history.json').write_text(json.dumps(history,indent=2)+'\n',encoding='utf-8')
(folder/'wave_policy.json').write_text(json.dumps({'method':'CEM static wave crawl parameter search',
    'seed':43,'training_episodes':len(history),'parameters':best['parameters'],
    'selection':'Best quality score across 30 trials in the same nominal model',
    'objective':'Final distance, body velocity, support contacts, all-foot clearance and tilt',
    'neural_policy':False,'hardware_validated':False,'validated_motion':'straight slow crawl'},indent=2)+'\n',encoding='utf-8')
print('Saved wave_policy.json and wave_training_history.json',flush=True)
