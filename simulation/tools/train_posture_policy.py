"""Seeded cross-entropy policy parameter search over actual physics episodes.

This learns four controller parameters; it is not a neural walking policy.
"""
import json
from pathlib import Path
import numpy as np
from posture_simulation import episode, ROOT

keys=['support_shift','balance_kp','balance_kd','speed_scale']
lower=np.array([.25,.15,0.,.5])
upper=np.array([.95,1.5,.15,1.5])
rng=np.random.default_rng(42)
mean=np.array([.6,.6,.05,1.1])
sigma=np.array([.2,.5,.05,.4])
history=[]
best=None
for generation in range(3):
    candidates=np.clip(rng.normal(mean,sigma,size=(10,4)),lower,upper)
    candidates[0]=mean
    if generation==0:
        candidates[1]=[.6,.15,0.,1.5]
    scores=[]
    for index,values in enumerate(candidates):
        params={**dict(zip(keys,values.tolist())),'period':4.}
        # Each generation evaluates a specified terrain/payload, not arbitrary hidden changes.
        friction=[.8,.65,.9][generation]
        mass=[1.,1.1,.95][generation]
        try:
            result=episode(params,seconds=18.,friction=friction,mass_scale=mass)
        except ValueError as exc:
            result={'reward':-200.,'fault':str(exc),'ready_reached':False,'fallen':True}
        record={'generation':generation,'candidate':index,'parameters':params,'friction':friction,
                'mass_scale':mass,'result':result}
        history.append(record)
        scores.append(result['reward'])
        print(f"generation {generation+1}/3, episode {index+1}/10: reward={result['reward']:.3f}, x={result.get('walk_distance_m',0):.3f} m",flush=True)
    elite=candidates[np.argsort(scores)[-3:]]
    mean=elite.mean(0)
    sigma=np.maximum(elite.std(0),[.035,.08,.008,.08])
    best=history[-10+int(np.argmax(scores))]
output=ROOT/'simulation/generated/posture'
output.mkdir(parents=True,exist_ok=True)
(output/'training_history.json').write_text(json.dumps(history,indent=2)+'\n',encoding='utf-8')
(output/'learned_policy.json').write_text(json.dumps({'method':'Cross-entropy controller parameter search',
    'seed':42,'training_episodes':len(history),'parameters':best['parameters'],
    'selection':'Best in final generation; independently validate after training',
    'neural_policy':False,'hardware_validated':False},indent=2)+'\n',encoding='utf-8')
print('Saved learned_policy.json and training_history.json',flush=True)
