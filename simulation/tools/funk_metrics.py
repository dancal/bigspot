"""Independent dance metrics from actual sole positions and ground forces."""
import numpy as np
import json
from pathlib import Path


def actual_body_points(row,points):
    """Rigid CAD reference points projected using measured root pose only."""
    xyz=np.asarray(row[1:4],dtype=float);r,p,y=np.asarray(row[4:7],dtype=float)
    sr,cr,sp,cp,sy,cy=np.sin(r),np.cos(r),np.sin(p),np.cos(p),np.sin(y),np.cos(y)
    R=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    return xyz+np.asarray(points)@R.T


def measured_funk(sensors,physical):
    events=[];support=[];heights=[[] for _ in range(4)]
    for foot,name in enumerate(['FR','FL','RR','RL']):
        air_s=touch_s=0.;armed=False;peak=0.;start=None
        for s,row in zip(sensors,physical):
            dance=s.get('dance') or {}
            if not dance.get('dynamic') or row[30]!='DANCE':continue
            f=float(row[13+foot]);h=float(row[20+foot]);t=float(s['time_s'])
            elapsed=dance['elapsed_s'];main=dance.get('intro_s',1.)<=elapsed<dance.get('main_end_s',21.)
            if main:heights[foot].append(h)
            clear=f<.2 and h>.010
            air_s=air_s+.016 if clear else 0.
            touch_s=touch_s+.016 if f>1. else 0.
            if clear:
                if start is None:start=t
                peak=max(peak,h)
            if air_s>=.048:armed=True
            if armed and touch_s>=.048:
                if main:events.append({'foot':foot,'name':name,'touchdown_s':t,'elapsed_s':elapsed,
                                      'peak_clearance_m':peak,'liftoff_s':start})
                armed=False;peak=0.;start=None
    events.sort(key=lambda e:e['touchdown_s'])
    clusters=[]
    for e in events:
        if clusters and e['touchdown_s']-clusters[-1][0]['touchdown_s']<.12:clusters[-1].append(e)
        else:clusters.append([e])
    pairs=[g for g in clusters if set(e['foot'] for e in g) in ({0,3},{1,2})]
    pair_times=np.array([np.mean([e['touchdown_s'] for e in g]) for g in pairs])
    periods=np.diff(pair_times)
    for s,row in zip(sensors,physical):
        d=s.get('dance') or {}
        if d.get('dynamic') and d.get('intro_s',1.)<=d.get('elapsed_s',0)<d.get('main_end_s',21.) and row[30]=='DANCE':
            support.append(sum(float(row[13+i])>1 for i in range(4)))
    description=json.loads((Path(__file__).resolve().parents[2]/'simulation/ros2_ws/src/bigspot_sim/config/model.json').read_text())
    points=np.array([np.mean([l['joints'][0]['origin'] for l in description['legs'][a:a+2]],axis=0) for a in (0,2)])
    phrases={}
    names=list(dict.fromkeys((s.get('dance') or {}).get('phrase') for s in sensors))
    for name in names:
        if name in (None,'intro','finish'):continue
        values=np.array([[float(v) for v in row[:7]] for s,row in zip(sensors,physical)
                         if (s.get('dance') or {}).get('phrase')==name and row[30]=='DANCE'])
        if len(values):
            references=np.array([actual_body_points(row,points) for row in values])
            yaw=values[0,6]
            heading=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
            local_xy=references[:,:,:2]@heading
            phrases[name]={'xy_range_m':np.ptp(values[:,1:3],axis=0).tolist(),
            'roll_pitch_range_deg':(np.ptp(values[:,4:6],axis=0)*180/np.pi).tolist(),
            'yaw_range_deg':float(np.ptp(np.unwrap(values[:,6]))*180/np.pi),
            'front_body_xy_range_m':np.ptp(local_xy[:,0],axis=0).tolist(),
            'rear_body_xy_range_m':np.ptp(local_xy[:,1],axis=0).tolist(),
            'front_body_z_range_m':float(np.ptp(references[:,0,2])),
            'rear_body_z_range_m':float(np.ptp(references[:,1,2]))}
    return {'measured_clear_paw_landings':events,'measured_diagonal_pair_steps':len(pairs),'measured_phrase_motion':phrases,
        'median_pair_step_interval_s':float(np.median(periods)) if len(periods) else None,
        'pair_interval_10_90_s':np.percentile(periods,[10,90]).tolist() if len(periods) else [],
        'per_paw_peak_clearance_m':[max(h) if h else 0. for h in heights],
        'dynamic_two_or_more_support_fraction':float(np.mean(np.array(support)>=2)) if support else 0.,
        'dynamic_min_support_count':int(min(support)) if support else None,
        'body_reference_points_local_m':points.tolist()}
