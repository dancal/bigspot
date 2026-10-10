"""Derive clap metrics from saved physical traces, without rerunning physics."""
from pathlib import Path
import csv,json
f=Path(__file__).resolve().parents[2]/'simulation/generated/show'
for path in f.glob('ros_*.json'):
    if path.name.endswith('_sensors.json'):continue
    r=json.loads(path.read_text())
    if 'paw_contact_pair_enabled' not in r:continue
    sensors=json.loads(path.with_name(path.stem+'_sensors.json').read_text())
    physical=list(csv.DictReader(path.with_suffix('.csv').open()))
    clap=[s for s,p in zip(sensors,physical) if p['state']=='SEATED_CLAP' and s.get('clap')]
    r['rear_support_min_margin_during_clap_m']=min((s['seat'][4] for s in clap),default=None)
    r['rear_support_min_force_during_clap_n']=min((s['seat'][0] for s in clap),default=None)
    r['front_paw_min_height_during_clap_m']=min((min(s['seat'][5:7]) for s in clap),default=None)
    r['seated_clap_success']=bool(r['scenario'] in ('seated_clap','clap_buttons') and not r['fallen'] and not r['fault']
        and r['end_state']=='READY' and r['measured_paw_clap_count']>=3 and any(e.get('action')=='seated_clap'
            and e.get('result')=='Completed seated clap; three measured paw contacts' for e in r['action_events']))
    path.write_text(json.dumps(r,indent=2)+'\n')
    print(path.name+': '+str(r['seated_clap_success']))
