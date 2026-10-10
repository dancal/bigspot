import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
deps=ROOT.parent/'bigspot/simulation/generated/python_deps'
if sys.platform=='win32' and deps.exists():sys.path.insert(0,str(deps))
from bigspot_autonomy.design import load_design
from bigspot_autonomy.physics import rollout
from bigspot_autonomy.navigation import Navigator

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--render',action='store_true');p.add_argument('--case',default='all',choices=['all','walk','startup','navigation','faults']);args=p.parse_args()
    d=load_design(ROOT/'config/selected_design.json');out=ROOT/'generated/validation';reports={}
    cases=['walk','startup','navigation','faults'] if args.case=='all' else [args.case]
    if 'walk' in cases:
        reports['walk']=rollout(d,speed=.30,seconds=15,render=args.render,output=out/'walk')
        reports['holdout']=rollout(d,speed=.30,seconds=15,friction=.55,mass_scale=1.15,seed=77,output=out/'holdout')
    if 'startup' in cases:reports['startup']=rollout(d,speed=0,seconds=10,startup=True,render=args.render,output=out/'startup')
    if 'navigation' in cases:reports['navigation']=rollout(d,speed=.24,seconds=32,navigator=Navigator(),render=args.render,output=out/'navigation')
    if 'faults' in cases:
        for source in ('imu','joints'):reports['fault_'+source]=rollout(d,seconds=9,speed=.2,fault=source,output=out/('fault_'+source))
        reports['camera_dropout']=rollout(d,seconds=12,speed=.24,navigator=Navigator(camera_dropout_at=6),output=out/'camera_dropout')
    for name,r in reports.items():print(json.dumps({'case':name,'passed':r['passed'],'thermal':r['thermal_margin_ratio'],'tilt':r['max_tilt_deg'],'navigation':r.get('navigation'),'shutdown':r.get('expected_shutdown_passed'),'safety':r['last_safety_reason']}),flush=True)
