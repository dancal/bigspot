"""Run the new assembly mass/inertia candidate without replacing baseline results."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
if sys.platform=='win32':sys.path.insert(0,str(ROOT.parent/'bigspot/simulation/generated/python_deps'))
from bigspot_autonomy.design import load_design
from bigspot_autonomy.physics import rollout

if __name__=='__main__':
    d=load_design(ROOT/'config/pi4_design.json');out=ROOT/'generated/pi4_validation';out.mkdir(parents=True,exist_ok=True)
    reports={}
    for name,kw in [('startup',dict(speed=0,seconds=10,startup=True)),('walk',dict(speed=.25,seconds=15)),('holdout',dict(speed=.25,seconds=15,mass_scale=1.15,friction=.55,seed=77))]:
        r=rollout(d,output=out/name,**kw);reports[name]=r
        print(json.dumps({'case':name,**{k:r.get(k) for k in ('passed','estimated_mass_kg','max_tilt_deg','thermal_margin_ratio','self_collision_impulse_ns','net_x_m','fallen')}}),flush=True)
    target=ROOT/'validation/pi4_dynamics.json'
    target.write_text(json.dumps({'geometry':'analytic collision proxies, CAD-derived link inertia','hardware_tested':False,'reports':reports},indent=2)+'\n',encoding='utf-8')
    if not all(r['passed'] for r in reports.values()):raise SystemExit(2)
