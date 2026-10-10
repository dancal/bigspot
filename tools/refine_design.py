"""Second round changes foot placement; original failed search remains intact."""
import json, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
deps=ROOT.parent/'bigspot/simulation/generated/python_deps'
if sys.platform=='win32' and deps.exists():sys.path.insert(0,str(deps))
from bigspot_autonomy.design import load_design,variant
from bigspot_autonomy.physics import rollout

base=load_design();out=ROOT/'generated/design_search_round2';out.mkdir(parents=True,exist_ok=True)
candidates=[]
for layout in ('hybrid','all02','power_knees'):
    for ratio in (1.70,1.80,1.85):
        for offset in (0.025,0.04):
            tag=f'{layout}_014_{ratio:.2f}_offset{offset:.3f}'
            d=variant(base,layout,.14,.14*ratio);d['neutral_foot_x_m']=offset
            try:r=rollout(d,speed=.30,seconds=10,output=out/tag)
            except ValueError as exc:
                print(json.dumps({'tag':tag,'pass':False,'reason':str(exc)}),flush=True)
                candidates.append({'tag':tag,'design':d,'result':{'passed':False,'failure':str(exc)}})
                continue
            candidates.append({'tag':tag,'design':d,'result':r})
            print(json.dumps({'tag':tag,'pass':r['passed'],'thermal':round(r['thermal_margin_ratio'],3),'x':round(r['net_x_m'],3)}),flush=True)
feasible=sorted((x for x in candidates if x['result']['passed']),key=lambda x:x['result']['motor_cost_usd']+100*x['result']['estimated_mass_kg'])
selected=None
for e in feasible:
    h=rollout(e['design'],speed=.30,seconds=15,friction=.55,mass_scale=1.15,seed=29,output=out/(e['tag']+'_holdout'))
    e['holdout']=h
    print(json.dumps({'holdout':e['tag'],'pass':h['passed'],'thermal':round(h['thermal_margin_ratio'],3)}),flush=True)
    if h['passed']:selected=e;break
summary={'candidates':candidates,'candidate_count':len(candidates),'selected':selected,'hardware_tested':False,'global_optimum_proven':False}
(out/'selection.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
if selected:(ROOT/'config/selected_design.json').write_text(json.dumps(selected['design'],indent=2)+'\n',encoding='utf-8')
print(json.dumps({'selected':selected['tag'] if selected else None}),flush=True)
