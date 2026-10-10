"""Finite candidate search with held-out friction/mass validation, not global optimisation."""
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
dependency=ROOT.parent/"bigspot/simulation/generated/python_deps"
if sys.platform=="win32" and dependency.exists():sys.path.insert(0,str(dependency))
from bigspot_autonomy.design import load_design,variant
from bigspot_autonomy.physics import rollout

base=load_design()
output=ROOT/"generated/design_search"
output.mkdir(parents=True,exist_ok=True)
candidates=[]
for layout in ("all00","hybrid","all02"):
    for length in (0.14,0.16,0.18):
        for ratio in (1.55,1.70,1.80):
            tag=f"{layout}_{length:.2f}_{ratio:.2f}"
            design=variant(base,layout,length,length*ratio)
            result=rollout(design,speed=0.30,seconds=10,output=output/tag)
            entry={"tag":tag,"design":design,"result":result}
            candidates.append(entry)
            print(json.dumps({"tag":tag,"passed":result["passed"],"thermal":round(result["thermal_margin_ratio"],3),"x":round(result["net_x_m"],3),"tilt":round(result["max_tilt_deg"],2)}),flush=True)
# Score only feasible candidates. Weight and price dominate; speed is measured.
feasible=sorted((x for x in candidates if x["result"]["passed"]),key=lambda x:x["result"]["motor_cost_usd"]+100*x["result"]["estimated_mass_kg"]-100*x["result"]["net_x_m"])
selected=None
for entry in feasible:
    holdout=rollout(entry["design"],speed=0.30,seconds=12,friction=0.55,mass_scale=1.15,seed=29,output=output/(entry["tag"]+"_holdout"))
    entry["holdout"]=holdout
    print(json.dumps({"holdout":entry["tag"],"passed":holdout["passed"],"thermal":round(holdout["thermal_margin_ratio"],3)}),flush=True)
    if holdout["passed"]:
        selected=entry
        break
summary={"candidate_count":len(candidates),"feasible_nominal_count":len(feasible),"selected":selected,"candidates":candidates,"global_optimum_proven":False,"hardware_tested":False}
(output/"selection.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
if selected:
    (ROOT/"config/selected_design.json").write_text(json.dumps(selected["design"],indent=2)+"\n",encoding="utf-8")
print(json.dumps({"selected":selected["tag"] if selected else None,"candidate_count":len(candidates)}),flush=True)
