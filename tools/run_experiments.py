import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
if sys.platform=="win32":
    dependency=ROOT.parent/"bigspot/simulation/generated/python_deps"
    if dependency.exists():sys.path.insert(0,str(dependency))
from bigspot_autonomy.design import load_design, variant
from bigspot_autonomy.physics import rollout

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--layout",choices=("all00","hybrid","all02"),default="hybrid")
    parser.add_argument("--length",type=float,default=0.16)
    parser.add_argument("--height",type=float)
    parser.add_argument("--speed",type=float,default=0.25)
    parser.add_argument("--seconds",type=float,default=12)
    parser.add_argument("--startup",action="store_true")
    parser.add_argument("--render",action="store_true")
    parser.add_argument("--friction",type=float,default=0.8)
    parser.add_argument("--mass-scale",type=float,default=1.)
    parser.add_argument("--tag",default="prototype")
    args=parser.parse_args()
    design=variant(load_design(),args.layout,args.length,args.height)
    report=rollout(design,args.speed,args.seconds,args.friction,args.mass_scale,args.startup,render=args.render,output=ROOT/"generated/experiments"/args.tag)
    print(json.dumps(report,indent=2))
