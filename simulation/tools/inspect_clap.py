import csv,json
from pathlib import Path
p=Path(__file__).resolve().parents[2]/'simulation/generated/show'
rows=list(csv.DictReader((p/'local_clap_q.csv').open()))
sensors=json.loads((p/'local_clap_q_sensors.json').read_text())
for t in [34,36,38,40,42,44,46,47]:
 r=min(rows,key=lambda r:abs(float(r['time_s'])-t));s=min(sensors,key=lambda r:abs(r['time_s']-t))
 print(json.dumps({k:r[k] for k in ['time_s','pitch_rad','x_m','z_m','FR_force_N','FL_force_N','RR_force_N','RL_force_N','FR_sole_z_m','state']})+' '+str(s.get('seat'))+' '+str(s.get('measured_joints_rad')))
