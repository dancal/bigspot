"""Original servo constants as a numerical boundary, not measured CAD calibration."""
import copy
import json
from pathlib import Path
import numpy as np


def load_reference():
    source=Path(__file__).resolve().parents[1]/'config/hardware_reference.json'
    if not source.exists():
        from ament_index_python.packages import get_package_share_directory
        source=Path(get_package_share_directory('bigspot_sim'))/'config/hardware_reference.json'
    return json.loads(source.read_text(encoding='utf-8'))


def pulse_angles(q,reference):
    q=np.asarray(q,dtype=float).ravel()
    if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('Expected 12 finite joint angles')
    values=np.array([np.ceil(np.rad2deg(a)*s['direction']+s['def_angle_deg'])+s['trim_deg']
        for a,s in zip(q,reference['servos'])])
    if np.any(values<0.) or np.any(values>180.):
        raise ValueError('Joint command exceeds original servo calibration')
    return values


def reference_model(model,reference):
    model=copy.deepcopy(model)
    joints=[j for leg in model['legs'] for j in leg['joints']]
    for j,s in zip(joints,reference['servos']):
        if j['name']!=s['input_joint']:raise ValueError('Original hardware reference joint order differs')
        margin=reference['servo_margin_deg']
        bounds=np.deg2rad((np.array([margin,180.-margin])-s['def_angle_deg']-s['trim_deg'])/s['direction'])
        j['lower']=max(j['lower'],float(bounds.min()));j['upper']=min(j['upper'],float(bounds.max()))
        if j['lower']>=j['upper']:raise ValueError('CAD and original servo limits do not overlap')
    model['initial_angles']=copy.deepcopy(reference['legacy_standing_ik_rad'])
    model['hardware_reference']=reference
    pulse_angles(model['initial_angles'],reference)
    return model
