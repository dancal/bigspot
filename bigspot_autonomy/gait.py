import math
import numpy as np
from .design import LEGS
from .kinematics import leg_ik, smoothstep


class Gait:
    def __init__(self, design):
        self.d = design
        self.velocity = np.zeros(3)
        self.phase = 0.0

    def stand(self, height=None, roll=0.0, pitch=0.0):
        return self.targets(0.0, np.zeros(3), height, roll, pitch)

    def targets(self, dt, requested, height=None, roll=0.0, pitch=0.0):
        d, cfg = self.d, self.d["control"]
        requested = np.asarray(requested, dtype=float)
        if requested.shape != (3,) or not np.isfinite(requested).all():
            raise ValueError("Expected finite vx, vy, yaw rate")
        requested = np.clip(requested, [-cfg["maximum_speed_m_s"]]*2+[-0.7], [cfg["maximum_speed_m_s"]]*2+[0.7])
        self.velocity += np.clip(requested-self.velocity, -0.7*dt, 0.7*dt)
        self.phase = (self.phase+dt*cfg["frequency_hz"]) % 1.0
        active = np.linalg.norm(self.velocity) > 0.005
        output = []
        for i, (leg, front, side) in enumerate(LEGS):
            phase = (self.phase + (0.0 if i in (0,3) else 0.5)) % 1.0
            foot_height = height or d["standing_leg_height_m"]
            # Feedback tilt compensation uses measured body attitude.
            z = -foot_height + float(np.clip(side*d["hip_y_m"]*roll-front*d["hip_x_m"]*pitch,-0.035,0.035))
            vx = self.velocity[0]-self.velocity[2]*side*(d["hip_y_m"]+d["hip_offset_m"])
            vy = self.velocity[1]+self.velocity[2]*front*d["hip_x_m"]
            stride = np.array([vx,vy])*cfg["duty_factor"]/cfg["frequency_hz"]
            if active:
                if phase < cfg["duty_factor"]:
                    xy = (0.5-phase/cfg["duty_factor"])*stride
                else:
                    swing = (phase-cfg["duty_factor"])/(1-cfg["duty_factor"])
                    xy = (smoothstep(swing)-0.5)*stride
                    z += cfg["swing_height_m"]*math.sin(math.pi*swing)
            else:
                xy = np.zeros(2)
            output.extend(leg_ik((xy[0]+d.get("neutral_foot_x_m",0),side*d["hip_offset_m"]+xy[1],z),d["upper_length_m"],d["lower_length_m"],d["hip_offset_m"],side))
        return np.array(output)
