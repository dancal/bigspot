"""Timestamp-based interlocks shared by the simulation and ROS controller."""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Decision:
    torque_enabled: bool
    motion_enabled: bool
    reason: str

class Supervisor:
    def __init__(self, sensor_timeout=.15, command_timeout=.3):
        self.sensor_timeout=sensor_timeout;self.command_timeout=command_timeout
        self.stamps={};self.estop=False;self.latched=None

    def update(self, sensor, stamp):
        if not math.isfinite(stamp):raise ValueError('Invalid sensor timestamp')
        if stamp < self.stamps.get(sensor,-math.inf):raise ValueError('Time went backwards')
        self.stamps[sensor]=stamp

    def evaluate(self, now, autonomous=False, tilt=0., motor_temperature=25., voltage=44.4):
        if self.estop:self.latched='ESTOP'
        if not all(math.isfinite(x) for x in (now,tilt,motor_temperature,voltage)):self.latched='INVALID_STATE'
        if abs(tilt)>math.radians(35):self.latched='EXCESS_TILT'
        if motor_temperature>65:self.latched='MOTOR_OVERHEAT'
        if not 36<=voltage<=54:self.latched='BUS_VOLTAGE'
        if not self.latched and any(sensor not in self.stamps for sensor in ('imu','joints')):
            return Decision(False,False,'WAITING_SENSORS')
        for sensor in ('imu','joints'):
            age=now-self.stamps.get(sensor,-math.inf)
            if age < 0 or age>self.sensor_timeout:self.latched=f'STALE_{sensor.upper()}'
        if self.latched:return Decision(False,False,self.latched)
        if now-self.stamps.get('command',-math.inf)>self.command_timeout:return Decision(True,False,'STALE_COMMAND')
        if autonomous and now-self.stamps.get('camera',-math.inf)>.3:return Decision(True,False,'STALE_CAMERA')
        return Decision(True,True,'OK')

    def reset(self):
        # A caller must explicitly reset after fixing the fault. Never auto-rearm.
        self.latched=None
