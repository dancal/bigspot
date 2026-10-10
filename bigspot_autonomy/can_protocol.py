"""RobStride private 29-bit protocol. RS00/RS02 July 2026 manuals, tables 4.1.2/3.

RS00 example macro V_MIN=+33 is a manual typo: the packet table says -33..+33.
Position uses the table's +/-4*pi range. Verify motor firmware before hardware use.
"""
import math,struct
from dataclasses import dataclass

@dataclass(frozen=True)
class Frame:
    arbitration_id:int
    data:bytes
    is_extended_id:bool=True

LIMITS={'RS00':(14.,33.),'RS02':(17.,44.)}

def uint16(value,low,high):
    if not math.isfinite(value) or not low<=value<=high:raise ValueError('CAN value outside range')
    return int((value-low)*65535/(high-low))

def value16(value,low,high):return low+value*(high-low)/65535

def control(model,motor_id,position,velocity,kp,kd,torque=0.):
    if not 1<=motor_id<=127:raise ValueError('Invalid motor ID')
    t,v=LIMITS[model]
    values=(uint16(position,-4*math.pi,4*math.pi),uint16(velocity,-v,v),uint16(kp,0,500),uint16(kd,0,5))
    return Frame((1<<24)|(uint16(torque,-t,t)<<8)|motor_id,struct.pack('>4H',*values))

def enable(motor_id,host_id=253):return Frame((3<<24)|(host_id<<8)|motor_id,b'\0'*8)
def stop(motor_id,host_id=253):return Frame((4<<24)|(host_id<<8)|motor_id,b'\0'*8)

def feedback(model,frame):
    if not frame.is_extended_id or (frame.arbitration_id>>24)&31!=2 or len(frame.data)!=8:raise ValueError('Not a feedback frame')
    p,v,t,temp=struct.unpack('>4H',frame.data);maxt,maxv=LIMITS[model]
    return {'motor_id':(frame.arbitration_id>>8)&255,'fault_flags':(frame.arbitration_id>>16)&63,
            'mode':(frame.arbitration_id>>22)&3,'position':value16(p,-4*math.pi,4*math.pi),
            'velocity':value16(v,-maxv,maxv),'torque':value16(t,-maxt,maxt),'temperature_c':temp/10}

def bus_load(axes=6,rate_hz=200,bitrate=1_000_000):
    # Upper bound including extended frame overhead, bit stuffing and inter-frame space.
    return axes*rate_hz*2*163/bitrate
