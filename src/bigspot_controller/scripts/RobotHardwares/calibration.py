"""Original board/channel, direction and horn trims. No motor imports.

The original driver consumed the IK array by slot, despite different leg names
on the wired servo list. Preserve that correspondence until measured otherwise.
"""
import math

IK_NAMES=('FRS','FRL','FRF','FLS','FLL','FLF','RRS','RRL','RRF','RLS','RLL','RLF')
# wired name, I2C address, channel, defAngle, direction, restPos (degrees)
SERVO_SPECS=(
    ('FLS',0x41,0,90,1,5),('FLL',0x41,1,60,1,40),('FLF',0x41,2,160,1,15),
    ('FRS',0x41,3,90,1,-10),('FRL',0x41,4,122,-1,-35),('FRF',0x41,5,22,1,-35),
    ('RLS',0x40,0,90,1,-5),('RLL',0x40,1,48,-1,35),('RLF',0x40,2,158,1,10),
    ('RRS',0x40,3,90,1,-3),('RRL',0x40,4,136,1,-12),('RRF',0x40,5,22,1,-35),
)
WIRED_NAMES=tuple(s[0] for s in SERVO_SPECS)
STANDING_HEIGHT_M=.20
CROUCH_HEIGHT_M=.16
RISE_SECONDS=4.


def pulse_angles(joint_angles):
    values=list(joint_angles)
    if len(values)!=12 or not all(math.isfinite(a) for a in values):
        raise ValueError('Expected 12 finite original IK angles')
    result=[math.ceil(math.degrees(a)*s[4]+s[3])+s[5] for a,s in zip(values,SERVO_SPECS)]
    if any(a<0 or a>180 for a in result):
        raise ValueError('Joint command outside original calibrated servo range; batch rejected')
    return result
