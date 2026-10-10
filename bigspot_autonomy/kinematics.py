import math
import numpy as np


def leg_ik(target, upper, lower, offset, side):
    x, y, z = map(float, target)
    radius2 = y*y + z*z - offset*offset
    if radius2 <= 0:
        raise ValueError("Foot target crosses abduction offset")
    down = math.sqrt(radius2)
    abduction = math.atan2(y, -z) - math.atan2(side*offset, down)
    cosine = (x*x + down*down - upper*upper - lower*lower)/(2*upper*lower)
    if not -1.000001 <= cosine <= 1.000001:
        raise ValueError("Foot target outside workspace")
    knee = math.acos(float(np.clip(cosine, -1, 1)))
    hip = math.atan2(-x, down) - math.atan2(lower*math.sin(knee), upper+lower*math.cos(knee))
    return np.array([abduction, hip, knee])


def leg_fk(q, upper, lower, offset, side):
    a, h, k = q
    x = -upper*math.sin(h) - lower*math.sin(h+k)
    z = -upper*math.cos(h) - lower*math.cos(h+k)
    y = side*offset
    return np.array([x, math.cos(a)*y-math.sin(a)*z, math.sin(a)*y+math.cos(a)*z])


def smoothstep(t):
    t = float(np.clip(t, 0, 1))
    return t*t*t*(10+t*(-15+6*t))
