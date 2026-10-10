"""Slow wave crawl: transfer weight with four soles down, then move one sole.

World coordinates are planner coordinates, not measured odometry. Supported
foot targets stay fixed while the commanded torso moves. Stop finishes the
current swing and retains the landed footprint instead of snapping to rest.
"""
import numpy as np


class StaticCrawl:
    def __init__(self, rest, period, margin=.012, transfer=.45,com_x=-.020):
        if period <= 0 or not 0 < margin < .05 or not .2 <= transfer <= .7 or not np.isfinite(com_x):
            raise ValueError('Invalid static crawl timing/support geometry')
        self.rest = rest.copy()
        self.feet = rest.copy()
        self.body = np.zeros(2)
        self.nominal = np.zeros(2)
        self.period, self.margin, self.transfer = period, margin, transfer
        self.com_x = com_x
        self.sequence = [3, 1, 2, 0]  # left rear, left front, right rear, right front
        self.index = 0
        self.running = False
        self.elapsed = 0.

    def support_y(self, x, triangle):
        # Keep x at the forward plan; choose y inside all three support edges.
        p = triangle.copy()
        ab,ac = p[1]-p[0],p[2]-p[0]
        if ab[0]*ac[1]-ab[1]*ac[0] < 0:
            p = p[::-1]
        lo, hi = -np.inf, np.inf
        for a, b in zip(p,np.roll(p,-1,axis=0)):
            dx,dy = b-a
            if abs(dx) < 1e-9:
                if -dy*(x-a[0]) < self.margin*abs(dy):
                    raise ValueError('Torso x is outside support triangle')
                continue
            y = a[1]+(dy*(x-a[0])+self.margin*np.hypot(dx,dy))/dx
            if dx > 0:
                lo=max(lo,y)
            else:
                hi=min(hi,y)
        if lo > hi:
            raise ValueError('No feasible static support at planned torso x')
        return float(np.clip(self.nominal[1],lo,hi))

    def begin(self, speed):
        self.leg = self.sequence[self.index]
        step_time = self.period/4
        self.nominal += speed[:2]*step_time
        triangle = np.delete(self.feet[:,:2],self.leg,axis=0)
        self.body_from = self.body.copy()
        self.body_to = np.array([self.nominal[0],self.support_y(self.nominal[0]+self.com_x,triangle)])
        self.foot_from = self.feet[self.leg].copy()
        self.foot_to = self.rest[self.leg].copy()
        self.foot_to[:2] += self.nominal + speed[:2]*self.period*.5
        self.elapsed, self.running = 0., True

    @staticmethod
    def smooth(u):
        return u*u*u*(10-15*u+6*u*u)

    def targets(self, dt, speed, requested, lift):
        if abs(speed[2]) > 1e-4:
            raise ValueError('Validated static crawl supports translation; yaw requires a separate gait')
        if not self.running and np.linalg.norm(speed[:2]) > .002 and requested:
            self.begin(speed)
        result = self.feet.copy()
        if self.running:
            self.elapsed += dt
            u = min(self.elapsed/(self.period/4),1.)
            if u < self.transfer:
                self.body = self.body_from + self.smooth(u/self.transfer)*(self.body_to-self.body_from)
            else:
                self.body = self.body_to.copy()
                swing = (u-self.transfer)/(1-self.transfer)
                result[self.leg] = self.foot_from+self.smooth(swing)*(self.foot_to-self.foot_from)
                result[self.leg,2] += lift*16*swing*swing*(1-swing)*(1-swing)
            if u >= 1:
                self.feet[self.leg] = self.foot_to
                result = self.feet.copy()
                self.index = (self.index+1)%4
                self.running = False
        result[:,:2] -= self.body
        return result
