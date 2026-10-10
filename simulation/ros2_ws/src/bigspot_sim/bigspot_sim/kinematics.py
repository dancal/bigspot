"""URDF-derived kinematics; never assume the old controller dimensions match CAD."""
import numpy as np


def rotation(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)


class Leg:
    def __init__(self, description):
        self.name = description['name']
        self.joints = description['joints']
        self.names = [j['name'] for j in self.joints]
        self.lower = np.array([j['lower'] for j in self.joints])
        self.upper = np.array([j['upper'] for j in self.joints])
        self.foot = np.array(description['foot'])
        self.support_vertices=np.array(description.get('support_vertices_m',[]))

    def support_height(self,q,pitch):
        """Root height at first CAD distal-mesh floor contact; geometric only."""
        if self.support_vertices.size==0:raise ValueError('Rear support CAD geometry missing')
        r,p=np.eye(3),np.zeros(3)
        for joint,angle in zip(self.joints,q):
            p+=r@np.array(joint['origin']);r=r@rotation(joint['axis'],angle)
        world=(p+self.support_vertices@r.T)@rotation([0,1,0],pitch).T
        return -float(world[:,2].min())-.0005

    def fk(self, q):
        r, p = np.eye(3), np.zeros(3)
        for j, angle in zip(self.joints, q):
            p += r @ np.array(j['origin'])
            r = r @ rotation(j['axis'], angle)
        return p + r @ self.foot

    def ik(self, target, seed, tolerance=.0005):
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('Invalid foot target')
        q = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)
        for _ in range(60):
            p = self.fk(q)
            error = target-p
            if np.linalg.norm(error) < tolerance:
                return q
            eps = 1e-5
            jac = np.column_stack([(self.fk(q+np.eye(3)[i]*eps)-p)/eps for i in range(3)])
            step = jac.T @ np.linalg.solve(jac@jac.T + np.eye(3)*1e-5, error)
            q = np.clip(q + np.clip(step, -.15, .15), self.lower, self.upper)
        raise ValueError(f'{self.name}: unreachable target; residual={np.linalg.norm(target-self.fk(q)):.4f} m')


class Gait:
    """Continuous foot trajectories with bounded commands. Dynamics need GPU validation."""
    def __init__(self, model, mode='crawl', period=2.0, lift=.02, support_shift=0.0,
                 support_forward_ratio=1.0, wave_sequence=False, center_stance=False,crawl_duty=.8,
                 static_crawl=False,support_margin=.012,transfer_fraction=.45,com_x=-.020,
                 stance_center_x=0.,max_forward_speed=.015,max_lateral_speed=.01,
                 max_yaw_rate=.1,acceleration=.03,trot_duty=.6,standing_height=0.,
                 smooth_velocity=False,linear_jerk=1.2,yaw_acceleration=.6,yaw_jerk=3.):
        if mode not in ('crawl', 'trot') or period <= 0 or not 0 < lift <= .05:
            raise ValueError('Invalid gait parameters')
        self.legs = [Leg(d) for d in model['legs']]
        self.q = np.array(model['initial_angles'], dtype=float)
        self.rest = np.array([leg.fk(q) for leg, q in zip(self.legs, self.q)])
        if center_stance:
            # The imported joint seed places rear soles ~70 mm in front of the
            # rear hips. Use the hip x coordinates for a symmetric support stance.
            self.rest[:, 0] = [leg.joints[0]['origin'][0]+stance_center_x for leg in self.legs]
        self.rest[:, 2] = np.mean(self.rest[:, 2])
        if standing_height > 0:
            self.rest[:,2] = -standing_height
        self.q = np.array([leg.ik(p, q) for leg, p, q in zip(self.legs, self.rest, self.q)])
        self.mode, self.period, self.lift = mode, period, lift
        if not 0 <= support_shift <= 1:
            raise ValueError('Invalid support shift')
        self.support_shift = support_shift
        if not 0 <= support_forward_ratio <= 1:
            raise ValueError('Invalid support forward ratio')
        self.support_forward_ratio = support_forward_ratio
        self.wave_sequence = wave_sequence
        if not .75 <= crawl_duty <= .92:
            raise ValueError('Invalid crawl stance fraction')
        self.crawl_duty = crawl_duty
        if min(max_forward_speed,max_lateral_speed,max_yaw_rate,acceleration) <= 0 or not .5 <= trot_duty <= .8:
            raise ValueError('Invalid speed/acceleration/trot duty')
        self.speed_limits=np.array([max_forward_speed,max_lateral_speed,max_yaw_rate])
        self.acceleration=np.array([acceleration,acceleration,.2])
        if min(linear_jerk,yaw_acceleration,yaw_jerk)<=0:
            raise ValueError('Invalid jerk or yaw acceleration')
        self.smooth_velocity=smooth_velocity
        if smooth_velocity:
            self.acceleration[2]=yaw_acceleration
        self.jerk_limits=np.array([linear_jerk,linear_jerk,yaw_jerk])
        self.speed_acceleration=np.zeros(3)
        self.speed_jerk=np.zeros(3)
        self.trot_duty=trot_duty
        self.phase = 0.0
        self.speed = np.zeros(3)
        self.blend = 0.0
        self.static = None
        if static_crawl:
            from .static_crawl import StaticCrawl
            self.static = StaticCrawl(self.rest,period,support_margin,transfer_fraction,com_x)

    def targets(self, dt, velocity, marching=False):
        dt = float(np.clip(dt, .001, .04))
        desired = np.clip(np.asarray(velocity, dtype=float), -self.speed_limits, self.speed_limits)
        if desired.shape != (3,) or not np.all(np.isfinite(desired)):
            raise ValueError('Non-finite velocity')
        if self.smooth_velocity:
            # A critically damped second-order command filter. Acceleration is
            # continuous under command changes and its derivative is bounded.
            jerk=np.clip(64*(desired-self.speed)-16*self.speed_acceleration,
                         -self.jerk_limits,self.jerk_limits)
            previous=self.speed_acceleration.copy()
            self.speed_acceleration=np.clip(previous+jerk*dt,-self.acceleration,self.acceleration)
            self.speed_jerk=(self.speed_acceleration-previous)/dt
            self.speed+=self.speed_acceleration*dt
        else:
            self.speed += np.clip(desired-self.speed, -self.acceleration*dt, self.acceleration*dt)
        if self.static is not None and self.mode == 'crawl':
            result = self.static.targets(dt,self.speed,np.linalg.norm(desired[:2])>1e-4,self.lift)
            self.blend = float(self.static.running or np.linalg.norm(self.speed)>1e-4)
            return result
        active = float(marching or np.linalg.norm(self.speed) > 1e-4)
        self.blend += np.clip(active-self.blend, -dt/.5, dt/.5)
        self.phase = (self.phase + dt/self.period) % 1
        offsets = ([0, .5, .25, .75] if self.wave_sequence else [0, .5, .75, .25]) if self.mode == 'crawl' else [0, .5, .5, 0]
        duty = self.crawl_duty if self.mode == 'crawl' else self.trot_duty
        result = self.rest.copy()
        if self.mode == 'crawl' and self.support_shift:
            # Shift the torso away from the leg about to lift. Ramp before liftoff
            # and return after touchdown; this is a geometric feedforward term.
            # No contact/IMU feedback or real center-of-mass identification is implied.
            shift = np.zeros(2)
            center = self.rest[:, :2].mean(0)
            for i, offset in enumerate(offsets):
                s = (self.phase+offset) % 1
                distance = abs((s-(1+duty)/2+.5) % 1-.5)
                u = np.clip((.18-distance)/.08, 0, 1)
                blend = u*u*(3-2*u)
                shift += blend*(center-self.rest[i, :2])/3
            shift[0] *= self.support_forward_ratio
            result[:, :2] -= self.blend*self.support_shift*shift
        for i, offset in enumerate(offsets):
            s = (self.phase+offset) % 1
            center = self.rest[i]
            velocity_i = np.array([self.speed[0]-self.speed[2]*center[1], self.speed[1]+self.speed[2]*center[0]])
            stride = velocity_i*self.period*duty
            if s < duty:
                xy = (.5-s/duty)*stride
                height = 0.0
            else:
                u = (s-duty)/(1-duty)
                # Match the backward stance velocity at both swing endpoints.
                # Zero horizontal swing velocity caused an abrupt joint-rate
                # change whenever a planted foot started or finished its swing.
                b = u*u*u*(10-15*u+6*u*u)
                slope=-(1-duty)/duty
                b=(1-slope)*b+slope*u
                xy = (b-.5)*stride
                height = self.lift * 16*u*u*(1-u)*(1-u)
            result[i, :2] += self.blend*xy
            result[i, 2] += self.blend*height
        return result

    def step(self, dt, velocity):
        targets = self.targets(dt, velocity)
        solved = np.array([leg.ik(p, q) for leg, p, q in zip(self.legs, targets, self.q)])
        # Cap command rate below the unverified 8 rad/s URDF servo limits.
        self.q += np.clip(solved-self.q, -1.5*dt, 1.5*dt)
        return self.q.ravel().tolist()
