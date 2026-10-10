"""CPU depth-camera/A* experiment. Odometry is simulation truth, NOT visual SLAM."""
import heapq, math
import numpy as np
import mujoco

class DepthCamera:
    def __init__(self,width=48,height=28):
        self.width=width;self.height=height
        self.fx=self.fy=height/(2*math.tan(math.radians(58)/2))
        xx,yy=np.meshgrid(np.arange(width),np.arange(height))
        # MuJoCo camera: local +X right, +Y up, -Z forward.
        self.rays=np.stack(((xx-(width-1)/2)/self.fx,-(yy-(height-1)/2)/self.fy,-np.ones_like(xx)),axis=-1)
        self.rays/=np.linalg.norm(self.rays,axis=-1,keepdims=True)
        self.depth=np.full((height,width),np.nan)

    def capture(self,model,data):
        cid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,'front_depth')
        origin=data.cam_xpos[cid].copy();rotation=data.cam_xmat[cid].reshape(3,3)
        points=[];self.depth.fill(np.nan)
        groups=np.array([1,0,1,0,0,0],dtype=np.uint8)
        for y in range(self.height):
            for x in range(self.width):
                direction=rotation@self.rays[y,x];geom=np.array([-1],dtype=np.int32)
                distance=mujoco.mj_ray(model,data,origin,direction,groups,1,-1,geom)
                if .1<=distance<=4.0:
                    # Optical-axis depth, not radial range.
                    self.depth[y,x]=distance*(-self.rays[y,x,2])
                    point=origin+distance*direction
                    if .06<point[2]<.9:points.append(point)
        return np.asarray(points).reshape(-1,3)

class GridPlanner:
    def __init__(self,resolution=.08,radius=.36):
        self.resolution=resolution;self.radius=radius
        self.origin=np.array([-1.,-2.]);self.shape=(64,51)
        self.raw=np.zeros(self.shape,dtype=bool);self.blocked=self.raw.copy()

    def cell(self,point):return tuple(np.rint((np.asarray(point)[:2]-self.origin)/self.resolution).astype(int))
    def point(self,cell):return self.origin+np.asarray(cell)*self.resolution
    def inside(self,c):return 0<=c[0]<self.shape[0] and 0<=c[1]<self.shape[1]

    def integrate(self,points):
        for p in points:
            c=self.cell(p)
            if self.inside(c):self.raw[c]=True
        self.blocked=self.raw.copy();r=math.ceil(self.radius/self.resolution)
        for cx,cy in np.argwhere(self.raw):
            for dx in range(-r,r+1):
                for dy in range(-r,r+1):
                    c=(cx+dx,cy+dy)
                    if self.inside(c) and math.hypot(dx,dy)*self.resolution<=self.radius:self.blocked[c]=True

    def path(self,start,goal):
        a,b=self.cell(start),self.cell(goal)
        if not self.inside(a) or not self.inside(b) or self.blocked[b]:return []
        # Unknown cells are provisionally free; this is a bounded indoor experiment.
        queue=[(0.,0.,a)];cost={a:0.};parent={}
        while queue:
            _,value,c=heapq.heappop(queue)
            if value>cost[c]+1e-8:continue
            if c==b:
                route=[b]
                while route[-1]!=a:route.append(parent[route[-1]])
                return [self.point(x) for x in route[::-1]]
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                n=(c[0]+dx,c[1]+dy)
                if not self.inside(n) or self.blocked[n]:continue
                if dx and dy and (self.blocked[c[0]+dx,c[1]] or self.blocked[c[0],c[1]+dy]):continue
                trial=value+math.hypot(dx,dy)
                if trial<cost.get(n,math.inf):
                    cost[n]=trial;parent[n]=c
                    heapq.heappush(queue,(trial+math.dist(n,b),trial,n))
        return []

class Navigator:
    def __init__(self,goal=(3.,0.),camera_dropout_at=None):
        self.camera=DepthCamera();self.planner=GridPlanner();self.goal=np.asarray(goal)
        self.last_capture=-math.inf;self.next_capture=0.;self.route=[]
        self.camera_dropout_at=camera_dropout_at;self.reached=False;self.log=[];self.velocity=np.zeros(3)

    def __call__(self,model,data,t):
        xy=data.qpos[:2].copy()
        mat=data.xmat[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')].reshape(3,3)
        yaw=math.atan2(mat[1,0],mat[0,0])
        if t>=self.next_capture and (self.camera_dropout_at is None or t<self.camera_dropout_at):
            points=self.camera.capture(model,data);self.planner.integrate(points)
            self.route=self.planner.path(xy,self.goal);self.last_capture=t;self.next_capture=t+.20
            self.log.append({'t':t,'xy':xy.tolist(),'obstacle_cells':int(self.planner.raw.sum()),'path':[p.tolist() for p in self.route]})
        distance=np.linalg.norm(self.goal-xy)
        if distance<.16:self.reached=True
        if self.reached or t-self.last_capture>.3 or not self.route:self.velocity=np.zeros(3);return self.velocity
        target=self.route[-1]
        for point in self.route:
            if np.linalg.norm(point-xy)>.30:target=point;break
        delta=target-xy;norm=np.linalg.norm(delta)
        # Holonomic quadruped; camera continues to face +X for this scoped course.
        world_velocity=delta/max(norm,1e-9)*min(.24,1.2*norm,.7*distance)
        body=np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])@world_velocity
        self.velocity=np.array([body[0],body[1],np.clip(-1.5*yaw,-.3,.3)])
        return self.velocity

    def report(self,data):
        return {'goal_xy_m':self.goal.tolist(),'goal_distance_m':float(np.linalg.norm(data.qpos[:2]-self.goal)),
                'goal_reached':self.reached,'depth_frames':len(self.log),'obstacle_cells':int(self.planner.raw.sum()),
                'perception':'pinhole depth raycasting; floor and robot excluded',
                'odometry':'MuJoCo ground truth; cuVSLAM NOT executed','unknown_cells':'assumed free; bounded test course'}
