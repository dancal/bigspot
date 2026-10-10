"""Synthetic pinhole geometry shared by the Isaac camera and contract tests."""
import numpy as np

WIDTH, HEIGHT = 640, 480
FOCAL_MM, HORIZONTAL_APERTURE_MM = 18., 20.955
BASELINE_M = .05
FX = FOCAL_MM/HORIZONTAL_APERTURE_MM*WIDTH
CX, CY = (WIDTH-1)/2, (HEIGHT-1)/2


def depth_points(depth,stride=4):
    z=np.asarray(depth,float)
    if z.shape!=(HEIGHT,WIDTH):raise ValueError('Unexpected depth resolution')
    y,x=np.mgrid[0:HEIGHT:stride,0:WIDTH:stride];z=z[::stride,::stride]
    xyz=np.stack(((x-CX)*z/FX,(y-CY)*z/FX,z),axis=-1).reshape(-1,3)
    return np.asarray(xyz[np.isfinite(xyz).all(axis=1)&(xyz[:,2]>.1)&(xyz[:,2]<4.)],dtype='<f4')


def camera_matrices(right=False):
    k=[FX,0.,CX,0.,FX,CY,0.,0.,1.]
    p=[FX,0.,CX,-FX*BASELINE_M if right else 0.,0.,FX,CY,0.,0.,0.,1.,0.]
    return k,p
