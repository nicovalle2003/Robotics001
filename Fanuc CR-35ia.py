import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt 
import numpy as np
import roboticstoolbox as rtb
from matplotlib.widgets import Slider

joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)  #Offset
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0)
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

print("Denavit-Hartenberg Parameters WITH TOOLBOX: ")

d1, a2, d4, d6 = 0.185, 0.410, 0.430, 0.145
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = np.pi/-2, 0, np.pi/2, np.pi/2, np.pi/-2, 0

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=float(d1), a=0, alpha=float(alpha1), offset=0, qlim=[-0,0]),
        rtb.RevoluteDH(d=0, a=float(a2), alpha=float(alpha2), offset=float(np.pi/-2), qlim=[-0,0]),
        rtb.RevoluteDH(d=0, a=0, alpha=float(alpha3), offset=0, qlim=[-0,0]),
        rtb.RevoluteDH(d=float(d4), a=0, alpha=float(alpha4), offset=0,qlim=[-0,0]),
        rtb.RevoluteDH(d=0, a=0, alpha=float(alpha5), offset=0, qlim=[-0,0]),
        rtb.RevoluteDH(d=float(d6), a=0, alpha=float(alpha6), offset=0, qlim=[-0,0])
    ], name = "Fanuc CR-35ia", base = SE3(0, 0, 0)
)

print(robot)
q =np.array([joint1, joint2, joint3, joint4, joint5, joint6])
robot.plot(q=q, backend = 'pyplot', dt = 10, limits = [-0.8,0.8,-0.8,0.8,-0.4,0.6,], shadow = True, jointaxes = True)
plt.show()
#q1 = np.array([0, np.pi/2, 0, 0, 0, 0])
print(robot.fkine(q))