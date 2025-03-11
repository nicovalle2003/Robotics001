import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from VerTrayectoria import plot_robot_trajectory

d1, a1, a2, a3, d4, d6 = 1.180, 0.150, 0.790, 0.150, 0.860, 0.100
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=float(d1), a=0, alpha=float(alpha1), offset=0, qlim=[-2.87979, 2.87979]),
        rtb.RevoluteDH(d=0, a=float(a2), alpha=float(alpha2), offset=float(np.pi/2), qlim=[-1.91986,1.91986]),
        rtb.RevoluteDH(d=0, a=float(a3), alpha=float(alpha3), offset=0, qlim=[-1.91986,1.22173]),
        rtb.RevoluteDH(d=float(d4), a=0, alpha=float(alpha4), offset=0,qlim=[-2.79253,2.79253]),
        rtb.RevoluteDH(d=0, a=0, alpha=float(alpha5), offset=0, qlim=[-2.0944,2.0944]),
        rtb.RevoluteDH(d=float(d6), a=0, alpha=float(alpha6), offset=0, qlim=[-6.98132,6.98132])
    ], name = "Fanuc CR-35ia", base = SE3(0, 0, 0)
)
print(robot)
q1 = np.array([0, np.pi/2, 0, 0, 0, 0])
#robot.teach(q1)

robot.tool = SE3.OA([1, 0, 0], [0, -1, 0])
robot.configurations_str('ru')  # right, elbow up
robot.qz = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # zero angles
robot.plot(robot.qz, limits=[-0.8, 0.8, -0.8, 0.8, -0.1, 2.5], eeframe=True, 
           backend='pyplot', shadow=True, jointaxes=False, block=True)

J_1 = np.array([
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], #1
    [0.0, 1.39, -19.98, 0.0, 18.59, 0.0], #2
    [18.32, -2.03, -16.54, 46.11, 25.86, -43.09], # 3
    [18.32, -3.45, 3.54, 90.28, 18.32, -90.30], # 4
    [26.34, 15.91, -13.94, 93.97, 26.41, -94.43], # 5
    [26.34, 18.16, -34.27, 60.73, 30.57, -56.94], # 6
    [0.0, 23.56, -37.99, 0.0, 14.44, 0.0], # 7
    [0.0, 20.70, -17.37, 0.0, -3.33, 0.0], # 8
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # 1
    [18.32, -3.45, 3.54, 90.28, 18.32, -90.30], #4
    [26.34, 15.91, -13.94, 93.97, 26.41, -94.43], #5
    [0.0, 20.70, -17.37, 0.0, -3.33, 0.0], #8
    [0.0, 23.56, -37.99, 0.0, 14.44, 0.0], #7
    [0.0, 1.39, -19.98, 0.0, 18.59, 0.0], #2
    [18.32, -2.03, -16.54, 46.11, 25.86, -43.09], #3
    [26.34, 18.16, -34.27, 60.73, 30.57, -56.94], #6
])

J = np.deg2rad(J_1)

p_lim=[-1, 1, -1, 1, -0.15, 1.5]
plot_robot_trajectory(
    robot=robot,
    q_trajectory = J,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.15,
    block=True
)