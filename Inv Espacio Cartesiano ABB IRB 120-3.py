import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from VerTrayectoria import plot_robot_trajectory

d1, a2, a3, d4, d6 = 0.290, 0.270, 0.070, 0.302, 0.072
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=float(d1), a=0, alpha=float(alpha1), offset=0, qlim=[-2.87979, 2.87979]),
        rtb.RevoluteDH(d=0, a=float(a2), alpha=float(alpha2), offset=float(np.pi/2), qlim=[-1.91986,1.91986]),
        rtb.RevoluteDH(d=0, a=float(a3), alpha=float(alpha3), offset=0, qlim=[-1.91986,1.22173]),
        rtb.RevoluteDH(d=float(d4), a=0, alpha=float(alpha4), offset=0,qlim=[-2.79253,2.79253]),
        rtb.RevoluteDH(d=0, a=0, alpha=float(alpha5), offset=0, qlim=[-2.0944,2.0944]),
        rtb.RevoluteDH(d=float(d6), a=0, alpha=float(alpha6), offset=0, qlim=[-6.98132,6.98132])
    ], name = "ABB IRB 120-3/0.6", base = SE3(0, 0, 0)
)
print(robot)
q1 = np.array([0, 0, 0, 0, 0, 0])
robot.teach(q1)
print(robot.fkine(q1))
# joint1 = 0  # In degrees
# joint2 = 0
# joint3 = 0
# joint4 = 0
# joint5 = 0
# joint6 = 0

# T = robot.fkine([np.deg2rad(joint1), np.deg2rad(joint2), np.deg2rad(joint3), 
#                  np.deg2rad(joint4), np.deg2rad(joint5), np.deg2rad(joint6)])
# Tn = np.array(T).astype(np.float64)
# x, y, z = Tn[0, 3], Tn[1, 3], Tn[2, 3]
# print(f"Los ángulos en la Cdirecta son ({joint1 :.2f}, {joint2 :.2f}, {joint3 :.2f}, {joint4 :.2f}, {joint5 :.2f}, {joint6 :.2f}) que pasamos a la inversa...")
# print(f"La coordenada es ({x :.3f}, {y :.3f}, {z :.3f}) que pasamos a la inversa...")

robot.tool = SE3.OA([1, 0, 0], [0, -1, 0])
robot.configurations_str('ru')  # right, elbow up
robot.qz = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # zero angles
robot.plot(robot.qz, limits=[-0.8, 0.8, -0.8, 0.8, -0.1, 1], eeframe=True, 
           backend='pyplot', shadow=True, jointaxes=True, block=True)

T = np.array([
    [0.374, 0.000, 0.630],  # 1 Check
    [0.374, 0.000, 0.630 + 0.1],  # 2 Check
    [0.374, 0.000 + 0.1, 0.630 + 0.1],  # 3 Check
    [0.374, 0.000 + 0.1, 0.630],  # 4 Check
    [0.374 + 0.1, 0.000 + 0.1, 0.630],  # 5 Check
    [0.374 + 0.1, 0.000 + 0.1, 0.630 + 0.1],  # 6 Check
    [0.374 + 0.1, 0.000, 0.630 + 0.1],  # 7 Check
    [0.374 + 0.1, 0.000, 0.630],  # 8
    [0.374, 0.000, 0.630],  # 1
    [0.374, 0.000 + 0.1, 0.630], # 4
    [0.374 + 0.1, 0.000 + 0.1, 0.630], # 5
    [0.374 + 0.1, 0.000, 0.630], # 8
    [0.374 + 0.1, 0.000, 0.630 + 0.1], # 7
    [0.374, 0.000, 0.630 + 0.1], # 2
    [0.374, 0.000 + 0.1, 0.630 + 0.1], # 3
    [0.374 + 0.1, 0.000 + 0.1, 0.630 + 0.1] # 6
])

via = np.empty((0, 3))
for punto in T:
    xyz = np.array(punto)
    via = np.vstack((via, xyz))

xyz_traj = rtb.mstraj(via, qdmax=[0.5, 0.5, 0.5], dt=0.25, tacc=0.5).q

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
plt.plot(xyz_traj[:, 0], xyz_traj[:, 1], xyz_traj[:, 2])
ax.scatter(xyz_traj[0, 0], xyz_traj[0, 1], xyz_traj[0, 2], color='red', marker='*')  # Start
ax.scatter(xyz_traj[-1, 0], xyz_traj[-1, 1], xyz_traj[-1, 2], color='blue', marker='o')  # End
plt.show()

T_Tool2 = [SE3(-0.15, 0, 0) * SE3(pose) for pose in T]
T_Tool2_array = np.array([pose.A for pose in T_Tool2])
sol2 = robot.ikine_LM(T_Tool2_array, q0=robot.qz, mask=[1, 1, 1, 0, 0, 0], ilimit=3000, slimit=500)
print('Método 1, sin trayectoria continua')
print(sol2.success)

T_Tool = SE3(-0.15, 0, 0) * SE3.Trans(xyz_traj) #SE3.OA([0, -1, 0], [1, 0, 0])
sol = robot.ikine_LM(T_Tool, q0=robot.qz, mask=[1, 1, 1, 0, 0, 0])
print('Método 2, usando mstraj')
print(sol.success)
#robot.plot(q=sol.q, backend='pyplot', block=True, limits=[-0.8, 0.8, -0.8, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=True)
#robot.plot(q=sol2.q, backend='pyplot', block=True, limits=[-0.8, 0.8, -0.8, 0.8, -0.1, 1], eeframe=True, shadow=True, jointaxes=True)

# if not sol2.success:
#     print("Inverse kinematics failed for sol2.")
#     print("Target pose:", SE3(T))
#     print("Initial guess:", robot.qz)
#     print("Solution:", sol2.q)
#     print("Residual:", sol2.residual)
#     print("Reason:", sol2.reason)

p_lim=[-1, 1, -1, 1, -0.15, 1.5]
plot_robot_trajectory(
    robot=robot,
    q_trajectory=sol2.q,
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

plot_robot_trajectory(
    robot=robot,
    q_trajectory=sol.q,
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