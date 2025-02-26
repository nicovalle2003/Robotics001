import sympy as sp
from sympy.matrices import rot_axis3
from spatialmath import *
from spatialmath.base import *
import matplotlib.pyplot as plt 
import numpy as np
import roboticstoolbox as rtb
from matplotlib.widgets import Slider

theta, d, a, alpha = sp.symbols('theta, d, a, alpha')
T = sp.Matrix([
[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
[sp.sin(theta), sp.cos(theta)* sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
[0, sp.sin(alpha), sp.cos(alpha), d],
[0, 0, 0, 1]
])

#theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1, theta2, theta3, theta4, theta5, theta6')

#print("Denavit-Hartenberg Parameters NO TOOLBOX: ")

#T01 = T.subs({d: 0.290, a: 0, alpha: sp.pi/2})
#T01 = T01.subs({theta: theta1})
#sp.pprint(T01)

#T12 = T.subs({d: 0, a: 0.270, alpha: 0}) #theta2 + sp.pi/2
#T12 = T12.subs({theta: theta2})
#sp.pprint(T12)

#T23 = T.subs({d: 0, a: 0.070, alpha: sp.pi/2})
#T23 = T23.subs({theta: theta3})
#sp.pprint(T23)

#T34 = T.subs({d:0.302, a: 0, alpha: -sp.pi/2})
#T34 = T34.subs({theta: theta4})
#sp.pprint(T34)

#T45 = T.subs({d: 0, a: 0, alpha: sp.pi/2})
#T45 = T45.subs({theta: theta5})
#sp.pprint(T45)

#T56 = T.subs({d: 0.072, a: 0, alpha: 0})
#T56 = T56.subs({theta: theta6})
#sp.pprint(T56)

#T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
#T06_s = T06.applyfunc(sp.simplify)

joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)  #Offset
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0)
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

#T06_Solved = T06_s.subs({theta1: joint1, theta2: joint2, theta3: joint3, theta4: joint4, theta5: joint5, theta6: joint6})
#sp.pprint(T06_Solved)

print("Denavit-Hartenberg Parameters WITH TOOLBOX: ")

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
q =np.array([joint1, joint2, joint3, joint4, joint5, joint6])
robot.plot(q=q, backend = 'pyplot', dt = 10, limits = [-0.8,0.8,-0.8,0.8,-0.4,0.6,], shadow = True, jointaxes = True)
plt.show()
#q1 = np.array([0, np.pi/2, 0, 0, 0, 0])
print(robot.fkine(q))