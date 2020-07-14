import numpy as np 
import math
import modern_robotics as mr

#Q4

H0 = np.array([ [-5, 1, -1],
                [ 5, 1,  1],
                [ 5, 1, -1],
                [-5, 1,  1]])

u = np.array([-1.18, 0.68, 0.02, -0.52])

Hinverse = np.linalg.pinv(H0)
V = np.dot(Hinverse, u)
# print V

#Q5
V6 = np.array([0, 0, V[0], V[1], V[2], 0])
se3mat = mr.VecTose3(V6)
T = mr.MatrixExp6(se3mat)
R, p = mr.TransToRp(T)
so3mat = mr.MatrixLog3(R)
omega = mr.so3ToVec(so3mat)
# print p
# print omega

#Q06
F = np.array([  [ 0,     0    ],
                [ 0,     0    ],
                [-0.25,  0.25 ],
                [ 0.25,  0.25 ],
                [ 0,     0    ],
                [ 0,     0    ]])
# Finding Tbe
omega = np.array([0, 0, math.pi/2])
p = np.array([2, 3, 0])
so3mat = mr.VecToso3(omega)
R = mr.MatrixExp3(so3mat)
Tbe = mr.RpToTrans(R, p)
Teb = np.linalg.inv(Tbe)
Ad_Teb = mr.Adjoint(Teb)
Jbase = np.dot(Ad_Teb, F)
print Tbe
print Jbase

# #Q07
Jb = [0, 0, 1, -3, 0, 0]
Jx = [0, 0, 1, 0, -2, 0]
# print np.dot(Ad_Teb, Jx)

# [0,-1,0,2;1,0,0,3;0,0,1,0;0,0,0,1]
# 0, 1, 0, -3, 
# -1, 0, 0, 2
# 0, 0, 1, 0,
# 0, 0, 0, 1

# 0, 1, 0, 0, 0, 0, 
# -1, 0, 0, 0, 0, 0, 
# 0, 0, 1, 0, 0, 0, 
# 0, 0, 2, 0, 1, 0,
# 0, 0, 3, -1, 0, 0,
# 3, -2, 0, 0, 0, 1

