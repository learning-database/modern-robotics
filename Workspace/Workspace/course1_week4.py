import modern_robotics as mr
import numpy as np
import math

Tsa = [[0,-1,0,0],[0,0,-1,0],[1,0,0,1],[0,0,0,1]]
Tsb = [[1,0,0,0],[0,0,1,2],[0,-1,0,0],[0,0,0,1]]

Tas = [[0,0,1,-1],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]
Tbs = [[1,0,0,0],[0,0,-1,0],[0,1,0,-2],[0,0,0,1]]

Vs = np.array([3,2,1,-1,-2,-3])

AdjointTas = mr.Adjoint(Tas)
AdjointTbsTranspose = np.transpose(mr.Adjoint(Tbs))

Fb = np.array([1,0,0,2,1,0])
T = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])

V = np.array([1,0,0,0,2,3])

q = np.array([0,0,2])
h = 1
s = np.array([1,0,0])
ScrewAxis = mr.ScrewToAxis(q, s, h)

matExponent14 = np.array([[0, -1.5708, 0, 2.3562],[1.5708, 0, 0, -2.3562], [0, 0, 0, 1], [0, 0, 0, 0]])
T14 = mr.MatrixExp6(matExponent14)

T15 = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])
matExponent15 = mr.MatrixLog6(T15)
print matExponent15