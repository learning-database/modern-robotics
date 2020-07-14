import modern_robotics as mr
import numpy as np
import math

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]


thetaList = np.array([0, math.pi/6, math.pi/4, math.pi/3, math.pi/2, 2 * math.pi/3])
dthetaList = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
ddthetaList = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
g = np.array([0, 0, -9.81])
Ftip = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

massMatrix = mr.MassMatrix(thetaList, Mlist, Glist, Slist)
QuadraticForces = mr.VelQuadraticForces(thetaList, dthetaList, Mlist, Glist, Slist)
gravityForces = mr.GravityForces(thetaList, g, Mlist, Glist, Slist)
# with np.printoptions(precision=3, suppress=True): print mr.EndEffectorForces(thetaList, Ftip, Mlist, Glist, Slist)

tauList = np.array([0.0128, -41.1477, -3.7809, 0.0323, 0.0370, 0.1034])
with np.printoptions(precision=3, suppress=True): print mr.ForwardDynamics(thetaList, dthetaList, tauList, g, Ftip,Mlist, Glist, Slist)