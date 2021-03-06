import modern_robotics as mr
import numpy as np
import math

M = np.array([[1,0,0,3.732],[0,1,0,0],[0,0,1,2.732],[0,0,0,1]])
Slist = np.array([[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,0,1,-0.732,0,0],[-1,0,0,0,0,-3.732],[0,1,2.732,3.732,1,0]])
Blist = np.array([[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,2.732,3.732,2,0,0],[2.732,0,0,0,0,0],[0,-2.732,-1,0,1,0]])
thetaList = np.array([-math.pi/2, math.pi/2, math.pi/3, -math.pi/4, 1, math.pi/6])

R = mr.FKinSpace(M, Slist, thetaList)
T = mr.FKinBody(M, Blist, thetaList)

print R
print T