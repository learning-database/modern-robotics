import modern_robotics as mr
import numpy as np
import math

Slist = np.array([[0,0,1,0,0,0],[1,0,0,0,2,0],[0,0,0,0,1,0]]).T
thetaList = np.array([math.pi/2,math.pi/2,1])

Blist = np.array([[0,1,0,3,0,0],[-1,0,0,0,3,0],[0,0,0,0,0,1]]).T

Js = mr.JacobianSpace(Slist, thetaList)
Jb = mr.JacobianBody(Blist, thetaList)

# Slist = np.array([[0, 0, 1,   0, 0.2, 0.2],  
#                       [1, 0, 0,   2,   0,   3],  
#                       [0, 1, 0,   0,   2,   1],  
#                       [1, 0, 0, 0.2, 0.3, 0.4]]).T  
# thetalist = np.array([0.2, 1.1, 0.1, 1.2])  


Jvel = np.array([[-0.105,0,0.006,-0.045,0,0.006,0],[-0.889,0.006,0,-0.844,0.006,0,0],[0,-0.105,0.889,0,0,0,0]])
A = np.dot(Jvel, Jvel.T)
eigenA = np.linalg.eig(A)
print eigenA
