import modern_robotics as mr
import numpy as np
import math

print mr.QuinticTimeScaling(5, 3)


Xstart = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
Xend = np.array([[0,0,1,1],[1,0,0,2],[0,1,0,3],[0,0,0,1]])

# tragectoryPoints = mr.ScrewTrajectory(Xstart, Xend, 10, 10, 3)
tragectoryPoints = mr.CartesianTrajectory(Xstart, Xend, 10, 10, 5)
with np.printoptions(precision=3, suppress=True): print tragectoryPoints[8]

