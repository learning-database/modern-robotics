import modern_robotics as mr
import numpy as np
import math

so3mat = np.array([ [0, 0, 1],
                    [-1, 0, 0],
                    [0, -1, 0]])


Tsa = np.array([[0,-1,0,0],
                [0,0,-1,0],
                [1,0,0,1],
                [0,0,0,1]])

Tsb = np.array([[1,0,0,0],
                [0,0,1,2],
                [0,-1,0,0],
                [0,0,0,1]])

PointB = np.array([1,2,3,1])

# Vs = ([3,2,1,-1,-2,-3])


# omegaTheta = mr.VecToso3(np.array([0,1,2]))

# Theta = 2.236067977

# print mr.MatrixExp3(omegaTheta)
# print ( np.identity(3) * Theta + (1-math.cos(Theta))*omegaTheta + (Theta-math.sin(Theta)) * (omegaTheta * omegaTheta)) * np.transpose(np.array([3,0,0]))
# print mr.MatrixExp6(mr.VecTose3(np.array([0,1,2,3,0,0])))

#R = mr.MatrixLog6(Tsa)
#R = mr.Adjoint(mr.TransInv(Tsa)) * np.transpose(Vs)
# R = Tsa * PointB
#R = mr.TransInv(Tsb) 
#R = mr.MatrixLog3(so3mat)
# print R

omega = np.array([1/math.sqrt(5), 2/math.sqrt(5), 0])
theta = math.sqrt(5)

i = np.identity(3)
R = i + mr.VecToso3(omega) * math.sin(theta)  + np.dot(mr.VecToso3(omega), mr.VecToso3(omega)) * (1-math.cos(theta))

print R