import modern_robotics as mr
import numpy as np
import math

def forwardFunction(input):
    result = np.array([0.0,0.0])
    result[0] = math.pow(input[0],2) - 9 
    result[1] = math.pow(input[1],2) - 4
    return result

def pseudoInverse(input):
    x = input[0]
    y = input [1]
    J = np.array([[2*x, 0], [0, 2*y]])
    pseudoInverse = np.linalg.inv(J)
    return pseudoInverse

# Question 1
# f(x,y) = [x^2 - 9, y^2 - 4]
# Initial Guess
solution1 = np.array([1,1])
goal = np.array([0,0])

print solution1
for i in range (2):
    solution1 = solution1 + np.dot(pseudoInverse(solution1), (goal - forwardFunction(solution1)) )
    print solution1


# Question 2
Slist = np.array([[0,0,0],[0,0,0],[1,1,1],[0,0,0],[0,-1,-2],[0,0,0]])
M = np.array([[1,0,0,3],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
Tsd = np.array([[-0.585, -0.811, 0, 0.076],[0.811, -0.585, 0, 2.608],[0, 0, 1, 0],[0, 0, 0, 1]])
thetaList = np.array([math.pi/4, math.pi/4, math.pi/4])


Sol = mr.IKinSpace(Slist, M, Tsd, thetaList, 0.001, 0.0001)
print Sol








