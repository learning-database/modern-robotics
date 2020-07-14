from __future__ import division
from scipy.optimize import linprog as lp
import numpy as np
import math

################# Instructions to Run the code ##################
# Make sure you have python 2 installed with following libraries.
#       scipy
#       numpy
#
# Run the python file
#      Example: In linux, open a terminal and type "python course5_assignment1.py"
#
#################################################################

def checkFormClosure(contacts):
    """Checks planar form closure given a set of contact points and contact normals

    :param contacts: List contact points. Each element should be of format (x,y,theta)
                    i.e. (x,y) contact location
                        and the direction of the contact normal specified by theta (radians)
    :return result: A logical value where TRUE means form closure
    :return output: Solution of the linear programming problem, if form closure is true.
                    Else null list.

    Example Input:
        contacts = [[3,5,4.712],[3,5,0.785],[12,5,4.712],[12,5,0.785]]
    Output:
    """

    contactCount = len(contacts)

    # Generating wrench matrix F  : (Mz, Fx, Fy)
    F = [[0 for x in range(contactCount)] for y in range(3)]
    for contactID in range(contactCount):
        x, y, theta = contacts[contactID]

        Fx = math.cos(theta)
        Fy = math.sin(theta)
        forceVector = [ Fx, Fy, 0] # Force vector
        pointVector = [x, y, 0] # Vector from origin to contact point
        M = np.cross( pointVector, forceVector)
        Mx, My, Mz = M

        F[0][contactID] = Mz
        F[1][contactID] = Fx
        F[2][contactID] = Fy
    

    
    f = [1 for x in range(contactCount)]
    A = [[0 for x in range(contactCount)] for y in range(contactCount)]
    for x in range(contactCount):
        A[x][x] = -1
    b = [-1 for x in range(contactCount)]

    Aeq = F
    beq = [0 for x in range(3)]

    lpOutput = lp(f, A, b, Aeq, beq)
    k = []
    result = lpOutput.get("success")
    if result:
        k = lpOutput.get("x")
    return result, k


print "#### This program will check if a given planar body is in form closure with respect to a given set of stationary contacts ####\n"

# Reading Contact points. Each specified by the (x,y) contact location and the direction of the contact normal
print "Enter the list contact points. Each element should be of format (x,y,theta):\n",
print "(x,y) contact location and the direction of the contact normal specified by theta(radians)"
print "\ti.e. [[1,2,0],[2,-2,3.14]]"
contacts = input("Contacts List: ")

result, k = checkFormClosure(contacts)
if result:
    # Rounding off k
    k = ['%.2f' % ki for ki in k]
    print "Form closure successfull. k vector: [ " + ", ".join(map(str, k)) + " ]"
else:
    print "Form closure fail"
