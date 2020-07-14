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
#      Example: In linux, open a terminal and type "python course5_assignment2.py"
#
#################################################################


def forceToWrench(point, theta, force = 1):
    """Given a force an a point on the force action line, returns the wrench associated

    :param point : (x,y) coordinates of a point on force action line
    :param theta : Angle of force in 2D x-y plane (in radians)
    :param force (optional) : Magnitude of force in Newtons (Default 1)
    """

    Fx = force * math.cos(theta)
    Fy = force * math.sin(theta)
    forceVector = [ Fx, Fy, 0] # Force vector
    pointVector = [point[0], point[1], 0] # Vector from origin to contact point
    Mx, My, Mz = np.cross( pointVector, forceVector)

    return [Mz, Fx, Fy]


def checkForceClosure(bodies, contacts):
    """Checks planar force closure given a set of objects and contact information

    :param masses: List of object bodies. Each element should be of format (ID,x,y,mass)
                    i.e. 
                        ID : Body ID (0 to N-1 if the number of bodies are N)
                        (x,y): Location of mass center
                        mass : Mass of the body in kg
    :param contacts: List contact points. Each element should be of format (a,b,x,y,theta,u)
                    i.e. 
                        a,b : Bodies in contact
                        (x,y) : contact location
                        theta : of the contact normal specified in radians 
                                (Contact normal is into body a)
                        u : Coulomb Friction coefficient
    :return result: A logical value where TRUE means force closure
    :return output: Solution of the linear programming problem, if force closure is true.
                    Else null list.

    Examples 
    ========    
    Input:
        bodies = [[1, 25, 35, 2], [2, 66, 42, 10]]
        contacts = [[1, 2, 60, 60, 3.1416, 0.5], [1, 0, 0, 0, 1.5708, 0.5], [2, 0, 60, 0, 1.5708, 0.5], [2, 0, 72, 0, 1.5708, 0.5]]
    Output:
        Force closure successfull. k vector: [ 0.00, 6.21, 3.42, 15.84, 15.04, 12.91, 48.62, 38.34 ]

    Input:
        bodies = [[1, 25, 35, 2], [2, 66, 42, 5]]
        contacts = [[1, 2, 60, 60, 3.1416, 0.5], [1, 0, 0, 0, 1.5708, 0.1], [2, 0, 60, 0, 1.5708, 0.5], [2, 0, 72, 0, 1.5708, 0.5]]
    Output:
        Force closure fail
    """

    bodyCount = len(bodies) + 1 # Body 0 (Stationary Ground) in not provided as a seperate input item
    contactCount = len(contacts)
    wrenchCount = contactCount * 2

    # Generating Ext Wrench matrix (Body Weight Wrenches) Fext : (Mz, Fx, Fy)
    FextList = [0 for x in range(bodyCount)]
    for body in bodies:
        ID, x, y, mass = body

        point = [x,y]
        theta = 3 / 2 * math.pi # Gravity acts downwards - 270 degrees
        force = mass * 10  # Gravitational acceleration taken as 10 m/s2
        FextList[ID] = forceToWrench(point, theta, force)
    
    # Generating List of contact wrench matrices. Each element F  : (Mz, Fx, Fy)
    Flist = [ [[0 for x in range(wrenchCount)] for y in range(3)] for z in range(bodyCount)]
    for contactID in range(contactCount):
        bodyA, bodyB, x, y, theta, u = contacts[contactID]
        frictionAngle = math.atan(u)

        # Friction cone left edge
        WrenchL = forceToWrench((x,y), theta + frictionAngle)
        wrenchLID = contactID * 2
        # Friction cone right edge
        WrenchR = forceToWrench((x,y), theta - frictionAngle)
        wrenchRID = (contactID * 2) + 1
        # bodyA : Contact normal is into body A
        for var in range(3):
            Flist[bodyA][var][wrenchLID] = WrenchL[var]
            Flist[bodyA][var][wrenchRID] = WrenchR[var]
        # bodyB : Contact normal is out of body B
        for var in range(3):
            Flist[bodyB][var][wrenchLID] = -WrenchL[var]
            Flist[bodyB][var][wrenchRID] = -WrenchR[var]
    
    # Remove Stationary ground from Wrench lists
    FextList.pop(0)
    Flist.pop(0)
    
    f = [1 for x in range(wrenchCount)]
    A = [[0 for x in range(wrenchCount)] for y in range(wrenchCount)]
    for x in range(wrenchCount):
        A[x][x] = -1
    b = [0 for x in range(contactCount * 2)]

    Aeq = []
    for F_ in Flist:
        Aeq = Aeq + F_
    beq = []
    for b_ in FextList:
        beq = beq + b_
    beq = map(lambda x: -x, beq)  # Since Fk = -Fext

    lpOutput = lp(f, A, b, Aeq, beq, method='interior-point')
    k = []
    result = lpOutput.get("success")
    if result:
        k = lpOutput.get("x")
    return result, k


print "#### This program will check if a given planar assembly is in force closure ####"

# Reading body descriptions. Each specified by body ID, (x,y) location of body mass center and body mass in kg
print "\nEnter the list of bodies. \nEach element should be of format (ID,x,y,mass):\n",
print "\tBodyID : Unique ID from 0 to Number of bodies"
print "\t(x,y)  : coordinates of body mass center"
print "\t mass  : body mass in Kilograms"
print "Note: BodyID 0 is stationary ground"
print "\ti.e. [[1, 25, 35, 2], [2, 66, 42, 10]]"
bodies = input("Bodies List: ")

# Reading Contact points. Each specified by the (x,y) contact location and the direction of the contact normal
print "\nEnter the list contact points. \nEach element should be of format (bodyA, bodyB, x, y, theta, u):\n",
print "\tContact from bodyB into bodyA"
print "\t(x,y) : Coordinates of contact point"
print "\ttheta : Angle of contact normal into bodyA (in Radians)"
print "\t u : Coulomb Friction Coefficient"
print "\ti.e. \n\t\t[[1, 2, 60, 60, 3.1416, 0.5], \n\t\t[1, 0, 0, 0, 1.5708, 0.5], \n\t\t[2, 0, 60, 0, 1.5708, 0.5], \n\t\t[2, 0, 72, 0, 1.5708, 0.5]]"
contacts = input("Contacts List: ")

result, k = checkForceClosure(bodies, contacts)
if result:
    # Rounding off k
    k = ['%.2f' % ki for ki in k]
    print "\nForce closure successfull. k vector: [ " + ", ".join(map(str, k)) + " ]"
else:
    print "\nForce closure fail"
