from __future__ import division
import modern_robotics as mr
import numpy as np
import math

################# Kinematic Model of the youBot ##################

# Body Screw Axes list at home configuration
B1 = [0,  0, 1,  0,      0.033, 0]
B2 = [0, -1, 0, -0.5076, 0,     0]
B3 = [0, -1, 0, -0.3526, 0,     0]
B4 = [0, -1, 0, -0.2176, 0,     0]
B5 = [0,  0, 1,  0,      0,     0]
Blist = np.array([B1, B2, B3, B4, B5]).T

# Home configuration Transform
M = [[1, 0, 0, 0.033    ], 
        [0, 1, 0, 0        ], 
        [0, 0, 1, 0.6546   ], 
        [0, 0, 0, 1        ]]

# Robot base to Arm base transform
Tb0 = [ [1, 0, 0, 0.1662], 
        [0, 1, 0, 0     ], 
        [0, 0, 1, 0.0026], 
        [0, 0, 0, 1     ]]

# u = H * V (U : wheel speeds, V : body twist)
l = 0.47 / 2
w = 0.3 / 2
r = 0.0475
H = np.array([  [ (-l-w)/r,    1/r,    -1/r],
                [ ( l+w)/r,    1/r,     1/r],
                [ ( l+w)/r,    1/r,    -1/r],
                [ (-l-w)/r,    1/r,     1/r]])

jointLimits = [ [-0.5,   +0.5],
                [-1.117, +1.1],
                [-2.0,   +0.2],     #-2.0,   -0.2
                [-0.6,  +1.78],    #+0.01,  +1.78
                [-2.89,  +2.89]]
                # TODO

def configurationToTransform(configuration):
    """Converts the youBot trajectory configuration to a homogenius transformation matrix

    :param configuration: r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state (optional)
    :return: Homogeneous Transformation matrix corresponding to given configuration
    """
    T = np.zeros((4,4))
    T[0, :3] = configuration[0:3]
    T[1, :3] = configuration[3:6]
    T[2, :3] = configuration[6:9]
    T[:3, 3] = configuration[9:12]
    T[3][3]  = 1
    return T

def poseToTransform(pose):
    """Converts a 3D pose (Wx, Wy, Wz, x, y, z) to a homogeneous transformation matrix

    :param pose: 2D pose [Wx, Wy, Wz, x, y, z]
                    Wx, Wy, Wz : Rotation angles
                    x, y, z : Position
    :return: Homogeneous Transformation matrix corresponding to given pose
    """
    [Wx, Wy, Wz, x, y, z] = pose
    rotation = [Wx, Wy, Wz]
    position = [x, y, z]
    R_so3 = mr.VecToso3(rotation)
    R = mr.MatrixExp3(R_so3)
    T = mr.RpToTrans(R, position)
    return T

def transformToPose(T):
    """Converts a homogeneous transformation matrix to a 3D pose (Wx, Wy, Wz, x, y, z) 6-vector 

    :param: Homogeneous Transformation matrix corresponding to given pose
    :return pose: 2D pose [Wx, Wy, Wz, x, y, z]
                    Wx, Wy, Wz : Rotation angles
                    x, y, z : Position
    """
    R, position = mr.TransToRp(T)
    R_so3 = mr.MatrixLog3(R)
    rotation = mr.so3ToVec(R_so3)
    return np.concatenate((rotation, position))


def youBot_Tbe(thetaList):
    """Computes the transform of youBot End effector in robot base frame

    :param configuration: A 5-vector representing the current configuration of the robot arm. 
                                5 variables for the arm configuration (J1, J2, J3, J4, J5), 
    :return: Transform of End effector in base frame
    """

    T0e = mr.FKinBody(M, Blist, thetaList)
    Tbe = np.dot(Tb0, T0e)
    return Tbe

def youBot_Tbase(theta,x,y):
    """Computes the transform of youBot base given a base configuration

    :param configuration: A 3-vector representing the current configuration of the robot base. 
                                    3 variables for the chassis configuration (theta, x, y), 
    :return: Transform of robot base in space frame
    """
    rotation = [0, 0, theta]
    position = [x, y, 0.0963]
    R_so3 = mr.VecToso3(rotation)
    R = mr.MatrixExp3(R_so3)
    Tbase = mr.RpToTrans(R, position)
    return Tbase

def youBot_Tse(configuration):
    """Computes the transform of youBot End effector given a robot configuration

    :param configuration: A 12-vector representing the current configuration of the robot. 
                                    3 variables for the chassis configuration (theta, x, y), 
                                    5 variables for the arm configuration (J1, J2, J3, J4, J5), 
                                    and 4 variables for the wheel angles (W1, W2, W3, W4)
    :return: Transform of End effector in space frame
    """
    theta, x, y = configuration[0:3]
    rotation = [0, 0, theta]
    position = [x, y, 0.0963]
    R_so3 = mr.VecToso3(rotation)
    R = mr.MatrixExp3(R_so3)
    Tsb = mr.RpToTrans(R, position)
    Tbe = youBot_Tbe(configuration[3:8])
    Tse = np.dot(Tsb, Tbe)
    return Tse

def baseConfiguration(Tbase):
    """Computes the configuration of youBot base given a base transformation in space frame

    :param configuration: Transform of robot base in space frame
    :return: A 3-vector representing the current configuration of the robot base. 
                3 variables for the chassis configuration (theta, x, y), 
    """
    R, position = mr.TransToRp(Tbase)
    R_so3 = mr.MatrixLog3(R)
    rotation = mr.so3ToVec(R_so3)
    theta = rotation[2]
    x, y = position[0:2]
    return theta, x, y

def youBot_Je(configuration):
    """Computes the jacobian of youBot End effector given a robot configuration

    :param configuration: A 12-vector representing the current configuration of the robot. 
                                    3 variables for the chassis configuration (theta, x, y), 
                                    5 variables for the arm configuration (J1, J2, J3, J4, J5), 
                                    and 4 variables for the wheel angles (W1, W2, W3, W4)
    :return: Robot jacobian [Jbase, Jb_arm]
    """

    # Generating Arm body jacobian
    thetaList = configuration[3:8]
    Jarm = mr.JacobianBody(Blist, thetaList)

    # Generating Jbase
    F3 = np.linalg.pinv(H)
    F6 = np.zeros((6, 4))
    F6[2:5,:] = F3
    Tbe = youBot_Tbe(thetaList)
    Teb = np.linalg.inv(Tbe)
    Ad_Teb = mr.Adjoint(Teb)
    Jbase = np.dot(Ad_Teb, F6)
    Je = np.concatenate((Jbase,Jarm),axis=1)

    return Je

def youBot_testJointLimits(thetaList):
    """Checks if the provided robot arm configuration is exceeding joint limits

    :param configuration: A 5-vector representing the current configuration of the robot arm 
                                5 variables for the arm configuration (J1, J2, J3, J4, J5), 
    :return: status, jointVector

            status: Boolean value. 
                    True - No jointlimit exceeded.
                    False - One or more joint values exceeding limits 
            JointVector: A 5-vector array indicating which joint is exeeding collision limits.
                            1 - Within limits. 0 - Exceeds joint limits
                            ex. [1, 1, 0, 1, 1]
                                Joint 3 is exeeding joint limits
    """
    results = []
    status = True
    for x in range(len(thetaList)):
        jointLimitLow = jointLimits[x][0]
        jointLimitHigh = jointLimits[x][1]
        jointValue = thetaList[x]
        if (jointValue < jointLimitLow) or (jointLimitHigh < jointValue):
            results.append(0)
            status = False
        else:
            results.append(1)
    return status, results

def youBot_selfCollisionCheck(thetaList):
    """Checks if the provided robot arm configuration is in self collision with youBot

    :param configuration: A 5-vector representing the current configuration of the robot arm 
                                5 variables for the arm configuration (J1, J2, J3, J4, J5), 
    :return: A boolean value, status
                    True - No self collision.
                    False - Self collision

    This funtion checks whether the end effector of the youBot arm is not in restricted / self colliding 
    areas. The restricted area is designed such that, if the end effector position is in a non restricted area, 
    the arm will not be in self collision with the robot or the arm links themselves. 

    The restricted area is modeled as a combination of a cuboid (representing robot base) 
    and a cylinder (representing arm base)
    """
    noCollision = True

    T0e = mr.FKinBody(M, Blist, thetaList)
    r, [x,y,z] = mr.TransToRp(T0e)

    # T0e position must not be in the cylinder region of the robot arm base.
    # Cylinder region radius : 0.1562m. Height 0.3604m
    min_radius = 0.1562
    min_height = 0.3604
    radial_distance = math.sqrt ( math.pow(x, 2) + math.pow(y, 2) )
    if (radial_distance < min_radius) and (z < min_height):
        noCollision = False
        return noCollision
    
    Tbe = np.dot(Tb0, T0e)
    r, [x,y,z] = mr.TransToRp(T0e)
    # Tbe position must not be in the cuboid region of the youBot base
    # Cuboid region coordinates:
    x_min = -0.35
    x_max = +0.35
    y_min = -0.336
    y_max = +0.336
    z_min = 0
    z_max = 0.175

    if ( (x_min < x) and (x < x_max)) and ((y_min < y) and (y < y_max)) and ((z_min < z and z < z_max)):
        noCollision = False
        return noCollision
    
    return noCollision