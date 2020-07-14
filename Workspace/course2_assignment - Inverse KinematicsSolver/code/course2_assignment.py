import modern_robotics as mr
import numpy as np
import math
import csv

################# Instructions to Run the code ##################
# Make sure you have python 2 installed with following libraries.
#       Numpy
#       Modern Robotics library
#       CSV Library
#
# Run the python file
#      Example: In linux, open a terminal and type "python course2_assignment.py"
#
# The code will automatically generate a CSV file named iterates.csv
# in the execution directory.

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    print "Starting Inverse Kinematic Calculation in Body Frame, with initial joint vector: ", thetalist

    i = 0
    maxiterations = 20
    err = True 
    JointVectorMatrix = []
    
    while i < maxiterations:
        print "========== Iteration ", i," =========="
        print "Joint Vector: ", thetalist
        JointVectorMatrix.append(thetalist.tolist())
        

        # Calculating forward kinematics (Current Configuration) : Tsb
        Tsb = mr.FKinBody(M, Blist, thetalist)
        print "SE(3) end-effector config: "
        with np.printoptions(precision=3, suppress=True): print(Tsb)

        # Calculating error Twist : V
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
        print "Error twist V_b: "
        with np.printoptions(precision=3, suppress=True): print(Vb)

        #Calculating error
        angularVelError = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        linearVelError = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err =  angularVelError > eomg or linearVelError > ev
        print "Angular error magnitude ||omega_b||: ", angularVelError
        print "Linear error magnitude ||v_b||: ", linearVelError

        if err:
            # Error is not within acceptable limits. Calculate new joint angles
            thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
            i = i + 1
        else:
            # Error is within acceptable limits. End iterations
            break
    
    # Save joint vectors to CSV file
    np.savetxt("iterates.csv", np.array(JointVectorMatrix), delimiter=",")
    return (thetalist, not err)


# Robot link dimensions in meters
w1 = 0.109
w2 = 0.082
l1 = 0.425
l2 = 0.392
h1 = 0.089
h2 = 0.095

Blist = np.array([[    0,      0,    0,    0,    0,    0],
                  [    1,      0,    0,    0,   -1,    0],
                  [    0,      1,    1,    1,    0,    1],
                  [w1+w2,     h2,   h2,   h2,  -w2,    0],
                  [    0, -l1-l2,  -l2,    0,    0,    0],
                  [l1+l2,      0,    0,    0,    0,    0]])

M = np.array([[-1, 0, 0, l1+l2],
              [ 0, 0, 1, w1+w2],
              [ 0, 1, 0, h1-h2],
              [ 0, 0, 0,   1  ]])

T = ([[  0,  1,  0, -0.5],
      [  0,  0, -1,  0.1],
      [ -1,  0,  0,  0.1],
      [  0,  0,  0,  1  ]])

thetaList0 = np.array([2.512,-1.326,1.954,-0.488,5.582,-1.465])
errOmg = 0.001  
errV =  0.0001

solution = IKinBodyIterates(Blist, M, T, thetaList0, errOmg, errV)
if(solution[1]): print ("Inverse Kinematics Converged Successfully")

