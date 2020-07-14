from __future__ import division
import modern_robotics as mr
import numpy as np
import math

################# Feedback controller for youBot ##################

# Error matrix se3 twist : [V]
XerrSum = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Kp = [ [0.0 for x in range(6)] for y in range(6) ]
Ki = [ [0.0 for x in range(6)] for y in range(6) ]

def FeedbackControl(X, Xd, Xd_next, gains, timeStep):
    """Calculates the kinematic task-space feedforward plus feedback control law
       
         :param Tse: The current actual end-effector configuration X (also written Tse)
         :param Tse_d: The current end-effector reference configuration Xd (i.e., Tse,d)
         :param Tse_dNext: The end-effector reference configuration at the next timestep in the reference trajectory, 
                       Xd,next (i.e., Tse,d,next), at a time delta_t later
         :param gains: The PI gain matrices Kp and Ki
         :param timeStep: The timestep delta_t between reference trajectory configurations

         :return: The commanded end-effector twist V expressed in the end-effector frame {e}
                  and Xerr

         Calculates the kinematic task-space feedforward plus feedback control law, 
         written both as Equation (11.16) and (13.37) in the textbook
    """

    global XerrSum

    Terr = np.dot(np.linalg.inv(X), Xd)
    Xerr_se3 = mr.MatrixLog6(Terr)
    Xerr = mr.se3ToVec(Xerr_se3)
    Ad_Terr = mr.Adjoint(Terr)

    # Updating integral
    XerrSum  = XerrSum + (Xerr * timeStep)

    T_dNext = np.dot( np.linalg.inv(Xd), Xd_next)
    Vd_se3 = mr.MatrixLog6(T_dNext)
    # Time scaling Vd_se3
    Vd_se3 = Vd_se3 / timeStep  
    Vd = mr.se3ToVec(Vd_se3)

    # Generating gain matrices
    kp = gains[0]
    ki = gains[1]

    for x in range(6):
        Kp[x][x] = kp
        Ki[x][x] = ki

    # Control law    
    V = np.dot(Ad_Terr, Vd) + np.dot(Kp, Xerr) + np.dot(Ki, XerrSum)

    return V, Xerr


