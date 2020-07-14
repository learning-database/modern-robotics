from __future__ import division
import modern_robotics as mr
import kinematic_model
import numpy as np
import math

################# simulator for the kinematics of the youBot ##################
def velocityLimiter(control, velocityLimits):
        """ Limits the control values to [-limit, +limit]
        """
        for x in range(len(control)):
            if (control[x] > velocityLimits[x]):
                control[x] = velocityLimits[x]
            if (control[x] < -velocityLimits[x]):
                control[x] = -velocityLimits[x]


def NextState(configuration, control, timeStep, velocityLimits = [1000 for x in range(12)] ):
    """Computes the configuration of youBot after a timeStep

    :param configuration: A 12-vector representing the current configuration of the robot. 
                                    3 variables for the chassis configuration (theta, x, y), 
                                    5 variables for the arm configuration (J1, J2, J3, J4, J5), 
                                    and 4 variables for the wheel angles (W1, W2, W3, W4)
    :param control: A 9-vector of controls indicating the arm joint speeds 
                        omega (5 variables) 
                        wheel speeds u (4 variables)
    :param timeStep: A timestep delta_t
    :param velocityLimits (optional): A positive real value vector indicating the maximum angular speed of the arm joints and the wheels. 
                                        omega (5 variables) 
                                        wheel speeds u (4 variables)
                                    If a value is not provided, default will be set to 1000 (Approx. no velocity limit)
    
    :return nextConfiguration: A 12-vector representing the configuration of the robot time delta_t later
                                    3 variables for the chassis configuration (theta, x, y), 
                                    5 variables for the arm configuration (J1, J2, J3, J4, J5), 
                                    and 4 variables for the wheel angles (W1, W2, W3, W4)
    
    The function NextState is based on a simple first-order Euler step, i.e.,

    new arm joint angles = (old arm joint angles) + (joint speeds) * delta_t
    new wheel angles = (old wheel angles) + (wheel speeds) * delta_t
    new chassis configuration is obtained from odometry, as described in Chapter 13.4 

    Note: All measurements are in meters, radians and seconds 

    Example Input:
        
    Output:
        
    """

    ## Limiting control velocities
    velocityLimiter(control, velocityLimits)
    
    ## Calculating arm configuration
    armConfig = np.array(configuration[3:8])
    armControl = np.array(control[0:5])
    armConfig = armConfig + armControl * timeStep


    ## Calculating wheel configuration
    wheelConfig = np.array(configuration[8:12])
    wheelControl = np.array(control[5:9])
    wheelConfig = wheelConfig + wheelControl * timeStep

    ## Calculating robot configuration
    theta, x, y = configuration[0:3]
    T = kinematic_model.youBot_Tbase(theta, x, y)

    # u = H * V (U : wheel speeds, V : body twist)
    # Using Eq. 13.10 in Modern Robotics book,
    H = kinematic_model.H
    V = np.dot(np.linalg.pinv(H), wheelControl)
    V6 = np.array([0, 0, V[0], V[1], V[2], 0])
    V6_se3mat = mr.VecTose3(V6)
    T_delta = mr.MatrixExp6(V6_se3mat * timeStep)  

    T = np.dot(T, T_delta)  
    baseConfig = kinematic_model.baseConfiguration(T)

    ## Updating configuration
    configuration = np.concatenate((baseConfig, armConfig, wheelConfig))
    ## Rounding off values to 8 decimal places
    configuration = np.around(configuration, 8)
    return configuration