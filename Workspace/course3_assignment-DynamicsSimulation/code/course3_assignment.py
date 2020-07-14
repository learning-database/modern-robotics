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
# Run the python file and enter the simulation number (1 or 2)
#      Example: In linux, open a terminal and type "python course3_assignment.py"
#               To Run first simulation, type 1 in the terminal and press Enter.
#      
# The code will automatically generate a CSV file named simulationX.csv in the execution directory.
# where X is 1 or 2 depending on which simulation you ran.


def UR5Simulation(thetalist0, time):
    """Computes the free falling forward dynamics of UR5 Robot, given initial configuration

    :param thetalist0: An initial joint angles, at time = 0
    :param time: Simulation time in seconds
    :return JointThetaMatrix: The list of all calculated joint angle values, as a list over time.

    Uses an Modern Robotics Library. Simulation time-step is 10ms

    Example Input:
        thetaList = np.array([0, 0, 0, 0, 0, 0])
        time = 3
    Output:
        JointThetaMatrix as an numpy array
    """

    M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
    M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
    M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
    M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
    M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
    M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
    M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
    G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
    Glist = [G1, G2, G3, G4, G5, G6]
    Mlist = [M01, M12, M23, M34, M45, M56, M67] 
    Slist = [[0,         0,         0,         0,        0,        0],
            [0,         1,         1,         1,        0,        1],
            [1,         0,         0,         0,       -1,        0],
            [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
            [0,         0,         0,         0,  0.81725,        0],
            [0,         0,     0.425,   0.81725,        0,  0.81725]]
    g = np.array([0, 0, -9.81])

    # Simulation is done under no joint torques and End Effector Wrenches
    tauList = np.array([0, 0, 0, 0, 0, 0])
    Ftip = np. array([0, 0, 0, 0, 0, 0])

    # Joint Configuration and Velocity matrices, for storing data over simulation time
    JointThetaMatrix = []
    JointVelMatrix = []
    JointThetaMatrix.append(thetalist0.tolist())
    JointVelMatrix.append(np.array([0,0,0,0,0,0]))

    # Forward Dynamics calculation at 100Hz. (10ms time step)
    iterationFrequency = 100

    totalTimeSteps = time * iterationFrequency
    for i in range(totalTimeSteps):
        ddthetaList = mr.ForwardDynamics(JointThetaMatrix[i],JointVelMatrix[i], tauList, g, Ftip, Mlist, Glist, Slist) 
        dthetaListNew = JointVelMatrix[i] + ddthetaList / iterationFrequency
        thetaListNew = JointThetaMatrix[i] + dthetaListNew / iterationFrequency

        # Append new values to Joint Velocity and Joint Configuration matrices
        JointThetaMatrix.append(thetaListNew.tolist())
        JointVelMatrix.append(dthetaListNew.tolist())

        print "Time step ", i, " out of ", totalTimeSteps

    # End of simulation. Return JointThetaMatrix
    return JointThetaMatrix

try:
    n = input('Enter Simulation number (1 or 2) and Press Enter: ')
    if (n == 1):
        thetaList = np.array([0, 0, 0, 0, 0, 0])
        print "Running Simulation 1"
        JointThetaMatrix = UR5Simulation(thetaList, 3)
        # Save joint vectors to simulation1 CSV file
        np.savetxt("simulation1.csv", np.array(JointThetaMatrix), delimiter=",")
        print "Simulation Complete! Output saved to simulation1.csv"
    elif (n == 2):
        thetaList = np.array([0, -1, 0, 0, 0, 0])
        print "Running Simulation 2"
        JointThetaMatrix = UR5Simulation(thetaList, 5)
        # Save joint vectors to simulation2 CSV file
        np.savetxt("simulation2.csv", np.array(JointThetaMatrix), delimiter=",")
        print "Simulation Complete! Output saved to simulation2.csv"
    else:
        print "Invalid Simulation number entered. Exiting program! Please enter 1 or 2"
    
except:
  print "Invalid character entered as simulation number. Exiting program! Please enter 1 or 2"
