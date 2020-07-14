from __future__ import division
import kinematic_simulator
import trajectory_planner
import kinematic_model
import controller
import csv_writer
import numpy as np
import math
import modern_robotics as mr
import sys
import matplotlib.pyplot as plt

################# Instructions to Run the code ##################
# Make sure you have python 2 installed with following libraries.
#       scipy
#       numpy
#       matplotlib
#
# This program has been tested on python 2.7 on Ubuntu 16.04
#
# Run the python file
#      Example: In linux, open a terminal and type "python main.py"
#
#################################################################

## Program Inputs
config = input("Enter Tse (End effector) initial configuration\n[theta, x, y, J1, J2, J3, J4, J5] : ") 
configuration = config[0:8] + [0, 0, 0, 0]
Tse_initial = kinematic_model.youBot_Tse(configuration)

[x, y, theta] = input("Enter Tsc (Cube) initial configuration\n [x, y, theta] : ")
pose = [0, 0, theta, x, y, 0.025] 
Tsc_initial = kinematic_model.poseToTransform(pose)

[x, y, theta] = input("Enter Tsc (Cube) goal configuration\n [x, y, theta] : ") 
pose = [0, 0, theta, x, y, 0.025] 
Tsc_goal = kinematic_model.poseToTransform(pose)

Tse_refInitial = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]

gains = input("Enter Feedback Controller gains [kp, ki] : ")

# Program constants
timeStep = 0.01
velocityLimits = [10, 10, 10, 20, 20, 20, 20, 20, 10, 10, 10, 10]
K = 10

result_configuration = []
result_Xerr = []

# Generating stadoff and gripping transforms
theta = [0, 3 * math.pi/4, 0]
R_so3 = mr.VecToso3(theta)
R = mr.MatrixExp3(R_so3)

position = [0.00, 0, 0.20]  #[-0.01, 0, 0.20]
Tce_standoff = mr.RpToTrans(R, position)

position = [0.00, 0, 0.00] #[-0.01, 0, 0.00]
Tce_gripping = mr.RpToTrans(R, position)

# Displaying initial End Effector configuration error
initialRef = kinematic_model.transformToPose(Tse_refInitial)
initialConf = kinematic_model.transformToPose(Tse_initial)
initialError = initialRef - initialConf
print 'Initial End Effector configuration Error (Theta_x, Theta_y, Theta_z, x, y, z):\n', np.around(initialError, 3)

## Trajectory generation
print 'Generating trajectory'
trajectory = trajectory_planner.TrajectoryGenerator(Tse_refInitial, Tsc_initial, Tsc_goal, Tce_gripping, Tce_standoff, timeStep, K )
print 'Trajectory generation successfull'


## Control generation
for x in range(len(trajectory) - 1):
    Xd = kinematic_model.configurationToTransform(trajectory[x])
    Xd_next = kinematic_model.configurationToTransform(trajectory[x+1])
    Tse = kinematic_model.youBot_Tse(configuration)
    
    V, Xerr = controller.FeedbackControl(Tse, Xd, Xd_next, gains, timeStep)

    # Calculate Je
    Je = kinematic_model.youBot_Je(configuration)

    while True:
        # Calculating control
        cmd = np.dot( np.linalg.pinv(Je), V)
        # Control : omega (5 variables), wheel speeds u (4 variables)
        control = np.concatenate((cmd[4:9], cmd[0:4]))

        next_config = kinematic_simulator.NextState(configuration, control, timeStep, velocityLimits)
        # Collision check. If collision detected, set Jarm to zero 
        status = kinematic_model.youBot_selfCollisionCheck(next_config[3:8])
        JarmCoefficients= [0, 0, 0, 0, 0]
        if status:
            configuration = next_config
            break
        else:
            Je[:,4:9] = Je[:,4:9] * JarmCoefficients

    # Save configuration (Tse) and Xerr per every K iterations
    if (x % K) == 0:
        # Append with gripper state
        gripper_state = trajectory[x][12]
        youBot_configuration = np.concatenate((configuration, [gripper_state]))
        result_configuration.append(youBot_configuration)
        result_Xerr.append(Xerr)
        completion = round( ( x * 100.0 /(len(trajectory) -1) ), 2)
        sys.stdout.write("\033[F")
        print ('Generating youBot trajectory controls. Progress ' + str(completion) + '%\r')

# Displaying final Controller error    
print "Final Error (Xerr : Wx, Wy, Wz, Vx, Vy, Vz) : \n", np.around(Xerr, 5)

## Saving Configuration and Xerr data to CSV files
configComments = ["A 12-vector representing the current configuration of the robot",
                    "\t3 variables for the chassis configuration (theta, x, y)",
                    "\t5 variables for the arm configuration (J1, J2, J3, J4, J5)",
                    "\tand 4 variables for the wheel angles (W1, W2, W3, W4)"]
csv_writer.writeDataList(result_configuration, name = "youBot_Trajectory", comments = configComments)

XerrComments = ["A 6-vector representing the error twist",
                "\t3 variables for the angular velocity error (Wx, Wy, Wz)",
                "\t3 variables for the linear velocity error (Vx, Vy, Vz)"]
csv_writer.writeDataList(result_Xerr, name = "trajectory_Xerr", comments = XerrComments)

## Displaying Error Plots
labels = ['Wx', 'Wy', 'Wz', 'Vx', 'Vy', 'Vz']
for x in range(6):
    plt.plot(np.array(result_Xerr)[:,x], label = labels[x])
plt.ylabel('Xerr')
plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True, shadow=True, ncol=6)
plt.show()