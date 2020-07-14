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

def milestone1_test1():
    """
        Milestone 1 : Test 1
        The robot chassis should drive forward by 0.475 meters.
        CoppeliaSim : Scene6_youBot_cube
    """
    configuration = [0 for x in range(12)]
    timeStep = 0.01
    configurations = []
    configurations.append(configuration)
    control = [0, 0, 0, 0, 0, 10, 10, 10, 10]
    velocityLimits = [5 for x in range(9)]

    for t in range(100):
        configuration = kinematic_simulator.NextState(configuration, control, timeStep)
        print configuration.tolist()
        configurations.append(configuration)

    csv_writer.writeDataList(configurations, 'milestone1_test1')

def milestone1_test2():
    """
        Milestone 1 : Test 2
        The robot chassis should slide sideways by 0.475 meters. 
        CoppeliaSim : Scene6_youBot_cube
    """
    configuration = [0 for x in range(12)]
    timeStep = 0.01
    configurations = []
    configurations.append(configuration)
    control = [0, 0, 0, 0, 0, -10, 10, -10, 10]
    velocityLimits = [5 for x in range(9)]

    for t in range(100):
        configuration = kinematic_simulator.NextState(configuration, control, timeStep)
        print configuration.tolist()
        configurations.append(configuration)

    csv_writer.writeDataList(configurations, 'milestone1_test2')

def milestone1_test3():
    """
        Milestone 1 : Test 2
        The robot chassis should spin counterclockwise in place by 1.234 radians
        CoppeliaSim : Scene6_youBot_cube
    """
    configuration = [0 for x in range(12)]
    timeStep = 0.01
    configurations = []
    configurations.append(configuration)
    control = [0, 0, 0, 0, 0, -10, 10, 10, -10]
    velocityLimits = [5 for x in range(9)]

    for t in range(100):
        configuration = kinematic_simulator.NextState(configuration, control, timeStep)
        print configuration.tolist()
        configurations.append(configuration)

    csv_writer.writeDataList(configurations, 'milestone1_test3')

def milestone1_test4():
    """
        Milestone 1 : Test 1
        The robot chassis should drive forward by 0.475 meters. 
        CoppeliaSim : Scene6_youBot_cube
    """
    configuration = [0 for x in range(12)]
    timeStep = 0.01
    configurations = []
    configurations.append(configuration)
    control = [0, 0, 0, 0, 0, 10, 10, 10, 10]
    velocityLimits = [5 for x in range(9)]

    for t in range(100):
        configuration = kinematic_simulator.NextState(configuration, control, timeStep, velocityLimits)
        print configuration.tolist()
        configurations.append(configuration)

    csv_writer.writeDataList(configurations, 'milestone1_test4')

def milestone2_test():
    """
        Milestone 2 : Test
        The robot manipulator traectory generation for pick and place task
        CoppeliaSim : Scene8_gripper_csv
    """
    Tse_initial = [[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.5], [0, 0, 0, 1]]
    Tsc_initial = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    Tsc_goal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]

    # Generating stadoff and gripping transforms
    theta = [0, 3 * math.pi/4, 0]
    R_so3 = mr.VecToso3(theta)
    R = mr.MatrixExp3(R_so3)

    position = [-0.01, 0, 0.20]
    Tce_standoff = mr.RpToTrans(R, position)

    position = [-0.01, 0, 0.01]
    Tce_gripping = mr.RpToTrans(R, position)

    trajectory = trajectory_planner.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_gripping, Tce_standoff, 0.01, 10 )
    csv_writer.writeDataList(trajectory, 'milestone2_test')

def milestone3_test1():
    """
        Milestone 3 : Test1
        Check robot controller feed forward values (kp = 0, ki = 0)
    """
    configuration = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0]
    Xd = [[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
    Xd_next = [[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]]
    # X = [[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]]
    gains = [0, 0]
    timeStep = 0.01
    
    Tse = kinematic_model.youBot_Tse(configuration)
    V, Xerr = controller.FeedbackControl(Tse, Xd, Xd_next, gains, timeStep)
    print V

    # print Je
    Je = kinematic_model.youBot_Je(configuration)
    print np.around(Je,3)

    # Calculating control
    control = np.dot( np.linalg.pinv(Je), V)
    control = np.around(control, 3)
    print control.tolist()

def milestone3_test2():
    """
        Milestone 3 : Test2
        Check robot controller feed forward and feedback execution (kp = 1, ki = 0)
    """

    configuration = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0]
    Xd = [[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
    Xd_next = [[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]]
    # X = [[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]]
    gains = [1, 0]
    timeStep = 0.01
    
    Tse = kinematic_model.youBot_Tse(configuration)
    V, Xerr = controller.FeedbackControl(Tse, Xd, Xd_next, gains, timeStep)
    print V
    
    # print Je
    Je = kinematic_model.youBot_Je(configuration)
    print Je.tolist()

    # Calculating control
    control = np.dot( np.linalg.pinv(Je), V)
    control = np.around(control, 3)
    print control.tolist()

def milestone3_test3():
    """
        Milestone 3 : Test3
        Check robot controller feed forward values (kp = 0, ki = 0) with collision checking
    """

    configuration = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0]
    Xd = [[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]]
    Xd_next = [[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]]
    # X = [[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]]
    gains = [1, 0]
    timeStep = 0.01
    
    Tse = kinematic_model.youBot_Tse(configuration)
    V, Xerr = controller.FeedbackControl(Tse, Xd, Xd_next, gains, timeStep)

    # Calculate Je
    Je = kinematic_model.youBot_Je(configuration)

    while True:
        # Calculating control
        control = np.dot( np.linalg.pinv(Je), V)
    
        next_config = kinematic_simulator.NextState(configuration, control, timeStep)
        status, jointVector = kinematic_model.youBot_testJointLimits(next_config[3:8])

        if status:
            break
        else:
            Je = Je * jointVector
    
def milestone3_test4_debug():
    """
        Milestone 3 : Test4
        Check robot controller for generated tragectory (kp = 0, ki = 0)
    """

    # configuration = [0, 0, 0, 0, -0.35, -0.698, -0.505, 0, 0, 0, 0, 0]
    configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    Tsc_initial = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    Tsc_goal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    Tse_initial = kinematic_model.youBot_Tse(configuration)
    
    result_configuration = []
    result_Xerr = []

    gains = [0, 0]  # kp, ki
    timeStep = 0.5
    K = 1

    # Generating stadoff and gripping transforms
    theta = [0, 3 * math.pi/4, 0]
    R_so3 = mr.VecToso3(theta)
    R = mr.MatrixExp3(R_so3)

    position = [-0.01, 0, 0.20]
    Tce_standoff = mr.RpToTrans(R, position)

    position = [-0.01, 0, 0.01]
    Tce_gripping = mr.RpToTrans(R, position)

    # Trajectory generation
    # trajectory = trajectory_planner.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_gripping, Tce_standoff, timeStep, K )
    Xstart = kinematic_model.youBot_Tse(configuration)
    XendConfig = [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    Xend = kinematic_model.youBot_Tse(XendConfig)
    trajectory = mr.ScrewTrajectory(Xstart, Xend, 3, 6, 3)
    csv_writer.writeDataList(trajectory, "milestone3_trajectory")

    # Control generation
    for x in range(len(trajectory) - 1):
        Xd = trajectory[x]
        Xd_next = trajectory[x+1]
        
        # Xd = kinematic_model.configurationToTransform(trajectory[x])
        # Xd_next = kinematic_model.configurationToTransform(trajectory[x+1])
        Tse = kinematic_model.youBot_Tse(configuration)
        print "####### Iteration #######"
        print "Tse"
        print np.around(Tse,3).tolist()
        print "Xd"
        print np.around(Xd,3).tolist()
        print "Xd_next"
        print np.around(Xd_next,3).tolist()
        
        V, Xerr = controller.FeedbackControl(Tse, Xd, Xd_next, gains, timeStep)
        print "V"
        print np.around(V,3).tolist()
        print "Xerr"
        print np.around(Xerr,3).tolist()
        # Calculate Je
        Je = kinematic_model.youBot_Je(configuration)

        while True:
            # Calculating control
            cmd = np.dot( np.linalg.pinv(Je), V)
            # Control : omega (5 variables), wheel speeds u (4 variables)
            control = np.concatenate((cmd[4:9], cmd[0:4]))

            print "control"
            print np.around(control,3).tolist()
            next_config = kinematic_simulator.NextState(configuration, control, timeStep)
            print "next_config"
            print np.around(next_config,3).tolist()
            # Joint limit checking
            status, jointVector = kinematic_model.youBot_testJointLimits(next_config[3:8])
            if status:
                configuration = next_config
                break
            else:
                Je[:,4:9] = Je[:,4:9] * jointVector
    
        # Save configuration (Tse) and Xerr per every K iterations
        if (x % K) == 0:
            result_configuration.append(configuration)
            result_Xerr.append(Xerr)
            # print np.around(configuration, 3).tolist()
    
    csv_writer.writeDataList(result_configuration, "milestone3_configurations")
    # print "Xerr"
    # for x in result_Xerr:
    #     print np.around(x,3).tolist()
    
def milestone3_test4():
    """
        Milestone 3 : Test4
        Check robot controller for generated tragectory (kp = 0, ki = 0)
    """

    configuration = [0.0, 0.0, 0.0, 0.0, -0.35, -0.698, -0.505, 0.0, 0.0, 0.0, 0.0, 0.0]
    Tsc_initial = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    Tsc_goal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    # Tsc_goal = [[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    Tse_initial = kinematic_model.youBot_Tse(configuration)
    
    result_configuration = []
    result_Xerr = []

    gains = [0, 0]  # kp, ki
    timeStep = 0.01
    velocityLimits = [10, 10, 10, 20, 20, 20, 20, 20, 10, 10, 10, 10]
    K = 1

    # Generating stadoff and gripping transforms
    theta = [0, 3 * math.pi/4, 0]
    R_so3 = mr.VecToso3(theta)
    R = mr.MatrixExp3(R_so3)

    position = [-0.01, 0, 0.20]
    Tce_standoff = mr.RpToTrans(R, position)

    position = [-0.005, 0, 0.005]
    Tce_gripping = mr.RpToTrans(R, position)

    # Trajectory generation
    print 'Generating trajectory'
    trajectory = trajectory_planner.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_gripping, Tce_standoff, timeStep, K )
    print 'Trajectory generation successfull'


    # Control generation
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
            # Joint limit checking
            # status, jointVector = kinematic_model.youBot_testJointLimits(next_config[3:8])
            # status = True # TODO
            status = kinematic_model.youBot_selfCollisionCheck(next_config[3:8])
            jointVector = [0, 0, 0, 0, 0]
            if status:
                configuration = next_config
                break
            else:
                Je[:,4:9] = Je[:,4:9] * jointVector
    
        # Save configuration (Tse) and Xerr per every K iterations
        if (x % K) == 0:
            # Append with gripper state
            gripper_state = trajectory[x][12]
            youBot_configuration = np.concatenate((configuration, [gripper_state]))
            result_configuration.append(youBot_configuration)
            result_Xerr.append(Xerr)
            completion = round(((x+1) * 100.0 /len(trajectory)), 2)
            sys.stdout.write("\033[F")
            print ('Generating youBot trajectory controls. Progress ' + str(completion) + '%\r')
            
    print "Final Error (Xerr) : \n", np.around(Xerr, 5)

    configComments = ["A 12-vector representing the current configuration of the robot",
                      "\t3 variables for the chassis configuration (theta, x, y)",
                      "\t5 variables for the arm configuration (J1, J2, J3, J4, J5)",
                      "\tand 4 variables for the wheel angles (W1, W2, W3, W4)"]
    csv_writer.writeDataList(result_configuration, name = "milestone3_configurations", comments = configComments)

    XerrComments = ["A 6-vector representing the error twist",
                    "\t3 variables for the angular velocity error (Wx, Wy, Wz)",
                    "\t3 variables for the linear velocity error (Vx, Vy, Vz)"]
    csv_writer.writeDataList(result_Xerr, name = "milestone3_Xerr", comments = XerrComments)
    
    # Displaying Error Plots
    labels = ['Wx', 'Wy', 'Wz', 'Vx', 'Vy', 'Vz']
    for x in range(6):
        plt.plot(np.array(result_Xerr)[:,x], label = labels[x])
    plt.ylabel('Xerr')
    plt.legend()
    plt.show()
    
    
