from __future__ import division
import modern_robotics as mr
import numpy as np
import math

################# TrajectoryGenerator to generate the reference trajectory for the end-effector frame ##################

def screwTragectoryGenerator(T_start, T_end, motionTime, gripperState, timeStep, K):
    """Generates a screw trajectory given a start and end configuration
       
         :param T_start: start configuration (SE3 Matrix)
         :param T_end: goal configuration   (SE3 Matrix)
         :param motionTime: Total time of motion
         :param gripperState: Open / Close state of gripper.  gripper state: 0 = open, 1 = 
         :param timeStep: A timestep delta_t
         :param K: The number of trajectory reference configurations per timeStep

         :return: Configuration Tse of the end-effector, expressed as 13 variables separated by commas. The 13 variables are, in order
                    
                    r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state


         Generated number of trajectory points per second will be equal to K * timeStep
         This method uses fifth-order polynomial time scaling for trajectory generation.
    """
    trajectory = mr.ScrewTrajectory(T_start, T_end, motionTime, motionTime * K / timeStep, 5)
    formattedTragectory = []
    for config in trajectory:
        # Rounding off values to 8 decimal places
        configuration = np.around(config, 8).tolist()
        formattedConfiguration = configuration[0][0:3] + configuration[1][0:3] + configuration[2][0:3] + \
        [configuration[0][3], configuration[1][3], configuration[2][3], gripperState]
        formattedTragectory.append(formattedConfiguration)
    return formattedTragectory

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, timeStep, K = 1):
    """Generates the reference trajectory for the end-effector frame

        :param Tse_initial: The initial configuration of the end-effector in the reference trajectory: Tse,initial. (SE3 Matrix)
        :param Tsc_initial: The cube's initial configuration: Tsc,initial   (SE3 Matrix)
        :param Tsc_final: The cube's desired final configuration: Tsc,final (SE3 Matrix)
        :param Tce_grasp: The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp (SE3 Matrix)
        :param Tce_standoff: The end-effector's standoff configuration above the cube,  
                            before and after grasping, relative to the cube: Tce,standoff. (SE3 Matrix)
                            
                            This specifies the configuration of the end-effector {e} relative to the cube frame {c} 
                            before lowering to the grasp configuration Tce,grasp, for example.
        :param timeStep: A timestep delta_t
        :param K (optional): The number of trajectory reference configurations per 0.01 seconds: k. 
                The value k is an integer with a value of 1 or greater. Default 1
        
        :return: A representation of the N configurations of the end-effector 
                along the entire concatenated eight-segment reference trajectory. 
                Each of these N reference points represents a transformation matrix Tse 
                of the end-effector frame {e} relative to {s} at an instant in time, 
                plus the gripper state (0 or 1). 
                
                r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state

        Note: All measurements are in meters, radians and seconds 

        This trajectory consists of eight concatenated trajectory segments
        1.    A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
        2.    A trajectory to move the gripper down to the grasp position.
        3.    Closing of the gripper.
        4.    A trajectory to move the gripper back up to the "standoff" configuration.
        5.    A trajectory to move the gripper to a "standoff" configuration above the final configuration.
        6.    A trajectory to move the gripper to the final configuration of the object.
        7.    Opening of the gripper.
        8.    A trajectory to move the gripper back to the "standoff" configuration. 
        Example Input:
            
        Output:
            
        """

    # gripper states
    open_ = 0
    closed_ = 1

    ## Tragectory 1
    ## A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
    motionTime = 3
    Tse_end = np.dot(Tsc_initial, Tce_standoff)
    trajectory1 = screwTragectoryGenerator(Tse_initial, Tse_end, motionTime, open_, timeStep, K)
    Tse_current = Tse_end

    ## Tragectory 2
    ## A trajectory to move the gripper down to the grasp position.
    motionTime = 1.5
    Tse_end = np.dot(Tsc_initial, Tce_grasp)
    trajectory2 = screwTragectoryGenerator(Tse_current, Tse_end, motionTime, open_, timeStep, K)
    Tse_current = Tse_end

    ## Tragectory 3
    ## Closing of the gripper.
    motionTime = 1
    trajectory3 = screwTragectoryGenerator(Tse_current, Tse_current, motionTime, closed_, timeStep, K)

    ## Tragectory 4
    ## A trajectory to move the gripper back up to the "standoff" configuration.
    motionTime = 1.5
    Tse_end = np.dot(Tsc_initial, Tce_standoff)
    trajectory4 = screwTragectoryGenerator(Tse_current, Tse_end, motionTime, closed_, timeStep, K)
    Tse_current = Tse_end

    ## Tragectory 5
    ## A trajectory to move the gripper to a "standoff" configuration above the final configuration.
    motionTime = 3
    Tse_end = np.dot(Tsc_final, Tce_standoff)
    trajectory5 = screwTragectoryGenerator(Tse_current, Tse_end, motionTime, closed_, timeStep, K)
    Tse_current = Tse_end

    ## Tragectory 6
    ## A trajectory to move the gripper to the final configuration of the object.
    motionTime = 1.5
    Tse_end = np.dot(Tsc_final, Tce_grasp)
    trajectory6 = screwTragectoryGenerator(Tse_current, Tse_end, motionTime, closed_, timeStep, K)
    Tse_current = Tse_end

    ## Tragectory 7
    ## Opening of the gripper.
    motionTime = 1
    trajectory7 = screwTragectoryGenerator(Tse_current, Tse_current, motionTime, open_, timeStep, K)

    ## Tragectory 8
    ## A trajectory to move the gripper back to the "standoff" configuration. 
    motionTime = 1.5
    Tse_end = np.dot(Tsc_final, Tce_standoff)
    trajectory8 = screwTragectoryGenerator(Tse_current, Tse_end, motionTime, open_, timeStep, K)
    Tse_current = Tse_end

    return trajectory1 + \
            trajectory2 + \
            trajectory3 + \
            trajectory4 + \
            trajectory5 + \
            trajectory6 + \
            trajectory7 + \
            trajectory8