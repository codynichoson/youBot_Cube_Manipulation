# Robotic Manipulation Final Project
import numpy as np
from numpy import pi, sin, cos
import modern_robotics as mr

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    csv_list = []
    
    # INITIAL TO STANDOFF ABOVE BLOCK
    t = 2 #seconds
    t_grip = 1

    Tse_standoff = Tsc_initial@Tce_standoff
    traj1 = mr.ScrewTrajectory(Tse_initial, Tse_standoff, t, t*k/0.01, 5)
    csv_list = BuildcsvList(traj1, 0, csv_list)

    # STANDOFF ABOVE BLOCK DOWN TO BLOCK
    Tse_grasp = Tsc_initial@Tce_grasp
    traj2 = mr.ScrewTrajectory(Tse_standoff, Tse_grasp, t, t*k/0.01, 5)
    csv_list = BuildcsvList(traj2, 0, csv_list)

    # GRAB BLOCK
    traj3 = mr.ScrewTrajectory(Tse_grasp, Tse_grasp, t_grip, t_grip*k/0.01, 5)
    csv_list = BuildcsvList(traj3, 1, csv_list)

    # MOVE BACK TO STANDOFF POSITION
    traj4 = mr.ScrewTrajectory(Tse_grasp, Tse_standoff, t, t*k/0.01, 5)
    csv_list = BuildcsvList(traj4, 1, csv_list)

    # MOVE TO FINAL STANDOFF POSITION
    Tse_standoff2 = Tsc_final@Tce_standoff
    traj5 = mr.ScrewTrajectory(Tse_standoff, Tse_standoff2, t, t*k/0.01, 5)
    csv_list = BuildcsvList(traj5, 1, csv_list)

    # STANDOFF2 ABOVE BLOCK DOWN TO BLOCK
    Tse_grasp2 = Tsc_final@Tce_grasp
    traj6 = mr.ScrewTrajectory(Tse_standoff2, Tse_grasp2, t, t*k/0.01, 5)
    csv_list = BuildcsvList(traj6, 1, csv_list)

    # RELEASE BLOCK
    traj7 = mr.ScrewTrajectory(Tse_grasp2, Tse_grasp2, t_grip, t_grip*k/0.01, 5)
    csv_list = BuildcsvList(traj7, 0, csv_list)
    
    # MOVE BACK TO STANDOFF2 POSITION
    traj8 = mr.ScrewTrajectory(Tse_grasp2, Tse_standoff2, t, t*k/0.01, 5)
    csv_list = BuildcsvList(traj8, 0, csv_list)

    return csv_list

    # np.savetxt('milestone2.csv',csv_list, delimiter=',')

def BuildcsvList(traj, gripper, csv_list):
    for i in range(len(traj)):
        Tse = traj[i]

        r11 = Tse[0][0]
        r12 = Tse[0][1]
        r13 = Tse[0][2]
        px = Tse[0][3]

        r21 = Tse[1][0]
        r22 = Tse[1][1]
        r23 = Tse[1][2]
        py = Tse[1][3]

        r31 = Tse[2][0]
        r32 = Tse[2][1]
        r33 = Tse[2][2]
        pz = Tse[2][3]

        csv_list.append([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper])

#####################################################################################

# End-effector intitial configuration relative to s (world) frame
Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])

# Cube ititial configuration relative to s (world) frame
Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])

# Cube final configuration relative to s (world) frame
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

# End-effector standoff configuration relative to block
Tce_standoff = np.array([[cos(3*pi/4),0,sin(3*pi/4),0],[0,1,0,0],[-sin(3*pi/4),0,cos(3*pi/4),0.2],[0,0,0,1]])

# End-effector grasping configuration relative to block
Tce_grasp = np.array([[cos(3*pi/4),0,sin(3*pi/4),0],[0,1,0,0],[-sin(3*pi/4),0,cos(3*pi/4),-0.01],[0,0,0,1]])

# Time multiplier
k = 5

# Initialize empty csv list
# csv_list = []

# Run the generator
TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)


