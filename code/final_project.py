"""
Robotic Manipulation Final Project
"""

import numpy as np
from numpy import pi, sin, cos
import modern_robotics as mr
from NextState import NextState
from TrajectoryGenerator import TrajectoryGenerator
from FeedbackControl import FeedbackControl
import logging
import matplotlib.pyplot as plt

chassis_phi = -0.3
chassis_x   = -0.4
chassis_y   = 0.2
joint_1     = 0.4
joint_2     = 0.2
joint_3     = -0.2
joint_4     = -1.57
joint_5     = 0
wheel_1     = 0
wheel_2     = 0
wheel_3     = 0
wheel_4     = 0
gripper     = 0

initial_config = np.array([chassis_phi, chassis_x, chassis_y,  joint_1,  joint_2, joint_3, joint_4, joint_5,  wheel_1,  wheel_2,  wheel_3,  wheel_4, gripper])

# Max joint speed
thetadotMax = 20

# End-effector intitial configuration relative to s (world) frame
Tse_initial = np.array([[ 0, 0, 1,   0],
                        [ 0, 1, 0,   0],
                        [-1, 0, 0, 0.5],
                        [ 0, 0, 0,   1]])

# Cube ititial configuration relative to s (world) frame
Tsc_initial = np.array([[1, 0, 0,     1],
                        [0, 1, 0,     0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0,     1]])

# Cube final configuration relative to s (world) frame
Tsc_final = np.array([[ 0, 1, 0,     0],
                      [-1, 0, 0,    -1],
                      [ 0, 0, 1, 0.025],
                      [ 0, 0, 0,     1]])
# Tsc_final = np.array([[ -1,  0, 0,     0],
#                       [  0, -1, 0,    -1],
#                       [  0,  0, 1, 0.025],
#                       [  0,  0, 0,     1]])

# End-effector standoff configuration relative to block
Tce_standoff = np.array([[ cos(3*pi/4), 0, sin(3*pi/4),   0],
                         [           0, 1,           0,   0],
                         [-sin(3*pi/4), 0, cos(3*pi/4), 0.2],
                         [           0, 0,           0,   1]])

# End-effector grasping configuration relative to block
Tce_grasp = np.array([[ cos(3*pi/4), 0, sin(3*pi/4),     0],
                      [           0, 1,           0,     0],
                      [-sin(3*pi/4), 0, cos(3*pi/4), -0.01],
                      [           0, 0,           0,     1]])

# Screw axes (in end-effector frame for 5 dof arm at its home configuration
Blist = np.array([[0, 0, 1,       0, 0.033, 0],
                  [0,-1, 0, -0.5076,     0, 0],
                  [0,-1, 0, -0.3526,     0, 0],
                  [0,-1, 0, -0.2176,     0, 0],
                  [0, 0, 1,       0,     0, 0]])

# End-effector frame relative to base frame in home configuration
M0e = np.array([[1,0,0, 0.033],
                [0,1,0,     0],
                [0,0,1,0.6546],
                [0,0,0,     1],])

# Base frame of arm relative to robot chassis frame
Tb0 = np.array([[1,0,0,0.1662],
                [0,1,0,     0],
                [0,0,1,0.0026],
                [0,0,0,     1],])

# Proportional and integral gain constants
# Kp = 1
# Ki = 0.1
Kp = 4
Ki = 0.01
Kp = np.identity(6) * Kp
Ki = np.identity(6) * Ki

# Simulation settings
time = 14                  # total simluation time
tf = time/7                     # time per traj movement
dt = 0.01                  # delta time
k = 1                      # time scaling constant

# DOING THE STUFF

traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, tf, k)
logging.debug("Trajectory calculated")
iterations = len(traj)  # total iterations
# print(f"iterations: {iterations}")

# Initialize empty csv list
config_csv = np.zeros((iterations, 13))
Xerr_csv = np.zeros((iterations, 6))
integXerr = np.zeros(6)

config_csv[0] = initial_config

trajdirectory = '/home/codynichoson/Q1/robotic_manipulation/final_project'
np.savetxt('%s/traj_test.csv' %(trajdirectory), traj, delimiter=',')

Xerr1 = []
Xerr2 = []
Xerr3 = []
Xerr4 = []
Xerr5 = []
Xerr6 = []

logging.debug("Beginning iterations...")
# for i in range(1, iterations-1):
for i in range(1, iterations-1):
    previous_config = config_csv[i-1]

    theta = previous_config[0]
    x = previous_config[1]
    y = previous_config[2]

    Tsb = np.array([[cos(theta), -sin(theta), 0,      x],
                    [sin(theta),  cos(theta), 0,      y],
                    [         0,           0, 1, 0.0963],
                    [         0,           0, 0,      1]])

    thetalist = previous_config[3:8]

    T0e = mr.FKinBody(M0e, Blist.T, thetalist)

    Tbe = Tb0@T0e

    X = Tsb@Tbe

    Xd = np.array([[traj[i][0], traj[i][1], traj[i][2], traj[i][9] ],
                   [traj[i][3], traj[i][4], traj[i][5], traj[i][10]],
                   [traj[i][6], traj[i][7], traj[i][8], traj[i][11]],
                   [         0,          0,          0,           1]])
    
    Xdnext = np.array([[traj[i+1][0], traj[i+1][1], traj[i+1][2], traj[i+1][9] ],
                       [traj[i+1][3], traj[i+1][4], traj[i+1][5], traj[i+1][10]],
                       [traj[i+1][6], traj[i+1][7], traj[i+1][8], traj[i+1][11]],
                       [           0,            0,            0,             1]])

    V, speeds, Xerr, integXerr = FeedbackControl(previous_config, X, Xd, Xdnext, Kp, Ki, dt, integXerr)
    speedsflipped = np.concatenate((speeds[4:9], speeds[:4]), axis=None)
    
    previous_config = NextState(previous_config[:12], speedsflipped, dt, thetadotMax)

    config_csv[i] = np.concatenate((previous_config, traj[i][-1]), axis=None)

    Xerr1.append(Xerr[0])
    Xerr2.append(Xerr[1])
    Xerr3.append(Xerr[2])
    Xerr4.append(Xerr[3])
    Xerr5.append(Xerr[4])
    Xerr6.append(Xerr[5])

print(len(Xerr1))

# xaxis = np.linspace(0, time, iterations-2)
# plt.plot(xaxis, Xerr1)
# plt.plot(xaxis, Xerr2)
# plt.plot(xaxis, Xerr3)
# plt.plot(xaxis, Xerr4)
# plt.plot(xaxis, Xerr5)
# plt.plot(xaxis, Xerr6)
# plt.show()

logging.debug("Creating .csv file")
directory = '/home/codynichoson/Q1/robotic_manipulation/final_project'
np.savetxt('%s/final.csv' %(directory),config_csv, delimiter=',')