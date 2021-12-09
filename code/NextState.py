"""
Robotic Manipulation Final Project
Milestone 1
"""
import numpy as np
from numpy import pi, sin, cos
import modern_robotics as mr

def NextState(robotConfig, speeds, dt, thetadotMax):

    chassisConfig = robotConfig[:3]
    phi, x, y = chassisConfig

    oldJointAngles = robotConfig[3:8]
    jointSpeeds = speeds[:5]
    for i in range(len(jointSpeeds)):
        if jointSpeeds[i] > thetadotMax:
            jointSpeeds[i] = thetadotMax
        elif jointSpeeds[i] < -thetadotMax:
            jointSpeeds[i] = -thetadotMax

    oldWheelAngles = robotConfig[8:12]
    wheelSpeeds = speeds[5:]
    for i in range(len(wheelSpeeds)):
        if wheelSpeeds[i] > thetadotMax:
            wheelSpeeds[i] = thetadotMax
        elif wheelSpeeds[i] < -thetadotMax:
            wheelSpeeds[i] = -thetadotMax
    
    # print(wheelSpeeds)

    r = 0.0475
    l = 0.47/2
    w = 0.3/2

    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [       1,       1,       1,        1],
                        [      -1,       1,      -1,        1]])
    deltaTheta = np.array([i*dt for i in wheelSpeeds])
    
    # Vb = [wbz, vbx, vby]
    Vb = F@deltaTheta.T
    wbz, vbx, vby = Vb

    if wbz == 0:
        delta_qb = np.array([0, vbx, vby])
    elif wbz != 0:
        delta_qb = np.array([wbz, (vbx*sin(wbz)+vby*(cos(wbz)-1))/wbz, (vby*sin(wbz)+vbx*(1-cos(wbz)))/wbz])

    R = np.array([[1,        0,         0],
                  [0, cos(phi), -sin(phi)],
                  [0, sin(phi),  cos(phi)]])
    delta_q = R@delta_qb.T

    newchassisConfig = np.add(chassisConfig, delta_q)
    newJointAngles = np.add(oldJointAngles, [i*dt for i in jointSpeeds])
    newWheelAngles = np.add(oldWheelAngles, [i*dt for i in wheelSpeeds])

    new_robotConfig = np.concatenate((newchassisConfig, newJointAngles, newWheelAngles), axis=None)
    new_robotConfig = list(new_robotConfig)
    # print(new_robotConfig)

    return new_robotConfig

if __name__ == '__main__':
    # chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
    robotConfig = [0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0]

    # speeds = [0, 0, 0, 0, 0, 10, 10, 10, 10] # Forward
    # speeds = [0, 0, 0, 0, 0, -10, 10, -10, 10] # Sideways
    speeds = [0, 0, 0, 0, 0, -10, 10, 10, -10] # Spin counterclockwise

    dt = 0.01

    # Maximum joint speed
    thetadotMax = 10

    # Initialize .csv list
    # csv_list = [robotConfig]
    csv_list = []

    # Iterate 100 times (1 second at dt = 0.01)
    for i in range(100):
        robotConfig = NextState(robotConfig, speeds, dt, thetadotMax)
        # print(robotConfig)
        csv_list.append(robotConfig)

    # Save to file
    print(len(csv_list))
    directory = '/home/codynichoson/Q1/robotic_manipulation/final_project'
    np.savetxt('%s/spin_test2.csv' %(directory),csv_list, delimiter=',')




