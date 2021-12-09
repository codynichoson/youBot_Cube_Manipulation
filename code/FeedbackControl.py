"""
Robotic Manipulation Final Project
Milestone 3
"""

import numpy as np
from numpy import pi, sin, cos
import modern_robotics as mr

# X is Tse (current actual ee configuration)
# Xd is Tse_d (current ee reference configuration)
def FeedbackControl(q, X, Xd, Xdnext, Kp, Ki, dt, integXerr):

    Xinv = np.linalg.inv(X)

    Xdinv = np.linalg.inv(Xd)
    Vd = (1/dt)*mr.MatrixLog6(Xdinv@Xdnext)
    Vd = np.array([0, 0, 0, Vd[0,3], Vd[1,3], Vd[2,3]])
    # print(f"Vd: {Vd}")

    # 6 vector of w_err and v_err
    Xerr = mr.se3ToVec(mr.MatrixLog6(Xinv@Xd))
    # Xerr = np.array([0,Xerr[0,2],0,Xerr[0,3], Xerr[1,3], Xerr[2,3]])
    
    Adjoint = mr.Adjoint(Xinv@Xd)@Vd
    V = Adjoint + Kp@Xerr + [i*integXerr for i in Ki] #Ki@integXerr
    V = V[0]
    # print(f"V: {V}")
    # print(f"Xerr: {Xerr}")

    integXerr = integXerr + Xerr*dt

    Blist = np.array([[0, 0, 1,       0, 0.033, 0],
                      [0,-1, 0, -0.5076,     0, 0],
                      [0,-1, 0, -0.3526,     0, 0],
                      [0,-1, 0, -0.2176,     0, 0],
                      [0, 0, 1,       0,     0, 0]]).T
    
    thetalist = np.array(q[3:8])

    # Find arm Jacobian
    Jarm = mr.JacobianBody(Blist, thetalist)
    # # if thetalist[0] < -2.9 or thetalist[0] > 2.9:      # Joint 1 limits
    # if thetalist[0] < -1 or thetalist[0] > 1:      # Joint 1 limits
    #     for row in Jarm:
    #         row[0] = 0
    # # elif thetalist[1] < -1.13 or thetalist[1] > 1.57:  # Joint 2 limits
    # elif thetalist[1] < -0.5 or thetalist[1] > 0.5:  # Joint 2 limits
    #     for row in Jarm:
    #         row[1] = 0
    # # # elif thetalist[2] < -2.64 or thetalist[2] > 2.55:  # Joint 3 limits
    # elif thetalist[2] < -0.5 or thetalist[2] > 0.5:  # Joint 3 limits
    #     for row in Jarm:
    #         row[2] = 0
    # # elif thetalist[3] < -1.79 or thetalist[3] > 1.79:  # Joint 4 limits
    # elif thetalist[3] < -0.5 or thetalist[3] > 0.5:  # Joint 4 limits
    #     for row in Jarm:
    #         row[3] = 0
    # # elif thetalist[4] < -2.88 or thetalist[4] > 2.88:  # Joint 5 limits
    # elif thetalist[4] < -0.5 or thetalist[4] > 0.5:  # Joint 5 limits
    #     for row in Jarm:
    #         row[4] = 0

    # print(Jarm)

    r = 0.0475
    l = 0.47/2
    w = 0.3/2

    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [       1,       1,       1,        1],
                        [      -1,       1,      -1,        1]])
    zeros = np.zeros(F.shape[1])
    F6 = np.array([zeros,zeros,F[0],F[1],F[2],zeros])

    M0e = np.array([[1, 0, 0,  0.033],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0,      1],])

    T0e = mr.FKinBody(M0e, Blist, thetalist)
    
    T0einv = mr.TransInv(T0e)

    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1],])

    Tb0inv = mr.TransInv(Tb0)

    Teb = T0einv@Tb0inv

    Jbase = mr.Adjoint(Teb)@F6

    Je = np.concatenate((Jbase, Jarm),axis=1)
    # print(f"Je: {np.around(Je,3)}")

    Vee = np.linalg.pinv(Je)@V
    # print(f"speeds: {np.around(Vee,3)}")

    return V, Vee, Xerr, integXerr


if __name__ == '__main__':
    q = np.array([0,0,0,0,0,0.2,-1.6,0])

    X = np.array([[  0.17,     0, 0.985, 0.387],
                [     0,     1,     0,     0],
                [-0.985,     0,  0.17,  0.57],
                [     0,     0,     0,     1]])

    Xd = np.array([[     0,     0,     1,   0.5],
                [     0,     1,     0,     0],
                [    -1,     0,     0,   0.5],
                [     0,     0,     0,     1]])

    Xdnext = np.array([[     0,     0,     1,   0.6],
                    [     0,     1,     0,     0],
                    [    -1,     0,     0,   0.3],
                    [     0,     0,     0,     1]])

    I = np.identity(6)

    Kp = 0
    Kp = [Kp*i for i in I]
    Ki = 0
    Ki = [Ki*i for i in I]

    dt = 0.01

    FeedbackControl(q, X, Xd, Xdnext, Kp, Ki, dt, integXerr=0)