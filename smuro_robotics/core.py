# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%                      Smuro Robotics Parameters                      %%%
# %%%                          ZEFFIRETTI, HIESH                          %%%
# %%%                   Beijing Institute of Technology                   %%%
# %%%                zeffiretti@bit.edu.cn, hiesh@mail.com                %%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import math

import modern_robotics as mr
import numpy as np


def eul2rotm(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


class Smuro(object):
    def __init__(self, mass_scale_=1, inertia_scale_=1):
        # key constance
        self.smuro_dof = 7
        M_PI = 3.141592653589793
        # pitch and yaw axis w.r.t. space frame
        w_pitch = np.array([0, -1, 0])
        w_yaw = np.array([0, 0, 1])
        self.g = np.array([0, 0, -9.8])
        w = np.array([w_pitch, w_pitch, w_yaw, w_yaw, w_pitch, w_yaw, w_pitch])

        # qi refers to a point on joint i, w.r.t space frame
        q = np.array([[0,       0,  0.0265],
                      [0.0368,  0,  0.0433],
                      [0.0368,  0,  0.0433],
                      [0.0768,  0,  0.0433],
                      [0.0768,  0,  0.0433],
                      [0.13042, 0,  0.0433],
                      [0.14255, 0,  0.0433]])

        SList = np.ones((self.smuro_dof, 6), dtype=np.float32)
        for i in range(self.smuro_dof):
            SList[i] = mr.ScrewToAxis(q[i], w[i], 0)
        self.SList = SList.T
        self.M = np.array([[1, 0,  0, 0.14253],
                           [0, 0, -1, 0],
                           [0, 1,  0, 0.0433],
                           [0, 0,  0, 1]])

        # com List of links center of mass position, w.r.t link frame
        com = np.array([[0.009,     0.008, 0],
                        [0.0,      -0.002, 0],
                        [0.02,      0.0,   0],
                        [-0.00085, -0.002, 0],
                        [0.0317,   -0.001, 0],
                        [0.0317,   -0.001, 0],
                        [0.018,     0.0,   0],
                        [0.0,       0.0,   0]])

        # Mi refers to com configuration w.r.t link frame i, corresponding with com [i]
        M1 = np.r_[np.c_[np.eye(3), com[0]], [[0, 0, 0, 1]]]
        M2 = np.r_[np.c_[np.eye(3), com[1]], [[0, 0, 0, 1]]]
        M3 = np.r_[np.c_[np.eye(3), com[2]], [[0, 0, 0, 1]]]
        M4 = np.r_[np.c_[np.eye(3), com[3]], [[0, 0, 0, 1]]]
        M5 = np.r_[np.c_[np.eye(3), com[4]], [[0, 0, 0, 1]]]
        M6 = np.r_[np.c_[np.eye(3), com[5]], [[0, 0, 0, 1]]]
        M7 = np.r_[np.c_[np.eye(3), com[6]], [[0, 0, 0, 1]]]
        M8 = np.r_[np.c_[np.eye(3), com[7]], [[0, 0, 0, 1]]]
        Mis = np.array([M1, M2, M3, M4, M5, M6, M7, M8])
        # print(Mis)
        # qtrans[i] link farme i+1 origin position w.r.t link frame i, defined with
        #           joint xyz properity in urdf file
        qtrans = np.array([[0,        0,       0.0265],
                           [0.0368,   0.0168,  0],
                           [0,        0,       0],
                           [0.04,     0,       0],
                           [0,        0,       0],
                           [0.05362,  0,       0],
                           [0.01213,  0,       0],
                           [0,        0,       0]])
        # rpy{i} rpy angle between link frames, defined with joint rpy
        #             properity in urdf file
        rpy = np.array([[M_PI / 2,    0,  0],
                        [0,           0,  0.4286],
                        [-M_PI / 2,   0, -0.4286],
                        [0,           0,  0],
                        [M_PI / 2,    0,  0],
                        [-M_PI / 2,   0,  0],
                        [M_PI / 2,    0,  0],
                        [0,           0,  0]])

        Tijs = np.ones((self.smuro_dof + 1, 4, 4), dtype=np.float32)
        for i in range(self.smuro_dof + 1):
            rotm = eul2rotm(rpy[i])
            Tijs[i] = np.r_[np.c_[rotm, qtrans[i]], [[0, 0, 0, 1]]]
        M01 = np.dot(Tijs[0], Mis[0])
        MList = np.ones((self.smuro_dof + 1, 4, 4), dtype=np.float32)
        MList[0] = M01
        for i in range(self.smuro_dof):
            MList[i + 1] = np.dot(np.linalg.inv(Mis[i]),
                                  np.dot(Tijs[i + 1], Mis[i + 1]))
            # MList[i + 1] = np.linalg.inv(Mis[i]) * Tijs[i + 1] * Mis[i + 1]
        MList[0] = np.array([[1,            0.0,            0.0,          9e-3],
                             [0,            0.0,           -1.0,          0.0],
                             [0,            1.0,            0.0,          3.45e-2],
                             [0,            0.0,            0.0,          1.0]])
        MList[1] = np.array([[9.0955e-01,  -4.1560e-01,     0.0,          2.8631e-02],
                             [4.1560e-01,   9.0955e-01,     0.0,          6.9809e-03],
                             [0,            0.0,            1.0,          0.0],
                             [0,            0.0,            0.0,          1.0]])
        MList[2] = np.array([[9.0955e-01,   0.0,            4.1560e-01,   1.8191e-02],
                             [-4.1560e-01,  0.0,            9.0955e-01,  -6.3120e-03],
                             [0,           -1.0,            0.0,          0],
                             [0,            0.0,            0.0,          1]])
        MList[3] = np.array([[1.0,          0.0,            0.0,          1.9150e-02],
                             [0,            1.0,            0.0,         -2.0000e-03],
                             [0,            0.0,            1.0,          0.0],
                             [0,            0.0,            0.0,          1.0]])
        MList[4] = np.array([[1.0,          0.0,            0.0,          3.2550e-02],
                             [0,            0.0,           -1.0,          2.0000e-03],
                             [0,            1.0,            0.0,         -1.0000e-03],
                             [0,            0.0,            0.0,          1.0]])
        MList[5] = np.array([[1.0,          0.0,            0.0,          3.4420e-02],
                             [0,            0.0,            1.0,         -2.2000e-03],
                             [0,           -1.0,            0.0,          0.0],
                             [0,            0.0,            0.0,          1.0]])
        MList[6] = np.array([[1.0,          0.0,            0.0,          1.7630e-02],
                             [0,            0.0,           -1.0,          0.0],
                             [0,            1.0,            0.0,          3.2000e-03],
                             [0,            0.0,            0.0,          1.0]])
        MList[7] = np.array([[1.0,          0.0,            0.0,         -1.8000e-02],
                             [0,            1.0,            0.0,          0.0],
                             [0,            0.0,            1.0,          0.0],
                             [0,            0.0,            0.0,          1.0]])
        self.MList = MList
        mass_scale = mass_scale_
        inertia_scale = inertia_scale_
        mass = mass_scale * \
            np.array([0.0086, 0.002, 0.14, 0.002, 0.0417, 0.0092, 0.0078])
        inertia = inertia_scale * np.array([[3.0e-6,     3.00e-6,     2.00e-6],
                                            [1.5e-7,     4.00e-7,     1.80e-7],
                                            [5.0e-5,     5.00e-5,     4.00e-5],
                                            [1.8e-7,     1.80e-7,     1.00e-8],
                                            [7.0e-6,     9.00e-6,     8.00e-6],
                                            [4.5e-7,     3.90e-7,     3.20e-7],
                                            [9.3e-7,     1.07e-6,    1.19e-6]])
        GList = np.ones((self.smuro_dof, 6, 6))
        for i in range(self.smuro_dof):
            GList[i] = np.r_[np.c_[np.diag(inertia[i]), np.zeros((3, 3))],
                             np.c_[np.zeros((3, 3)), mass[i] * np.identity(3, dtype=np.float32)]]
        self.GList = GList

        # inverse dynamics test
        # theta = np.zeros(self.smuro_dof, dtype=np.float32)

    def fkinSpace(self, thetalist):
        """
        Function: Compute end effector frame (used for current spatial position calculation)
        Inputs: A list of joint coordinates.
        Returns: Transfomation matrix representing the end-effector frame when the joints are
                        at the specified coordinates
        Notes: FK means Forward Kinematics
        """
        return mr.FKinSpace(self.M, self.Slist, thetalist)

    def ikinSpace(self, T, thetalist, emog=1e-6, ev=1e-6):
        """
        Function: Computes inverse kinematics in the space frame for an open chain robot
        Inputs:
                T: The desired end-effector configuration Tsd
                thetalist[in][out]: An initial guess and result output of joint angles that are close to
                 satisfying Tsd
                emog: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
                ev: A small positive tolerance on the end-effector linear position
                 error. The returned joint angles must give an end-effector
                 position error less than ev
        Outputs:
                success: A logical value where TRUE means that the function found
                 a solution and FALSE means that it ran through the set
                 number of maximum iterations without finding a solution
                 within the tolerances eomg and ev.
                thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
        """
        return mr.IKinSpace(self.SList, self.M, T, thetalist, emog, ev)

    def inverseDynamics(self, thetalist, dthetalist, ddthetalist, ftip):
        """
        Function: This function uses forward-backward Newton-Euler iterations to solve the
        equation:
            taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
                    + g(thetalist) + Jtr(thetalist) * Ftip
        Inputs:
            thetalist: n-vector of joint variables
            dthetalist: n-vector of joint rates
            ddthetalist: n-vector of joint accelerations
        Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
        Outputs:
            taulist: The n-vector of required joint forces/torques
        """
        return mr.InverseDynamics(thetalist, dthetalist, ddthetalist, self.g, ftip, self.MList, self.GList, self.SList)

    def printInfo(self):
        print("################### Smuro Robotics Parameters Brief ###################")
        print("01. Screw axes Si of the joints in a space frame SList:")
        print(self.SList)
        print("-----------------------------------------------------------------------")
        print("02. The home configuration of the end-effector M:")
        print(self.M)
        print("-----------------------------------------------------------------------")
        print("03. List of link com frames {i} relative to {i-1} at the")
        print("         home position MList:")
        print(self.MList)
        print("-----------------------------------------------------------------------")
        print("04. List of spatial inertia matrices GList:")
        print(self.GList)
        print("-----------------------------------------------------------------------")
        print("05. Gravity vector g:")
        print(self.g)
        print("################### Smuro Robotics Parameters End. ####################")
