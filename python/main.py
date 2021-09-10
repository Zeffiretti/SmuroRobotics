# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

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


def parameters():
    smuro_dof = 7
    # pitch and yaw axis w.r.t. space frame
    w_pitch = np.array([0, -1, 0])
    w_yaw = np.array([0, 0, 1])
    M_PI = 3.141592653589793
    g = np.array([0, 0, -9.8])
    # wi refers to roration axis of joint i, w.r.t space frame
    w = np.array([w_pitch, w_pitch, w_yaw, w_yaw, w_pitch, w_yaw, w_pitch])

    # qi refers to a point on joint i, w.r.t space frame
    q = np.array([[0, 0, 0.0265],
                  [0.0368, 0, 0.0433],
                  [0.0368, 0, 0.0433],
                  [0.0768, 0, 0.0433],
                  [0.0768, 0, 0.0433],
                  [0.13042, 0, 0.0433],
                  [0.14255, 0, 0.0433]])

    SList = np.ones((smuro_dof, 6), dtype=np.float32)
    for i in range(smuro_dof):
        SList[i] = mr.ScrewToAxis(q[i], w[i], 0)
    SList = SList.T
    M = np.array([[1, 0, 0, 0.14253],
                  [0, 0, -1, 0],
                  [0, 1, 0, 0.0433],
                  [0, 0, 0, 1]])

    # com List of links center of mass position, w.r.t link frame
    com = np.array([[0.009, 0.008, 0],
                    [0.0, -0.002, 0],
                    [0.02, 0.0, 0],
                    [-0.00085, -0.002, 0],
                    [0.0317, -0.001, 0],
                    [0.0317, -0.001, 0],
                    [0.018, 0.0, 0],
                    [0.0, 0.0, 0]])

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
    qtrans = np.array([[0, 0, 0.0265],
                       [0.0368, 0.0168, 0],
                       [0, 0, 0],
                       [0.04, 0, 0],
                       [0, 0, 0],
                       [0.05362, 0, 0],
                       [0.01213, 0, 0],
                       [0, 0, 0]])
    # rpy{i} rpy angle between link frames, defined with joint rpy
    #             properity in urdf file
    rpy = np.array([[M_PI / 2, 0, 0],
                    [0, 0, 0.4286],
                    [-M_PI / 2, 0, -0.4286],
                    [0, 0, 0],
                    [M_PI / 2, 0, 0],
                    [-M_PI / 2, 0, 0],
                    [M_PI / 2, 0, 0],
                    [0, 0, 0]])

    Tijs = np.ones((smuro_dof + 1, 4, 4), dtype=np.float32)
    for i in range(smuro_dof + 1):
        rotm = eul2rotm(rpy[i])
        Tijs[i] = np.r_[np.c_[rotm, qtrans[i]], [[0, 0, 0, 1]]]
    M01 = np.dot(Tijs[0], Mis[0])
    MList = np.ones((smuro_dof + 1, 4, 4), dtype=np.float32)
    MList[0] = M01
    for i in range(smuro_dof):
        MList[i + 1] = np.dot(np.linalg.inv(Mis[i]), np.dot(Tijs[i + 1], Mis[i + 1]))
        # MList[i + 1] = np.linalg.inv(Mis[i]) * Tijs[i + 1] * Mis[i + 1]
    # print(MList)
    mass_scale = 1.0
    inertia_scale = 1.0
    mass = mass_scale * np.array([0.0086, 0.002, 0.14, 0.002, 0.0417, 0.0092, 0.0078])
    inertia = inertia_scale * np.array([[3e-6, 3e-6, 2e-6],
                                        [1.5e-7, 4e-7, 1.8e-7],
                                        [5e-5, 5e-5, 4e-5],
                                        [1.8e-7, 1.8e-7, 1e-8],
                                        [7e-6, 9e-6, 8e-6],
                                        [4.5e-7, 3.9e-7, 3.2e-7],
                                        [9.3e-7, 1.07e-6, 1.19e-6]])
    GList = np.ones((smuro_dof, 6, 6))
    for i in range(smuro_dof):
        GList[i] = np.r_[np.c_[np.diag(inertia[i]), np.zeros((3, 3))],
                         np.c_[np.zeros((3, 3)), mass[i] * np.identity(3, dtype=np.float32)]]

    # inverse dynamics test
    theta = np.zeros(smuro_dof, dtype=np.float32)

    return smuro_dof


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    parameters()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
