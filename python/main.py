# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np


def parameters():
    smuro_dof = 7
    # pitch and yaw axis w.r.t. space frame
    w_pitch = np.array([0, -1, 0])
    w_yaw = np.array([0, 0, 1])
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
    M0 = np.r_[np.c_[np.eye(3), com[0]], [[0, 0, 0, 1]]]
    M1 = np.r_[np.c_[np.eye(3), com[1]], [[0, 0, 0, 1]]]
    M2 = np.r_[np.c_[np.eye(3), com[2]], [[0, 0, 0, 1]]]
    M3 = np.r_[np.c_[np.eye(3), com[3]], [[0, 0, 0, 1]]]
    M4 = np.r_[np.c_[np.eye(3), com[4]], [[0, 0, 0, 1]]]
    M5 = np.r_[np.c_[np.eye(3), com[5]], [[0, 0, 0, 1]]]
    M6 = np.r_[np.c_[np.eye(3), com[6]], [[0, 0, 0, 1]]]
    M7 = np.r_[np.c_[np.eye(3), com[7]], [[0, 0, 0, 1]]]


    return smuro_dof


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    parameters()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
