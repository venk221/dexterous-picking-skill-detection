import math
from ntpath import join
import numpy as np


class Kinematics():
    def __init__(self, joint_angles):
        self.joint_angles = joint_angles
    
        DH_params = {"joint1": [0, 0.333,             0,   joint_angles[0]],
                    "joint2": [0,     0,  -(math.pi/2),   joint_angles[1]],
                    "joint3": [0, 0.316,   (math.pi/2),   joint_angles[2]],
                    "joint4": [0.0825,     0,   (math.pi/2),   joint_angles[3]],
                    "joint5": [-0.0825, 0.384,  -(math.pi/2),   joint_angles[4]],
                    "joint6": [0,     0,   (math.pi/2),   joint_angles[5]],
                    "joint7": [0.088,     0,   (math.pi/2),   joint_angles[6]], }

        self.DH_params = DH_params

    def forward(self):
        # print(self.DH_params)

        joints = ["joint1", "joint2", "joint3",
                  "joint4", "joint5", "joint6", "joint7"]

        # joints = ["joint1"]

        T_old = np.eye(4)
        T_lst = []

        for joint in joints:
            T_new = self.t_matrix(joint)
            T = T_old @ T_new
            T = np.around(T, decimals=3)
            T_lst.append(T)
            T_old = T

        # print(len(T_lst))

        # for i in T_lst:
        #     print(i)

        return T_lst

    def t_matrix(self, joint):
        a, d, alpha, theta = self.DH_params[joint]
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        T = np.array([[ct,   -st*ca,  st*sa,  a*ct],
                      [st,    ct*ca, -ct*sa,  a*st],
                      [0,       sa,     ca,     d],
                      [0,        0,      0,     1]])
        return T


# if __name__ == '__main__':
#     joint_angles = [0.523599, 0.523599, 0.523599,
#                     0.523599, 0.523599, 0.523599, 0.523599]

#     forward = Kinematics(joint_angles)
#     forward.forward()

    # print(np.around(forward.t_matrix("joint1"),decimals=3))
