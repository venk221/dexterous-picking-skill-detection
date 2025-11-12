#!/usr/bin/env python3.8
import numpy as np
from img_pix_conv import camera_ht

def world_coords():

    t0 = [0.000, 0.000, 0.000]
    rot0 = [0.000, 0.000, 0.000, 1.000]

    ht0 = camera_ht(t0, rot0)

    print("ht0 = ", ht0)
    

    t1 = [0.000, 0.000, 0.333]
    rot1 = [0.000, 0.000, 0.001, 1.000]

    ht1 = camera_ht(t1, rot1)

    print("ht1 = ", ht1)
    # print("ht[0]", ht[0])    

    t2 = [0.000, 0.000, 0.333]
    rot2 = [-0.679, -0.196, -0.195, 0.680]


    ht2 = camera_ht(t2, rot2)

    print("ht2 = ", ht2)
    # print("ht[1] = ", ht[1])   

    t3 = [-0.168, -0.000, 0.601]
    rot3 = [-0.000, -0.277, 0.003, 0.961]

    ht3 = camera_ht(t3, rot3)

    print("ht3 = ", ht3)
    # print("ht[2]= ", ht[2])    

    t4 =  [-0.098, 0.000, 0.644]
    rot4 = [0.608, 0.360, -0.357, 0.611]

    ht4 = camera_ht(t4, rot4)

    print("ht4 = ", ht4)
    # print("ht[3] = ", ht[3])    

    t5 = [0.198, 0.003, 0.903]
    rot5 = [-0.005, 0.507, -0.002, 0.862]

    ht5 = camera_ht(t5, rot5)

    print("ht5 = ", ht5)
    # print("ht[4] = ", ht[4])    

    t6 = [0.198, 0.003, 0.903]

    rot6 = [0.641, -0.290, 0.294, 0.647]

    ht6 = camera_ht(t6, rot6)

    print("ht6 = ", ht6)
    # print("ht[5] = ", ht[5])    

    t7 = [0.256, 0.003, 0.969]
    rot7 = [0.821, 0.394, 0.370, 0.182]

    ht7 = camera_ht(t7, rot7)

    print("ht7 = ", ht7)
    # print("ht[6] = ", ht[6])

    p_j0 = np.dot(ht0, np.transpose([0, 0, 0, 1]))
    p_j1 = np.dot(ht1,np.transpose([0, 0, 0, 1]))
    # print(p_j1)
    p_j2 = np.dot(ht2, np.transpose([0, 0, 0, 1]))
    # print(p_j2)
    p_j3 = np.dot(ht3, np.transpose([0, 0, 0, 1]))
    p_j4 = np.dot(ht4, np.transpose([0, 0, 0, 1]))
    p_j5 = np.dot(ht5, np.transpose([0, 0, 0, 1]))
    p_j6 = np.dot(ht6, np.transpose([0, 0, 0, 1]))
    p_j7 = np.dot(ht7, np.transpose([0, 0, 0, 1]))

    world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4, p_j5, p_j6, p_j7]

    # print(p_j1,p_j2,p_j3,p_j4,p_j5,p_j6,p_j7)
    return world_coords

if __name__ == "__main__":
    world_coords()