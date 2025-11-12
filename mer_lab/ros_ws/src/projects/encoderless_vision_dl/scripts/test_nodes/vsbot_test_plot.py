#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
import csv
from std_msgs.msg import Float32MultiArray, Int32
from PIL import Image


def main(args):    
    # Initialize ROS
    rospy.init_node('test_plotter')
    print("started plotting node")
    # read the model error from saved csv
    # rospy.sleep(2) 
    print("is main getting called")
    with open('/home/jc-merlab/Pictures/Data/plotting_data/keypoints.csv','r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        list_data1 = []
        for row in plots:
            list_data1.append(row)

    with open('/home/jc-merlab/Pictures/Data/plotting_data/controlpoints.csv','r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        list_data2 = []
        for row in plots:
            list_data2.append(row)

    file1 = "/home/jc-merlab/Pictures/Data/plotting_data/dl_feature_velocity.csv"
    f1 = open(file1, 'w')
    csvwriter1 = csv.writer(f1)
    csvwriter1.writerow(["vel1", "vel2", "vel3"])

    file2 = "/home/jc-merlab/Pictures/Data/plotting_data/shape_feature_velocity.csv"
    f2 = open(file2, 'w')
    csvwriter2 = csv.writer(f2)
    csvwriter2.writerow(["vel1", "vel2", "vel3"])

    # print(list_data)

    kpx1 = []
    kpx2 = []
    kpx3 = []

    kpy1 = []
    kpy2 = []
    kpy3 = []

    cpx1 = []
    cpx2 = []
    cpx3 = []

    cpy1 = []
    cpy2 = []
    cpy3 = []
    
    for i in range(len(list_data1[1:])):
        # print("Rows", i)
        # print("list_data", list_data[i+1])
        x1 = np.int64(list_data1[i+1][0])   
        kpx1.append(x1)
        y1 = np.int64(list_data1[i+1][1])
        kpy1.append(y1)


        x2 = np.int64(list_data1[i+1][2])
        y2 = np.int64(list_data1[i+1][3])

        kpx2.append(x2)
        kpy2.append(y2)        

        x3 = np.int64(list_data1[i+1][4])
        y3 = np.int64(list_data1[i+1][5])

        kpx3.append(x3)
        kpy3.append(y3)

    kp_vel1 = []
    kp_vel2 = []
    kp_vel3 = []

    velx1 = [(j-i) for i, j in zip(kpx1[:-1], kpx1[1:])]
    vely1 = [(j-i) for i, j in zip(kpy1[:-1], kpy1[1:])]

    for i in range(len(velx1)):
        vel1 = np.sqrt((velx1[i])**2+(vely1[i])**2)
        kp_vel1.append(vel1/(1/15))

    velx2 = [(j-i) for i, j in zip(kpx2[:-1], kpx2[1:])]
    vely2 = [(j-i) for i, j in zip(kpy2[:-1], kpy2[1:])]

    for i in range(len(velx2)):
        vel2 = np.sqrt((velx2[i])**2+(vely2[i])**2)
        kp_vel2.append(vel2/(1/15))

    velx3 = [(j-i) for i, j in zip(kpx3[:-1], kpx3[1:])]
    vely3 = [(j-i) for i, j in zip(kpy3[:-1], kpy3[1:])]

    for i in range(len(velx3)):
        vel3 = np.sqrt((velx3[i])**2+(vely3[i])**2)
        kp_vel3.append(vel3/(1/15))

    print(len(kp_vel1))

    for i in range(len(kp_vel1)):
        v1 = kp_vel1[i]        
        v2 = kp_vel2[i]
        v3 = kp_vel3[i]
        csvwriter1.writerow([v1, v2, v3])

    for i in range(len(list_data2[1:])):
        # print("Rows", i)
        # print("list_data", list_data[i+1])
        x1 = np.float64(list_data2[i+1][6])   
        cpx1.append(x1)
        y1 = np.float64(list_data2[i+1][7])
        cpy1.append(y1)

        x2 = np.float64(list_data2[i+1][8])
        y2 = np.float64(list_data2[i+1][9])

        cpx2.append(x2)
        cpy2.append(y2)        

        x3 = np.float64(list_data2[i+1][10])
        y3 = np.float64(list_data2[i+1][11])

        cpx3.append(x3)
        cpy3.append(y3)

    cp_vel1 = []
    cp_vel2 = []
    cp_vel3 = []

    cpvelx1 = [(j-i) for i, j in zip(cpx1[:-1], cpx1[1:])]
    cpvely1 = [(j-i) for i, j in zip(cpy1[:-1], cpy1[1:])]

    for i in range(len(cpvelx1)):
        vel1 = np.sqrt((cpvelx1[i])**2+(cpvely1[i])**2)
        cp_vel1.append(vel1/(1/15))

    cpvelx2 = [(j-i) for i, j in zip(cpx2[:-1], cpx2[1:])]
    cpvely2 = [(j-i) for i, j in zip(cpy2[:-1], cpy2[1:])]

    for i in range(len(cpvelx1)):
        vel2 = np.sqrt((cpvelx2[i])**2+(cpvely2[i])**2)
        cp_vel2.append(vel2/(1/15))

    cpvelx3 = [(j-i) for i, j in zip(cpx3[:-1], cpx3[1:])]
    cpvely3 = [(j-i) for i, j in zip(cpy3[:-1], cpy3[1:])]
    
    for i in range(len(cpvelx1)):
        vel3 = np.sqrt((cpvelx3[i])**2+(cpvely3[i])**2)
        cp_vel3.append(vel3/(1/15))

    for i in range(len(cp_vel1)):
        v1 = cp_vel1[i]        
        v2 = cp_vel2[i]
        v3 = cp_vel3[i]
        csvwriter2.writerow([v1, v2, v3])
       
    # Image.open('error_plot.png').convert("RGB").save('error_plot.jpg','JPEG')

   # Initialize subscribers
    # status_sub = rospy.Subscriber("vsbot/plot/status", Int32, statusCallback, queue_size = 1)

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")

    # plotting complete, shut down
    # rospy.signal_shutdown("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
