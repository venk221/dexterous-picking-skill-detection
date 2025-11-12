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
    with open('/home/janch-ros/.ros/coeff_record.csv','r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        list_data = []
        for row in plots:
            list_data.append(row)

        for i in range(len(list_data[1:])):
            print("Rows", i)
            print("list_data", list_data[i+1])
            a = float(list_data[i+1][0])
            print(type(a))
            b = float(list_data[i+1][1])
            c = float(list_data[i+1][2])
            x_start = float(list_data[i+1][6])
            x_end = float(list_data[i+1][7])
            x = np.linspace(x_start,x_end,10)
            y = a*x**2 + b*x + c
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.plot(x, -y, 'r+')
            ax.set_ylabel('generated y')
            ax.set_xlabel('x within limits')
            ax.legend(['curve segment 1'])
            plt.savefig('/home/janch-ros/Pictures/first_curve_plot'+str(i+1)+'.png')
        
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
