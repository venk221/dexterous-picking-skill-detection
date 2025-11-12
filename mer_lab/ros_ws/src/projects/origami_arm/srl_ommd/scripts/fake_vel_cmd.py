#!/usr/bin/env python3
# This node is for Soft Robotics Lab Origami Module Motor Driver

import rospy
from std_msgs.msg import Int32MultiArray

import math
import time

def fake_vel_cmd():
    pub = rospy.Publisher('origami_vs/OMMD_velocity', Int32MultiArray, queue_size=1)
    rospy.init_node('fake_vel', anonymous=True)
    rate = rospy.Rate(100)

    u = [0]*32
    tic = time.time()
    while not rospy.is_shutdown():
        tt = time.time() - tic
        for k in range(32):
            u[k] = int(math.sin(tt + k)*1000)
        pub_vel_cmd = Int32MultiArray()
        pub_vel_cmd.data = u
        pub.publish(pub_vel_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_vel_cmd()
    except rospy.ROSInterruptException:
        pass
    
    
