#!/usr/bin/env python3

# ROS deps
import rospy
from std_msgs.msg import Float64MultiArray

# Services
from origami_vision.srv import curve_msg

import time

def main():
    rospy.init_node('clothoid_fit_client')
    
    # Subscribers
    # marker_sub = rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, getMarkers, queue_size=1)

    # Service server
    # clothoid_server = rospy.Service('clothoid_fit_srv', curve_msg, fitClothoid)
    rospy.wait_for_service("clothoid_fit_srv")
    fit_clothoid = rospy.ServiceProxy('clothoid_fit_srv', curve_msg)

    while not rospy.is_shutdown():
        t0 = time.time()
        resp = fit_clothoid()
        t1 = time.time()
        print(t1-t0,"s")

    # print("Initialized clothoids")

    rospy.spin()


if __name__ == '__main__':
    main()
