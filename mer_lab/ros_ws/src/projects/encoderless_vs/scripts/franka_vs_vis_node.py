#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import glob
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import msg
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

bridge = CvBridge()

traj_pts_x = []
traj_pts_y = []

goalX = None
goalY = None

# status = -2
itr = 0

ros_img = None

def eePosCallback(traj_msg):
    # Read end effector position for visualization
    global traj_pts_x, traj_pts_y

    # if status == 2:
    traj_pts_x.append(traj_msg.data[0])
    traj_pts_y.append(traj_msg.data[1])    


def visCallback(msg):
    # Draw all results on current image frame
    global bridge, img_pub, itr, ros_img
    
    # convert published ros image to cv
    cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")

    # build an array of pose
    points = np.c_[traj_pts_x, traj_pts_y]

    # draw the pose as the robot moves
    for i in range(len(points)):
        x = np.int(points[i][0])
        y = np.int(points[i][1])
        cv2.circle(cv_img,(x, y),3,(255, 0, 0),-1)
        cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 4, color=(0,255,0), thickness=2)
        cv_img = cv2.putText(cv_img, 'goal', (goalX, goalY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
    cv2.imwrite("frames"+str(itr)+".jpg", cv_img)
    itr = itr+1
    
    # draw the final trajectory image
    cv2.imwrite("traj.jpg",cv_img)
    print("Trajectory saved")

    # convert cv_img to ros_img to be published
    try:
        ros_img = bridge.cv2_to_imgmsg(cv_img,"bgr8")
    except CvBridgeError as e:
        print(e)

def main(args):
    global goalX, goalY, img_pub
    
    # Initialize ROS
    rospy.init_node('franka_visualizer')
    print("Initialized vis")

    # Initialize subscribers
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, visCallback, queue_size = 1) 
    ee_sub = rospy.Subscriber("aruco/Pose", Float32MultiArray, eePosCallback, queue_size=1)

    # Read goal positions from config params
    goalX = rospy.get_param("vsbot/control/goal_pos_x")
    goalY = rospy.get_param("vsbot/control/goal_pos_y")

    # publish robot trajectory
    img_pub = rospy.Publisher("franka/vs/vis", Image, queue_size=1)

    r = rospy.get_param("vsbot/estimation/rate")
    rate = rospy.Rate(r)

    while not rospy.is_shutdown():
        if ros_img is not None:
            img_pub.publish(ros_img)
        rate.sleep()

    # publish cartesian velocity vector
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
