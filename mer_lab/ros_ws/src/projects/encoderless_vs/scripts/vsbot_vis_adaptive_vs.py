#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import glob
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int32

bridge = CvBridge()

goalX = 0
goalY = 0

img_pub = rospy.Publisher("vsbot/vis", Image, queue_size=1)

r = rospy.get_param("vsbot/estimation/rate")

traj_pts_x = []
traj_pts_y = []

status = -2
itr = 0
height = 0
width = 0


def statusCallback(msg):
    global status
    status = msg.data


def eePosCallback(msg):
    global traj_pts_x, traj_pts_y, status

    if status == 2:
        traj_pts_x.append(int(msg.data[0]))
        traj_pts_y.append(int(msg.data[1]))


def visCallback(msg):
    # print("Received img")
    global bridge, goalX, goalY, img_pub, itr, height, width
    # rate = rospy.Rate(r)
    try:
        cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        print(e)

    if status == 2:
        # draw goal marker if servoing
        cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,255,0), thickness=2)

        # draw trajectory if servoing
        for i in range(len(traj_pts_x) - 1):
            start_pt = (traj_pts_x[i], traj_pts_y[i])
            end_pt = (traj_pts_x[i+1], traj_pts_y[i+1])
            cv_img = cv2.line(cv_img, start_pt, end_pt,(0,0,255), 2)
    elif status == -1:
        # draw goal marker if servoing
        cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,255,0), thickness=2)

        # draw trajectory if servoing
        for i in range(len(traj_pts_x) - 1):
            start_pt = (traj_pts_x[i], traj_pts_y[i])
            end_pt = (traj_pts_x[i+1], traj_pts_y[i+1])
            cv_img = cv2.line(cv_img, start_pt, end_pt,(0,0,255), 2)

        # save trajectory image
        cv2.imwrite("traj.jpg",cv_img)
        print("Trajectory saved")

        # stitch video frames
        img_array = []
        file_list = []

        for file in glob.glob("*.png"):
            file_tup = file.partition('.')
            file_list.append(int(file_tup[0]))

        file_list.sort()

        for i in file_list:
            img = cv2.imread(str(i)+".png")
            height, width, layers = img.shape
            size = (width, height)
            img_array.append(img)
        
        out = cv2.VideoWriter("exp_vid.avi",cv2.VideoWriter_fourcc(*'XVID'), r, size)

        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()
        print("Video saved")

        for i in file_list:
            os.remove(str(i)+".png")

        print("Removed temp files")

        # Shut down
        rospy.signal_shutdown("Shutting down")

    if status >= 1:
        # capture video
        fname = str(itr) + ".png"
        cv2.imwrite(fname, cv_img)
        itr += 1

    
    # convert cv image to ROS msg
    try:
        ros_img = bridge.cv2_to_imgmsg(cv_img,"bgr8")
    except CvBridgeError as e:
        print(e)

    img_pub.publish(ros_img)
    # rate.sleep()






def main(args):
    global goalX, goalY, img_pub
    
    # Initialize ROS
    rospy.init_node('visualizer')
    print("Initialized vis")
    
    # Read goal posn from YAML
    goalX = rospy.get_param("vsbot/control/goal_pos_x")
    goalY = rospy.get_param("vsbot/control/goal_pos_y")

    # Initialize subscribers
    img_sub = rospy.Subscriber("vsbot/camera1/image_raw", Image, visCallback, queue_size = 1)
    ee_sub = rospy.Subscriber("marker_segmentation", Float64MultiArray, eePosCallback, queue_size=1)
    status_sub = rospy.Subscriber("vsbot/status", Int32, statusCallback, queue_size = 1)

    # publish robot trajectory
    # publish cartesian velocity vector
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
