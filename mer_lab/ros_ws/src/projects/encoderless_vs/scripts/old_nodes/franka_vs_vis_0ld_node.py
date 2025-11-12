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
from std_msgs.msg import Float32MultiArray, Int32

bridge = CvBridge()

# img_pub = rospy.Publisher("franka/vs/vis", Image, queue_size=1)


traj_pts_x = []
traj_pts_y = []

goalX = None
goalY = None

# status = -2
itr = 0
height = 0
width = 0

ros_img = None

# def statusCallback(status_msg):
#     # Read current status
#     global status
#     status = status_msg.data


def eePosCallback(traj_msg):
    # Read end effector position for visualization
    global traj_pts_x, traj_pts_y

    print("latest EE", int(traj_msg.data[0]), int(traj_msg.data[1]))

    # if status == 2:
    traj_pts_x.append(int(traj_msg.data[0]))
    traj_pts_y.append(int(traj_msg.data[1]))


def visCallback(msg):
    # Draw all results on current image frame

    # print("Received img")
    global bridge, img_pub, itr, height, width, ros_img
    
    cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")

    # # if status == 1:
    #     # print("Status published", status)
    #     # text
    # cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,255,0), thickness=2)
    # cv_img = cv2.putText(cv_img, 'Initialization', (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
    # cv_img = cv2.putText(cv_img, 'x2', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 2, cv2.LINE_AA)
    # cv_img = cv2.putText(cv_img, 'Frame #: '+str(itr), (100, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)

    # elif status == 2:

        # for i in range(len(traj_cp1_x) - 1):
    for i in range(len(traj_pts_x) - 1):
        # using ee pose in place of control points
        start_pt1 = (int(traj_pts_x[i]), int(traj_pts_y[i]))
        end_pt1 = (int(traj_pts_x[i+1]), int(traj_pts_y[i+1]))

        cv_img = cv2.line(cv_img, start_pt1, end_pt1,(255,0,0), 4)
        cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 5, color=(0,255,0), thickness=2)
        cv_img = cv2.putText(cv_img, 'goal', (goalX, goalY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)

        # cv_img = cv2.resize(cv_img, (640, 480), interpolation = cv2.INTER_AREA)

        # text
        # cv_img = cv2.putText(cv_img, 'Servoing', (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'x2', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 2, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'Frame #: '+str(itr), (100, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
        cv2.imwrite("frames"+str(itr)+".jpg", cv_img)
        itr = itr+1
    
    # Continue publishing vis when servoing is complete
    # Save images to file
    # if status == -1:
        # print("Status published", status)
        # draw goal curve
    # cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,255,0), thickness=2)
    # draw trajectory
    # for i in range(len(traj_cp1_x) - 1):
    # for i in range(len(traj_pts_x) - 1):
    #     # using ee pose in place of control points
    #     start_pt1 = (int(traj_pts_x[i]), int(traj_pts_y[i]))
    #     end_pt1 = (int(traj_pts_x[i+1]), int(traj_pts_y[i+1]))
    #     cv_img = cv2.line(cv_img, start_pt1, end_pt1,(255,0,0), 2)

        # text
        # cv_img = cv2.putText(cv_img, 'Servoing completed', (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'x2', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 2, cv2.LINE_AA)

        # save trajectory image
    cv2.imwrite("traj.jpg",cv_img)
    print("Trajectory saved")

    # stitch video frames
    # img_array = []
    # file_list = []
    # for file in glob.glob("*.png"):
    #     file_tup = file.partition('.')
    #     file_list.append(int(file_tup[0]))
    # file_list.sort()
    # for i in file_list:
    #     img = cv2.imread(str(i)+".png")
    #     height, width, layers = img.shape
    #     # size = (width, height)
    #     img_array.append(img)
    
    # out = cv2.VideoWriter("exp_vid.avi",cv2.VideoWriter_fourcc(*'XVID'), 2*r, (width, height))
    # for i in range(len(img_array)):
    #     out.write(img_array[i])
    # out.release()
    # print("Video saved")

    # for i in file_list:
    #     os.remove(str(i)+".png")
    # print("Removed temp files")
    # # Shut down
    # rospy.signal_shutdown("Shutting down")

    # # if status >= 1:
    # # capture video
    # fname = str(itr) + ".png"
    # cv2.imwrite(fname, cv_img)
    # itr += 1

    
    # convert cv image to ROS msg
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
    # status_sub = rospy.Subscriber("vsbot/status", Int32, statusCallback, queue_size = 1)

    # Read params
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
