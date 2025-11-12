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

img_pub = rospy.Publisher("franka/vs/vis", Image, queue_size=1)

r = None

# traj_pts_x = []
# traj_pts_y = []

traj_cp1_x = []
traj_cp1_y = []

traj_cp2_x = []
traj_cp2_y = []

traj_cp3_x = []
traj_cp3_y = []

# traj_cp4_x = []
# traj_cp4_y = []

# traj_cp5_x = []
# traj_cp5_y = []

status = -2
itr = 0
height = 0
width = 0
goal_image = None

cp1x = 0
cp1y = 0

cp2x = 0
cp2y = 0

cp3x = 0
cp3y = 0

# cp4x = 0
# cp4y = 0

# cp5x = 0
# cp5y = 0

def controlPointCallback(cp_msg):
    # Read current control points
    global cp1x, cp1y, cp2x, cp2y, cp3x, cp3y, cp4x, cp4y, cp5x, cp5y, traj_cp1_x, traj_cp1_y, \
        traj_cp2_x, traj_cp2_y, traj_cp3_x, traj_cp3_y
            # traj_cp4_x, traj_cp4_y
    
    cp1x = cp_msg.data[0]
    cp1y = cp_msg.data[1]
    
    cp2x = cp_msg.data[2]
    cp2y = cp_msg.data[3]

    cp3x = cp_msg.data[4]
    cp3y = cp_msg.data[5]

    # cp4x = cp_msg.data[6]
    # cp4y = cp_msg.data[7]

    # cp5x = cp_msg.data[8]
    # cp5y = cp_msg.data[9]

    if status == 2:
        traj_cp1_x.append(cp1x)
        traj_cp1_y.append(cp1y)
        traj_cp2_x.append(cp2x)
        traj_cp2_y.append(cp2y)
        traj_cp3_x.append(cp3x)
        traj_cp3_y.append(cp3y)
        # traj_cp4_x.append(cp4x)
        # traj_cp4_y.append(cp4y)
        # traj_cp5_x.append(cp5x)
        # traj_cp5_y.append(cp5y)


def goalImgCallback(img_msg):
    # Read goal curve image
    global goal_image, bridge
    goal_image = bridge.imgmsg_to_cv2(img_msg,"bgr8")


def statusCallback(status_msg):
    # Read current status
    global status
    status = status_msg.data


# def eePosCallback(traj_msg):
#     # Read end effector position for visualization
#     global traj_pts_x, traj_pts_y

#     if status == 2:
#         traj_pts_x.append(int(traj_msg.data[0]))
#         traj_pts_y.append(int(traj_msg.data[1]))


def visCallback(msg):
    # Draw all results on current image frame

    # print("Received img")
    global bridge, img_pub, itr, height, width
    
    cv_img = bridge.imgmsg_to_cv2(msg,"bgr8")

    # Draw control points
    cv2.circle(cv_img, (int(cp1x), int(cp1y)), 5, (255,0,0),-1)
    cv2.circle(cv_img, (int(cp2x), int(cp2y)), 5, (0,255,0),-1)
    cv2.circle(cv_img, (int(cp3x), int(cp3y)), 5, (255,255,0),-1)
    # cv2.circle(cv_img, (int(cp4x), int(cp4y)), 3, (0,255,255),-1)
    # cv2.circle(cv_img, (int(cp5x), int(cp5y)), 3, (255,0,255),-1)

    if status == 1:
        # text
        cv_img = cv2.putText(cv_img, 'Initialization', (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'x2', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 2, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'Frame #: '+str(itr), (100, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)

    elif status == 2:
        # draw goal curve if servoing
        cv_img = cv2.addWeighted(cv_img, 0.4, goal_image, 0.6, 0)

        for i in range(len(traj_cp1_x) - 1):
            start_pt1 = (int(traj_cp1_x[i]), int(traj_cp1_y[i]))
            end_pt1 = (int(traj_cp1_x[i+1]), int(traj_cp1_y[i+1]))

            start_pt2 = (int(traj_cp2_x[i]), int(traj_cp2_y[i]))
            end_pt2 = (int(traj_cp2_x[i+1]), int(traj_cp2_y[i+1]))

            start_pt3 = (int(traj_cp3_x[i]), int(traj_cp3_y[i]))
            end_pt3 = (int(traj_cp3_x[i+1]), int(traj_cp3_y[i+1]))

            # start_pt4 = (int(traj_cp4_x[i]), int(traj_cp4_y[i]))
            # end_pt4 = (int(traj_cp4_x[i+1]), int(traj_cp4_y[i+1]))

            cv_img = cv2.line(cv_img, start_pt1, end_pt1,(255,0,0), 3)
            cv_img = cv2.line(cv_img, start_pt2, end_pt2,(0,255,0), 3)
            cv_img = cv2.line(cv_img, start_pt3, end_pt3,(255,255,0), 3)
            # cv_img = cv2.line(cv_img, start_pt4, end_pt4,(0,255,255), 1)

        # text
        cv_img = cv2.putText(cv_img, 'Servoing', (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'x2', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 2, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'Frame #: '+str(itr), (100, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1, cv2.LINE_AA)
    
    # Continue publishing vis when servoing is complete
    # Save images to file
    if status == -1:

        # draw goal curve
        # cv_img = cv2.circle(cv_img,(goalX, goalY), radius = 3, color=(0,255,0), thickness=2)
        cv_img = cv2.addWeighted(cv_img, 0.4, goal_image, 0.6, 0)

        # draw trajectory
        for i in range(len(traj_cp1_x) - 1):
            start_pt1 = (int(traj_cp1_x[i]), int(traj_cp1_y[i]))
            end_pt1 = (int(traj_cp1_x[i+1]), int(traj_cp1_y[i+1]))

            start_pt2 = (int(traj_cp2_x[i]), int(traj_cp2_y[i]))
            end_pt2 = (int(traj_cp2_x[i+1]), int(traj_cp2_y[i+1]))

            start_pt3 = (int(traj_cp3_x[i]), int(traj_cp3_y[i]))
            end_pt3 = (int(traj_cp3_x[i+1]), int(traj_cp3_y[i+1]))

            # start_pt4 = (int(traj_cp4_x[i]), int(traj_cp4_y[i]))
            # end_pt4 = (int(traj_cp4_x[i+1]), int(traj_cp4_y[i+1]))

            # start_pt5 = (int(traj_cp5_x[i]), int(traj_cp5_y[i]))
            # end_pt5 = (int(traj_cp5_x[i+1]), int(traj_cp5_y[i+1]))

            cv_img = cv2.line(cv_img, start_pt1, end_pt1,(255,0,0), 3)
            cv_img = cv2.line(cv_img, start_pt2, end_pt2,(0,255,0), 3)
            cv_img = cv2.line(cv_img, start_pt3, end_pt3,(255,255,0), 3)
            # cv_img = cv2.line(cv_img, start_pt4, end_pt4,(0,255,255), 1)
            # cv_img = cv2.line(cv_img, start_pt5, end_pt5,(255,0,255), 1)

        # text
        # cv_img = cv2.putText(cv_img, 'Servoing completed', (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 2, cv2.LINE_AA)
        # cv_img = cv2.putText(cv_img, 'x2', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 2, cv2.LINE_AA)

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
        
        out = cv2.VideoWriter("exp_vid.avi",cv2.VideoWriter_fourcc(*'XVID'), 2*r, size)

        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()
        print("Video saved")

        # for i in file_list:
        #     os.remove(str(i)+".png")

        # print("Removed temp files")

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


def main(args):
    # global img_pub
    global r
    # Initialize ROS
    rospy.init_node('visualizer')
    print("Initialized vis")

    # Initialize subscribers
    img_sub = rospy.Subscriber("camera/color/image_raw", Image, visCallback, queue_size = 1)
    # ee_sub = rospy.Subscriber("marker_segmentation", Float64MultiArray, eePosCallback, queue_size=1)
    status_sub = rospy.Subscriber("vsbot/status", Int32, statusCallback, queue_size = 1)
    goal_sub = rospy.Subscriber("franka/dl_goal_image",Image, goalImgCallback, queue_size=1)
    cp_sub = rospy.Subscriber("vsbot/control_points", Float64MultiArray, controlPointCallback)
    r = rospy.get_param("vsbot/estimation/rate")
    # publish robot trajectory
    # publish cartesian velocity vector
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
