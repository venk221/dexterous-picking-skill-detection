#!/usr/bin/env python3
# This node is for visualizing useful info during experiment runtime

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

# Placeholder objects
goal = []
ee_pose = []
base_pose = []
ee_traj = []
end_flag = False

# Visualization publisher
img_pub = rospy.Publisher("origami_vs/vis", Image, queue_size=1)

bridge = CvBridge()

def getEndFlag(flag):
    global end_flag

    # Update status of end flag
    end_flag = flag.data


def imgPlotter(img_msg):
    global bridge

    if not end_flag:
        # Convert to cv img
        img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        # Write raw img to folder

        # Draw goal position
        img = cv2.circle(img, (goal[0], goal[1]), 5, (166, 176, 64), -1 )
        # Draw EE trajectory
        for i in range(len(ee_traj) -1):
            start = (int(ee_traj[i][0]),int(ee_traj[i][1]))
            end = (int(ee_traj[i+1][0]), int(ee_traj[i+1][1]))

            img = cv2.line(img, start, end,(89,17,212), 2)

        # Write annotated image to folder
        
        # Convert to ROS img
        img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        img_pub.publish(img_msg)

    else:
        # Initiate shutdown request
        # rospy.signal_shutdown("End Signal Received")
        pass
        # The bag writer is initiating shutdown request


def getPose(pose_msg):
    global ee_pose, ee_traj, base_pose

    # Update current marker positions in image space
    ee_pose = pose_msg.data[0:2]
    base_pose = pose_msg.data[2:4]
    ee_traj.append(ee_pose)
    

def main():
    global goal

    # Initialize ROS
    rospy.init_node('visualizer')
    print("Initialized visualization")

    # Read goal from YAML
    # goal_x = rospy.get_param("origami_vs/goal_pose_x")
    # goal_y = rospy.get_param("origami_vs/goal_pose_y")
    # goal = [goal_x, goal_y]
    goal = rospy.get_param("origami_adaptive_vs/goal_pose")
    
    # Initialize Subscribers
    img_sub = rospy.Subscriber("origami_vs/aruco/result", Image, imgPlotter, queue_size = 1)
    pose_sub = rospy.Subscriber("origami_vs/aruco/pose", Float64MultiArray, getPose, queue_size=1)
    end_flag = rospy.Subscriber("origami_vs/end_flag", Bool, getEndFlag, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__ == "__main__":
    main()