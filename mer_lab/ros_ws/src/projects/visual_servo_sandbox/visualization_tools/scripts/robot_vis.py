#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

cur_img = None
cur_features = []
cur_target = []
bridge = CvBridge()
end_flag = False
rate = 0
traj_x = []
traj_y = []

def getFlag(msg):
    global end_flag
    end_flag = msg.data

def getReference(msg):
    global cur_target
    cur_target = msg.data

def getFeatures(msg):
    global cur_features
    cur_features = msg.data
    traj_x.append(cur_features[0])
    traj_y.append(cur_features[1])

def getImage(msg):
    global bridge, cur_img
    cur_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def main():
    global rate
    rospy.init_node("visualizer")

    image_sub = rospy.Subscriber("two_link/camera1/image_raw", Image, getImage, queue_size=1)
    feature_sub = rospy.Subscriber("two_link_vs/features", Float64MultiArray, getFeatures, queue_size=1)
    flag_sub = rospy.Subscriber("two_link_vs/end_flag", Bool, getFlag, queue_size=1)
    goal_sub = rospy.Subscriber("two_link_vs/reference", Float64MultiArray, getReference, queue_size=1)

    annotated_img_pub = rospy.Publisher("two_link_vs/annotated_img", Image, queue_size=1)
    
    rate = rospy.get_param("two_link_vs/control_rate")
    
    # vis code
    r = rospy.Rate(rate)
    
    while cur_img is None:
        r.sleep()

    
    while not end_flag:
        # Draw end effector center
        cv2.circle(cur_img, (int(cur_features[0]), int(cur_features[1])), 3, (0, 79, 153), -1)
        
        # Draw target location
        if(len(cur_target)>0):
            cv2.circle(cur_img, (int(cur_target[0]), int(cur_target[1])), 3, (10, 194, 255), -1)
        
        # Draw end effector trajectory
        for i in range(0, len(traj_x)-1):
            cv2.line(cur_img, (int(traj_x[i]), int(traj_y[i])), (int(traj_x[i+1]), int(traj_y[i+1])),(0, 79, 153), 2)
        
        # Publish image
        img_msg = bridge.cv2_to_imgmsg(cur_img, "bgr8")
        annotated_img_pub.publish(img_msg)

        r.sleep()

    # Write final image
    rospy.spin()

if __name__ == "__main__":
    main()