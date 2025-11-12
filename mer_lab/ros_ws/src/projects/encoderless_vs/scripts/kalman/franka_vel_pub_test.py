#!/usr/bin/env python3
# license removed for brevity
import sys
import rospy
import cv2
import numpy as np
from skimage.morphology import skeletonize
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
from scipy.interpolate import splprep
from cv_bridge import CvBridge, CvBridgeError
from controller_manager_msgs.srv import SwitchController, LoadController
from encoderless_vs.srv import bin_img, bin_imgResponse, cp_goal, cp_goalResponse

# Declaring cvBridge for cv to ros conversion and vice versa
bridge = CvBridge()
current_ros_image = None
control_points_output = None
status = None

def update_vel(msg):
    global status
    status = msg.data   

def main():
    # Initialize the node
    rospy.init_node('pos_control_dream_depth_ds_gen')    
    status_sub =  rospy.Subscriber("/vel/status", Float64, update_vel, queue_size=1)

    pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
    joint_vel_01 = Float64MultiArray()
    joint_vel_02 = Float64MultiArray()
    joint_vel_01.data = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    joint_vel_02.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    while not rospy.is_shutdown():
        if status == 1.0: 
            pub.publish(joint_vel_01)        
        else:
            pub.publish(joint_vel_02)

    rospy.spin()


if __name__ == '__main__':
    main()
    
