#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image, JointState # Image is the message type
from std_msgs.msg import Float64MultiArray, Int64MultiArray, Float64, Bool
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import rosbag

bag_1 = rosbag.Bag('visual_data.bag', 'w')

bag_2 = rosbag.Bag('motor_data.bag', 'w')

shutdown_flag = False

def motor_callback(data):
	bag_2.write('q1', data.position[0])
	bag_2.write('q2', data.position[1])

def visual_callback(data):
	bag_1.write('s', data)
	
def flag_callback(data):
	global shutdown_flag
	shutdown_flag = data.data
	
	if shutdown_flag:
		bag_1.close()
		bag_2.close()
		rospy.signal_shutdown('Mission Complete')

def main():
	flag_sub = rospy.Subscriber('/shutdown_flag', Bool, flag_callback)

	visual_sub = rospy.Subscriber('/pos', Float64, visual_callback)
	
	motor_sub = rospy.Subscriber('/vsbot/joint_states', JointState, motor_callback)
	
	rospy.spin()

if __name__ == '__main__':
    	rospy.init_node('bagwriter', anonymous=False, disable_signals=True)
    	main()
