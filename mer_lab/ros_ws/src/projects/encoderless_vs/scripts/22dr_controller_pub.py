#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image, JointState # Image is the message type
from std_msgs.msg import Float64MultiArray, Float64, Int32, Bool
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import rosbag
import time
import csv
import random

Xc = 0
Yc = 0

Xd = 0
Yd = 0

q1 = 0.52;
q2 = -0.52;

# Link 1 and 2 lengths from Gazebo file
a1 = 0.5
a2 = 0.5

# Proportional gain constant
k = 1

# Declare publishers to be used
joint1_pub = rospy.Publisher('/vsbot/joint1_velocity_controller/command', Float64, queue_size=1)
joint2_pub = rospy.Publisher('/vsbot/joint2_velocity_controller/command', Float64, queue_size=1)

def pos_callback(data):
	# Global variables to be passed between functions
	global Xc
	# Data is in pixel values
	Xc = data.data[0]
	global Yc
	Yc = data.data[1]
	
	global Xd
	Xd = data.data[2]
	global Yd
	Yd = data.data[3]
	
	# print (data.data[0])
	# print (data.data[1])
	
def joint_callback(data):
	global q1
	q1 = data.position[0]
	global q2
	q2 = data.position[1]
	
def controller():
	# Jacobian matrix calculations
	j00 = a1*np.cos(q1)+a2*np.cos(q1+q2)
	j01 = a2*np.cos(q1+q2)
	j10 = a1*np.sin(q1)+a2*np.sin(q1+q2)
	j11 = a2*np.sin(q1+q2)
	
	# Formation of the Jacobian
	J = np.matrix([[j00, j01], [j10, j11]])
	
	# Calculate error values, place in error matrix
	ex = Xd - Xc
	ey = Yd - Yc
	e = np.matrix([[ex], [ey]])
	
	# print(e)
	
	# Psuedo-inverse of Jacobian matrix
	J_i = np.linalg.pinv(J)
	# J_i = np.matmul(np.linalg.inv(np.matmul(np.transpose(J),J)),np.transpose(J))
	# J_i = np.linalg.inv(J)
	
	# Controller input
	u = k*np.matmul(J_i,e)
	
	now = rospy.get_rostime()
	seconds = now.to_sec()
	
	# Load velocity values to be published
	q1_dot = u[0]
	q2_dot = u[1]
	
	# max_vel = 1
	# if q1_dot > max_vel:
	# 	q1_dot = max_vel
	# if q1_dot < -max_vel:
	# 	q1_dot = -max_vel
	# if q2_dot > max_vel:
	# 	q2_dot = max_vel
	# if q2_dot < -max_vel:
	# 	q2_dot = -max_vel
	
	# Publish joint velocity values
	joint1_pub.publish(q1_dot)
	joint2_pub.publish(q2_dot)
	
	return seconds, j00, j01, j10, j11, q1_dot, q2_dot
	
# def csv_write():
	
# 	csv_writer.writerow([q1, q2, Xc, Yc])	

def main():
	# Subscriber to get current and desired position values
	pos_sub = rospy.Subscriber('/pos', Float64MultiArray, pos_callback)
	
	# Subscriber to get current joint state values
	joint_sub = rospy.Subscriber('/vsbot/joint_states', JointState, joint_callback)
	
	with open('direct_velocity_tracker_data.csv', 'w') as f:
		csv_writer = csv.writer(f)
		
		csv_writer.writerow(['Q1', 'Q2', 't', 'X', 'Y', 'j00', 'j01', 'j10', 'j11', 'Q1_dot', 'Q2_dot'])
	
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			t, j00, j01, j10, j11, q1_input, q2_input = controller()
		
			csv_writer.writerow([q1, q2, t, Xc, Yc, j00, j01, j10, j11, q1_input, q2_input])	
		
			rate.sleep()
	
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('bot_controller', anonymous=False)
	
	time.sleep(5)
	
	main()
