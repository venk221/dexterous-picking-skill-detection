#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image, JointState # Image is the message type
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import rosbag
import time
import csv

Xc = 0
Yc = 0

Xd = 0
Yd = 0

q1 = 0.52
q2 = -0.52

q1d = 0
q2d = 0

# Link 1 and 2 lengths from Gazebo file
a1 = 0.5
a2 = 0.5

# Proportional gain constant
k = 1

window_size = 1

disturbance = 0

refresh_rate = 5

update_gain = 1

update_range = 0.2

# Declare publishers to be used
joint1_pub = rospy.Publisher('/vsbot/joint1_velocity_controller/command', Float64, queue_size=1)
joint2_pub = rospy.Publisher('/vsbot/joint2_velocity_controller/command', Float64, queue_size=1)

def pos_callback(data):
	# Global variables to be passed between functions
	# Data is in pixel values
	
	# Xc, Yc are current coordinates
	global Xc
	global Yc
	
	# Coordinates come in as pixel data
	Xc = data.data[0]
	Yc = data.data[1]
	
	# Xd, Yd are desired visual data values
	global Xd
	global Yd
	
	# Coordinates come in as pixel data
	Xd = data.data[2]
	Yd = data.data[3]
	
	# Q1d, Q2d are desired motor data values
	# global q1d
	# q1d = data.data[4]
	# global q2d
	# q2d = data.data[5]
	
def joint_callback(data):
	# q1, q2 are current joint position values
	global q1
	q1_a = data.position[0]
	q1_b = np.sin(q1_a)
	q1 = np.arcsin(q1_b)
	global q2
	q2_a = data.position[1]
	q2_b = np.sin(q2_a)
	q2 = np.arcsin(q2_b)
	
def initialJacobian():
	
	# Jacobian matrix calculations, ground truth calculations
	j00 = a1*np.cos(q1)+a2*np.cos(q1+q2)
	j01 = a2*np.cos(q1+q2)
	j10 = a1*np.sin(q1)+a2*np.sin(q1+q2)
	j11 = a2*np.sin(q1+q2)
	
	j00_dist = j00 * disturbance
	j01_dist = j01 * disturbance
	j10_dist = j10 * disturbance
	j11_dist = j11 * disturbance
	
	j00 = j00 + j00_dist
	j01 = j01 + j01_dist
	j10 = j10 + j10_dist
	j11 = j11 + j11_dist
	
	J = np.matrix([[j00, j01], [j10, j11]])
	
	# J = np.matrix([[1, 2], [3, 4]])
	
	return J
	
def controller(J):
	# Calculate error values, place in error matrix
	ex = Xd - Xc
	ey = Yd - Yc
	e = np.matrix([[ex], [ey]])
		
	# Psuedo-inverse of Jacobian matrix
	J_i = np.linalg.pinv(J)
		
	# Controller input
	u = k*np.matmul(J_i,e)
	# print (u)
		
	# Load velocity values to be published
	q1_dot = u[0]
	q2_dot = u[1]
	
	max_vel = 1
	if q1_dot > max_vel:
		q1_dot = max_vel
	if q1_dot < -max_vel:
		q1_dot = -max_vel
	if q2_dot > max_vel:
		q2_dot = max_vel
	if q2_dot < -max_vel:
		q2_dot = -max_vel
		
	# Publish joint velocity values
	joint1_pub.publish(q1_dot)
	joint2_pub.publish(q2_dot)
	
def updateJacobian(J, J_data, q_data, s_data):
	# Store visual and motor data for calculations
	x_current = Xc
	y_current = Yc
	q1_current = q1
	q2_current = q2
	
	J_sum = np.zeros((2, 2))
	
	for x in range(window_size):
	
		# Observed change in visual features
		dY_x = x_current - s_data[x, 0]
		dY_y = y_current - s_data[x, 1]
		dY = np.matrix([[dY_x], [dY_y]])
		
		# Known change in motor values
		dX_1 = q1_current - q_data[x, 0]
		dX_2 = q2_current - q_data[x, 1]
		dX = np.matrix([[dX_1], [dX_2]])
		
		# Current motor values
		X = np.matrix([[q1_current], [q2_current]])

	# J_k+1' = J_k + (dY - J_k X) X^T / X^T X
	# Where:
	# 	 dY is the change in visual features
	#	 X is the motor features
	
		J_temp = np.matrix([[J_data[x, 0], J_data[x, 1]], [J_data[x, 2], J_data[x, 3]]])
	
	# Calculate the expected movement given J
		movement = np.matmul(J_temp, X)
	
	# Calculate the model error
		model_error = update_gain*(dY - movement)
	
	# Jacobian numerator
		J_num = np.matmul(model_error, np.transpose(X))
	
	# Jacobian denominator
		J_den = np.matmul(np.transpose(X), X)
		# print(J_den)
	
	# if J_den < 1:
	# 	J_den = 1
		
	# Calculate the new Jacobian
		J_calc = J_num/J_den
		# J_calc = J_num
		J = J_temp + J_calc
		
		J_sum = J + J_sum
		
	J = J_sum / window_size
	
	# J_k = a J_k' + (1 - a) J_k-1
	# a = min(1, ||dY||/Ndi)
	
	# Ndi = 1
	
	# dY_n = np.linalg.norm(dY)
	
	# minimum = dY_n / Ndi
	
	# a = min(1, minimum)
	# a = 0.5
	
	# J = a*J + (1-a)*J_temp
	
	if window_size > 1:
		
		for x in range(window_size-1):
		
			x_sub = window_size - x - 1
			x_sub_1 = x_sub - 2
		
			J_data[x_sub, :] = J_data[x_sub_1, :]
			q_data[x_sub, :] = q_data[x_sub_1, :]
			s_data[x_sub, :] = s_data[x_sub_1, :]
	
	x_prev = x_current
	y_prev = y_current
	q1_prev = q1_current
	q2_prev = q2_current
	
	J_data[0, 0] = J[0, 0]
	J_data[0, 1] = J[0, 1]
	J_data[0, 2] = J[1, 0]
	J_data[0, 3] = J[1, 1]
	
	q_data[0, 0] = q1_current
	q_data[0, 1] = q2_current
	
	s_data[0, 0] = x_current
	s_data[0, 1] = y_current
	
	return J, J_data, q_data, s_data
	
def dataWriter(J, csv_writer):
	# Get time and convert to seconds
	now = rospy.get_rostime()
	seconds = now.to_sec()
		
	j00_act = a1*np.cos(q1)+a2*np.cos(q1+q2)
	j01_act = a2*np.cos(q1+q2)
	j10_act = a1*np.sin(q1)+a2*np.sin(q1+q2)
	j11_act = a2*np.sin(q1+q2)
	
	J_act = np.matrix([[j00_act, j01_act], [j10_act, j11_act]])
	
	j_calc = J - J_act
	
	j_err = np.linalg.norm(j_calc)
	
	x_exp = a1*np.sin(q1)+a2*np.sin(q1+q2)
	
	y_exp = -a1*np.cos(q1)-a2*np.cos(q1+q2)
		
	csv_writer.writerow([q1, q2, seconds, Xc, Yc, Xd, Yd, J[0,0], J[0,1], J[1,0], J[1,1], j00_act, j01_act, j10_act, j11_act, j_err, j_calc[0,0], j_calc[0,1], j_calc[1,0], j_calc[1,1], x_exp, y_exp])
	


def main():
	# Subscriber to get current and desired position values
	pos_sub = rospy.Subscriber('/pos', Float64MultiArray, pos_callback)
	
	# Subscriber to get current joint state values
	joint_sub = rospy.Subscriber('/vsbot/joint_states', JointState, joint_callback)
	
	# Open file to write data to
	f1 = open('adaptive_results.csv', 'w')
	# f2 = open('delta_data.csv', 'w')
	
	csv_writer1 = csv.writer(f1)
	# csv_writer2 = csv.writer(f2)
		
	# Write the titles of each column
	csv_writer1.writerow(['Q1', 'Q2', 't', 'X', 'Y', 'X_des', 'Y_des', 'j00', 'j01', 'j10', 'j11', 'j00_act', 'j01_act', 'j10_act', 'j11_act', 'j_err', 'j_err_00', 'j_err_01', 'j_err_10', 'j_err_11', 'X_exp', 'Y_exp'])
	# csv_writer2.writerow(['delta Q1', 'delta Q2', 'delta Sx', 'delta Sy', 'Q1 NN','Q2 NN', 'Sx NN', 'Sy NN'])
	
	rate = rospy.Rate(refresh_rate)
	
	# Calculate the initial Jacobian
	J = initialJacobian()
	
	# Initialize data matrices
	J_data = np.zeros((window_size, 4))
	q_data = np.zeros((window_size, 2))
	s_data = np.zeros((window_size, 2))
	
	# Store visual and motor data for future calculations
	x_prev = Xc
	y_prev = Yc
	q1_prev = q1
	q2_prev = q2
	
	# Store initial information
	J_data[0, 0] = J[0, 0]
	J_data[0, 1] = J[0, 1]
	J_data[0, 2] = J[1, 0]
	J_data[0, 3] = J[1, 1]
	
	q_data[0, 0] = q1_prev
	q_data[0, 1] = q2_prev
	
	s_data[0, 0] = x_prev
	s_data[0, 1] = y_prev
	
	while not rospy.is_shutdown():
		
		# Move the Robot
		controller(J)
		
		dist = np.sqrt((Xc-Xd)**2 + (Yc-Yd)**2)
		
		if dist > update_range:
		
			# Update the Jacobian and the previous data values
			J, J_data, q_data, s_data = updateJacobian(J, J_data, q_data, s_data)
		
		# Log the data
		dataWriter(J, csv_writer1)
		
		rate.sleep()
	
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('bot_controller', anonymous=False)

	main()
