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

# Amount of data to be stored in sk and qk matrices
data_size = 100

# Constant to make MAD consistent with normal distribution
B = 1.4826

# Proportional gain constant
k = 0.1

pass_flag = 0

# Declare publishers to be used
joint1_pub = rospy.Publisher('/vsbot/joint1_velocity_controller/command', Float64, queue_size=1)
joint2_pub = rospy.Publisher('/vsbot/joint2_velocity_controller/command', Float64, queue_size=1)

def pos_callback(data):
	# Global variables to be passed between functions
	# Data is in pixel values
	
	# Xc, Yc are current coordinates
	global Xc
	Xc = data.data[0]
	global Yc
	Yc = data.data[1]
	
	# Xd, Yd are desired visual data values
	global Xd
	Xd = data.data[2]
	global Yd
	Yd = data.data[3]
	
	# Q1d, Q2d are desired motor data values
	# global q1d
	# q1d = data.data[4]
	# global q2d
	# q2d = data.data[5]
	
def joint_callback(data):
	# q1, q2 are current joint position values
	global q1
	q1 = data.position[0]
	global q2
	q2 = data.position[1]
	
def initialize_offline():
	
	# Define data arrays to store incoming visual-motor data
	q_raw = np.array([[0, 0]])
	s_raw = np.array([[0, 0]])
	
	# Open the visual-motor data file
	with open('visual_motor_data.csv', newline='') as f:
		csv_reader = csv.DictReader(f)
		
		# Store data in the defined data arrays
		for row in csv_reader:
			
			# New Q1 and Q2 motor data to add
			q_data = np.array([[float(row['Q1']), float(row['Q2'])]])
			
			# Motor (Q) data from file
			q_raw = np.append(q_raw, q_data, 0)
			
			# New X and Y visual data to add
			s_data = np.array([[float(row['X']), float(row['Y'])]])
			
			# Visual (S) data from file
			s_raw = np.append(s_raw, s_data, 0)
	
	i_counter = 0	
		
	# Clock algorithm to parse through data and maximize q1 and q2 to 2*pi.
	# If value is larger than 2*pi it reduces number to below 2*pi.
	# This is necessary for accurate weighting function and may occur due to revolute joints spinning around multiple times during data collection.
	while i_counter < 100:
		for x in range(q_raw.shape[0]):
			if q_raw[x, 0] >= 2*np.pi:
				q_raw[x, 0] = q_raw[x, 0] - 2*np.pi
			if q_raw[x, 1] >= 2*np.pi:
				q_raw[x, 1] = q_raw[x, 1] - 2*np.pi
		
		i_counter = i_counter + 1
		
	# Jacobian matrix calculations
	j00 = a1*np.cos(q1)+a2*np.cos(q1+q2)
	j01 = a2*np.cos(q1+q2)
	j10 = a1*np.sin(q1)+a2*np.sin(q1+q2)
	j11 = a2*np.sin(q1+q2)
	
	# Formation of the Jacobian
	J = np.matrix([[j00, j01], [j10, j11]])
			
	return q_raw, s_raw, J
	
def initialize_online(csv_writer):
	# Define data arrays to store incoming visual-motor data
	q_raw = np.array([[0, 0]])
	s_raw = np.array([[0, 0]])
	
	# Get time and convert to seconds
	now = rospy.get_rostime()
	seconds = now.to_sec()
	
	rate = rospy.Rate(10)
	
	# Collect data for the first X seconds
	while seconds < 10:
		
		# Get time and convert to seconds
		now = rospy.get_rostime()
		seconds = now.to_sec()
	
		# New Q1 and Q2 motor data to add
		q_data = np.array([[q2, q1]])
			
		# Motor (Q) data from file
		q_raw = np.append(q_raw, q_data, 0)
			
		# New X and Y visual data to add
		s_data = np.array([[Xc, Yc]])
			
		# Visual (S) data from file
		s_raw = np.append(s_raw, s_data, 0)
	
		# Jacobian matrix calculations, ground truth calculations
		j00 = a1*np.sin(q1)+a2*np.sin(q1+q2)
		j01 = a2*np.sin(q1+q2)
		j10 = a1*np.cos(q1)+a2*np.cos(q1+q2)
		j11 = a2*np.cos(q1+q2)
	
		# Formation of the Jacobian
		J = np.matrix([[j10, j11], [j00, j01]])
	
		q1_dot = 0.25*np.sin(seconds)
		q2_dot = 0.25*np.cos(seconds)
		
		joint1_pub.publish(q1_dot)
		joint2_pub.publish(q2_dot)
		
		# Write data to file
		csv_writer.writerow([q1, q2, seconds, Xc, Yc, j00, j01, j10, j11])
		
		rate.sleep()
	
	q1_dot = 0
	q2_dot = 0
	
	joint1_pub.publish(q1_dot)
	joint2_pub.publish(q2_dot)
	
	return q_raw, s_raw, J
	
def neighbors(q_raw, s_raw):

	# Define empty arrays for the visual data vector magnitudes
	s_mag = np.zeros((s_raw.shape[0], 1))
	
	# Calculate a vector for the query location
	# s_e_mag = np.sqrt(Xc**2+Yc**2)
	
	# Calculate the distance by subtracting the query location vector from the visual data vector
	for x in range(s_raw.shape[0]):
		# s_mag[x] = np.absolute(np.sqrt(s_raw[x, 0]*s_raw[x, 0]+s_raw[x, 1]*s_raw[x, 1]) - s_e_mag)
		s_x_e = Xc - s_raw[x, 0]
		s_y_e = Yc - s_raw[x, 1]
		s_mag[x] = np.absolute(np.sqrt(s_x_e**2+s_y_e**2))
	
	# Transpose the error array
	s_mag = np.transpose(s_mag)
	
	# Organize the arrays with the data_size least amounts first
	# np.argpartition organizes by the location in the array, not the value
	# Example:	s_mag = [5 8 2 1 3]
	# 		s_idx = np.argpartition(s_mag, s_mag.size) = [3 2 4 0 1]
	s_idx = np.argpartition(s_mag, data_size)
	
	# This makes the returned index searchable
	s_idx_i = s_idx[0]
	
	# Create empty matrices for the visual-motor data
	q_nn = np.zeros((data_size, 2))
	s_nn = np.zeros((data_size, 2))
	
	# Find the data_size nearest neighbors using the partition location pointers and load them into the matrices
	for x in range(data_size):
	
		q_nn[x, :] = q_raw[s_idx_i[x], :]
		
		s_nn[x, :] = s_raw[s_idx_i[x], :]
	
	return q_nn, s_nn


def scale(qk, sk):

	dsk = np.zeros((data_size, 2))
	
	sc = np.array([Xc, Yc])
	
	# Delta sk or the distance to the query point from each visual memory point
	for x in range(data_size):
		dsk[x, :] = sc - sk[x, :]

	# Define an empty array for the xk data
	xk = np.zeros((data_size, 1))
	
	# Convert visual data into a scalar format
	for x in range(data_size):
		xk[x] = np.sqrt(dsk[x, 0]**2 + dsk[x, 1]**2)
	
	# Find median of visual data
	xj = np.median(xk)
	
	# Define the array for the below calculation
	x_calc = np.zeros((data_size, 1))
	
	# Calculate |x_i - med(x_j)|
	# x_i is the visual data
	# med(x_j) is the median value xj found above
	for x in range(data_size):
		x_calc[x] = np.absolute(xk[x] - xj)
	
	# Calculate sigma
	sigma = B*np.median(x_calc)
	
	return sigma, xk


def weight(sigma, xk):

	# Define empty w_k array for below calculations
	w_k = np.zeros((data_size, 1))

	# Define empty weight matrix to be filled
	W = np.zeros((data_size, data_size))

	# Initial weight matrix Wo is binary, if the value falls outside of 2.5 sigma the value is 0, otherwise it is 1
	for x in range(data_size):
		if np.absolute(xk[x]) <= 2.5*sigma:
			w_k[x] = 1
		else:
			w_k[x] = 0
	
	# Populate the weight matrix
	for x in range(data_size):
		W[x, x] = w_k[x]
	
	return W
	
def jacobianEstIRLS(qk, sk, sigma, Wo, J_prev, csv_writer, err_writer):

	# Define empty dS and dQ
	dS = np.zeros((data_size, 2))
	dQ = np.zeros((data_size, 2))
	
	# Define empty J
	J = np.zeros((2,2))
	# J[0, 0] = 1
	# J[1, 1] = 1
	
	# Initial weight is binary
	W = Wo
	
	# Determine delta values (sc, qc) - (sk, qk)
	# (sc, qc) are query (current) visual-motor data
	# (sk, qk) are memory (1 to K) visual-motor data
	for x in range(data_size):
		dQ[x, 0] = q1 - qk[x, 0]
		dQ[x, 1] = q2 - qk[x, 1]
		
		dS[x, 0] = Xc - sk[x, 0]
		dS[x, 1] = Yc - sk[x, 1]
	
	error = np.zeros((data_size,1))	
	w = np.zeros((data_size,1))
	
	run = 0
	
	# While loop ||Ju(t) - Ju(t-1)|| > e
	while np.linalg.norm(J - J_prev) > 1:
	# while run < 2:
		
		WdQ = np.matmul(W, dQ)
		WdS = np.matmul(W, dS)
		
		# Singular Value Decoposition of W dQ
		U, S, V = np.linalg.svd(WdQ, full_matrices=True)
		Vt = np.transpose(V)
		S_mat = np.zeros((data_size, 2))
		
		for x in range(2):
		 	S_mat[x, x] = S[x]
		
		USV = np.matmul(U, np.matmul(S_mat, Vt))
		
		# USV = np.matmul(W, dQ)
		
		USVt = np.transpose(USV)
		
		J_prev = J
		
		# Ju(t) = ((UEV^T)^TWdS)^T	
		J = np.transpose(np.matmul(USVt, WdS))
		Jt = np.transpose(J)
		
		# J dQ = dQ J^T
		
		j00 = J[0, 0]
		j01 = J[0, 1]
		j10 = J[1, 0]
		j11 = J[1, 1]
		
		# Get time and convert to seconds
		now = rospy.get_rostime()
		seconds = now.to_sec()
		
		# Jacobian matrix calculations, ground truth
		j00_act = a1*np.cos(q1)+a2*np.cos(q1+q2)
		j01_act = a2*np.cos(q1+q2)
		j10_act = a1*np.sin(q1)+a2*np.sin(q1+q2)
		j11_act = a2*np.sin(q1+q2)
	
		# Formation of the Jacobian
		J_act = np.matrix([[j00_act, j01_act], [j10_act, j11_act]])
		
		J_err = np.linalg.norm(J - J_act)
		
		eps = np.linalg.norm(J - J_prev)
			
		# Determine new error values [e1...eK]^T = ||W dQ Ju(t) - W dS||
		error_calc = np.matmul(WdQ, Jt) - WdS
		
		# error_calc = [K, 2]
		
		for x in range(data_size):
			error_x = error_calc[x,0]**2
			error_y = error_calc[x,1]**2
			
			error[x] = np.sqrt(error_x + error_y)
		
		s2 = sigma**2
		
		# e_zero = 2*s2/s2**2
		
		# Determine weight values w(ek) = 1/e d/de (e^2 / (e^2 + o^2))
		for x in range(data_size):
		
			e2 = error[x]**2
			
			w[x] = 2*s2/(e2+s2)**2
			
			# w[x] = w[x]/e_zero
		
		# Set new weight matrix, global variable
		for x in range(data_size):
			W[x, x] = w[x]
			# W[x, x] = 1
			
		error_norm = np.linalg.norm(error)
			
		# Write data to file
		csv_writer.writerow([q1, q2, seconds, Xc, Yc, j00, j01, j10, j11, J_err, error_norm, w[0], eps])
		
		run = run+1
			
		global pass_flag
			
		if pass_flag < 1:
			for x in range(data_size):
				err_writer.writerow([dQ[x, 0], dQ[x, 1], dS[x, 0], dS[x, 1], qk[x, 0], qk[x, 1], sk[x, 0], sk[x, 1]])	
		
			pass_flag = 1
		# err_writer.writerow([error[0], w[0]])
			
		print(np.linalg.norm(J - J_prev))
	
	# Return Ju(t)
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
	
	# max_vel = 0.5
	
	# if u[0] > max_vel:
	# 	u[0] = max_vel
		
	# if u[0] < -max_vel:
	# 	u[0] = -max_vel
	
	# if u[1] > max_vel:
	# 	u[1] = max_vel
		
	# if u[1] < -max_vel:
	# 	u[1] = -max_vel
		
	# Load velocity values to be published
	q1_dot = u[0]
	q2_dot = u[1]
		
	# Publish joint velocity values
	joint1_pub.publish(q1_dot)
	joint2_pub.publish(q2_dot)
	
def updateMemory(q_raw, s_raw):
	
	global q1
	global q2
	
	i_counter = 0	
	
	while i_counter < 100:
		if q1 >= 2*np.pi:
			q1 = q1 - 2*np.pi
		if q2 >= 2*np.pi:
			q2 = q2 - 2*np.pi
		
		i_counter = i_counter + 1
		
	q_data = np.array([[q1, q2]])
	s_data = np.array([[Xc, Yc]])
	
	q_raw = np.append(q_raw, q_data, 0)
	s_raw = np.append(s_raw, s_data, 0)
	
	return q_raw, s_raw


def main():
	# Subscriber to get current and desired position values
	pos_sub = rospy.Subscriber('/pos', Float64MultiArray, pos_callback)
	
	# Subscriber to get current joint state values
	joint_sub = rospy.Subscriber('/vsbot/joint_states', JointState, joint_callback)
	
	# Open file to write data to
	f1 = open('direct_velocity_sinesoidal.csv', 'w')
	f2 = open('delta_data.csv', 'w')
	
	csv_writer1 = csv.writer(f1)
	csv_writer2 = csv.writer(f2)
		
	# Write the titles of each column
	csv_writer1.writerow(['Q1', 'Q2', 't', 'X', 'Y', 'j00', 'j01', 'j10', 'j11', 'J_err', 'Error', 'Weight', 'Epsilon'])
	csv_writer2.writerow(['delta Q1', 'delta Q2', 'delta Sx', 'delta Sy', 'Q1 NN','Q2 NN', 'Sx NN', 'Sy NN'])
	
	# A1:  Initialize Visual-Motor Memory
	# Utilizes offline data
	# q_raw, s_raw, J = initialize_offline()
	
	# Does not utilize offline data - performs data collection phase
	q_raw, s_raw, J = initialize_online(csv_writer1)
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# A2:  Determine Neighbors
		q_nn, s_nn = neighbors(q_raw, s_raw)
	
		# A3:  Estimate Initial Scale
		sigma, xk = scale(q_nn, s_nn)
	
		# A4:  Find Initial Weights
		W = weight(sigma, xk)
					
		# A5:  Estimate the Jacobian
		J = jacobianEstIRLS(q_nn, s_nn, sigma, W, J, csv_writer1, csv_writer2)
	
		# A6:  Update Control Signal
		controller(J)
	
		# A7:  Update Memory
		# q_raw, s_raw = updateMemory(q_raw, s_raw)
		# print("Done")
		
		rate.sleep()
	
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('bot_controller', anonymous=False)

	main()
