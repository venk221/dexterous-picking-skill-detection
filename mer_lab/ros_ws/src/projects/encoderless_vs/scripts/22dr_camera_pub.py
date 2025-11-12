#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image, JointState # Image is the message type
from std_msgs.msg import Float64MultiArray, Int64MultiArray, Float64, Bool
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import time
import random
import csv
from skimage.morphology import skeletonize
from skimage import data, morphology, filters
import matplotlib.pyplot as plt
from skimage.util import invert
from scipy.interpolate import splprep, splev
import scipy.special as sc

bridge = CvBridge()

Xd_px = 175
Yd_px = 75

Xc_px = 175
Yc_px = 75

q1 = 0
q2 = 0

q1d = -0.335631987854796
q2d = -4.47042354331878

# Number of segments for clothoid
# Do not exceed 30 data points
# Equations used begin to break down due to the small difference between the points and the rounding necessary for display on the image
num_points = 10

# Define data arrays to store incoming visual-motor data
q_raw = np.array([[0, 0]])
s_raw = np.array([[0, 0]])

# Define publishers for the base image, spline image, clothoid image, and skeleton image
img_pub = rospy.Publisher('/camera', Image, queue_size=1)
spline_pub = rospy.Publisher('/spline', Image, queue_size=1)
clothoid_pub = rospy.Publisher('/clothoid', Image, queue_size=1)
skel_pub = rospy.Publisher('/skeleton', Image, queue_size=1)

# Define publisher for the position of the visual feature(s)
pos_pub = rospy.Publisher('/pos', Float64MultiArray, queue_size=10)

# Position is posted as a Float64 Multi Array
position = Float64MultiArray()

# Set color bounds
orange_lower_bounds = np.array([5,50,50], np.uint8)
orange_upper_bounds = np.array([50,255,255], np.uint8)

blue_lower_bounds = np.array([100,150,0], np.uint8)
blue_upper_bounds = np.array([140,255,255], np.uint8)

splinerow1 = 0
splinerow2 = 0
splinerow3 = 0
splinerow4 = 0
splinerow5 = 0
	
splinecolumn1 = 0
splinecolumn2 = 0
splinecolumn3 = 0
splinecolumn4 = 0
splinecolumn5 = 0

# Empty arrays for clothoid points
points_x = np.zeros(num_points, np.float64)
points_y = np.zeros(num_points, np.float64)
			
def camera_callback(data):
	global cv_image
	global cv_copy
	
	global Xd_px
	global Yd_px
	
	global Xc_px
	global Yc_px
	
	global Xd
	global Yd
	
	global q1d
	global q2d
	
	global q_raw
	global s_raw
	
	global criteria
	
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	
	cv_copy = cv_image
	cv_copy_2 = cv_image
	
	cv_hsv = cv2.cvtColor(cv_copy, cv2.COLOR_BGR2HSV)
		
	# Convert image to grayscale
	cv_image = cv2.cvtColor(cv_copy, cv2.COLOR_BGR2GRAY)
	
	# Define image masks for orange and blue objects
	cv_image_orange = cv2.inRange(cv_hsv, orange_lower_bounds, orange_upper_bounds)
	cv_image_blue = cv2.inRange(cv_hsv, blue_lower_bounds, blue_upper_bounds)
	
	# Combine object masks into singular mask
	cv_image_2 = cv_image_orange + cv_image_blue
	
	# Create additional blank images
	cv_image_3 = np.zeros((300,300), np.uint8)
	cv_image_4 = np.zeros((300,300), np.uint8)
		
	# Binarize the image
	ret, cv_image = cv2.threshold(cv_image, 50, 255, cv2.THRESH_BINARY_INV)
	ret, cv_image_2 = cv2.threshold(cv_image_2, 177, 255, cv2.THRESH_BINARY)
	
	##############
	##############
	# Main Image #
	##############
	##############
	
	# Calculate moments matrix
	M = cv2.moments(cv_image_blue)
		
	# Calculate the end-effector position	
	if M["m00"] != 0:
		Xc_px = int(M["m10"] / M["m00"])
		Yc_px = int(M["m01"] / M["m00"])
	else:
		# set values as what you need in the situation
		Xc_px, Yc_px = 0, 0
		
	# Add locator for end-effector
	cv2.circle(cv_copy, (Xc_px, Yc_px), 2, (255, 255, 255), -1)
	cv2.putText(cv_copy, "centroid", (Xc_px, Yc_px + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)
		
	# Add locator for goal
	cv2.circle(cv_copy, (Xd_px, Yd_px), 2, (0, 255, 255), -1)
	cv2.putText(cv_copy, "desired", (Xd_px, Yd_px + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 255), 1)
	
	##############
	##############
	#  Skeleton  #
	##############
	##############
	
	# Define a kernel to use in opening operation
	kernel = np.ones((5,5),np.uint8)
	
	# Open image to remove any holes or gaps
	cv_image_2 = cv2.morphologyEx(cv_image_2, cv2.MORPH_OPEN, kernel)
	
	# Binarize the image to use in skeletonization
	binary = cv_image_2 > filters.threshold_otsu(cv_image_2)
	np.unique(binary)
	
	# Skeletonize the image
	cv_image_skeleton = skeletonize(binary)
	
	# Increase the intensity of the skeleton
	cv_image_skeleton = cv_image_skeleton.astype(np.uint8) * 255
	
	##############
	##############
	#  B Spline  #
	##############
	##############
	
	row_look, column_look = np.nonzero(cv_image_skeleton)
	
	x_look = column_look
	y_look = 300 - row_look
	
	cv_image_3 = b_spline_calc(x_look, y_look, cv_image_3)
	
	# row_look_2, column_look_2 = np.nonzero(cv_image_3)
	
	# for x in range(row_look_2.size):
	# 	cv2.circle(cv_copy_2, (column_look_2[x], row_look_2[x]), 2, (0, 255, 255), -1)
	
	##############
	##############
	#  Clothoid  #
	##############
	##############
	
	points_x, points_y = clothoid_calc(Xc_px, Yc_px, q1, q2)
			
	for x in range(num_points):
		cv_image_4[(300 - points_x[x]), points_y[x]] = 255
		# cv_image_4[points_x[x], points_y[x]] = 255
		
	print_points_y_1, print_points_x_1 = np.nonzero(cv_image_3)
	print_points_y_2, print_points_x_2 = np.nonzero(cv_image_4)
	
	# print('b-spline:', print_points_y_1, print_points_x_1)
	# print('clothoid:', print_points_y_2, print_points_x_2)
	
	##############
	##############
	# Publishing #
	##############
	##############
		
	# Convert the image to be published
	msg = bridge.cv2_to_imgmsg(cv_copy, "bgr8")
	msg2 = bridge.cv2_to_imgmsg(cv_image_3, "mono8")
	msg3 = bridge.cv2_to_imgmsg(cv_image_4, "mono8")
	msg4 = bridge.cv2_to_imgmsg(cv_image_skeleton, "mono8")

	img_pub.publish(msg)
	spline_pub.publish(msg2)
	clothoid_pub.publish(msg3)
	skel_pub.publish(msg4)
	
	# Convert from pixel to distance
	Xc_dist = (Xc_px - 150)/100
	Yc_dist = (Yc_px - 150)/100
	
	# Rotate from camera coordinates to world coordinates
	Xc = Xc_dist
	Yc = Yc_dist
	
	# Convert from pixel to distance
	Xd_dist = (Xd_px - 150)/100
	Yd_dist = (Yd_px - 150)/100
	
	# Rotate from camera coordinates to world coordinates
	Xd = Xd_dist
	Yd = Yd_dist
	
	# Xc = Xc_px
	# Yc = Yc_px
	
	# Xd = Xd_px
	# Yd = Yd_px
	
	# Define position data to publish
	global position
	position.data = [Xc, Yc, Xd, Yd, q1d, q2d]
	
	# now = rospy.get_rostime()
	# seconds = now.to_sec()
	
def b_spline_calc(x_look, y_look, cv_image_3):

	# B-Spline functions
	tck, u = splprep([x_look, y_look], k = 3)
	new_points = splev(u, tck)
	
	# Round points to nearest whole integer
	new_points_x = np.rint(new_points[0])
	new_points_y = np.rint(new_points[1])
	
	# Change type of numbers to unsigned integers
	new_points_x = new_points_x.astype(np.uint8)
	new_points_y = new_points_y.astype(np.uint8)
	
	# Extract data points from splprep function
	temp_tck = tck[1]
	
	# Extract control points from data holder
	control_points_x = temp_tck[0]
	control_points_y = temp_tck[1]
	
	# Round control points to nearest whole integers
	control_points_x = np.rint(control_points_x)
	control_points_y = np.rint(control_points_y)
	
	# Change type of numbers to unsigned integers
	control_points_x = control_points_x.astype(np.uint8)
	control_points_y = control_points_y.astype(np.uint8)
	
	# Define number of metric to extract a set number of control points from the control points data
	# Metric 0 is the first data point
	# Metric 1 is the data point half way between the first and the median
	# Metric 2 is the median data point
	# Metric 3 is the data point half way between the median and the last
	# Metric 4 is the last data point
	
	metric0 = 0
	
	metric1 = np.rint(control_points_x.size/4)
	metric1 = metric1.astype(np.int8)
	
	metric2 = np.rint(control_points_x.size/2)
	metric2 = metric2.astype(np.int8)
	
	metric3 = np.rint(3*control_points_x.size/4)
	metric3 = metric3.astype(np.int8)
	
	metric4 = np.rint(control_points_x.size) - 1
	metric4 = metric3.astype(np.int8)
	
	global splinerow1
	global splinerow2
	global splinerow3
	global splinerow4
	global splinerow5
	
	global splinecolumn1
	global splinecolumn2
	global splinecolumn3
	global splinecolumn4
	global splinecolumn5
	
	# Use the metric to call the data of the control points
	# Convert to image coordinate values
	splinerow1 = (300-control_points_y[metric0])
	splinerow2 = (300-control_points_y[metric1])
	splinerow3 = (300-control_points_y[metric2])
	splinerow4 = (300-control_points_y[metric3])
	splinerow5 = (300-control_points_y[metric4])
	
	splinecolumn1 = control_points_x[metric0]
	splinecolumn2 = control_points_x[metric1]
	splinecolumn3 = control_points_x[metric2]
	splinecolumn4 = control_points_x[metric3]
	splinecolumn5 = control_points_x[metric4]
	
	# Display control points onto the image
	cv_image_3[splinerow1, splinecolumn1] = 255
	cv_image_3[splinerow2, splinecolumn2] = 255
	cv_image_3[splinerow3, splinecolumn3] = 255
	cv_image_3[splinerow4, splinecolumn4] = 255
	cv_image_3[splinerow5, splinecolumn5] = 255
	
	return cv_image_3
	

def clothoid_calc(Xc_px, Yc_px, q1, q2):
	
	# Distance to be covered by each segment
	deltax = (Xc_px - 150) / num_points
	deltay = (Yc_px - 150) / num_points
	
	# Angle values and total angle
	theta1 = q1
	theta2 = q2
	thetat = theta1 + theta2
	
	# Angle to be changed by each segment
	deltatheta = (thetat - theta1) / num_points
	
	# Total length between initial point (base) and final point (visual feature)	
	length = np.sqrt((Xc_px - 150)**2 + (Yc_px - 150)**2)
	
	# Total length to be covered by each segment
	deltalength = length / num_points
	
	global points_x
	global points_y
	
	for x in range(num_points):
		if x < 1:
			points_x[x] = 150
			points_y[x] = 150
		else:
			points_x[x] = points_x[x - 1] + deltalength*np.cos(theta1 + deltatheta*(x-1))
			points_y[x] = points_y[x - 1] + deltalength*np.sin(theta1 + deltatheta*(x-1))
			
	# Round numbers to nearest whole integer
	points_x = np.rint(points_x)
	points_y = np.rint(points_y)
	
	# Change format of numbers to unsigned integers
	points_x = points_x.astype(np.uint8)
	points_y = points_y.astype(np.uint8)
	
	return points_x, points_y

	
def joint_callback(data):
	# q1, q2 are current joint position values
	global q1
	q1 = data.position[0] # % 2*np.pi
	global q2
	q2 = data.position[1] # % 2*np.pi
	
	# Modulo operator should be used to ensure that values do not go over 2*Pi
	# ROS was providing incorrect/varying joint values, though, so this was disabled
		
def des_callback(data):
	global Xd_px
	global Yd_px
	
	Xd_px = data.data[0]
	Yd_px = data.data[1]
		
def publisher():
	global position
	
	global Xd_px
	global Yd_px
	
	# Publish data
	pos_pub.publish(position)
		

def main():
	rate = rospy.Rate(10)
	
	# Subscriber to get camera image
	camera_sub = rospy.Subscriber('/vsbot/camera1/image_raw', Image, camera_callback)
	
	# Subscriber to get desired position
	# Manually publish desired position if you want to update the desired Xd and Yd values	
	des_sub = rospy.Subscriber('/des_pos', Int64MultiArray, des_callback)
	
	# Subscriber to get current joint state values
	joint_sub = rospy.Subscriber('/vsbot/joint_states', JointState, joint_callback)
	
	while not rospy.is_shutdown():
		# Publish data
		publisher()
	
		# Post data at controlled rate
		rate.sleep()
	
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('camera_read', anonymous=False)
	
	main()
