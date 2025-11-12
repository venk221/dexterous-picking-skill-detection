#!/usr/bin/env python3
import rospy
import sys
import cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# Declaring cvbridge for ROS
bridge = CvBridge()

# cv image to visualize the spline points
cv_binary = None

# the spline on binary image converted to ROS image
ros_binary = None

# this callback function is subscribing to the binary image from the image_sub node
def callback_vsbot_binary(img_msg):
		global cv_binary, bridge
		try:
			# converting the subscribed binary image to CV
			cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)

def callback_control_pts(cp):

	

		
# This call back functions subscribes to the 
# evaluated points as computed in the control_node
def callback_vsbot_points(msg):
			global bridge, ros_binary

			# separating the x and y points from the evaluated points list
			mid_index = len(msg.data)//2

			print("Test Visualization Node", len(msg.data))
			x = msg.data[:mid_index]
			y = msg.data[mid_index:]

			# converting them to an array to be used in cv2.circle method
			points = np.c_[x, y]

			# creating the visualization for the Spline points and
			#  placing it on the existing binary image
			for i in range(len(points)):
					x = np.int(points[i][0])
					y = np.int(points[i][1]) 
					cv2.circle(cv_binary,(x, y),2,(0,0,0),-1)						 
			
			#converting the cv image of spline points to ros 
			try:
				if cv_binary is not None:
					ros_binary = bridge.cv2_to_imgmsg(cv_binary, "mono8")
			except CvBridgeError as e:
				print(e)
			
def skeletonCallback(msg):
	global cv_binary, ros_binary
	skel_img = bridge.imgmsg_to_cv2(msg,"mono8")
	cv_binary = cv2.addWeighted(cv_binary, 0, skel_img, 1, 1)

	if cv_binary is not None:
		ros_binary = bridge.cv2_to_imgmsg(cv_binary,"mono8")



def main(args):
	rospy.init_node('spline_visualization_node', anonymous=True)

	# subscriber for the evaluated points
	eval_pts_sub = rospy.Subscriber("vsbot/evaluated_points", Float64MultiArray, callback_vsbot_points, queue_size=1)

	# subscriber for the binary image on which the spline will be placed
	binary_sub = rospy.Subscriber("vsbot/camera1/image_raw", Image, callback_vsbot_binary, queue_size=1)

	# skeleton sub
	# skeleton_sub = rospy.Subscriber("vsbot/skeleton_points", Image, skeletonCallback)

	# publisher for the final image for spline on binary
	binary_spline_pub = rospy.Publisher("/vsbot/spline_on_binary", Image,queue_size=1)
	
	# setting up a rate to publish the spline
	r = rospy.get_param('vsbot/estimation/rate')
	rate = rospy.Rate(r)

	while not rospy.is_shutdown():
		# spline image is published @ 30Hz
		if ros_binary !=None:
			binary_spline_pub.publish(ros_binary)
		rate.sleep()
		# rospy.spin()

		# ROS will sleep for the remaining of 1/30 secs
		# once a message is published, meanwhile the subscriber will get refreshed
		# r.sleep()
	rospy.spin()
		


if __name__ == '__main__':
		main(sys.argv)



		
