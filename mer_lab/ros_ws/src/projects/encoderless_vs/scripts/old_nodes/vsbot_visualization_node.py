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
cv_img = None

# the spline on binary image converted to ROS image
ros_img = None

# this callback function is subscribing to the binary image from the image_sub node
def callback_vsbot_img(image):
		global cv_img, bridge
		try:
			# converting the subscribed binary image to CV
			cv_img = bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)


# def callback_cp(cp):
	
# 	print(cp.data)

def callback_kp(kp):
	print(kp.data)
		
# This call back functions subscribes to the 
# evaluated points as computed in the control_node
def callback_vsbot_points(msg):
			global bridge, ros_img

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
					cv2.circle(cv_img,(x, y),2,(0,0,0),-1)						 
			
			#converting the cv image of spline points to ros 
			try:
				if cv_img is not None:
					ros_img = bridge.cv2_to_imgmsg(cv_img, "bgr8")
			except CvBridgeError as e:
				print(e)
			




def main(args):
	rospy.init_node('spline_visualization_node', anonymous=True)

	# subscriber for the evaluated points
	eval_pts_sub = rospy.Subscriber("vsbot/evaluated_points", Float64MultiArray, callback_vsbot_points, queue_size=1)

	# subscriber for the image on which the spline will be placed
	image_sub = rospy.Subscriber("vsbot/camera1/image_raw", Image, callback_vsbot_img, queue_size=1)

	# cp_sub = rospy.Subscriber("vsbot/control_points", Float64MultiArray, callback_cp, queue_size=1)

	knot_sub = rospy.Subscriber("vsbot/knot_points", Float64MultiArray, callback_kp, queue_size=1)

	# skeleton sub
	# skeleton_sub = rospy.Subscriber("vsbot/skeleton_points", Image, skeletonCallback)

	# publisher for the final image for spline on binary
	img_spline_pub = rospy.Publisher("/vsbot/spline_img", Image,queue_size=1)
	
	# setting up a rate to publish the spline
	r = rospy.get_param('vsbot/estimation/rate')
	rate = rospy.Rate(r)

	while not rospy.is_shutdown():
		# spline image is published @ 30Hz
		if ros_img !=None:
			img_spline_pub.publish(ros_img)
		rate.sleep()
		# rospy.spin()

		# ROS will sleep for the remaining of 1/30 secs
		# once a message is published, meanwhile the subscriber will get refreshed
		# r.sleep()
	rospy.spin()
		


if __name__ == '__main__':
		main(sys.argv)



		
