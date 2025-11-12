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

def update_pose():
    global control_points_output
    # Declare the publisher for joint position 
    pub1 = rospy.Publisher('/3linkbot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/3linkbot/joint2_position_controller/command', Float64, queue_size=10)    
    pub3 = rospy.Publisher('/3linkbot/joint3_position_controller/command', Float64, queue_size=10)    
    
    # Switch the controller from Velocity to joint
    # the sleep is a must as the controller needs to load completely before switching
    rospy.sleep(1)
    rospy.wait_for_service('/3linkbot/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/3linkbot/controller_manager/switch_controller', SwitchController)
        start_controllers = ['joint1_position_controller','joint2_position_controller', 'joint3_position_controller']
        stop_controllers = ['joint1_velocity_controller','joint2_velocity_controller', 'joint3_velocity_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
    except rospy.ServiceException as e:
        print("Service Call Failed")
    
    # Defining the joint positions as defined in the config.yaml            
    q1_pos = rospy.get_param("3linkbot/shape_control/joint1_goal")
    q2_pos = rospy.get_param("3linkbot/shape_control/joint2_goal")
    q3_pos = rospy.get_param("3linkbot/shape_control/joint3_goal")

    # Publishing the goal position   
    pub1.publish(q1_pos)
    pub2.publish(q2_pos)
    pub3.publish(q3_pos)
    
    # The node waits for 6 secs for the robot to reach 
    # the goal position then calls the control_points_service 
    # to print the control points    
    rospy.sleep(6)
    # create a handle to call the service
    control_points_output = rospy.ServiceProxy('control_points_output', cp_goal)
    goal_srv_resp = control_points_output(1)
    # print('Control_points', goal_srv_resp.cp.data)
    # print('Coefficients', goal_srv_resp.coeff.data)

    ## Ouput a YAML file with these parameters
    yaml_file = open("features.yaml","w")
    s = ""
    s += "shape_controller:\n"
    s += "  goal_features: [" + (','.join(map(str,goal_srv_resp.cp.data))) +"]\n"
    # s += "  coefs: [" + (','.join(map(str,goal_srv_resp.coeff.data))) +"]"
    
    yaml_file.write(s)
    yaml_file.close()
    rospy.signal_shutdown("Wrote features to file")

def binary_image_service(msg):
    global bridge 
    # Convert ROS image to cv image
    # try:
    current_cv_image = bridge.imgmsg_to_cv2(current_ros_image, "bgr8")
    # except CvBridgeError as e:
      # print(e)
 
    # converting to hsv
    hsvimg = cv2.cvtColor(current_cv_image, cv2.COLOR_BGR2HSV)

    # Define image color bounds 
    orange_lower = np.array([10, 100, 20], np.uint8) 
    orange_upper = np.array([25, 255, 255], np.uint8) 
    white_lower = np.array([0, 0, 200], np.uint8)
    white_upper = np.array([145, 60, 255], np.uint8) 
    blue_lower = np.array([78,158,124])
    blue_upper = np.array([138,255,255])
	  
    # Binarizing individual colors & combining
    orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
    blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
    white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
	  	
    cv_binary = orange_mask + blue_mask + white_mask

    kernel = np.ones((5, 5), np.uint8)
    cv_binary = cv2.dilate(cv_binary, kernel, iterations=1)


    # Convert cv image to ROS image
    # try:
    binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
    # except CvBridgeError as e:
      # print(e)
    return bin_imgResponse(binary_image)

def image_callback(ros_image):
        #capturing the colored image from camera
        global current_ros_image
        current_ros_image = ros_image
        cv_image = bridge.imgmsg_to_cv2(current_ros_image, "bgr8")
        cv2.imwrite("goal2.tiff",cv_image)


def main():
    # Initialize the node
    rospy.init_node('goal_feature_node')
    
    # Declare the subscriber to Camera topic
    image_sub = rospy.Subscriber("/vsbot/camera1/image_raw",Image, image_callback,  queue_size = 1)

    # service declaration to receive the binary image
    bin_img_service = rospy.Service("binary_image_output", bin_img, binary_image_service )

    # wait for control points service to be up
    rospy.wait_for_service('control_points_output')   

    
    try:
        update_pose() # publishes joint positions to position controller
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__ == '__main__':
    main()
    
