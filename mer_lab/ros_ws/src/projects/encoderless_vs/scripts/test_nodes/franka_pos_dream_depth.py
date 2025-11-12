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
    pub = rospy.Publisher('/position_joint_trajectory_controller/command', Float64, queue_size=10)
    
    # Switch the controller from Velocity to joint
    # the sleep is a must as the controller needs to load completely before switching
    rospy.sleep(1)
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        start_controllers = ['position_joint_trajectory_controller']
        stop_controllers = ['joint_group_velocity_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
    except rospy.ServiceException as e:
        print("Service Call Failed")
    
    # Defining the joint positions as defined in the config.yaml            
    q1_pos = rospy.get_param("vsbot/shape_control/joint1_goal")
    q2_pos = rospy.get_param("vsbot/shape_control/joint2_goal")

    # Publishing the goal position   
    pub1.publish(q1_pos)
    pub2.publish(q2_pos)
    
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

def main():
    # Initialize the node
    rospy.init_node('pos_control_dream_depth_ds_gen')
    
    # Declare the subscriber to Camera topic
    rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image, image_callback,  queue_size = 1)
    dep_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image, image_callback,  queue_size = 1)

    
    try:
        update_pose() # publishes joint positions to position controller
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__ == '__main__':
    main()
    
