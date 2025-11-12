#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
from encoderless_vs.srv import cp_goal, cp_goalResponse
from controller_manager_msgs.srv import SwitchController
from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser

bridge = CvBridge()
def image_callback(msg):
    global bridge
    home = expanduser("~")    
    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite("robot_img_6.tiff", cv_img)

    

def main():
    # initialize the node
    rospy.init_node('goal_img_write_node')        

    pub1 = rospy.Publisher('/vsbot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/vsbot/joint2_position_controller/command', Float64, queue_size=10)    
    
    # Switch the controller from Velocity to joint
    # the sleep is a must as the controller needs to load completely before switching
    rospy.sleep(1)
    rospy.wait_for_service('/vsbot/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/vsbot/controller_manager/switch_controller', SwitchController)
        start_controllers = ['joint1_position_controller','joint2_position_controller']
        stop_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
    except rospy.ServiceException as e:
        print("Service Call Failed")
    
    # Defining the joint positions as defined in the config.yaml            
    q1_pos =  1.1803  #rospy.get_param("vsbot/shape_control/joint1_goal")
    q2_pos =  -0.75      #rospy.get_param("vsbot/shape_control/joint2_goal")

    # 0.6, -0.7

    # Publishing the goal position   
    pub1.publish(q1_pos)
    pub2.publish(q2_pos)

    rospy.sleep(10)

    img_sub = rospy.Subscriber("/vsbot/camera1/image_raw", Image, image_callback)

    rospy.spin()
         

if __name__ == '__main__':
    main()
