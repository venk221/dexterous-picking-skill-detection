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

# Declaring cvBridge for cv to ros conversion and vice versa
bridge = CvBridge()
cv_image_capture = None # colored image
cp = None # control points
cp_flag = False

def talker():
    # Declare the publisher for joint position 
    pub1 = rospy.Publisher('/vsbot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/vsbot/joint2_position_controller/command', Float64, queue_size=10)    
    
    # Switch the controller from Velocity to joint
    # the sleep is a must as the controller needs to load completely before switching
    rospy.wait_for_service('/vsbot/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/vsbot/controller_manager/switch_controller', SwitchController)
        start_controllers = ['joint1_position_controller','joint2_position_controller']
        stop_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
        print(res)
    except rospy.ServiceException as e:
        print("Service Call Failed")
    
    # Applying the joint positions as defined in the config.yaml            
    q1_pos = rospy.get_param("vsbot/shape_control/joint1_goal")
    q2_pos = rospy.get_param("vsbot/shape_control/joint2_goal")

    # Publishing the goal position   
    pub1.publish(q1_pos)
    pub2.publish(q2_pos)
    
    # The node waits for 5 secs for the robot to reach 
    # the goal position then calls the compute_func() to print the control points
    rospy.sleep(5)
    compute_func()

def image_callback(ros_image):
        #capturing the colored image from camera
        global cv_image_capture
        cv_image_capture = bridge.imgmsg_to_cv2(ros_image, "bgr8")

# Performing the skeletoniztion to Spline fit 
# and extraction of Control Points in this function        
def compute_func():
        global binary_image, cp
        # taking the hue, stauration , value of the colored image
        hsvimg = cv2.cvtColor(cv_image_capture, cv2.COLOR_BGR2HSV)
        
        #determining the lower and upper bound of each colored part of the image
        orange_lower = np.array([10, 100, 20], np.uint8) 
        orange_upper = np.array([25, 255, 255], np.uint8) 
            
        white_lower = np.array([0, 0, 200], np.uint8)
        white_upper = np.array([145, 60, 255], np.uint8) 

        blue_lower = np.array([78,158,124])
        blue_upper = np.array([138,255,255])

        # binarizing each color 
        orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
        blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
        white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
        
        # final binary image in cv by combining each binarized part
        cv_binary = orange_mask + blue_mask + white_mask

        #converting the binarized cv_image to ros    
        binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")

        # determining the white portion of the binary image for skeletnization
        binary = np.where(cv_binary > 0, 1, cv_binary)  

        # performing the skeletonization
        skeleton = skeletonize(binary, method='lee')
        skeleton = np.where(skeleton == 1, 255, skeleton)

        # determinig pixels of just the skeleton
        pixels = np.argwhere(skeleton > 0)
        xinit = pixels[:, 1]
        yinit = pixels[:, 0]

        # ordering pixels using nearest neighbor search 
        init_point = (149, 149)
        dist_mat = []
        for i in range(len(xinit)):
            dist = np.sqrt((xinit[i]-init_point[0])**2 + (yinit[i]-init_point[1])**2)
            dist_mat = np.append(dist_mat, dist)
            
        index = np.argsort(dist_mat)
        xinit, yinit = xinit[index], yinit[index]
        point_jump = (len(xinit))/2
        x = np.array(xinit[0::round(point_jump)])
        y = np.array(yinit[0::round(point_jump)])
        if (x[-1] != xinit[-1]) or (y[-1] != yinit[-1]) :
            x = np.append(x, xinit[-1])
            y = np.append(y, yinit[-1])
            if ((x[-1] - x[-2] == 1) or (x[-1] - x[-2] == -1)) and ((y[-1] - y[-2] == 1) or (y[-1] - y[-2] == -1)):
                x = np.delete(x, -2)
                y = np.delete(y, -2)

        # fitting the spline
        tck, u_params = splprep([x, y], k =2, s = 1)

        #extracting the control points    
        cx = tck[1][0]
        cx = cx.tolist()
        cy = tck[1][1]
        cy = cy.tolist()
        cp = [cx[0], cx[1], cx[2], cy[0], cy[1], cy[2]]
        print("Control Points", cp)

def main():
    # Initialize the node
    rospy.init_node('goal_feature_node')
    name1 = 'joint1_position_controller'
    name2 = 'joint2_position_controller'
    rospy.wait_for_service('/vsbot/controller_manager/switch_controller')
    load_svc = rospy.ServiceProxy('/vsbot/controller_manager/load_controller', LoadController)
    resp1 = load_svc(name1)
    resp2 = load_svc(name2)

    print(resp1.ok)


    # Declare the subscriber to Camera topic
    image_sub = rospy.Subscriber("/vsbot/camera1/image_raw",Image, image_callback,  queue_size = 1)

    # Declare publisher for control points
    control_pts_pub = rospy.Publisher('goal_feature/control_points', Float64MultiArray, queue_size= 1)

    # Declaring msgs to publish
    control_points = Float64MultiArray()

    # Publish control points
    control_points.data = cp
    control_pts_pub.publish(control_points)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
    
