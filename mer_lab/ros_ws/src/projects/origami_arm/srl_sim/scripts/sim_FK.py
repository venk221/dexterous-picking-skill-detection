#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from continuum_misc import continuum_fk_arc, cablelen_to_skp, skp_to_cablelength
from plot import generate_arc_point_list, plot_arc

bridge = CvBridge()
servo = False
cur_pose = []
cur_target = []
pose_pub = None
cur_cable_lengths = []

#* module prameters
min_cable_len = 80
max_cable_len = 200
module_d = 40
module_num = 2

#* module state
cable_pos = np.array([[min_cable_len]*3]*module_num) # 3 motors per module
skp_state = np.array([[0.0]*3]*module_num)
transMatrixList = []

#* base pose
T0 = np.array([ [-1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1]])

def getReference(msg):
    global cur_target, servo
    cur_target = msg.data
    servo = True

def getLengths(msg):
    global cur_cable_lengths
    change_in_lengths = msg.data
    # print("change",change_in_lengths)
    
    if module_num == 1:
        for i in range(3):
            cur_cable_lengths[0][i] = cur_cable_lengths[0][i] + change_in_lengths[i]
    else:
        for seg in range(module_num):
            for i in range(3):
                cur_cable_lengths[seg][i] = cur_cable_lengths[seg][i] + change_in_lengths[seg*3+i] 

def update_transformation_matrix(module_list, skp_list, module_n):
    s, kappa, phi = skp_list[module_n]
    T = continuum_fk_arc(s, kappa, phi)
    module_list[module_n] = T

def update_cable_lenth(cbl_list, skp_list, module_n):
    s, kappa, phi = skp_list[module_n]
    l1, l2, l3 = skp_to_cablelength(s, kappa, phi, module_d)
    cbl_list[module_n] = [l1, l2, l3]

def update_skp(skp_list, cbl_list, module_n):
    l1, l2, l3 = cbl_list[module_n]
    s, kappa, phi = cablelen_to_skp(l1, l2, l3, module_d)
    skp_list[module_n] = [s, kappa, phi]

def init_T_list(T_list):
    for _ in range(module_num):
        T_list.append(T0)

def update_EE_pose(new_cable_pose):
    global cable_pos, skp_state, transMatrixList
    
    Tee = T0
    for seg in range(module_num):
        cable_pos[seg] = new_cable_pose[seg]
        update_skp(skp_state, cable_pos, seg)
        update_transformation_matrix(transMatrixList, skp_state, seg)

        Tee = Tee @ transMatrixList[seg]

    return Tee

def get_rvecs(R):
    """Calculate axis-angle vector from given rotation matrix"""
    theta = np.arccos((np.trace(R) - 1)/2)
    if theta > np.pi:
        R = R * -1
        theta = np.arccos((np.trace(R) - 1)/2)

    w = (1/(2*np.sin(theta))) * np.array([[R[2, 1] - R[1, 2]],
                                          [R[0, 2] - R[2, 0]],
                                          [R[1, 0] - R[0, 1]]])

    return theta*w

def main():
    global module_num, pose_pub, cur_cable_lengths, T0

    # ROS node init
    rospy.init_node("sim_fk")

    # Read sim parameters
    module_num = rospy.get_param("origami_sim/num_of_modules")
    rate = rospy.get_param("origami_sim/control_rate")
    cur_cable_lengths = rospy.get_param("origami_sim/init_cable_lengths")
    points_per_segment = rospy.get_param("origami_sim/points_per_segment")

    # Add subscriber for new cable lengths
    arcl_sub = rospy.Subscriber("origami_vs/d_cable_length", Float64MultiArray, getLengths, queue_size=1)
    ref_sub = rospy.Subscriber("origami_vs/reference", Float64MultiArray, getReference, queue_size=1)

    # Define publishers
    pose_pub = rospy.Publisher("origami_vs/features", Float64MultiArray, queue_size=1)
    img_pub = rospy.Publisher("origami_vs/camera_image", Image, queue_size=1)
    cable_length_pub = rospy.Publisher("origami_vs/cable_lengths", Float64MultiArray, queue_size=1)
    
    # Initialize transformation matrix for each module
    init_T_list(transMatrixList)
    
    # Initialize visualization objects
    points = []
    
    # Define camera parameters
    fx = 230
    fy = 230
    cx = 320
    cy = 240

    # Prepare rotation
    # rotate pi/2 along x-axis to fit z-axis up global frame
    rot_x = np.array([[1, 0, 0],
                      [0, 0, -1],
                      [0, 1, 0]])
    
    rot_z = np.array([[0,1,0],
                      [-1,0,0],
                      [0,0,1]])
    
    # rotate -30 degree along z-axis so that motor1 and motor2 plane is parallel to camera's z axis
    # z_rot_angle = -np.pi/6
    # rot_z = np.array([[np.cos(z_rot_angle), -np.sin(z_rot_angle), 0],
    #                   [np.sin(z_rot_angle), np.cos(z_rot_angle), 0],
    #                   [0, 0, 1]])
    # R = rot_x @ rot_z
    R = rot_x    
    rx, ry, rz = get_rvecs(R)

    # Prepare translation 
    tx = 0
    ty = -200 # move camera lower so the base (0, 0, 0) will be higher on the image frame
    tz = 250 # set the object is 250 in depth

    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros((4, 1), dtype=np.float64)
    rvecs = np.array([rx, ry, rz], dtype=np.float64)  # rotation vector
    tvecs = np.array([tx, ty, tz], dtype=np.float64)  # translation vector

    # fig = plt.figure(1)
    # ax = fig.add_subplot(projection='3d')
    # Tn = T0

    # for seg in range(module_num):
    #     s, k, p = skp_state[seg]
    #     plot_arc(Tn, s, k, p, ax)
    #     Tn = Tn @ transMatrixList[seg]

    # ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
    # plt.ion()
    # plt.show()

    # Colors for drawing robot image
    color = [(0, 97, 230), (155, 58, 93)]

    r = rospy.Rate(rate)

    while not rospy.is_shutdown():
        # Define blank image
        img = np.zeros((480, 640, 3), np.uint8)
        
        # This condition prevents pose computation until controller is setup
        if len(cur_cable_lengths) > 0:
            # Compute current pose
            cur_pose = update_EE_pose(cur_cable_lengths)

        # Update robot state for vis
        points.clear()
        Tn = T0
        for seg in range(module_num):
            s, k, p = skp_state[seg]
            points.append(generate_arc_point_list(Tn, s, k, p, points_per_segment))
            Tn = Tn @ transMatrixList[seg]

        points_3d = np.array(points, dtype=np.float64).reshape(-1, 3)

        # Add origin point
        origin_point = T0[0:3, 3]
        points_3d = np.vstack((origin_point, points_3d))
        # print(points_3d[-1])
        # use cv2.projectPoints() project the points onto image frame
        points_2d, _ = cv2.projectPoints(points_3d, rvecs, tvecs, camera_matrix, dist_coeffs)
        # print("points_2d:", points_2d)
        
        # Draw base
        base_point = points_2d[0]
        cv2.rectangle(img, (int(base_point[0][0]-25), int(base_point[0][1])-2),(int(base_point[0][0]+25), int(base_point[0][1])+2),(255,255,255),2)
        
        # Draw each segment in a different color
        for seg in range(module_num):
            for i in range(points_per_segment):
                point = points_2d[seg*points_per_segment+i+1]  # The +1 is because index 0 is base point
                cv2.circle(img, point.ravel().astype(np.int32), 5, color[seg], -1)

        # Draw the end effector point
        end_effector = point.ravel().astype(np.int32)
        cv2.circle(img, end_effector, 5, (183, 95,211), -1)
        
        # fig = plt.figure(1)
        # ax = fig.add_subplot(projection='3d')
        # Tn = T0

        # for seg in range(module_num):
        #     s, k, p = skp_state[seg]
        #     plot_arc(Tn, s, k, p, ax)
        #     Tn = Tn @ transMatrixList[seg]

        # ax.set_box_aspect([ub - lb for lb, ub in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
        # plt.ion()
        # plt.show()

        if servo:
            # print("publishing latest features")
            # Publish features
            cur_features = Float64MultiArray()
            cur_features.data = [point[0][0], point[0][1], 0]
            pose_pub.publish(cur_features)

            # Publish cable lengths
            cable_length_msg = Float64MultiArray()
            cable_length_msg.data = list(np.concatenate(cur_cable_lengths).flat)
            cable_length_pub.publish(cable_length_msg)

            # Draw Target
            cv2.circle(img, (int(cur_target[0]), int(cur_target[1])), 7, (color[1]), 3)
        
        # Convert image to ros_msg and publish
        img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        img_pub.publish(img_msg)
        r.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    main()
