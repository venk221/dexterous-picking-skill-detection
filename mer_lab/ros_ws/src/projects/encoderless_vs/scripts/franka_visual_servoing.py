#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
import csv
import glob
import os
import scipy
from scipy import linalg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int32

# j2_vel = 0 # global variables for velocities to be published to joint2
# j4_vel = 0 # global variables for velocities to be published to joint2

vel_matrix = [] # global variable to publish group joint velocity
linvelbase = np.array([[0], [0], [0]]) # global linear velocity in base frame

# global end effector position
ee_x = 0
ee_y = 0

# global error
error_x = None
error_y = None

# global end effector velocity in image frame
xdotee = 0
ydotee = 0

max_error_bound = 1
min_error_bound = -1

# global angular velocity
j2_vel = 0
j4_vel = 0

# global joint angles
q2 = None
q4 = None

def joint_callback(msg):
    print("Joint Callback called")
    global vel_matrix, j2_vel, j4_vel, q2, q4

    # updating the joint positions from /joint_states topic
    q1 = msg.position[1]
    q2 = msg.position[3]

    # bounding parameters for stopping conditions 
    max_error_bound = 1
    min_error_bound = -1
    
    # joint limits for optimized arm movement
    max_vel_lim = 0.2
    min_vel_lim = -0.2

    # Robot jacobian matrix as calculated from Peter Corke toolbox
    # Jr = np.matrix([[0, 0.4822, 0, -0.1662, 0,0.226, 0],[0.0996, 0, 0.0996, 0, 0.0880,   0, 0],\
    #     [0,   -0.0996, 0, 0.0171, 0, 0.0726,  0],[0, 0, 0, 0, 0.0698,  0 , -0.0698],
    #  [0, 1.0000,  0, -1.0000, 0, -1.0000, 0],[1.0000, 0, 1.0000, 0, 0.9976, 0,  -0.9976]])

    Jr = [[-(79*np.sin(q1))/250 - (48*np.cos(q1)*np.sin(q2))/125 - \
        (48*np.cos(q2)*np.sin(q1))/125, - (48*np.cos(q1)*np.sin(q2))/125 - \
         (48*np.cos(q2)*np.sin(q1))/125], 
        [(79*np.cos(q1))/250 + (48*np.cos(q1)*np.cos(q2))/125 - (48*np.sin(q1)*np.sin(q2))/125,\
        (48*np.cos(q1)*np.cos(q2))/125 - (48*np.sin(q1)*np.sin(q2))/125]]

        

    # getting the inverse of Jacobian
    Jrinv = np.linalg.inv(Jr)

    # conveting base frame linear velocity to list for easier computation
    linvel = linvelbase.tolist()

    linvel_mat = [[linvel[0][0]], [linvel[2][0]]]    

    # matrix to generate angular velocities
    Jvel_matrix = (np.matmul(Jrinv, linvel_mat)).tolist()

    # velocities on joint 2 and joint 4
    j2_vel = (Jvel_matrix[0][0])
    j4_vel = (Jvel_matrix[1][0])

    # adding joint limits for optimized robot motion
    if j2_vel > max_vel_lim:
        j2_vel = 0.2
    
    elif j2_vel < min_vel_lim:
        j2_vel = -0.2

    # Similar adjustments as applied on joint2    
    if j4_vel > max_vel_lim:
        j4_vel = 0.2
    
    elif j4_vel < min_vel_lim :
        j4_vel = -0.2   
  
    # Stopping condition
    if (error_x is not None) and (error_y is not None) and (min_error_bound < error_x < max_error_bound) and (min_error_bound <= error_y <= max_error_bound):
        vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    
    else:
        vel = [0.0, j2_vel, 0.0, j4_vel, 0.0, 0.0, 0.0]
        # rospy.signal_shutdown("Shutting Down")
    
    vel_matrix = vel

def vs_callback(msg):
    print("vs callback called")
    global linvelbase, ee_x, ee_y, xdotee, ydotee, error_x, error_y
    
    ee_x = msg.data[0]
    ee_y = msg.data[1]

    # Read goal position params
    goalX = rospy.get_param("vsbot/control/goal_pos_x")
    goalY = rospy.get_param("vsbot/control/goal_pos_y")

    error_x = int(goalX - ee_x)
    error_y = int(goalY - ee_y)

    # rotation between image frame and base frame
    # the base frame is rotated +ve 180 around image frame y-axis and then +90 degrees around 
    # current x-axis after the first rotation

    r_img = np.matrix([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])


    # Applied gain on error
    xgain = rospy.get_param("vsbot/vs_baseline/xdot_gain")
    ygain = rospy.get_param("vsbot/vs_baseline/ydot_gain")
    
    # condition to make the robot move if error_bound is reached before reaching goal position.
    if (error_x is not None) and (error_y is not None) and (min_error_bound <= error_x <= max_error_bound) and (min_error_bound <= error_y <= max_error_bound):
        xgain = 0.005
        xgain = 0.005

    # error between current and goal end effector position
    xdotee = (goalX - ee_x)*xgain
    ydotee = (goalY - ee_y)*ygain

    linvelimg = np.array([[xdotee],[ydotee],[0]]) # linear end effector velocity in image frame    
    linvelbase = np.dot(r_img, linvelimg) # end effector velocity with respect to base frame    
    
    
    

def main():    
    # Initialize ROS
    rospy.init_node('franka_visual_servoing')

    # Initialize subscribers 
    ee_sub = rospy.Subscriber("aruco/Pose", Float32MultiArray, vs_callback)
    angle_sub = rospy.Subscriber("/joint_states", JointState, joint_callback)

    # Initialize publisher for group joint velocities
    vel_pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
    
    # loop rate to publish joint velocity
    rate = rospy.Rate(30)

    # creating a csv record file for published velocities
    f1 = open('joint_velocities_recorder.csv', 'w')
    writer_j_vel = csv.writer(f1)
    writer_j_vel.writerow(['Joint2_Vel', 'Joint4_vel'])

    # creating a csv record file for current end effector position
    f2 = open('end_effector_pose.csv', 'w')
    writer_ee_pos = csv.writer(f2)
    writer_ee_pos.writerow(['ee_x', 'ee_y'])

    # creating a csv record file for updated linear velocity
    f3 = open('end_effector_error.csv', 'w')
    writer_ee_vel = csv.writer(f3)
    writer_ee_vel.writerow(['error_x', 'error_y', 'current ee vel x', 'current ee vel y','current base vel x', 'current base vel y'])

    # creating a csv record file for updated joint2 and joint4 angles
    f4 = open('joint_angles.csv', 'w')
    writer_j_pos = csv.writer(f4)
    writer_j_pos.writerow(['Joint2Angle', 'Joint4Angle'])

    
    while not rospy.is_shutdown():
        velocity = Float64MultiArray()
        velocity.data = vel_matrix
        vel_pub.publish(velocity)              

        # add non-zero joint velocities in the csv file
        if j2_vel !=0 and j4_vel !=0:
            row1 = [j2_vel, j4_vel]
            writer_j_vel.writerow(row1)

        # add non-zero x and y poition in the csv file
        if ee_x != 0 and ee_y != 0:
            row2 = [ee_x, ee_y]
            writer_ee_pos.writerow(row2)

        # add non-zero linear velocity in the csv file
        if xdotee != 0 and ydotee != 0:
            row3 = [error_x, error_y, xdotee, ydotee, linvelbase[0][0], linvelbase[1][0]]
            writer_ee_vel.writerow(row3)
        
        # add current joint angles in the csv file
        if (q2 is not None) and (q4 is not None):
            row4 = [q2, q4]
            writer_j_pos.writerow(row4)
        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
