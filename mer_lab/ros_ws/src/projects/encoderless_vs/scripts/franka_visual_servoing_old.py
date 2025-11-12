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
linvelbase = np.array([[0], [0], [0]])

ee_x = 0
ee_y = 0

xdotee = 0
ydotee = 0

r01 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
r12 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
r23 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) 
r34 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) 
r45 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) 
r56 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) 
r67 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
r_img = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]) 

j2_vel = 0
j4_vel = 0

q2 = None
q4 = None

def joint_callback(msg):
    print("Joint Callback called")
    global vel_matrix, r12, r23, r34, r45, r56, r_img, j2_vel, j4_vel, q2, q4

    # updating the joint positions from /joint_states topic
    q1 = msg.position[0]
    q2 = msg.position[1]
    q3 = msg.position[2]
    q4 = msg.position[3]
    q5 = msg.position[4]
    q6 = msg.position[5]
    q7 = msg.position[6]

    # assigning cos and sine computatation to variables for better visibility
    c1 = np.cos(q1)
    c2 = np.cos(q2)
    c3 = np.cos(q3)
    c4 = np.cos(q4)
    c5 = np.cos(q5)
    c6 = np.cos(q6)
    c7 = np.cos(q7)

    s1 = np.sin(q1)
    s2 = np.sin(q2)
    s3 = np.sin(q3)
    s4 = np.sin(q4)
    s5 = np.sin(q5)
    s6 = np.sin(q6)
    s7 = np.sin(q7)

    c12 = np.cos(q1+q2)
    s12 = np.sin(q1+q2)

    # rotation matrices of the panda robot computed from matlab 
    r12 = np.array([[c1, -s1, 0], [s1,  c1, 0], [ 0, 0, 1]])
    r23 = np.array([[c2, 0, -s2], [s2,0, c2],[ 0, -1, 0]])
    r34 = np.array([[c3, 0, s3], [s3, 0, -c3], [ 0, 1, 0]])
    r45 = np.array([[c4, 0, s4], [s4, 0, -c4], [0, 1, 0]])
    r56 = np.array([[c5, 0, -s5], [s5, 0, c5], [ 0, -1, 0]])
    r67 = np.array([[c6, 0, s6], [s6, 0, -c6],[ 0, 1, 0]])

    # rotation between end effector frame and image frame
    # r_img = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    r_img = np.matrix([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

    # Robot jacobian matrix as calculated from Peter Corke toolbox
    # Jr = np.matrix([[0, 0.4822, 0, -0.1662, 0,0.226, 0],[0.0996, 0, 0.0996, 0, 0.0880,   0, 0],\
    #     [0,   -0.0996, 0, 0.0171, 0, 0.0726,  0],[0, 0, 0, 0, 0.0698,  0 , -0.0698],
    #  [0, 1.0000,  0, -1.0000, 0, -1.0000, 0],[1.0000, 0, 1.0000, 0, 0.9976, 0,  -0.9976]])

    Jr = [[(79*s1*s2)/250-(79*c1*c2)/250+(33*c5*(c12*s4+s12*c3*c4))\
        /400+(48*c12*c4)/125-(33*c12*s4)/400-(33*s12*c3*c4)/400-(48*s12*c3*s4)/125+(33*s12*s3*s5)/400,\
        (79*s1*s2)/250-(79*c1*c2)/250+(33*c5*(c12*s4+s12*c3*c4))/400+(48*c12*c4)/125-(33*c12*s4)\
        /400-(33*s12*c3*c4)/400-(48*s12*c3*s4)/125+(33*s12*s3*s5)/400,\
         -(3*c12*(55*c4*s3+55*c3*s5+256*s3*s4-55*c4*c5*s3))/2000, (33*c5*(s12*c4+c12*c3*s4))/400\
        -(33*s12*c4)/400-(48*s12*s4)/125+(48*c12*c3*c4)/125-(33*c12*c3*s4)/400,\
        -(33*s5*(s12*s4-c12*c3*c4))/400-(33*c12*c5*s3)/400, 0, 0],[(33*c5*(s12*s4-c12*c3*c4))\
        /400-(79*c2*s1)/250-(79*c1*s2)/250+(48*s12*c4)/125-(33*s12*s4)/400+(33*c12*c3*c4)\
        /400+(48*c12*c3*s4)/125-(33*c12*s3*s5)/400, (33*c5*(s12*s4-c12*c3*c4))/400-(79*c2*s1)\
        /250-(79*c1*s2)/250+(48*s12*c4)/125-(33*s12*s4)/400+(33*c12*c3*c4)/400+(48*c12*c3*s4)/125\
        -(33*c12*s3*s5)/400, -(3*s12*(55*c4*s3+55*c3*s5+256*s3*s4-55*c4*c5*s3))/2000,\
        (33*c12*c4)/400-(33*c5*(c12*c4-s12*c3*s4))/400+(48*c12*s4)/125+(48*s12*c3*c4)/125 \
        -(33*s12*c3*s4)/400,(33*s5*(c12*s4+s12*c3*c4))/400-(33*s12*c5*s3)/400, 0],\
        [0, 0, (33*s3*s5)/400-(48*c3*s4)/125-(33*c3*c4)/400+(33*c3*c4*c5)/400,\
        -(3*s3*(256*c4-55*s4+55*c5*s4))/2000,-(33*c3*c5)/400-(33*c4*s3*s5)/400, 0, 0],\
        [0, 0, -s12, c12*s3, s12*c4+c12*c3*s4, s5*(s12*s4-c12*c3*c4)+c12*c5*s3, s5*(s12*s4-c12*c3*c4)\
        +c12*c5*s3], [0, 0, c12, s12*s3, s12*c3*s4-c12*c4, s12*c5*s3-s5*(c12*s4+s12*c3*c4),\
        s12*c5*s3-s5*(c12*s4+s12*c3*c4)], [1,1,0,c3,-s3*s4, c3*c5+c4*s3*s5, c3*c5+c4*s3*s5]]

    Jrx2 = Jr[0][1]
    Jrx4 = Jr[0][3]
    Jry2 = Jr[1][1]
    Jry4 = Jr[1][3]

    Jr = np.matrix([[Jrx2, Jrx4], [Jry2, Jry4]])
    
    # print("Jacobian", Jr)

    # getting the pseduoinverse of Jacobian since Jacobian not square
    Jrinv = np.linalg.inv(Jr)

    # print("Inverted Jacobian", Jrinv)

    # complete 6X6 matrix of linear velocity
    linvel = linvelbase.tolist()

    linvel_mat = [[linvel[0][0]], [linvel[1][0]]] #[linvelbase[2][0]], [0], [0], [0]])    

    # matrix to generate angular velocities
    Jvel_matrix = (np.dot(Jrinv, linvel_mat)).tolist()

    j2_gain = rospy.get_param("vsbot/vs_baseline/j2_gain")
    j4_gain = rospy.get_param("vsbot/vs_baseline/j4_gain")

    j2_vel = Jvel_matrix[0][0]
    j4_vel = Jvel_matrix[1][0]

    print("Angular Velocities", j2_vel, j4_vel)

    # print(j2_vel, j4_vel)

    vel_matrix = [0.0, j2_vel, 0.0, j4_vel, 0.0, 0.0, 0.0]

    # print(vel_matrix)

def vs_callback(msg):
    print("vs callback called")
    global linvelbase, ee_x, ee_y, xdotee, ydotee
    
    ee_x = msg.data[0]
    ee_y = msg.data[1]

    # Read goal position params
    goalX = rospy.get_param("vsbot/control/goal_pos_x")
    goalY = rospy.get_param("vsbot/control/goal_pos_y")

         
    # Applied gain on error
    xgain = rospy.get_param("vsbot/vs_baseline/xdot_gain")
    ygain = rospy.get_param("vsbot/vs_baseline/ydot_gain")
    # error between current and goal end effector position
    xdotee = (goalX - ee_x)*xgain
    ydotee = (goalY - ee_y)*ygain
    linvelimg = np.array([[xdotee],[ydotee],[0]]) # linear end effector velocity in image frame
    linvelee = np.dot(r_img, linvelimg)  # end effector velocity with respect to ee frame
    linvel5 = np.dot(r56, linvelee) # end effector velocity with respect to frame 5
    linvel4 = np.dot(r45, linvel5) # end effector velocity with respect to frame 4
    linvel3 = np.dot(r34, linvel4) # end effector velocity with respect to frame 3
    linvel2 = np.dot(r23, linvel3) # end effector velocity with respect to frame 2
    linvelbase = np.dot(r12, linvel2) # end effector velocity with respect to base frame    
    
    # if (int(goalX - ee_x) == 0) and (int(goalY - ee_y)==0):
    #     vel_matrix = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     rospy.signal_shutdown("Shutting Down")


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
    writer_j_vel.writerow(['Join22', 'Joint4'])

    # creating a csv record file for current end effector position
    f2 = open('end_effector_pose.csv', 'w')
    writer_ee_pos = csv.writer(f2)
    writer_ee_pos.writerow(['ee_x', 'ee_y'])

    # creating a csv record file for updated linear velocity
    f3 = open('end_effector_error.csv', 'w')
    writer_ee_vel = csv.writer(f3)
    writer_ee_vel.writerow(['linvel_x', 'linvel_y'])

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
            row3 = [xdotee, ydotee]
            writer_ee_vel.writerow(row3)
        
        # add current joint angles in the csv file
        if (q2 is not None) and (q4 is not None):
            row4 = [q2, q4]
            writer_j_pos.writerow(row4)
        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
