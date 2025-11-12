#!/usr/bin/env python3
# license removed for brevity
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, Float64, Bool, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Declaring cvBridge for cv to ros conversion and vice versa
control_flag = None


def start_vel(msg):
    global control_flag
    control_flag = msg.data


def main():
    # Initialize the node
    rospy.init_node('pos_control_dream_depth_ds_gen')
    flag_sub = rospy.Subscriber(
        "/franka/control_flag", Bool, start_vel, queue_size=1)

    pub = rospy.Publisher('/position_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    stop_pub = rospy.Publisher('/node/stop', Float64, queue_size=1)

    points_1 = [0.05749443941250753, 0.0348567428632265, -0.06122166075159962, \
        -2.36481474250274, -0.14681326545088727, 2.3256683265897964, -0.6471021017803008]
    points_2 = [-0.14388429290369936, -0.9116789614192224, 0.007053329724550676, \
        -2.392968496516767, -0.006875019672653089, 1.5927911340925425, 0.5742785146241964]
    points_3 = [0.011813786179675502, 0.18208683474741638, -0.01134785583297878, \
        -2.285521212006048, -0.006860209093828165, 1.9723091645902067, -0.9213482787435928]

    joint_pos_1 = JointTrajectory()
    # joint_pos_2 = JointTrajectory()
    # joint_pos_1.header = Header()
    # joint_pos_1.header.stamp = rospy.Time.now()
    joint_pos_1.joint_names=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', \
                                'panda_joint5', 'panda_joint6', 'panda_joint7']
    point=JointTrajectoryPoint()
    # point.positions = [-0.14400032784855157, -0.9114426452563292, \
    #                     0.007202914413326142, -2.3928297259468323, -0.0067417885280624335, \
    #                     1.5928215515348643, 0.5744856126045371]

    


    point.positions = points_1
    point.time_from_start = rospy.Duration(10)
    joint_pos_1.points.append(point)

    # joint_pos_2.joint_names=['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', \
    #                             'panda_joint5', 'panda_joint6', 'panda_joint7']
    # point = JointTrajectoryPoint()
    # point.positions = points_2
    # point.time_from_start = rospy.Duration(10)
    # joint_pos_2.points.append(point)

    while not rospy.is_shutdown():
        pub.publish(joint_pos_1)
        # pub.publish(joint_pos_2)
        print("we have checked enough")

    # while not rospy.is_shutdown():
    #     # if control_flag == True:
    #     start_time = rospy.Time.now()
    #     to_wait = rospy.Duration(3)
    #     end_time_1 = start_time + to_wait    
    #     while rospy.Time.now() < end_time_1:
            
            

    rospy.spin()


if __name__ == '__main__':
    main()