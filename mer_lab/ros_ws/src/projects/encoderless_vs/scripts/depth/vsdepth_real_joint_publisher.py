#!/usr/bin/env python3
# license removed for brevity
import math
import random
import rospy
from std_msgs.msg import Float64MultiArray

def franka_mover():    
    vel_pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
    
    rospy.init_node('joint_vel_talker', anonymous=True)
    rate = rospy.Rate(1) # meaning 1 message published in 1 sec
    # random.seed()     
    # rospy.sleep(1)

    # vel_matrix = [0.0, random.uniform(-0.05, 0.05), 
    #                 0.0, 0.0, random.uniform(-0.1, 0.1), 0.0, 0.0]

    vel_matrix = [0.0, -0.05, 
                    0.0, 0.05, 0.0, 0.0, 0.0]

    velocity = Float64MultiArray()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        velocity.data = vel_matrix
        vel_pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
       franka_mover()
        # rospy.sleep(5)
    except rospy.ROSInterruptException:
        pass
