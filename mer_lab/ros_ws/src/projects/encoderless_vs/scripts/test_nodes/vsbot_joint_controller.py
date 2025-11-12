#!/usr/bin/env python3
# license removed for brevity
import math
import random
import rospy
from std_msgs.msg import Float64

def talker():    
    pub1 = rospy.Publisher('/vsbot/joint1_velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/vsbot/joint2_velocity_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/vsbot/joint3_velocity_controller/command', Float64, queue_size=10)
    
    rospy.init_node('joint_vel_talker', anonymous=True)
    rate = rospy.Rate(1) # meaning 1 message published in 1 sec
    # random.seed()
    i=0     
    while not rospy.is_shutdown():              
        # q1_vel = 2*math.sin(i*math.pi + 0.8) #random.uniform(-0.5, 0.5) #5 * math.sin(i)                    #
        # q2_vel = 2*math.cos(i*math.pi - 0.5) #random.uniform(-0.5, 0.5) #7 * math.sin(i)                    #
        # print("Joint1 Velocity", q1_vel)
        # print("Joint2 Velocity", q2_vel)
        q1_vel = random.uniform(-0.5, 0.5)
        q2_vel = random.uniform(-0.5, 0.5)
        q3_vel = random.uniform(-0.5, 0.5)
        pub1.publish(q1_vel)
        pub2.publish(q2_vel)
        pub3.publish(q3_vel)
        i = i+0.1
        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
