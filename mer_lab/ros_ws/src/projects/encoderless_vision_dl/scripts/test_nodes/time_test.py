#!/usr/bin/env python3.8

import time
import datetime
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Bool


control_flag = None


def start_vel(msg):
    global control_flag
    control_flag = msg.data


def main():
    # Initialize the node
    rospy.init_node('test_time')

    flag_sub = rospy.Subscriber("/franka/control_flag", Bool, start_vel, queue_size=1)
    while not rospy.is_shutdown():
        if control_flag == True:
            print(control_flag)
            start_time = rospy.Time.now()
            to_wait = rospy.Duration(2)
            print(to_wait)
            end_time = start_time + to_wait
            print(start_time, end_time)
            while rospy.Time.now() < end_time:
                print("We need to check the first one")
                rospy.sleep(rospy.Duration(0.1))

            start_time = rospy.Time.now()
            to_wait = rospy.Duration(4)
            end_time_2 = start_time + to_wait
            print(start_time, end_time)
            while end_time < rospy.Time.now() < end_time_2:
                print("We need to check the second one")
                rospy.sleep(rospy.Duration(0.1))

            print("we have checked enough")
            rospy.signal_shutdown("I have good reason")

            # while not rospy.is_shutdown():
            #     right_now = rospy.Time.now()
            #     print(right_now)
            #     rospy.sleep(5)
            #     print("5 secs have passed")
                # after_five = rospy.get_time()
            # print(after_five)

            # print(after_five - right_now)
    rospy.spin()

if __name__ == '__main__':
    main()


