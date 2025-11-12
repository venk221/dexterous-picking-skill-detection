#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import keyboard
from tkinter import*

vel_msg = Int32MultiArray()
num_actuators = 0
rate = 0

# Keyboard listener callback
def send_increment(key):
    global vel_msg
    # Use switch case or conditions to choose velocity msg data

def main():
    rospy.init_node("keyboard_node")

    global num_actuators, rate

    # Publishers
    input_pub = rospy.Publisher("origami_vs/OMMD_vel", Int32MultiArray, queue_size=1)
    mode_pub = rospy.Publisher("origami_vs/OMMD_control_mode", Int32, queue_size=1)

    # Read in parameters
    # num_actuators = rospy.get_param("origami_skeleton_vs/no_of_actuators")
    # rate = rospy.get_param("origami_skeleton_vs/control_rate")
    # TK instance
    root = Tk()

    # ROS msgs
    mode_msg = Int32()
    mode_msg.data = 1

    # Keyboard listener loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        # expand all
        # if keyboard.is_pressed('w'):
        #     send_increment(0)
        #     print('w')

        # # contract all
        # if keyboard.is_pressed('s'):
        #     send_increment(1)
        #     print('s')

        # # bend left
        # if keyboard.is_pressed('a'):
        #     send_increment(2)
        #     print('a')

        # # bend right
        # if keyboard.is_pressed('d'):
        #     send_increment(3)
        #     print('d')

        input_pub.publish(vel_msg)
        mode_pub.publish(mode_msg)

        # r.sleep()
        root.mainloop()

if __name__ == "__main__":
    main()