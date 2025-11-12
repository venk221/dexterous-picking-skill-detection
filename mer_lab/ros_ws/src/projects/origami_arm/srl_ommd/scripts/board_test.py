#!/usr/bin/env python3
# This node is for Soft Robotics Lab Origami Module Motor Driver

import rospy
from std_msgs.msg import Int32MultiArray

import hid
from importlib.metadata import version
import math
import numpy as np

def motorBytes(x):
    mb = [0] * (2 * len(x))
    for i, xi in enumerate(x):
        xi = max(-2047, min(2047, round(xi)))
        mb[2*i] = (xi >> 8) & 0x0F
        mb[2*i + 1] = xi & 0xFF
    return mb

def motorPWM(vel_list, motorEnableMask):
    
   # u = np.array(list(vel_list.data)) * np.array(motorEnableMask)
    u = [0] * 32
    for k in range(32):
        u[k] = motorEnableMask[k] * vel_list.data[k]
        
    return u
    
def write_vel_cmd_cb(vel_msg, args):
    motorEnableMask = args[1]
    h = args[0]
    
    u = motorPWM(vel_msg, motorEnableMask)
    h.write(bytes([0] + motorBytes(u)))

def main():
    """main function"""
    
    vid = 0xFFEF
    pid = 0x0004

    num_module = 4

    print(f'\nHID Module Version: {version("hidapi")}')

    try:
        h = hid.device()
        h.open(vid, pid)
        print(f'\nUSB Device Manufacturer: {h.get_manufacturer_string()}\nUSB Device Product Name: {h.get_product_string()}\n')

        h.write(bytes(65))
        h.read(64)

        inData = list(h.read(64))

        u = [0]*32

        motorEnable = [0, 1, 1, 1] * num_module + [0] * 4 * (8 - num_module)

        rospy.init_node('ommd_listener', anonymous=True)
        vel_sub = rospy.Subscriber("origami_vs/OMMD_velocity", Int32MultiArray, write_vel_cmd_cb, (h,motorEnable), queue_size = 1)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("done!")
    except IOError as ex:
        print(ex)
        print("You probably don't have the hard-coded device.")
        print("Update the h.open() line in this script with the one")
        print("from the enumeration list output above and try again.")

if __name__ == '__main__':
    main()
            
