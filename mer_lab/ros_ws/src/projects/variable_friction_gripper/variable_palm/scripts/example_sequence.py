#!/usr/bin/env python3
import rospy
from variable_palm.srv import *

def service_client(service_name, command):
    rospy.wait_for_service(service_name)
    try:
        serv = rospy.ServiceProxy(service_name, PosCommand)
        resp = serv(command)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    t=2
    service_client('set_palm_width',2.0)
    rospy.sleep(t)
    service_client('set_palm_width',1.0)
    rospy.sleep(t)
    service_client('hold_object',0.0)
    rospy.sleep(t)

    for i in range(0,2):
        service_client('slide_right_finger_down',0.3)
        rospy.sleep(t)
        service_client('slide_left_finger_down',0.3)
        rospy.sleep(t)

    for i in range(0,2):
        service_client('slide_left_finger_up',0.3)
        rospy.sleep(t)
        service_client('slide_right_finger_up',0.3)
        rospy.sleep(t)


    service_client('rotate_counterclockwise',0.3)
    rospy.sleep(t)
    service_client('rotate_clockwise',0.3)
    rospy.sleep(t)


    service_client('release_object',0.0)
    rospy.sleep(t)
