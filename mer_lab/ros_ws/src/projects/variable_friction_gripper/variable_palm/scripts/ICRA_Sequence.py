#!/usr/bin/env python3
import rospy
from variable_palm.srv import *
import numpy as np

def service_client(service_name, command):
    rospy.wait_for_service(service_name)
    try:
        serv = rospy.ServiceProxy(service_name, PosCommand)
        resp = serv(command)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    # Set the gripper in a configuraion to drag the card
    service_client('drag_card',0.9)

    # Pause while getting arm lines up with the card
    rospy.sleep(5)

    # Straighten the fingers to grasp the object
    # The card was roughly half way into the grasp for the tests
    service_client('hold_object',0.9)

    # Slowly closes width to hold the card
    widthlist = np.linspace(0.2, 0.9, 50)
    for width in widthlist:
        service_client('set_palm_width',width )
       

    # Pause to get the gripper in the horizontal position
    # Make sure the servo numbered five is facing upward when you excute sliding
    rospy.sleep(3)

    # Slide down in right direction
    service_client('credit_card_slide',0.1)

    # Slide down in left direction
    service_client('credit_card_slide_Left',0.1)

    

