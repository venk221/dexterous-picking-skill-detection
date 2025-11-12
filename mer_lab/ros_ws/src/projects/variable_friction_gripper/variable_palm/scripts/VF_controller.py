#!/usr/bin/env python3
# Create ROS Services to move the hand and fingers 

from VF_hands import *
from variable_palm.srv import *
import rospy
import numpy as np
import argparse

motorMax = 0.99
motorMin = 0.01


def set_friction_states(states):
    if states[0] == "high":
        W.moveMotor(4, 0)
    else:
        W.moveMotor(4, 1)

    if states[1] == "high":
        W.moveMotor(5, 0)
    else:
        W.moveMotor(5, 1)

    rospy.sleep(1)

        # Palm width 0.9 for test
def handle_set_palm_width(req):
    W.moveMotor(0, req.data)
    W.moveMotor(3, req.data)
    print("Palm width set to {}".format(req.data))
    return PosCommandResponse(1)



def handle_hold_object(req):
    set_friction_states(["high", "high"])
    #W.moveMotor(0, 0.75)
    #W.moveMotor(3, 0.75)
    #print("Palm width set to {}".format(0.8))
    W.moveMotor(1, 1900.0 / 4095.0 )
    W.moveMotor(2, 1900.0 / 4095.0 )
    # W.torqueMotor(1,0.7)
    # W.torqueMotor(4,0.7)
    W.setFingerSpeed()
    return PosCommandResponse(1)

def handle_drag_card(req):
    set_friction_states(["high", "high"])
    W.moveMotor(0, 0.2)
    W.moveMotor(3, 0.2)
    W.moveMotor(1, 0.9)
    W.moveMotor(2, 0.9)
    return PosCommandResponse(1)

def handle_credit_card_slide(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["high", "low"])
        pos2,_ = pos,_ = W.readMotor(2)
        
        # slide down right finger
        i = 0.01
        while pos < motorMax  and pos2 > motorMin: 
            pos3,_ = W.readMotor(3)
            W.moveMotor(2, pos2)
            W.moveMotor(1, pos)
            W.moveMotor(3, pos3 - i/200)
            pos = pos + 0.01
            pos2 = pos2 - 0.01
            i = i + 0.01
        i = 0.01
        
        # make fingers more parallel
        steplist = np.linspace(0, 0.18, 20)
        pos3,_ = W.readMotor(3)
        pos2,_ = W.readMotor(2)
        for ii in steplist:
            W.moveMotor(2, pos2 - ii)
            if pos3 - ii/1.5 >= 0:
                W.moveMotor(3, pos3 - ii/1.5)

        pos2,_ = W.readMotor(2)
        pos1,_ = W.readMotor(1)
        W.moveMotor(2, pos2 - 0.15 )
        W.moveMotor(1, pos1 - 0.05 )
       
        #switch from low friction to high friction
        while i < 0.99:
            if i < 0.5:
                pos1,_ = W.readMotor(1)
                W.moveMotor(1, pos1 - i/30 )
                pos0,_ = W.readMotor(0)
                W.moveMotor(0, pos0 + i/250  )
            W.moveMotor(5, 1 - i)
            
            i = i + 0.01

        #Switch from high friction to low friction
        i = 0.01
        Pos,_ = W.readMotor(3)
        while i < 1:
            W.moveMotor(4, 0 + i)  
            W.moveMotor(3, Pos - i/100  )
            i = i + 0.01
        
        i = 0.01
        # Slide down left finger
        pos2,_ = W.readMotor(2)
        pos,_ = W.readMotor(1)
        pos3,_ = W.readMotor(3)
        while pos2 < (1900.0 / 4095.0)  and pos1 > (1900.0 / 4095.0): 
            W.moveMotor(1, pos)
            W.moveMotor(2, pos2)
            W.moveMotor(3, pos3 + i)
            pos = pos - 0.01
            pos2 = pos2 + 0.01
            i = i + 0.005


        # Hold Object
        i = 0.01
        W.moveMotor(1, 1900.0 / 4095.0 )
        W.moveMotor(2, 1900.0 / 4095.0 )
        pos0,_ = W.readMotor(0)
        pos3,_ = W.readMotor(3)
        while i < 0.99:
            if pos0 + i/250 <= 0.9:
                W.moveMotor(0, pos0 + i/250  )
            if pos3 + i/250 <= 0.9:
                W.moveMotor(3, pos3 + i/250  )
            W.moveMotor(4, 1 - i)
            
            i = i + 0.01


    return PosCommandResponse(1)

def handle_credit_card_slide_Left(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["low", "high"])
        pos2,_ = W.readMotor(2)
        pos,_ = W.readMotor(1)
        
        # slide down left finger
        i = 0.01
        while pos > motorMin  and pos2 < motorMax: 
            pos0,_ = W.readMotor(0)
            W.moveMotor(2, pos2)
            W.moveMotor(1, pos)
            W.moveMotor(0, pos0 - i/200)
            pos = pos - 0.01
            pos2 = pos2 + 0.01
            i = i + 0.01
        i = 0.01
        
        # make fingers more parallel
        steplist = np.linspace(0, 0.18, 20)
        pos0,_ = W.readMotor(0)
        pos1,_ = W.readMotor(1)
        for ii in steplist:
            W.moveMotor(1, pos1 - ii)
            if pos0 - ii/1.5 >= 0:
                W.moveMotor(0, pos0 - ii/1.1)

        #pos2,_ = W.readMotor(2)
        #pos1,_ = W.readMotor(1)
        #W.moveMotor(2, pos2 - 0.05 )
        #W.moveMotor(1, pos1 + 0.15 )
       
        #switch from low friction to high friction
        while i < 0.99:
            if i < 0.5:
                pos2,_ = W.readMotor(2)
                W.moveMotor(2, pos2 + i/30 )
                pos3,_ = W.readMotor(3)
                W.moveMotor(3, pos3 + i/250  )
            W.moveMotor(4, 1 - i)
            
            i = i + 0.01

        #Switch from high friction to low friction
        i = 0.01
        Pos,_ = W.readMotor(0)
        while i < 1:
            W.moveMotor(5, 0 + i)  
            W.moveMotor(0, Pos - i/10  )
            i = i + 0.01
        
        i = 0.01
        # Slide down right finger
        pos2,_ = W.readMotor(2)
        pos,_ = W.readMotor(1)
        pos0,_ = W.readMotor(0)
        while pos2 > (1900.0 / 4095.0)  and pos1 < (1900.0 / 4095.0): 
            W.moveMotor(2, pos2)
            W.moveMotor(1, pos)
            
            W.moveMotor(0, pos0 + i)
            pos = pos + 0.01
            pos2 = pos2 - 0.01
            i = i + 0.005


        # Hold Object
        i = 0.01
        W.moveMotor(1, 1900.0 / 4095.0 )
        W.moveMotor(2, 1900.0 / 4095.0 )
        pos0,_ = W.readMotor(0)
        pos3,_ = W.readMotor(3)
        while i < 0.99:
            if pos0 + i/250 <= 0.9:
                W.moveMotor(0, pos0 + i/250  )
            if pos3 + i/250 <= 0.9:
                W.moveMotor(3, pos3 + i/250  )
            W.moveMotor(5, 1 - i)
            
            i = i + 0.01


    return PosCommandResponse(1)

def handle_slide_left_finger_down(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["low", "high"])
        W.readFingerTorque()
        W.setFingerTorque(finger='right')
        W.readFingerTorque()      
        W.moveMotor(2, motorMax)
        W.readFingerTorque()
        W.moveMotor(1, req.data)
        W.readFingerTorque()
        
        return PosCommandResponse(1)


def handle_slide_left_finger_up(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["low", "high"])
        W.readFingerTorque()
        W.setFingerTorque(finger='left')
        W.readFingerTorque()
        W.moveMotor(1, motorMax)
        W.readFingerTorque()
        W.moveMotor(2, req.data)
        W.readFingerTorque()
        return PosCommandResponse(1)


def handle_slide_right_finger_down(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["high", "low"])
        W.readFingerTorque()
        W.setFingerTorque(finger='left')
        W.readFingerTorque()
        W.moveMotor(1, motorMax)
        W.readFingerTorque()
        W.moveMotor(2, req.data)
        W.readFingerTorque()
        return PosCommandResponse(1)


def handle_slide_right_finger_up(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["high", "low"])
        W.readFingerTorque()
        W.setFingerTorque(finger='right')
        W.readFingerTorque()
        W.moveMotor(2, motorMax)
        W.readFingerTorque()
        W.moveMotor(1, req.data)
        W.readFingerTorque()
        return PosCommandResponse(1)


def handle_rotate_clockwise(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["high", "high"])
        W.readFingerTorque()
        W.setFingerTorque(finger='right')
        W.readFingerTorque()
        W.moveMotor(2, motorMax)
        W.readFingerTorque()
        W.moveMotor(1, req.data)
        W.readFingerTorque()
        return PosCommandResponse(1)


def handle_rotate_counterclockwise(req):
    if (req.data < motorMin or req.data > motorMax):
        return PosCommandResponse(0)
    else:
        W.readFingerTorque()
        set_friction_states(["high", "high"])
        W.readFingerTorque()
        W.setFingerTorque(finger='left')
        W.readFingerTorque()
        W.moveMotor(1, motorMax)
        W.readFingerTorque()
        W.moveMotor(2, req.data)
        W.readFingerTorque()
        return PosCommandResponse(1)


def handle_release_object(req):
    set_friction_states(["low", "low"])
    W.setFingerTorque(val=0)
    return PosCommandResponse(1)


def handle_read_motor(req):
    val, enc = W.readMotor(req.motor_no)
    resp = ReadMotorResponse()
    resp.val = val
    resp.enc = enc
    return resp


def handle_write_motor(req):
    W.moveMotor(req.motor_no, req.val)
    return WriteMotorResponse(1)


if __name__ == "__main__":
    rospy.init_node('variable_palm_controller_server')

    argparser = argparse.ArgumentParser()
    argparser.add_argument('--hand_port', default='/dev/ttyUSB0')
    argparser.add_argument('--finger_port', default='/dev/ttyUSB1')
    
    args = argparser.parse_args()
    
    W = Model_VF(args.hand_port, args.finger_port, 1, 2, 3, 4, 5, 6, "XM")
    # W = Model_Test("/dev/ttyUSB0", 20, 21, "XM")

    ccsL = rospy.Service("credit_card_slide_Left", PosCommand, handle_credit_card_slide_Left)
    ccs = rospy.Service("credit_card_slide", PosCommand, handle_credit_card_slide)
    dc = rospy.Service("drag_card", PosCommand, handle_drag_card)
    spw = rospy.Service("set_palm_width", PosCommand, handle_set_palm_width)
    ho = rospy.Service("hold_object", PosCommand, handle_hold_object)
    ro = rospy.Service("release_object", PosCommand, handle_release_object)
    slfd = rospy.Service("slide_left_finger_down", PosCommand, handle_slide_left_finger_down)
    slfu = rospy.Service("slide_left_finger_up", PosCommand, handle_slide_left_finger_up)
    srfd = rospy.Service("slide_right_finger_down", PosCommand, handle_slide_right_finger_down)
    srfu = rospy.Service("slide_right_finger_up", PosCommand, handle_slide_right_finger_up)
    rcw = rospy.Service("rotate_clockwise", PosCommand, handle_rotate_clockwise)
    rccw = rospy.Service("rotate_counterclockwise", PosCommand, handle_rotate_counterclockwise)

    read = rospy.Service("read_motor", ReadMotor, handle_read_motor)
    write = rospy.Service("write_motor", WriteMotor, handle_write_motor)

    rospy.spin()