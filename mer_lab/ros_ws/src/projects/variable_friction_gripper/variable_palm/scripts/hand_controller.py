#!/usr/bin/env python3
from hands import *
from variable_palm.srv import *
import rospy

motorMax = 0.9
motorMin = 0.1


def set_friction_states(states):
    if states[0] == "high":
        W.moveMotor(0, 0)
    else:
        W.moveMotor(0, 0.8)

    if states[1] == "high":
        W.moveMotor(3, 0.5)
    else:
        W.moveMotor(3, 1)

    rospy.sleep(1)


def handle_set_palm_width(req):
    W.moveMotor(2, req.data / 2)
    W.moveMotor(5, req.data / 2)
    print("Palm width set to {}".format(req.data))
    return PosCommandResponse(1)



def handle_hold_object(req):
    set_friction_states(["high", "high"])
    W.setFingerTorque()
    W.moveMotor(1, motorMax)
    W.moveMotor(4, motorMax)
    # W.torqueMotor(1,0.7)
    # W.torqueMotor(4,0.7)
    W.setFingerSpeed()
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
        W.moveMotor(4, motorMax)
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
        W.moveMotor(4, req.data)
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
        W.moveMotor(4, req.data)
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
        W.moveMotor(4, motorMax)
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
        W.moveMotor(4, motorMax)
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
        W.moveMotor(4, req.data)
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

    W = Model_W("/dev/ttyUSB0", 1, 2, 3, 4, 5, 6, "XL")

    spw = rospy.Service("set_palm_width", PosCommand, handle_set_palm_width)
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
