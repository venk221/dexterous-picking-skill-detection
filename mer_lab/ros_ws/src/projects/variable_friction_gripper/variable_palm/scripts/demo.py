## Demo to run the new gripper
import numpy as np
import argparse
from variable_palm.srv import *
from VF_hands import Model_VF
from VF_controller_functions import *
from autoCalib import autoCalib

if __name__=='__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--hand_port', default='/dev/ttyUSB0')
    argparser.add_argument('--friction_port', default='/dev/ttyUSB1')
    args = argparser.parse_args()

    #init VF-Hand class
    hand_port, friction_port = args.hand_port,args.friction_port
    motor_ids = {
        'hand_motor_right':1,
        'finger_motor_right':2,
        'finger_motor_left':3,
        'hand_motor_left':4,
        'friction_motor_left':5,
        'friction_motor_right':6,
    }
    hand = Model_VF(hand_port, friction_port, motor_ids['hand_motor_right'], motor_ids['finger_motor_right'], motor_ids['finger_motor_left'], motor_ids['hand_motor_left'], motor_ids['friction_motor_left'], motor_ids['friction_motor_right'], "XM")

    def demo(hand):
        autoCalib(hand, motor_ids)

        slide_object_on_right_finger(hand, motorDir='down', torqueVal=0.4, motorPos=0.2, motorMinPos=0.1, motorMaxPos= 0.3)
        
        #hold object
        hand.setFingerTorque(finger='left')
        hand.setFingerTorque(finger='right')
        set_friction_states(hand, states=['high','high'])
    
        slide_object_on_left_finger(hand, motorDir='up', torqueVal=0.4, motorPos=0.2, motorMinPos=0.1, motorMaxPos= 0.3)

    demo(hand)