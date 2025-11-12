#!/usr/bin/env python3
#Auto Calibration of Fingers and Hand
import numpy as np
# import rospy
import argparse
from variable_palm.srv import *
from VF_hands import Model_VF
from VF_controller_functions import *

def autoCalib(hand, motor_ids, LOAD_THRESHOLD=15):
    '''
    Inputs:
        FINGER_LEN
        HAND_MOVEMENT_LEN
        LOAD_THRESHOLD
    #! Steps:
        1 Set initial hand position: Take the hand motors to the outer most positions
        2. set friction state of both fingers to low
        2 Set the initial finger positions: Set finger positions to the outermost positions as well
        3 move the hand by 1 unit step
        4 move the fingers; 0->90, 
            if the load value crosses a threshold
                - stop
                - calib complete
            else
                - repeat step 2->4
        5 if no object found return to init position
    '''

    #set torque values
    # hand.setFingerTorque(val=0.8, finger='both')

    print([hand.servos[i].read_encoder for i in range(6)])

    #set init hand position
    # set_palm_width(hand, 0.1,dir=1)
    rospy.sleep(1)
    #TODO check if it hit the block at the end

    #set finger states
    set_friction_states(hand, states=['high','high'])

    #set init finger positions
    for i in range(0,1):
        # rospy.sleep(1)
        move_left_finger(hand,0.3,motorDir=1)
        # rospy.sleep(1)
        move_right_finger(hand,0.3,motorDir=-1)
        rospy.sleep(1)

        print('Movement successful')

    # move the palm/hand but one unit
    obj_found=False
    for step in np.arange(0,0.1,0.02):
        set_palm_width(hand, step, dir=-1)
        for finger_step in np.arange(0.2,0.5,0.01):
            move_left_finger(hand,finger_step, motorDir=-1)
            move_right_finger(hand,finger_step, motorDir=-1)
            
            # rospy.sleep(1)
            load_left = float(hand.readLoadForMotor(motor_ids['finger_motor_left']))
            load_right = float(hand.readLoadForMotor(motor_ids['finger_motor_right']))

            print('left finger load: '+ str(load_left) + '\n')
            print('right finger load: '+ str(load_right) + '\n')

            if(abs(load_left)>=LOAD_THRESHOLD or abs(load_right)>=LOAD_THRESHOLD):
                print('Load changed! BREAK THE LOOP!! BREAK THE LOOP\n\n\n')
                obj_found=True
                break
        if(obj_found==True):
            print('AutoCalib complete\n')
            break
        else:
            print("Couldn't find the sweet spot\n")
        rospy.sleep(1)

    return

if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--hand_port', default='/dev/ttyUSB0')
    argparser.add_argument('--friction_port', default='/dev/ttyUSB1')
    args = argparser.parse_args()

    motor_ids = {
    'hand_motor_right':1,
    'finger_motor_right':2,
    'finger_motor_left':3,
    'hand_motor_left':4,
    'friction_motor_left':5,
    'friction_motor_right':6,
    }

    hand = Model_VF(args.hand_port,args.friction_port, motor_ids['hand_motor_right'], motor_ids['finger_motor_right'], motor_ids['finger_motor_left'], motor_ids['hand_motor_left'], motor_ids['friction_motor_left'], motor_ids['friction_motor_right'], "XM")
    autoCalib(hand, motor_ids)