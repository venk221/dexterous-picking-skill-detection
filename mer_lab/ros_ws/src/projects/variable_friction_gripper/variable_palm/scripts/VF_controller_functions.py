from variable_palm.srv import *
import numpy as np
import rospy
from curses import baudrate
from turtle import left
from lib_robotis_mod import Robotis_Servo_X as Servo
from lib_robotis_mod import USB2Dynamixel_Device

def setTorque(torque, servo_port, servo_id):
    #Parameters
    max_torque = 0.4  # Dynamixel suggests capping max torque at 0.4 of stall torque
    baudrate_servo= 57600
    device = USB2Dynamixel_Device(servo_port,baudrate_servo)
    servo_id_left = servo_id

    servo = Servo(device,servo_id_left,series='X')
    servo.enable_torque_mode()
    servo.apply_torque(torque)

#TODO: Change motor IDs with Dictionary keys for readability
def set_friction_states(handObj, states):
    if states[0] == "high":
        handObj.moveMotor(4, 0)
    else:
        handObj.moveMotor(4, 1)

    if states[1] == "high":
        handObj.moveMotor(5, 0)
    else:
        handObj.moveMotor(5, 1)

    rospy.sleep(1)

def set_friction_state_for_motor(handObj, motorId, state):
    '''
    friction state motors are set at id 4,5 hence hard coded
    TODO: use dict keys istead of hard code.
    '''
    if motorId not in (4,5):
        return

    if state == "high":
        handObj.moveMotor(motorId, 0)
    else:
        handObj.moveMotor(motorId, 1)

    rospy.sleep(1)


def set_palm_width(handObj, width, dir=1):
    '''
    handObj = VF_Hands class obj with low level functions
    width = desired width of the hand/palm
    dir = motor direction for the two hand motors (1=normal, -1=reverse)
    Resets the motor dir back to normal after function exec to maintain consistency throughout
    '''
    
    oldMotorDir = handObj.motorDir
    if dir==-1:
        handObj.motorDir = [-1,1,1,-1,1,1]
    handObj.moveMotor(0, width)
    handObj.moveMotor(3, width)
    print("Palm width set to {}".format(width))

    if dir==-1:
        handObj.motorDir = oldMotorDir
    
    return PosCommandResponse(1)


def move_left_finger(handObj, motorPos, motorMinPos=0.01, motorMaxPos=0.99, motorDir=1):
    '''
    motor serial num:2 aka index:3 = left
    '''
    if (motorPos < motorMinPos or motorPos > motorMaxPos):
        print('invalid pos\n')
        return PosCommandResponse(0)
    else:
        oldMotorDir = handObj.motorDir
        if motorDir==-1:
            handObj.motorDir = [1,-1,-1,1,1,1]
    
        # handObj.readFingerTorque()
        # set_friction_states(["low", "low"])
        # handObj.readFingerTorque()
        print('setting right finger torque')
        handObj.setFingerTorque(finger='right')
        # handObj.readFingerTorque()      
        # handObj.moveMotor(2, motorMax)
        # handObj.readFingerTorque()
        print('moving left finger now')
        handObj.moveMotor(2, motorPos)
        # handObj.readFingerTorque()        

        if motorDir==-1:
            handObj.motorDir = oldMotorDir

        return PosCommandResponse(1)

def move_right_finger(handObj, motorPos, motorMinPos=0.01, motorMaxPos=0.99, motorDir=1):
    '''
    motor serial num:1 aka index:2 = right
    '''
    if (motorPos < motorMinPos or motorPos > motorMaxPos):
        return PosCommandResponse(0)
    else:
        oldMotorDir = handObj.motorDir
        if motorDir==-1:
            handObj.motorDir = [1,-1,-1,1,1,1]

        # handObj.readFingerTorque()
        # set_friction_states(["high", "low"])
        # handObj.readFingerTorque()
        print('setting left finger torque')
        handObj.setFingerTorque(finger='left')

        print('moving right finger now')
        handObj.moveMotor(1, motorPos)
        # handObj.readFingerTorque()

        if motorDir==-1:
            handObj.motorDir = oldMotorDir

        return PosCommandResponse(1)

def hold_object(handObj):
    set_friction_states(handObj,["high", "high"])
    #Move motors to the center
    handObj.moveMotor(1, 4000.0 / 4095.0 )
    handObj.moveMotor(2, 4000.0 / 4095.0 )
    
    handObj.setFingerSpeed()
    return PosCommandResponse(1)

def moveLeftFingerDifferentially(hand, position, step=0.01, motorMinPos=0.01, motorMaxPos=0.99, motorDir=1):
    current_pos,_ = hand.readMotor(2)
    print(str(current_pos)+'\n')
    diff = position-current_pos
    if diff<0:
        step=-step
    for pos in np.arange(current_pos, position, step):
        print(str(pos)+'\n')
        move_left_finger(hand,pos,motorMinPos, motorMaxPos,motorDir=motorDir)


def moveRightFingerDifferentially(hand, position, step=0.01, motorMinPos=0.01, motorMaxPos=0.99, motorDir=1):
    current_pos,_ = hand.readMotor(1)
    print(str(current_pos)+'\n')
    diff = position-current_pos
    if diff<0:
        step=-step
    for pos in np.arange(current_pos,position,step):
        move_right_finger(hand,pos, motorMinPos, motorMaxPos, motorDir=motorDir)


def slide_object_on_left_finger(handObj, motorDir, torqueVal, motorPos, motorMinPos, motorMaxPos):
    if (motorPos < motorMinPos or motorPos > motorMaxPos):
        return PosCommandResponse(0)
    else:
        set_friction_states(handObj, states=['low','high'])
        
        if motorDir=='up':
            print('moving l finger up\n')
            handObj.setFingerTorque(finger='left')
            moveRightFingerDifferentially(handObj, motorMaxPos)
            moveLeftFingerDifferentially(handObj, motorPos)
        elif motorDir=='down':
            print('moving l finger down\n')
            handObj.setFingerTorque(finger='right', )
            moveLeftFingerDifferentially(handObj, motorMaxPos)
            moveRightFingerDifferentially(handObj, motorPos)
        else:
            print('Invalid Motor Dir\n')
        handObj.readFingerTorque()

        # hold_object(handObj)
        return PosCommandResponse(1)
        

def slide_object_on_right_finger(handObj, motorDir, torqueVal, motorPos, motorMinPos, motorMaxPos):
    if (motorPos < motorMinPos or motorPos > motorMaxPos):
        print('motor outside min and max pos\n')
        return PosCommandResponse(0)
    else:
        set_friction_states(handObj, states=['high','low'])
        print('Friction states setup done\n')
        
        if motorDir=='up':
            print('moving right finger up\n')
            handObj.setFingerTorque(finger='left')
            moveLeftFingerDifferentially(handObj, motorMaxPos)
            moveRightFingerDifferentially(handObj, motorPos)

        elif motorDir=='down':
            print('moving right finger down\n')
            handObj.setFingerTorque(finger='left')
            # handObj.moveMotor(1, motorMaxPos)
            # handObj.moveMotor(2, motorPos)
            moveRightFingerDifferentially(handObj, motorMaxPos)
            moveLeftFingerDifferentially(handObj, motorPos)

        else:
            print('Invalid Motor Dir\n')
        handObj.readFingerTorque()
        return PosCommandResponse(1)


def slide_object(handObj, finger, motorDir, torqueVal, motorPos, motorMinPos, motorMaxPos):
    if finger=='left':
        slide_object_on_left_finger(handObj, motorDir, torqueVal, motorPos, motorMinPos, motorMaxPos)
    elif finger=='right':
        slide_object_on_right_finger(handObj, motorDir, torqueVal, motorPos, motorMinPos, motorMaxPos)
    else:
        print('Invalid finger input. Please choose between "left" and "right"\n')
        return PosCommandResponse(0)
    return PosCommandResponse(1)


##! EXAMPLE SEQUENCE 
def handle_credit_card_slide(handObj, motorPos, motorMinPos, motorMaxPos):
    if (motorPos < motorMinPos or motorPos > motorMaxPos):
        return PosCommandResponse(0)
    else:
        handObj.readFingerTorque()
        set_friction_states(handObj, states=['high','low'])
        pos2,_ = pos,_ = handObj.readMotor(2)
        
        # Differentially slide down right finger
        step = 0.01
        while pos < motorMaxPos  and pos2 > motorMinPos: 
            pos3,_ = handObj.readMotor(3)
            handObj.moveMotor(2, pos2)
            handObj.moveMotor(1, pos)
            handObj.moveMotor(3, pos3 - step/100)
            pos = pos + 0.01
            pos2 = pos2 - 0.01
            step += 0.01
        
        # make fingers more parallel
        steplist = np.linspace(0, 0.18, 20)
        pos3,_ = handObj.readMotor(3)
        pos2,_ = handObj.readMotor(2)
        for step in steplist:
            handObj.moveMotor(2, pos2 - step)
            if pos3 - step/1.5 >= 0:
                handObj.moveMotor(3, pos3 - step/1.5)

        pos2,_ = handObj.readMotor(2)
        pos1,_ = handObj.readMotor(1)
        handObj.moveMotor(2, pos2 - 0.15 )
        handObj.moveMotor(1, pos1 - 0.05 )
       
        #switch from low friction to high friction
        i = 0.01
        while i < 0.99:
            if i < 0.5:
                pos1,_ = handObj.readMotor(1)
                handObj.moveMotor(1, pos1 - i/30 )
                pos0,_ = handObj.readMotor(0)
                handObj.moveMotor(0, pos0 + i/250  )
            handObj.moveMotor(5, 1 - i)
            
            i = i + 0.01

        #Switch from high friction to low friction
        i = 0.01
        Pos,_ = handObj.readMotor(3)
        while i < 1:
            handObj.moveMotor(4, 0 + i)  
            handObj.moveMotor(3, Pos - i/100  )
            i = i + 0.01
        
        # Differentially slide down left finger
        step = 0.01
        pos2,_ = handObj.readMotor(2)
        pos,_ = handObj.readMotor(1)
        pos3,_ = handObj.readMotor(3)
        while pos2 < (1900.0 / 4095.0)  and pos1 > (1900.0 / 4095.0): 
            handObj.moveMotor(1, pos)
            handObj.moveMotor(2, pos2)
            handObj.moveMotor(3, pos3 + step)
            pos = pos - 0.01
            pos2 = pos2 + 0.01
            step += 0.005

    return PosCommandResponse(1)

def handle_credit_card_slide_Left(handObj, motorPos, motorMinPos, motorMaxPos):
    if (motorPos < motorMinPos or motorPos > motorMaxPos):
        return PosCommandResponse(0)
    else:
        handObj.readFingerTorque()
        set_friction_states(handObj, states=['high','low'])
        pos2,_ = handObj.readMotor(2)
        pos,_ = handObj.readMotor(1)
        
        # Differentially slide down left finger
        step = 0.01
        while pos > motorMinPos  and pos2 < motorMaxPos: 
            pos0,_ = handObj.readMotor(0)
            handObj.moveMotor(2, pos2)
            handObj.moveMotor(1, pos)
            handObj.moveMotor(0, pos0 - step/200)
            pos = pos - 0.01
            pos2 = pos2 + 0.01
            step += 0.01
        
        # make fingers more parallel
        step = 0.01
        steplist = np.linspace(0, 0.18, 20)
        pos0,_ = handObj.readMotor(0)
        pos1,_ = handObj.readMotor(1)
        for step in steplist:
            handObj.moveMotor(1, pos1 - step)
            if pos0 - step/1.5 >= 0:
                handObj.moveMotor(0, pos0 - step/1.1)
       
        #switch from low friction to high friction
        i = 0.01
        while i < 0.99:
            if i < 0.5:
                pos2,_ = handObj.readMotor(2)
                handObj.moveMotor(2, pos2 + i/30 )
                pos3,_ = handObj.readMotor(3)
                handObj.moveMotor(3, pos3 + i/250  )
            handObj.moveMotor(4, 1 - i)
            
            i += 0.01

        #Switch from high friction to low friction
        i = 0.01
        Pos,_ = handObj.readMotor(0)
        while i < 1:
            handObj.moveMotor(5, 0 + i)  
            handObj.moveMotor(0, Pos - i/10  )
            i += 0.01
        
        # Differentially Slide down right finger
        step = 0.01
        pos2,_ = handObj.readMotor(2)
        pos,_ = handObj.readMotor(1)
        pos0,_ = handObj.readMotor(0)
        while pos2 > (1900.0 / 4095.0)  and pos1 < (1900.0 / 4095.0): 
            handObj.moveMotor(2, pos2)
            handObj.moveMotor(1, pos)
            
            handObj.moveMotor(0, pos0 + step/200)
            pos = pos + 0.01
            pos2 = pos2 - 0.075
            step = step + 0.005
        
    return PosCommandResponse(1)

