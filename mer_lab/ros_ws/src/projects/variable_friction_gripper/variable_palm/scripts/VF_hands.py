#!/usr/bin/env python3

# Created by multiple users
# Yale Grablab
# Updated 09/2019

from __future__ import print_function
from lib_robotis_mod import *
import time
import numpy as np  # for array handling
import scipy.io
import math
from decimal import Decimal


#! Assumptions:
#! only dynamixel servos being used
#! either all RX or MX servos being used (no mixing)
#! different encoder limits for each type
#! motor limits and mov't designated in terms of proportion, not encoder value


class OpenHand():
    dyn = None  # USB2Dynamixel device
    dyn2 = None
    port = None  # mounted port, /devttyUSB# in Linux, COM# in Windows
    port2 = None
    servo_ids = []
    servos = []

    servo_speed = 1.0
    max_torque = 0.4  # Dynamixel suggests capping max torque at 0.4 of stall torque

    motorDir = []
    motorMin = []
    motorMax = []
    modes = []  # position control (True) or torque/pseudo-torque control (False)

    amnt_release = 0.0  # values [0,1.0] for degree of closing and opening
    amnt_close = 0.5

    pause = 0.3  # amount of time to wait for move commands and eeprom updates

    HAND_HEIGHT = 0.14  # hand height from base to palm (m) used for arm IK and approach vectors
    WRIST_OFFSET = -np.pi / 3  # J5 offset (rad) to accommodate final orientation

    def __init__(self, port, port2, servo_ids, series="RX"):
        self.port = port

        if series == 'XL':
            self.dyn = USB2Dynamixel_Device(port, 1000000)  # always only one

        else:
            self.dyn = USB2Dynamixel_Device(port, 57600)
            self.dyn2 = USB2Dynamixel_Device(port2, 1000000)

            #self.dyn2 = USB2Dynamixel_Device(port, 1000000)


        self.servo_ids = servo_ids
        num_servos = len(servo_ids)
        print("Initializing...")
        self.servos = []
        for servo_id in self.servo_ids:
            if series == "RX" or series == "MX":
                self.servos.append(Robotis_Servo(self.dyn, servo_id, series))
            elif series == "X":  # We will be using protocol 2 instead
                self.servos.append(Robotis_Servo_X(self.dyn, servo_id, series))
            else:
                if servo_id < 5:
                #self.servos.append(Robotis_Servo_XL(self.dyn, servo_id, series))
                    self.servos.append(Robotis_Servo_X(self.dyn, servo_id, series))
                else:
                    self.servos.append(Robotis_Servo_XL(self.dyn2, servo_id, "XL"))
                    #self.servos.append(Robotis_Servo_X(self.dyn2, servo_id, series))


            print("Adding servo id " + repr(servo_id))
            time.sleep(self.pause)

        if series != 'XL':
            for servo in self.servos:
                if servo.servo_id < 5:
                    servo.kill_cont_turn()  # make sure position mode limits are enabled

                    time.sleep(self.pause)  # in case eeprom delay is what is causing the issues
                if series == "RX":
                    servo.apply_speed(1)
                time.sleep(self.pause)
                servo.apply_max_torque(self.max_torque)

        self.modes = [
                         True] * num_servos  # default assignment (shouldn't have servos in torque mode during normal
        # operation)

        if len(self.motorDir) != num_servos or len(self.motorMin) != num_servos or len(self.motorMax) != num_servos:
            print("[ERR] Servo number mismatch, resetting motor limits")
            self.motorDir = [1] * num_servos
            self.motorMin = [self.amnt_release] * num_servos
            self.motorMax = [self.amnt_close] * num_servos

        if num_servos == 4:  # This is the model_O and we want to prevent gear shear
            for i in range(num_servos):
                enc = self.servos[i].read_encoder()
                if series == "RX":
                    if (self.motorDir[i] == 1 and enc > 512) or (self.motorDir[i] == -1 and enc < 512):
                        print("------------FAILSAFE-------------")
                        print("Failsafe is incorporated to prevent gear shear in Model O")
                        print("Motor encoder postion: ", enc)
                        input = raw_input("Your encoder position for motor index " + str(
                            i) + " may cause the motor to move backwards and break gears. We recommend you resetting the fingers to prevent gear shear, proceed? [ENTER]")
                    else:
                        print("Motor directions not set...")
                # These would then be the MX and XM motors
                elif (self.motorDir[i] == 1 and enc > 2048) or (self.motorDir[i] == -1 and enc < 2048):
                    print("------------FAILSAFE-------------")
                    print("Failsafe is incorporated to prevent gear shear in Model O")
                    print("Motor encoder postion: ", enc)
                    input = raw_input("As an XM Motor, we can automatically fix this issue for motor " + str(
                        i) + ", proceed? [ENTER]")
                    self.servos[i].enable_extended_position_control_mode()
                    self.servos[i].move_to_encoder(self.servos[i].settings['max_encoder'] + 100)
                    time.sleep(self.pause)
                    self.servos[i].disable_extended_position_control_mode()
                    time.sleep(self.pause)
                    print("Fixed servo from ID: " + repr(servo_ids[i]))
            # Finally, limit the abduction_torque if desired
            if self.abduction_limit != 1:
                self.servos[0].enable_current_position_control_mode(self.abduction_limit)

        time.sleep(self.pause)
        # self.reset()
        print("Initialization Complete.")

    def reset(self):  # returns everything to zeroed positions, different from release
        print("[ERR] reset() not implemented")
        return False

    def release(self):  # opens the finger components, doesn't necessarily move non-finger servos
        print("[ERR] release() not implemented\n")
        return False

    # close functions are normalized regardless of mode such that the operating range [0,1.0] makes sense
    def close(self, amnt=0.5):
        print("[ERR] close() not implemented\n")
        return False

    # tval: torque applied to servos, dpos: delta position overshoot beyond current
    def close_torque(self, tval=0.2, dpos=1.0):
        print("[ERR] close_torque() not implemented\n")
        return False

    # difference between close_torque should just be the particular servos that are actuated
    def _close_torques(self, tval=0.2, dpos=1.0, idxs=None):
        if idxs is None:  # effect on all servos if id's not specified
            idxs = range(len(self.servos))
        hp, he = self.readHand()
        for idx in idxs:
            self.torqueMotor(idx, tval, hp[idx] + dpos)
        i = 0
        while i < 15:  # some arbitrary limit on torque closing
            for idx in idxs:
                if not self.servos[idx].is_moving():
                    amnt, enc = self.readMotor(idx)
                    self.moveMotor(idx, amnt)  # exit out of torque mode
            time.sleep(self.pause)
            i += 1
        self.hold()

    # move servo according to amnt, not encoder value, scaled between designated min/max values
    def moveMotor(self, index, amnt):
        if amnt < 0. or amnt > 1.0:
            print("[WARNING] motion out of bounds, capping to [0,1]. Index: " + repr(index) + ", Cmd:" + repr(amnt))
            amnt = min(max(amnt, 0.), 1.0)
        if (index < 0 or index >= len(self.servos)):
            print("[ERR] invalid motor index " + repr(index))
        else:
            servo = self.servos[index]

            if self.motorDir[index] > 0:  # normal case
                servo.move_to_encoder(int(servo.settings["max_encoder"] * (
                        self.motorMin[index] + amnt * (self.motorMax[index] - self.motorMin[index]))))
            else:  # reverse
                servo.move_to_encoder(int(servo.settings["max_encoder"] * (
                        self.motorMax[index] - amnt * (self.motorMax[index] - self.motorMin[index]))))

            if not self.modes[index]:  # restore position-control mode if necessary - want to register new encoder target first before re-applying system torque
                self.modes[index] = True
                servo.apply_max_torque(self.max_torque)

    def getCurrDir(self):
        global currdir, take_no
        print('Current directory: ')
        currdir = raw_input()
        print('Take number: ')
        take_no = raw_input()

    def torqueMotor(self, index, val, pos_val=None):
        val = min(1.0, max(val, 0))  # by design, can exceed default max torque value
        self.modes[index] = False  # swap to torque mode in record keeping
        s = self.servos[index]
        if pos_val is None:
            enc = s.settings['max_encoder']
        else:
            pos_val = min(max(0, pos_val), 1.0)  # new target position is always max to force saturation of servo torque
            enc = int((pos_val * (self.motorMax[index] - self.motorMin[index]) + self.motorMin[index]) * s.settings[
                'max_encoder'])

        s.apply_max_torque(val)
        s.move_to_encoder(enc)

    def moveHand(self, vals):
        if len(vals) != len(self.servos):
            print("[ERR] Motor number mismatch")
        else:
            for i in range(len(vals)):
                self.moveMotor(i, vals[i])

    # returns motor position amnt, between designated min and max values
    def readMotor(self, index):
        servo = self.servos[index]
        enc = servo.read_encoder()
        if self.motorDir[index] > 0:
            val = (enc / float(servo.settings["max_encoder"]) - self.motorMin[index]) / (
                    self.motorMax[index] - self.motorMin[index])
        else:
            val = (self.motorMax[index] - enc / float(servo.settings["max_encoder"])) / (
                    self.motorMax[index] - self.motorMin[index])
        return val, enc

    def readLoads(self):
        for servo in self.servos:
            print("---")
            print("Servo ID: " + repr(servo.servo_id))
            print("Load: " + repr(servo.read_load()))

    def readLoadForMotor(self,servo_index):
        return repr(self.servos[servo_index].read_load())
            

    def readMotorMins(self):
        index = 0
        for servo in self.servos:
            print("---")
            print("Servo ID: " + repr(servo.servo_id))
            print("Motor Mins: " + repr(self.motorMin[index]))
            index = index + 1

    def readHand(self):
        amnts = np.array([0.] * len(self.servos))
        encs = np.array([0] * len(self.servos))

        for i in range(len(self.servos)):
            amnt, enc = self.readMotor(i)
            amnts[i] = amnt
            encs[i] = enc

        return amnts, encs

    # takes the current location and sets either the min or max
    def setMotorMin(self):
        amnts, encs = self.readHand()
        self.motorMin = (encs / float(self.servos[0].settings['max_encoder'])).tolist()

    def setMotorMax(self):
        amnts, encs = self.readHand()
        self.motorMax = (encs / float(self.servos[0].settings['max_encoder'])).tolist()

    # setting the max torque (shortcut for torque-based closing motions)
    def setMaxTorque(self, val=0.4):

        val = max(min(1.0, val), 0.0)
        for servo in self.servos:
            # servo.apply_max_torque(val)
            time.sleep(self.pause)  # helps mitigate eeprom delay issues?
        self.max_torque = val

    def setServoSpeed(self, val=1.0):
        val = max(min(1.0, val), 0.1)
        for servo in self.servos:
            servo.apply_speed(val)
            time.sleep(self.pause)
        self.servo_speed = val

    # moves to the current encoder value and locks servos in place to minimize current draw
    def preventAllLoadErrors(self, offset_scale=0):
        for i in range(len(self.servos)):
            self.preventLoadError(i, offset_scale)

    def preventLoadError(self, i, offset_scale=0):
        if abs(self.servos[i].read_load()) > 80:  # arbitrary load threshold
            value = offset_scale * 10 + self.servos[i].read_encoder()  # should never be negative
            if value < self.servos[i].settings['max_encoder']:
                self.servos[i].move_to_encoder(value)
            else:
                if value < 0:
                    self.servos[i].move_to_encoder(0)
                else:
                    self.servos[i].move_to_encoder(self.servos[i].settings['max_encoder'])

    # move to the current position for each motor - fast switch from torque mode and alternative to preventAllLoadErrors
    def hold(self):
        for i in range(len(self.servos)):
            amnt, enc = self.readMotor(i)
            self.moveMotor(i, amnt)

    def diagnostics(self):
        for servo in self.servos:
            print("---")
            print("Servo ID: " + repr(servo.servo_id))
            print("Load: " + repr(servo.read_load()))
            print("Temperature: " + repr(servo.read_temperature()))
            print("Target Encoder: " + repr(servo.read_target_encoder()))
            print("Current Encoder: " + repr(servo.read_encoder()))


# ------------------------------------------------------#

# Different hand types

# ------------------------------------------------------#


class Model_VF(OpenHand):
    defaults = {

        # left finger min encoder values
        'X1_MIN_ENC': 50.0,
        #'X2_MIN_ENC': 1750.0,
        #'X3_MIN_ENC': 1750.0,
        'X2_MIN_ENC': 1700.0,
        'X3_MIN_ENC': 1700.0,
        # right finger min encoder values
        'X4_MIN_ENC': 50.0,
        'X5_MIN_ENC': 250.0,
        'X6_MIN_ENC': 250.0,

        # left finger max encoder values
        'X1_MAX_ENC': 1350.0,
        #'X2_MAX_ENC': 2450.0,
        #'X3_MAX_ENC': 2450.0,
        'X2_MAX_ENC': 2333.0,
        'X3_MAX_ENC': 2333.0,

        # right finger max encoder values
        'X4_MAX_ENC': 1350.0,
        'X5_MAX_ENC': 525.0,
        'X6_MAX_ENC': 525.0,

    }

    motorDir = [1, 1, 1, 1, 1, 1]
    motorMinEnc = [defaults['X1_MIN_ENC'], defaults['X2_MIN_ENC'], defaults['X3_MIN_ENC'], defaults['X4_MIN_ENC'],
                   defaults['X5_MIN_ENC'], defaults['X6_MIN_ENC']]  # should always be symmetric here?
    motorMaxEnc = [defaults['X1_MAX_ENC'], defaults['X2_MAX_ENC'], defaults['X3_MAX_ENC'], defaults['X4_MAX_ENC'],
                   defaults['X5_MAX_ENC'], defaults['X6_MAX_ENC']]

    motorMin = [x / 4095.0 if i <= 3 else x / 1023.0 for i,x in enumerate(motorMinEnc)]
    motorMax = [x / 4095.0 if i <= 3 else x / 1023.0 for i,x in enumerate(motorMaxEnc)]

    #motorMin[4] = [x / 1023.0 for x in motorMinEnc]
    #motorMax = [x / 1023.0 for x in motorMaxEnc]

    HOLD_TORQUE = 0.5
    OVERSHOOT = 0.15

    pause = 0.2

    def __init__(self, port="/dev/ttyUSB0", port2="/dev/ttyUSB1",  s1=1, s2=2, s3=3, s4=4, s5=5, s6=6, dyn_model="XM"):
        """
        port = hand port
        port2 = finger port

        """
        OpenHand.__init__(self, port, port2, [s1, s2, s3, s4, s5, s6], dyn_model)
        # self.close_hand([0.,0.,0.,0.,0.,0.])
        #self.setHandLEDColor(4)

    def reset(self):
        self.release()

    def release(self):
        for x in range(0, 5):
            self.moveMotor(x, 0)
            time.sleep(0.01)

    def close_hand(self, amnt=[0.3, 0.3, 0.3, 0.3, 0.3, 0.3]):

        for x in range(0, len(amnt)):
            self.moveMotor(x, amnt[x])
            time.sleep(0.01)

    def close_left_finger(self, amnt=[0.3, 0.3, 0.3]):
        for x in range(0, len(amnt)):
            self.moveMotor(x, amnt[x])
            time.sleep(0.01)

    def close_right_finger(self, amnt=[0.3, 0.3, 0.3]):
        for x in range(0, len(amnt)):
            self.moveMotor(3 + x, amnt[x])
            time.sleep(0.01)

    #def setHandLEDColor(self, color):
        #for x in range(0, 6):
            #servo = self.servos[x]
            #servo.write_LED(color)

    def setHandTorque(self, val):
        val = min(max(val, 0), 1)
        for x in range(0, 6):
            servo = self.servos[x]
            servo.apply_max_torque(val)

    def setFingerTorque(self, val=0.8, finger='both'):
        val = min(max(val, 0), 1)
        if finger == 'left':
            servo = self.servos[2]
            servo.apply_max_torque(val)
        elif finger == 'right':
            servo = self.servos[1]
            servo.apply_max_torque(val)
        else:
            servo = self.servos[2]
            servo.apply_max_torque(val)

            servo = self.servos[1]
            servo.apply_max_torque(val)

    def setFingerSpeed(self, val=0.5):
        val = max(min(1.0, val), 0.1)
        servo_left = self.servos[2]
        servo_right = self.servos[1]
        servo_left.servo_speed = val
        servo_right.servo_speed = val

    def readFingerTorque(self):
        servo_left = self.servos[2]
        servo_right = self.servos[1]
        print("Left Finger:")
        print("Load: " + repr(servo_left.read_load()))
        print("Right Finger:")
        print("Load: " + repr(servo_right.read_load()))



class Model_Test(OpenHand):
    defaults = {

        # left finger min encoder values
        'X1_MIN_ENC': 452.0,
        'X2_MIN_ENC': 485.0,
        'X3_MIN_ENC': 542.0,
        # right finger min encoder values
        'X4_MIN_ENC': 264.0,
        'X5_MIN_ENC': 200.0,
        'X6_MIN_ENC': 200.0,

        # left finger max encoder values
        'X1_MAX_ENC': 920.0,
        'X2_MAX_ENC': 120.0,
        'X3_MAX_ENC': 210.0,

        # right finger max encoder values
        'X4_MAX_ENC': 700.0,
        'X5_MAX_ENC': 525.0,
        'X6_MAX_ENC': 525.0,

    }

    motorDir = [1, 1]
    motorMinEnc = [defaults['X1_MIN_ENC'], defaults['X2_MIN_ENC']]  # should always be symmetric here?
    motorMaxEnc = [defaults['X1_MAX_ENC'], defaults['X2_MAX_ENC']]

    motorMin = [x / 1023.0 for x in motorMinEnc]
    motorMax = [x / 1023.0 for x in motorMaxEnc]

    HOLD_TORQUE = 0.5
    OVERSHOOT = 0.15

    pause = 0.2

    def __init__(self,  port="/dev/ttyUSB1",  s1=1, s2=2, dyn_model="XL"):
        OpenHand.__init__(self, port, [s1, s2], dyn_model)
        # self.close_hand([0.,0.,0.,0.,0.,0.])
        #self.setHandLEDColor(4)

    def reset(self):
        self.release()

    def release(self):
        for x in range(0, 5):
            self.moveMotor(x, 0)
            time.sleep(0.01)

    def close_hand(self, amnt=[0.3, 0.3]):

        for x in range(0, len(amnt)):
            self.moveMotor(x, amnt[x])
            time.sleep(0.01)

    def close_left_finger(self, amnt=[0.3, 0.3]):
        for x in range(0, len(amnt)):
            self.moveMotor(x, amnt[x])
            time.sleep(0.01)

    def close_right_finger(self, amnt=[0.3, 0.3]):
        for x in range(0, len(amnt)):
            self.moveMotor(3 + x, amnt[x])
            time.sleep(0.01)

    #def setHandLEDColor(self, color):
        #for x in range(0, 6):
            #servo = self.servos[x]
            #servo.write_LED(color)

    def setHandTorque(self, val):
        val = min(max(val, 0), 1)
        for x in range(0, 6):
            servo = self.servos[x]
            servo.apply_max_torque(val)

    def setFingerTorque(self, val=0.8, finger='both'):
        val = min(max(val, 0), 1)
        if finger == 'left':
            servo = self.servos[0]
            servo.apply_max_torque(val)
        elif finger == 'right':
            servo = self.servos[1]
            servo.apply_max_torque(val)
        else:
            servo = self.servos[0]
            servo.apply_max_torque(val)

            servo = self.servos[1]
            servo.apply_max_torque(val)

    def setFingerSpeed(self, val=0.5):
        val = max(min(1.0, val), 0.1)
        servo_left = self.servos[0]
        servo_right = self.servos[1]
        servo_left.servo_speed = val
        servo_right.servo_speed = val

    def readFingerTorque(self):
        servo_left = self.servos[0]
        servo_right = self.servos[1]
        print("Left Finger:")
        print("Load: " + repr(servo_left.read_load()))
        print("Right Finger:")
        print("Load: " + repr(servo_right.read_load()))
