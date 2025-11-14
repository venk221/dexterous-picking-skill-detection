#!/usr/bin/python

import sys
sys.path.append('/home/merlab/mer_lab/ros_ws/src/projects/dexterous_picking/src') 

import time
import os
os.system("sudo chmod 666 /dev/ttyUSB0")
import openhand_node.hands as hands

class RobotHand:
    def __init__(self, port = '/dev/ttyUSB0'):
        self.T = hands.Model_O(port, 3, 4, 1, 2, "XM")
        self.T.release()
        self.T.moveMotor(2, 0.3)
        self.T.moveMotor(1, 0.1)
        self.T.adduct()
        self.T.moveMotor(1, 0.6)
        self.T.moveMotor(2, 0.57)

    def simple_pick_initialize(self):
        self.T.open()
        time.sleep(1)
        self.T.adduct(0.5)
        time.sleep(1)
        return True

    def push_to_horizontal_initialize(self, amount=0.12):
        self.T.open()
        time.sleep(1)
        self.T.adduct(0.5)
        time.sleep(1)
        self.T.moveMotor(3, amount)
        self.T.moveMotor(1, amount)
        self.T.moveMotor(2, amount)
        time.sleep(1)
        return True

    def slide_to_edge_initialize(self, amount=0.2):
        self.T.open()
        time.sleep(1)
        self.T.adduct(1)
        time.sleep(1)
        self.T.moveMotor(1, amount)
        self.T.moveMotor(2, amount)
        time.sleep(1)
        return True

    def push_to_vertical_initialize(self, amount=0.2):
        self.T.open()
        time.sleep(0.5)
        self.T.adduct(1)
        time.sleep(0.5)
        self.T.moveMotor(1, amount)
        self.T.moveMotor(2, amount)
        time.sleep(0.5)
        return True

    def flip_initialize(self):
        self.T.open()
        time.sleep(0.5)
        self.T.adduct(1)
        time.sleep(0.5)
        return True

    def flip_execute(self, grasp_amount=0.25, flip_amount=0.4):
        self.T.moveMotor(1, grasp_amount)
        self.T.moveMotor(2, grasp_amount)
        self.T.moveMotor(3, grasp_amount)
        time.sleep(1)
        self.T.moveMotor(3, flip_amount)
        time.sleep(0.5)
        self.T.moveMotor(1, grasp_amount + 0.05)
        self.T.moveMotor(2, grasp_amount + 0.05)
        time.sleep(0.5)
        return True

    def close(self, amount=0.35):
        self.T.close(amount)
        time.sleep(1) #hold object time
        return True

    def slide_to_edge_close(self, amount=0.4):
        self.T.close(amount)
        time.sleep(1) #hold object time
        return True

    def release(self):
        self.T.open()
        time.sleep(1)
        self.T.adduct(0)
        time.sleep(1)
        return True

def main():
    gripper = RobotHand()
    print("Executing simple pick initialize")
    gripper.simple_pick_initialize()
    print("Executing simple pick close")
    gripper.close()
    print("Executing simple pick release")
    gripper.release()
    print("Executing push to horizontal initialize")
    gripper.push_to_horizontal_initialize()
    print("Executing push to horizontal close")
    gripper.close()
    print("Executing push to horizontal release")
    gripper.release()
    print("Executing slide to edge initialize")
    gripper.slide_to_edge_initialize()
    print("Executing slide to edge close")
    gripper.close()
    print("Executing slide to edge release")
    gripper.release()
    print("Executing push to vertical initialize")
    gripper.push_to_vertical_initialize()
    print("Executing push to vertical close")
    gripper.close()
    print("Executing push to vertical release")
    gripper.release()
    print("Executing flip initialize")
    gripper.flip_initialize()
    print("Executing flip execute")
    gripper.flip_execute()
    print("Executing flip release")
    gripper.release()


if __name__ == "__main__":
    main()


