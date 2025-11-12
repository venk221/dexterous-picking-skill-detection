#!/usr/bin/env python3

import rospy
from ur3_moveit.msg import UR3JointStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def callback(data):
    joints = JointState()

    # rospy.loginfo(rospy.get_caller_id() + "I heard:\n%s", data)
    joints.header = Header()
    joints.header.stamp = rospy.Time.now()
    joints.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint','finger_joint', 'left_inner_knuckle_joint','left_inner_finger_joint','right_outer_knuckle_joint','right_inner_knuckle_joint', 'right_inner_finger_joint']
    for angle in data.joint_position:
        joints.position.append(angle)
    
    for i in range(6):
        joints.position.append(0.0)

    joints.velocity = []
    joints.effort = []
    pub.publish(joints)

def listener():
    rospy.init_node("Joint_State_Sub_and_Pub", anonymous=True)
    rospy.Subscriber("/ur3/joint_states", UR3JointStates, callback=callback)
    rospy.spin()

if __name__ == "__main__":
    pub = rospy.Publisher("/joint_states",JointState, queue_size = 10)
    listener()