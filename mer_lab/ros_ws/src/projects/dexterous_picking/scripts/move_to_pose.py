#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import sys

home_joint_state = [0.119575, -0.751142, -0.0703775, -2.08545, -0.06018, 1.3240, 0.94463]

def pose_callback(pose_msg):
    # Got new mesaage
    print("Planning....")

    # Initialize move group for Panda arm
    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_planner_id("BiTRRT")
    group.panda.go_to_joint_state(home_joint_state)
    # Set the target pose
    # group.set_pose_target(pose_msg)

    # # Plan and execute the motion without waiting for completion
    # success, plan, time, error = group.plan()  # Getting full tuple response
    # if success:
    #     group.execute(plan, wait=True)
    #     rospy.loginfo("Execution started.")
    # else:
    #     rospy.logerr(f"Failed to plan trajectory: Error code {error}")

    # # Clear targets after starting execution
    # group.clear_pose_targets()


def main():
    # Initialize ROS node
    rospy.init_node('panda_moveit_control')

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Subscribe to the pose topic
    rospy.Subscriber("/pose_topic", Pose, pose_callback)

    # Keep the script alive
    rospy.spin()

if __name__ == '__main__':
    main()
