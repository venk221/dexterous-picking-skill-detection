#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import sys

def publish_poses():
    # Initialize ROS node
    rospy.init_node('pose_publisher')

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create a publisher object
    pub = rospy.Publisher('/pose_topic', Pose, queue_size=10)

    # Set the rate of publishing
    rate = rospy.Rate(1)  # 1 Hz

    # Define increments and cycling behavior
    cycle_length = 5  # Number of steps before cycling back
    current_step = 0
    increment_value = 0.01
    direction = 1  # Start by incrementing

    while not rospy.is_shutdown():
        # Get current pose from MoveIt!

        # Compute new position
        new_pose = Pose()
        new_pose.position.x = 0.4 + current_step * direction * increment_value
        new_pose.position.y = 0.0 + current_step * direction * increment_value
        new_pose.position.z = 0.6 + current_step * direction * increment_value
        new_pose.orientation.w = 0.005634
        new_pose.orientation.x = 0.926374
        new_pose.orientation.y = -0.376433
        new_pose.orientation.z = -0.009869


        rospy.loginfo(f"Publishing pose: {new_pose}")
        pub.publish(new_pose)
        break

        # Update steps and adjust direction if needed
        current_step += 1
        if current_step == cycle_length:
            direction *= -1  # Reverse the increment direction
            current_step = 0  # Reset step counter

        rate.sleep()
        rospy.sleep(10)

if __name__ == '__main__':
    try:
        publish_poses()
    except rospy.ROSInterruptException:
        pass
