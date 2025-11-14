#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list

HOME_POSE = geometry_msgs.msg.Pose()
HOME_POSE.position.x = 0.400
HOME_POSE.position.y = 0.000
HOME_POSE.position.z = 0.687
orientation_quat = quaternion_from_euler(-3.118, -0.023, -0.752)
HOME_POSE.orientation.x = orientation_quat[0]
HOME_POSE.orientation.y = orientation_quat[1]
HOME_POSE.orientation.z = orientation_quat[2]
HOME_POSE.orientation.w = orientation_quat[3]

pose_1 = geometry_msgs.msg.Pose()
pose_1.position.x = 0.494
pose_1.position.y = 0.000
pose_1.position.z = 0.200
pose_1.orientation.x = orientation_quat[0]
pose_1.orientation.y = orientation_quat[1]
pose_1.orientation.z = orientation_quat[2]
pose_1.orientation.w = orientation_quat[3]

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MotionPlanningInterface(object):

    def __init__(self):
        super(MotionPlanningInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id("BiTRRT")
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
    def go_to_joint_state(self, goal_joint_values): # goal_joint_values is a list of 7 floats
        self.move_group.go(goal_joint_values, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return all_close(goal_joint_values, current_joints, 0.01)

    def go_to_pose_goal(self, goal_posestamped):
        self.move_group.set_pose_target(goal_posestamped)
        success, plan, planning_time, error_code = self.move_group.plan()
        # success = self.move_group.go(wait=True)
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(goal_posestamped, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints_poses, end_effector_step=0.01, jump_threshold=0.0):
        # current_pose = self.move_group.get_current_pose().pose
        # waypoints_poses.insert(0, current_pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints_poses, end_effector_step, jump_threshold)
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(waypoints_poses[-1], current_pose, 0.01)

def main():
    rospy.init_node("move_group_interface", anonymous=True)
    try:
        panda = MotionPlanningInterface()
        
        panda.go_to_pose_goal(HOME_POSE)

        for i in range(3):
            panda.go_to_pose_goal(pose_1)
            panda.go_to_pose_goal(HOME_POSE)

        waypoints = []
        waypoints.append(pose_1)

        panda.plan_cartesian_path(waypoints)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
