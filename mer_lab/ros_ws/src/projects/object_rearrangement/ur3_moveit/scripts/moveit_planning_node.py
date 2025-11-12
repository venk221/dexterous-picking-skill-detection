#!/usr/bin/env python3

from shutil import move
import sys
import copy
import math
import moveit_commander
import rospy
# import moveit_msgs.msg
# from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
# import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped

from ur3_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def serviceCallback(req):
    response = MoverServiceResponse()
    setScene()
    print(req)
    current_robot_joint_angles = [0.0,0.0,0.0,0.0,0.0,0.0]
    if req.pose_2.position.y == req.pose_1.position.y: 
        increment = (req.pose_2.position.x - req.pose_1.position.x)/10
        print("Increment_x = {}".format(increment))
        poselist = []
        temp_pose = copy.deepcopy(req.pose_1)
        poselist.append(temp_pose)
        move_group.set_planning_time(10)
        move_group.set_num_planning_attempts(20)
        trajectories = []
        temp_pose.position.x = temp_pose.position.x - increment
        poselist = []
        poselist.append(temp_pose)
        print(temp_pose)
        for i in range(10):
            temp_pose.position.x = temp_pose.position.x + increment
            print(temp_pose)
            trajectory = plan_trajectory(move_group, temp_pose, current_robot_joint_angles)
            
        #     # print(type(trajectory))
            if (len(trajectory.joint_trajectory.points) > 0):
                
                print("--------------------------")
                # print(trajectory)
                print("i = {}".format(i))
                trajectories.append(trajectory)
                print(trajectory.joint_trajectory.points[0].positions)
                print(trajectory.joint_trajectory.points[-1].positions)
                current_robot_joint_angles = trajectory.joint_trajectory.points[-1].positions
    elif req.pose_2.position.x == req.pose_1.position.x:
        increment = (req.pose_2.position.y - req.pose_1.position.y)/10
        print("Increment_y = {}".format(increment))
        poselist = []
        temp_pose = copy.deepcopy(req.pose_1)
        poselist.append(temp_pose)
        move_group.set_planning_time(10)
        move_group.set_num_planning_attempts(10)
        trajectories = []
        temp_pose.position.y = temp_pose.position.y - increment
        poselist = []
        poselist.append(temp_pose)
        print(temp_pose)
        for i in range(10):
            temp_pose.position.y = temp_pose.position.y + increment
            print(temp_pose)
            trajectory = plan_trajectory(move_group, temp_pose, current_robot_joint_angles)
            
        #     # print(type(trajectory))
            if (len(trajectory.joint_trajectory.points) > 0):
                
                print("--------------------------")
                # print(trajectory)
                print("i = {}".format(i))
                trajectories.append(trajectory)
                print(trajectory.joint_trajectory.points[0].positions)
                print(trajectory.joint_trajectory.points[-1].positions)
                current_robot_joint_angles = trajectory.joint_trajectory.points[-1].positions
    else:
        # executing PCA based trajectory
        increment_y = (req.pose_2.position.y - req.pose_1.position.y)/10
        increment_x = (req.pose_2.position.x - req.pose_1.position.x)/10
        print("Increment_y = {}".format(increment_y))
        print("Increment_x = {}".format(increment_x))
        poselist = []
        temp_pose = copy.deepcopy(req.pose_1)
        poselist.append(temp_pose)
        move_group.set_planning_time(10)
        move_group.set_num_planning_attempts(10)
        trajectories = []
        temp_pose.position.y = temp_pose.position.y - increment_y
        temp_pose.position.x = temp_pose.position.x - increment_x
        poselist = []
        poselist.append(temp_pose)
        print(temp_pose)
        for i in range(10):
            temp_pose.position.y = temp_pose.position.y + increment_y
            temp_pose.position.x = temp_pose.position.x + increment_x
            print(temp_pose)
            trajectory = plan_trajectory(move_group, temp_pose, current_robot_joint_angles)
            
        #     # print(type(trajectory))
            if (len(trajectory.joint_trajectory.points) > 0):
                
                print("--------------------------")
                # print(trajectory)
                print("i = {}".format(i))
                trajectories.append(trajectory)
                print(trajectory.joint_trajectory.points[0].positions)
                print(trajectory.joint_trajectory.points[-1].positions)
                current_robot_joint_angles = trajectory.joint_trajectory.points[-1].positions

    for i in trajectories:
        response.trajectories.append(i)

    move_group.clear_pose_targets()
    return response

def interpolate_poses(pose1,pose2):
    increment = (pose2.position.y - pose1.position.y)/10
    print("Increment = {}".format(increment))
    poselist = []
    temp_pose = pose1
    poselist.append(temp_pose)
    for i in range(10):
        temp_pose.position.y = temp_pose.position.y + increment
        print("Temp Pose")
        print(temp_pose)
        poselist.append(temp_pose)
    return poselist

def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)

    plan = move_group.plan()

    return plan[1]

def setScene():
    global move_group
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Add table collider to MoveIt scene
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = move_group.get_planning_frame()
    p = set_pose(p, [0, 0, -0.02+0.77])
    scene.add_box("table", p, (1.2, 1.8, 0.01))

def set_pose(poseStamped, pose):
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    return poseStamped

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3_moveit_server')
    s = rospy.Service('/ur3_moveit/plantrajectory', MoverService, serviceCallback)
    print("Ready to plan")
    rospy.spin()


if __name__ =="__main__":
    moveit_server()