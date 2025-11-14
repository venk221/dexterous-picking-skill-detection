#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from dexterous_picking.cfg import MotionPrimitiveConfig
from motion_planning_interface import MotionPlanningInterface
from gripper_action_interface import RobotHand
from franka_msgs.msg import ErrorRecoveryActionGoal
import geometry_msgs.msg
from dexterous_picking.srv import ExecuteMotionPrimitive, ExecuteMotionPrimitiveRequest, ExecuteMotionPrimitiveResponse
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray

class MotionPrimitive:
    
    def __init__(self):
        # Set the default positions
        self.home_joint_state = [0.119575, -0.751142, -0.0703775, -2.08545, -0.06018, 1.3240, 0.94463]
        self.home_pose = self.get_pose(0.400, 0.000, 0.687, -3.118, -0.023, -0.752)

        self.grasp_coords_sub = rospy.Subscriber('\grasp_coordinates', Float32MultiArray, self.grasp_coords_callback)
        try:
            grasp_data = rospy.wait_for_message('\grasp_coordinates', Float32MultiArray, timeout=10)
            if len(grasp_data.data) == 3:
                x, y, z = grasp_data.data
                self.grasp_pose = self.get_pose(x, y, z, 3.116, 0.037, -0.775)
            else:
                rospy.logwarn("Received incorrect number of values for grasp coordinates!")
        except rospy.ROSException:
            rospy.logerr("Failed to fetch grasp coordinates!")
    
        # self.simple_pick_pose = self.get_pose(0.474, 0.000, 0.198, 3.116, 0.037, -0.775)
        self.slide_to_edge_p1 = self.get_pose(0.301, -0.181, 0.302, -2.927, -0.290, -2.468)
        self.slide_to_edge_p2 = self.get_pose(0.301, -0.424, 0.290, -3.078, -0.062, -2.338)
        self.slide_to_edge_p3 = self.get_pose(0.301, -0.541, 0.214, 1.922, 0.771, -3.021)
        self.push_to_horizontal_p1 = self.get_pose(0.474, 0.000, 0.228, 3.116, 0.037, -0.775)
        self.push_to_horizontal_p2 = self.get_pose(0.474, 0.000, 0.198, 3.116, 0.037, -0.775)
        self.push_to_vertical_p1 = self.get_pose(0.291, 0.043, 0.153, -2.064, -0.628, 0.275)
        self.push_to_vertical_p2 = self.get_pose(0.291, 0.067, 0.126, -1.845, -0.688, 0.226)
        self.flip_pose = self.get_pose(0.500, 0.000, 0.316, 3.116, 0.037, -0.775)
        self.simple_pick_close_amt = 0.35
        self.push_to_horizontal_init_amt = 0.12
        self.push_to_horizontal_close_amt = 0.35
        self.push_to_vertical_init_amt = 0.2
        self.push_to_vertical_close_amt = 0.2
        self.slide_to_edge_init_amt = 0.2
        self.slide_to_edge_close_amt = 0.4
        self.flip_grasp_amt = 0.25
        self.flip_flip_amt = 0.4

        # initialize the motion planning interface
        self.panda = MotionPlanningInterface()
        # Initialize the gripper interface
        self.gripper = RobotHand()
        # Initialize Dynamic Reconfigure Server
        # self.dyn_reconf_server = Server(MotionPrimitiveConfig, self.dyn_reconf_callback)
        # Initialize the ros service server
        self.motion_primitive_service = rospy.Service('execute_motion_primitive', ExecuteMotionPrimitive, self.motion_primitive_callback)
        self.recovery_pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)

    def get_pose(self, x, y, z, roll, pitch, yaw):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose_quat = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = pose_quat[0]
        pose.orientation.y = pose_quat[1]
        pose.orientation.z = pose_quat[2]
        pose.orientation.w = pose_quat[3]
        return pose

    def simple_pick(self):
        rospy.loginfo("Simple Pick: Started")
        if self.gripper.simple_pick_initialize():
            rospy.loginfo("Simple Pick: Grasp initialized")
            if self.panda.go_to_pose_goal(self.simple_pick_pose):
                rospy.loginfo("Simple Pick: Reached the pick pose")
                if self.gripper.close(self.simple_pick_close_amt):
                    rospy.loginfo("Simple Pick: Grasp closed")
                    if self.panda.go_to_joint_state(self.home_joint_state):
                        rospy.loginfo("Simple Pick: Reached the home pose")
                        if self.gripper.release():
                            rospy.loginfo("Simple Pick: Release executed")
                            return True
                        else:
                            rospy.logerr("Simple Pick: Release failed")
                    else:
                        rospy.logerr("Simple Pick: Failed to reach the home pose")
                else:
                    rospy.logerr("Simple Pick: Grasp close failed")
            else:
                rospy.logerr("Simple Pick: Failed to reach the pick pose")
        else:
            rospy.logerr("Simple Pick: Grasp initialization failed")
        return False

    def slide_to_edge(self):
        rospy.loginfo("Slide to Edge: Started")
        if self.gripper.slide_to_edge_initialize(self.slide_to_edge_init_amt):
            rospy.loginfo("Slide to Edge: Grasp initialized")
            if self.panda.go_to_pose_goal(self.slide_to_edge_p1):
                rospy.loginfo("Slide to Edge: Reached the first pose")
                if self.panda.plan_cartesian_path([self.slide_to_edge_p2]):
                    rospy.loginfo("Slide to Edge: Reached the second pose")
                    if self.panda.plan_cartesian_path([self.slide_to_edge_p3]):
                        rospy.loginfo("Slide to Edge: Reached the third pose")
                        if self.gripper.slide_to_edge_close(self.slide_to_edge_close_amt):
                            rospy.loginfo("Slide to Edge: Grasp closed")
                            if self.panda.go_to_joint_state(self.home_joint_state):
                                rospy.loginfo("Slide to Edge: Reached the home pose")
                                if self.gripper.release():
                                    rospy.loginfo("Slide to Edge: Release executed")
                                    return True
                                else:
                                    rospy.logerr("Slide to Edge: Release failed")
                            else:
                                rospy.logerr("Slide to Edge: Failed to reach the home pose")
                        else:
                            rospy.logerr("Slide to Edge: Grasp close failed")
                    else:
                        rospy.logerr("Slide to Edge: Failed to reach the third pose")
                else:
                    rospy.logerr("Slide to Edge: Failed to reach the second pose")
            else:
                rospy.logerr("Slide to Edge: Failed to reach the first pose")
        else:
            rospy.logerr("Slide to Edge: Grasp initialization failed")
        return False

    def push_to_horizontal(self):
        rospy.loginfo("Push to Horizontal: Started")
        if self.gripper.push_to_horizontal_initialize(self.push_to_horizontal_init_amt):
            rospy.loginfo("Push to Horizontal: Grasp initialized")
            if self.panda.go_to_pose_goal(self.push_to_horizontal_p1):
                rospy.loginfo("Push to Horizontal: Reached the first pose")
                if self.panda.plan_cartesian_path([self.push_to_horizontal_p2]):
                    rospy.loginfo("Push to Horizontal: Reached the second pose")
                    if self.gripper.close(self.push_to_horizontal_close_amt):
                        rospy.loginfo("Push to Horizontal: Grasp closed")
                        if self.panda.go_to_joint_state(self.home_joint_state):
                            rospy.loginfo("Push to Horizontal: Reached the home pose")
                            if self.gripper.release():
                                rospy.loginfo("Push to Horizontal: Release executed")
                                return True
                            else:
                                rospy.logerr("Push to Horizontal: Release failed")
                        else:
                            rospy.logerr("Push to Horizontal: Failed to reach the home pose")
                    else:
                        rospy.logerr("Push to Horizontal: Grasp close failed")
                else:
                    rospy.logerr("Push to Horizontal: Failed to reach the second pose")
            else:
                rospy.logerr("Push to Horizontal: Failed to reach the first pose")
        else:
            rospy.logerr("Push to Horizontal: Grasp initialization failed")
        return False
        
    def push_to_vertical(self):
        rospy.loginfo("Push to Vertical: Started")
        if self.gripper.push_to_vertical_initialize(self.push_to_vertical_init_amt):
            rospy.loginfo("Push to Vertical: Grasp initialized")
            if self.panda.go_to_pose_goal(self.push_to_vertical_p1):
                rospy.loginfo("Push to Vertical: Reached the first pose")
                if self.panda.plan_cartesian_path([self.push_to_vertical_p2]):
                    rospy.loginfo("Push to Vertical: Reached the second pose")
                    if self.gripper.close(self.push_to_vertical_close_amt):
                        rospy.loginfo("Push to Vertical: Grasp closed")
                        if self.panda.go_to_joint_state(self.home_joint_state):
                            rospy.loginfo("Push to Vertical: Reached the home pose")
                            if self.gripper.release():
                                rospy.loginfo("Push to Vertical: Release executed")
                                return True
                            else:
                                rospy.logerr("Push to Vertical: Release failed")
                        else:
                            rospy.logerr("Push to Vertical: Failed to reach the home pose")
                    else:
                        rospy.logerr("Push to Vertical: Grasp close failed")
                else:
                    rospy.logerr("Push to Vertical: Failed to reach the second pose")
            else:
                rospy.logerr("Push to Vertical: Failed to reach the first pose")
        else:
            rospy.logerr("Push to Vertical: Grasp initialization failed")
        return False

    def flip(self):
        rospy.loginfo("Flip: Started")
        if self.gripper.flip_initialize():
            rospy.loginfo("Flip: Grasp initialized")
            if self.panda.go_to_pose_goal(self.flip_pose):
                rospy.loginfo("Flip: Reached the flip pose")
                if self.gripper.flip_execute(self.flip_grasp_amt, self.flip_flip_amt):
                    rospy.loginfo("Flip: Flip executed")
                    if self.panda.go_to_joint_state(self.home_joint_state):
                        rospy.loginfo("Flip: Reached the home pose")
                        if self.gripper.release():
                            rospy.loginfo("Flip: Release executed")
                            return True
                        else:
                            rospy.logerr("Flip: Release failed")
                    else:
                        rospy.logerr("Flip: Failed to reach the home pose")
                else:
                    rospy.logerr("Flip: Flip execution failed")
            else:
                rospy.logerr("Flip: Failed to reach the flip pose")
        else:
            rospy.logerr("Flip: Grasp initialization failed")
        return False

    def go_to_home(self):
        rospy.loginfo("Go to Home: Started")
        if self.panda.go_to_joint_state(self.home_joint_state):
            rospy.loginfo("Go to Home: Reached the home pose")
            if self.gripper.release():
                rospy.loginfo("Go to Home: Release executed")
                return True
            else:
                rospy.logerr("Go to Home: Release failed")
        else:
            rospy.logerr("Go to Home: Failed to reach the home pose")
        return False
    
    def reset(self):
        rospy.loginfo("Reset: Started")
        if self.gripper.release():
            rospy.loginfo("Reset: Gripper reset executed")
            self.recovery_pub.publish(ErrorRecoveryActionGoal())
            rospy.sleep(3)
            if self.panda.go_to_joint_state(self.home_joint_state):
                rospy.loginfo("Reset: Reached the home pose")
                return True
            else:
                rospy.logerr("Reset: Failed to reach the home pose")
        else:
            rospy.logerr("Reset: Gripper reset failed")
        return False
        
    def dyn_reconf_callback(self, config, level):
        rospy.loginfo("""Reconfiugre Request: {home_pose_x}, {home_pose_y}, {home_pose_z},\
            {simple_pick_pose_x}, {simple_pick_pose_y}, {simple_pick_pose_z}, {simple_pick_close_amount}, \
            {slide_to_edge_p1_x}, {slide_to_edge_p1_y}, {slide_to_edge_p1_z}, {slide_to_edge_initialize_amount}, \
            {slide_to_edge_p2_x}, {slide_to_edge_p2_y}, {slide_to_edge_p2_z}, \
            {slide_to_edge_p3_x}, {slide_to_edge_p3_y}, {slide_to_edge_p3_z}, {slide_to_edge_close_amount}, \
            {push_to_horizontal_p1_x}, {push_to_horizontal_p1_y}, {push_to_horizontal_p1_z}, {push_to_horizontal_initialize_amount}, \
            {push_to_horizontal_p2_x}, {push_to_horizontal_p2_y}, {push_to_horizontal_p2_z}, {push_to_horizontal_close_amount}, \
            {push_to_vertical_p1_x}, {push_to_vertical_p1_y}, {push_to_vertical_p1_z}, {push_to_vertical_initialize_amount}, \
            {push_to_vertical_p2_x}, {push_to_vertical_p2_y}, {push_to_vertical_p2_z}, {push_to_vertical_close_amount}, \
            {flip_pose_x}, {flip_pose_y}, {flip_pose_z}, {flip_grasp_amount}, {flip_flip_amount}""".format(**config))
        self.home_pose.position.x = config["home_pose_x"]
        self.home_pose.position.y = config["home_pose_y"]
        self.home_pose.position.z = config["home_pose_z"]
        self.simple_pick_pose.position.x = config["simple_pick_pose_x"]
        self.simple_pick_pose.position.y = config["simple_pick_pose_y"]
        self.simple_pick_pose.position.z = config["simple_pick_pose_z"]
        self.simple_pick_close_amt = config["simple_pick_close_amount"]
        self.slide_to_edge_p1.position.x = config["slide_to_edge_p1_x"]
        self.slide_to_edge_p1.position.y = config["slide_to_edge_p1_y"]
        self.slide_to_edge_p1.position.z = config["slide_to_edge_p1_z"]
        self.slide_to_edge_p2.position.x = config["slide_to_edge_p2_x"]
        self.slide_to_edge_p2.position.y = config["slide_to_edge_p2_y"]
        self.slide_to_edge_p2.position.z = config["slide_to_edge_p2_z"]
        self.slide_to_edge_p3.position.x = config["slide_to_edge_p3_x"]
        self.slide_to_edge_p3.position.y = config["slide_to_edge_p3_y"]
        self.slide_to_edge_p3.position.z = config["slide_to_edge_p3_z"]
        self.slide_to_edge_init_amt = config["slide_to_edge_initialize_amount"]
        self.slide_to_edge_close_amt = config["slide_to_edge_close_amount"]
        self.push_to_horizontal_p1.position.x = config["push_to_horizontal_p1_x"]
        self.push_to_horizontal_p1.position.y = config["push_to_horizontal_p1_y"]
        self.push_to_horizontal_p1.position.z = config["push_to_horizontal_p1_z"]
        self.push_to_horizontal_p2.position.x = config["push_to_horizontal_p2_x"]
        self.push_to_horizontal_p2.position.y = config["push_to_horizontal_p2_y"]
        self.push_to_horizontal_p2.position.z = config["push_to_horizontal_p2_z"]
        self.push_to_horizontal_init_amt = config["push_to_horizontal_initialize_amount"]
        self.push_to_horizontal_close_amt = config["push_to_horizontal_close_amount"]
        self.push_to_vertical_p1.position.x = config["push_to_vertical_p1_x"]
        self.push_to_vertical_p1.position.y = config["push_to_vertical_p1_y"]
        self.push_to_vertical_p1.position.z = config["push_to_vertical_p1_z"]
        self.push_to_vertical_p2.position.x = config["push_to_vertical_p2_x"]
        self.push_to_vertical_p2.position.y = config["push_to_vertical_p2_y"]
        self.push_to_vertical_p2.position.z = config["push_to_vertical_p2_z"]
        self.push_to_vertical_init_amt = config["push_to_vertical_initialize_amount"]
        self.push_to_vertical_close_amt = config["push_to_vertical_close_amount"]
        self.flip_pose.position.x = config["flip_pose_x"]
        self.flip_pose.position.y = config["flip_pose_y"]
        self.flip_pose.position.z = config["flip_pose_z"]
        self.flip_grasp_amt = config["flip_grasp_amount"]
        self.flip_flip_amt = config["flip_flip_amount"]
        return config

    def motion_primitive_callback(self, req):
        rospy.loginfo("Motion Primitive: Received request")
        success = False
        if req.grasp_id == ExecuteMotionPrimitiveRequest.NO_OBJECT_FOUND:
            rospy.loginfo("Motion Primitive: No object found")
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.SIMPLE_PICK:
            success = self.simple_pick()
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.SLIDE_TO_EDGE:
            success = self.slide_to_edge()
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.PUSH_TO_HORIZONTAL:
            success = self.push_to_horizontal()
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.PUSH_TO_VERTICAL:
            success = self.push_to_vertical()
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.FLIP:
            success = self.flip()
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.GO_TO_HOME:
            success = self.go_to_home()
        elif req.grasp_id == ExecuteMotionPrimitiveRequest.RESET:
            success = self.reset()
        else:
            rospy.logerr('Motion primitive not implemented')
        return ExecuteMotionPrimitiveResponse(success)

if __name__ == '__main__':
    rospy.init_node('motion_primitive_node')
    motion_primitive = MotionPrimitive()
    while not rospy.is_shutdown():
        rospy.spin()