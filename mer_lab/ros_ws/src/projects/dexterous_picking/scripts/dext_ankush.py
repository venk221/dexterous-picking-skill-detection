#!/usr/bin/env python

# import rospy
# import signal
# import sys
# from dexterous_picking.srv import ExecuteMotionPrimitive, ExecuteMotionPrimitiveRequest
# from dexterous_picking.srv import GetGrasp
# from recording import CameraRecorder


# # Function to handle the SIGINT signal
# def signal_handler(sig, frame):
#     rospy.loginfo("Exiting...")
#     rospy.signal_shutdown("KeyboardInterrupt")
#     sys.exit(0)

# # Set the signal handler for SIGINT
# signal.signal(signal.SIGINT, signal_handler)

# class DexterousPickingPipeline: 
#     def __init__(self):
#         rospy.wait_for_service('get_grasp')
#         rospy.wait_for_service('execute_motion_primitive')
#         rospy.loginfo('Services are ready')
#         self.get_grasp_service = rospy.ServiceProxy('get_grasp', GetGrasp)
#         self.get_grasp_rgb_service = rospy.ServiceProxy('get_grasp_rgb', GetGrasp)

#         self.get_grasp_points_service = rospy.ServiceProxy('get_grasp_points', GetGrasp)
#         self.get_grasp_points_rgb_service = rospy.ServiceProxy('get_grasp_points_rgb', GetGrasp)

#         self.execute_motion_primitive_service = rospy.ServiceProxy('execute_motion_primitive', ExecuteMotionPrimitive)

# if __name__ == '__main__':
#     rospy.init_node('dexterous_picking_pipeline_node')
#     pipeline = DexterousPickingPipeline()
#     camera_recorder = CameraRecorder()
#     # Move robot to home position
#     if pipeline.execute_motion_primitive_service(ExecuteMotionPrimitiveRequest.GO_TO_HOME).success:
#         rospy.loginfo('Robot moved to home position.')
#         while not rospy.is_shutdown():
#             inp = input("Is the object placed? Press Enter to continue with Depth mode & p for RGB mode...")
#             if inp == 'r' or inp == 'R':
#                 if pipeline.execute_motion_primitive_service(ExecuteMotionPrimitiveRequest.RESET).success:
#                     rospy.loginfo('Robot resetting.')
#                 else:
#                     rospy.logerr('Robot failed to reset.')
#                 continue

#             if inp == '':
#                 print("Depth")
#                 _ = pipeline.get_grasp_points_service()
#                 # _ = pipeline.get_grasp_points_rgb_service()
#                 get_grasp_response = pipeline.get_grasp_service()
                
#             elif inp == 'p':  # Backspace (ASCII code 8)
#                 print("RGB")
#                 # _ = pipeline.get_grasp_points_service()
#                 _ = pipeline.get_grasp_points_rgb_service()
#                 get_grasp_response = pipeline.get_grasp_rgb_service()

#             inp = input("Do you want to execute grasp? Press enter to execute... ")
#             if inp == '':
#                 print("Executing grasp...")
#             else:
#                 continue

#             # get_grasp_response = pipeline.get_grasp_service()
#             rospy.loginfo('Received grasp id: {}'.format(get_grasp_response.grasp_id))
#             if get_grasp_response.success and get_grasp_response.grasp_id != 0:
#                 camera_recorder.set_grasp_id(get_grasp_response.grasp_id)
#                 camera_recorder.start_recording()

#                 rospy.loginfo('Grasp found. Executing motion primitive...')
#                 execute_motion_primitive_response = pipeline.execute_motion_primitive_service(get_grasp_response.grasp_id)
#                 if execute_motion_primitive_response.success:
#                     rospy.loginfo('Motion primitive execute.')
#                 else:
#                     rospy.logerr('Motion primitive execution failed.')
#                     if pipeline.execute_motion_primitive_service(ExecuteMotionPrimitiveRequest.GO_TO_HOME).success:
#                         rospy.loginfo('Robot moved to home position.')
#                     else:
#                         rospy.logerr('Robot failed to move to home position.')
#             else:
#                 rospy.logerr('Grasp not found.')
            
#             camera_recorder.stop_recording()
#     else:
#         rospy.logerr('Robot failed to move to home position.')


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

import signal
import sys

from dexterous_picking.srv import GetGrasp
from recording import CameraRecorder



# Function to handle the SIGINT signal
def signal_handler(sig, frame):
    rospy.loginfo("Exiting...")
    rospy.signal_shutdown("KeyboardInterrupt")
    sys.exit(0)

# Set the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

class DexterousPickingPipeline: 
    def __init__(self):

        rospy.wait_for_service('get_grasp')
        rospy.wait_for_service('execute_motion_primitive')
        rospy.loginfo('Services are ready')
        self.get_grasp_service = rospy.ServiceProxy('get_grasp', GetGrasp)
        self.get_grasp_rgb_service = rospy.ServiceProxy('get_grasp_rgb', GetGrasp)

        self.get_grasp_points_service = rospy.ServiceProxy('get_grasp_points', GetGrasp)
        self.get_grasp_points_rgb_service = rospy.ServiceProxy('get_grasp_points_rgb', GetGrasp)

        self.execute_motion_primitive_service = rospy.ServiceProxy('execute_motion_primitive', ExecuteMotionPrimitive)
        
        self.panda = MotionPlanningInterface()
        # Initialize the gripper interface
        self.gripper = RobotHand()

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

    def reset(self):
        rospy.loginfo("Reset: Started")
        if self.gripper.release():
            rospy.loginfo("Reset: Gripper reset executed")
            self.recovery_pub.publish(ErrorRecoveryActionGoal())
            rospy.sleep(3)
            if self.panda.go_to_joint_state(self.home_pose):
                rospy.loginfo("Reset: Reached the home pose")
                return True
            else:
                rospy.logerr("Reset: Failed to reach the home pose")
        else:
            rospy.logerr("Reset: Gripper reset failed")
        return False
        


    def go_to_home(self):
        rospy.loginfo("Go to Home: Started")
        self.home_pose = self.get_pose(0.400, 0.000, 0.387, -3.118, -0.023, -0.752)
        if self.panda.go_to_pose_goal(self.home_pose):
            rospy.loginfo("Go to Home: Reached the home pose")
            if self.gripper.release():
                rospy.loginfo("Go to Home: Release executed")
                return True
            else:
                rospy.logerr("Go to Home: Release failed")
        else:
            rospy.logerr("Go to Home: Failed to reach the home pose")
        return False







if __name__ == '__main__':
    rospy.init_node('dexterous_picking_pipeline_node')
    pipeline = DexterousPickingPipeline()
    camera_recorder = CameraRecorder()
    # Move robot to home position
    pipeline.go_to_home()
