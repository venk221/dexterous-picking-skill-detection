#!/usr/bin/env python

import rospy
import signal
import sys
from dexterous_picking.srv import ExecuteMotionPrimitive, ExecuteMotionPrimitiveRequest
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

if __name__ == '__main__':
    rospy.init_node('dexterous_picking_pipeline_node')
    pipeline = DexterousPickingPipeline()
    camera_recorder = CameraRecorder()
    # Move robot to home position
    if pipeline.execute_motion_primitive_service(ExecuteMotionPrimitiveRequest.GO_TO_HOME).success:
        rospy.loginfo('Robot moved to home position.')
        while not rospy.is_shutdown():
            inp = input("Is the object placed? Press Enter to continue with Depth mode & p for RGB mode...")
            if inp == 'r' or inp == 'R':
                if pipeline.execute_motion_primitive_service(ExecuteMotionPrimitiveRequest.RESET).success:
                    rospy.loginfo('Robot resetting.')
                else:
                    rospy.logerr('Robot failed to reset.')
                continue

            if inp == '':
                print("Depth")
                _ = pipeline.get_grasp_points_service()
                get_grasp_response = pipeline.get_grasp_service()
                
            elif inp == 'p':  # Backspace (ASCII code 8)
                print("RGB")
                _ = pipeline.get_grasp_points_rgb_service()
                get_grasp_response = pipeline.get_grasp_rgb_service()

            inp = input("Do you want to execute grasp? Press enter to execute... ")
            if inp == '':
                print("Executing grasp...")
            else:
                continue

            # get_grasp_response = pipeline.get_grasp_service()
            rospy.loginfo('Received grasp id: {}'.format(get_grasp_response.grasp_id))
            if get_grasp_response.success and get_grasp_response.grasp_id != 0:
                camera_recorder.set_grasp_id(get_grasp_response.grasp_id)
                camera_recorder.start_recording()

                rospy.loginfo('Grasp found. Executing motion primitive...')
                execute_motion_primitive_response = pipeline.execute_motion_primitive_service(get_grasp_response.grasp_id)
                if execute_motion_primitive_response.success:
                    rospy.loginfo('Motion primitive execute.')
                else:
                    rospy.logerr('Motion primitive execution failed.')
                    if pipeline.execute_motion_primitive_service(ExecuteMotionPrimitiveRequest.GO_TO_HOME).success:
                        rospy.loginfo('Robot moved to home position.')
                    else:
                        rospy.logerr('Robot failed to move to home position.')
            else:
                rospy.logerr('Grasp not found.')
            
            camera_recorder.stop_recording()
    else:
        rospy.logerr('Robot failed to move to home position.')
# import rospy
# from dynamic_reconfigure.server import Server
# from dexterous_picking.cfg import MotionPrimitiveConfig
# from motion_planning_interface import MotionPlanningInterface
# from gripper_action_interface import RobotHand
# from franka_msgs.msg import ErrorRecoveryActionGoal
# import geometry_msgs.msg
# from dexterous_picking.srv import ExecuteMotionPrimitive, ExecuteMotionPrimitiveRequest, ExecuteMotionPrimitiveResponse
# from tf.transformations import quaternion_from_euler
# from std_msgs.msg import Float32MultiArray

# import signal
# import sys
# import numpy as np 

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
        
#         self.panda = MotionPlanningInterface()
#         #angle between the fingers
#         self.a = 120 * np.pi/180
#         self.rm = 40
#         self.rf = 30
#         self.l1i = 70
#         self.l2i = 40
#         # Initialize the gripper interface
#         #self.gripper = RobotHand()


    
 
#     def quaternion_multiply(self,Q0,Q1):
#         """
#         Multiplies two quaternions.
    
#         Input
#         :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
#         :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
    
#         Output
#         :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
    
#         """
#         # Extract the values from Q0
#         w0 = Q0[3]
#         x0 = Q0[0]
#         y0 = Q0[1]
#         z0 = Q0[2]
        
#         # Extract the values from Q1
#         w1 = Q1[3]
#         x1 = Q1[0]
#         y1 = Q1[1]
#         z1 = Q1[2]
        
#         # Computer the product of the two quaternions, term by term
#         Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
#         Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
#         Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
#         Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
        
#         # Create a 4 element array containing the final quaternion
#         final_quaternion = np.array([Q0Q1_x, Q0Q1_y, Q0Q1_z,Q0Q1_w])
        
#         # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
#         return final_quaternion
#     def get_pose(self, x, y, z, roll, pitch, yaw):
#             pose = geometry_msgs.msg.Pose()
#             pose.position.x = x
#             pose.position.y = y
#             pose.position.z = z
#             pose_quat = quaternion_from_euler(roll, pitch, yaw)
#             pose.orientation.x = pose_quat[0]
#             pose.orientation.y = pose_quat[1]
#             pose.orientation.z = pose_quat[2]
#             pose.orientation.w = pose_quat[3]
#             return pose
    
#     def get_pose_new(self, x, y, z, xo,yo,zo,w):
#             pose = geometry_msgs.msg.Pose()
#             pose.position.x = x
#             pose.position.y = y
#             pose.position.z = z
#             pose.orientation.x = xo
#             pose.orientation.y = yo
#             pose.orientation.z = zo
#             pose.orientation.w = w
#             return pose

#     def reset(self):
#         rospy.loginfo("Reset: Started")
#         if self.gripper.release():
#             rospy.loginfo("Reset: Gripper reset executed")
#             self.recovery_pub.publish(ErrorRecoveryActionGoal())
#             rospy.sleep(3)
#             if self.panda.go_to_joint_state(self.home_pose):
#                 rospy.loginfo("Reset: Reached the home pose")
#                 return True
#             else:
#                 rospy.logerr("Reset: Failed to reach the home pose")
#         else:
#             rospy.logerr("Reset: Gripper reset failed")
#         return False
        


#     def go_to_goal(self,goal):
#         rospy.loginfo("Go to Home: Started")
#         if self.panda.go_to_pose_goal(goal):
#             rospy.loginfo("Go to Home: Reached the home pose")
#             if self.gripper.release():
#                 rospy.loginfo("Go to Home: Release executed")
#                 return True
#             else:
#                 rospy.logerr("Go to Home: Release failed")
#         else:
#             rospy.logerr("Go to Home: Failed to reach the home pose")
#         return False

#     def inverse_kinematics(self,Ph):

#         alpha =self.a
#         Rm = self.rm
#         Rf = self.rf 

#         o = Ph[0:3]
#         a = Ph[3] * np.pi / 180
#         b = Ph[4] * np.pi / 180
#         c = Ph[5] * np.pi / 180

#         B = np.array([[0, -np.sin(a), np.sin(b)*np.cos(a)], [0, np.cos(a), np.sin(b)*np.sin(a)], [1, 0, np.cos(b)]])
#         T = np.block([[np.eye(3), np.zeros((3, 3))], [np.zeros((3, 3)), B]])
    

#         R1 = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
#         R2 = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
#         R3 = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])

#         R = R1.dot(R2).dot(R3)
#         # Calculating upper joint positions wrt. the upper coordinate frame
#         s = np.transpose(np.array([[Rm, 0, 0], [Rm*np.cos(alpha), Rm*np.sin(alpha), 0], [Rm*np.cos(-alpha), Rm*np.sin(-alpha), 0]]))

#         # Calculating lower joint positions wrt. the lower coordinate frame
#         u = np.transpose(np.array([[Rf, 0, 0], [Rf*np.cos(alpha), Rf*np.sin(alpha), 0], [Rf*np.cos(-alpha), Rf*np.sin(-alpha), 0]]))



#         l = np.zeros(3)
#         n = np.zeros((3,3))
#         for i in range(3):
#             L = o + R.dot(s[:, i]) - u[:, i]
#             l[i] = np.linalg.norm(L)
#             n[:, i] = L / l[i]

#         return l,n,R,s,T 
    
#     def Jacobian(self,n,R,s):
#         J = np.zeros((3, 6))       
#         for z in range(3):
#                 J[z, :3] = n[:, z]
#                 J[z, 3:] = np.cross((R.dot(s[:, z])), n[:, z])
#         J = np.transpose(J)

#         return J





# if __name__ == '__main__':
#     rospy.init_node('dexterous_picking_pipeline_node')
    


#     pipeline = DexterousPickingPipeline()
#     camera_recorder = CameraRecorder()




#     ##############################Robot Parameters##########################################
#     L1i, L2i = 70, 40
#     Rm, Rf = 40, 30
#     A = 120 * np.pi / 180

#     ##########################################
#     #Home position#
#     ##########################################

#     home = np.array([0.400, 0.000, 0.687, -3.118, -0.023, -0.752])
#     home_pose = pipeline.get_pose(home[0],home[1],home[2],home[3],home[4],home[5])

#     fixed_roll = home_pose.orientation.x
#     fixed_yaw = home_pose.orientation.z
#     fixed_w = home_pose.orientation.w

#     home_quat = quaternion_from_euler(home[3],home[4],home[5])

#     rospy.loginfo(home_pose)

#     # Move robot to home position
#     pipeline.go_to_goal(home_pose)

#     #  Update current end effector position 
#     curr_pose = home


#     #########################################
#     #Execute Test Case 1 for simple pick #
#     #########################################

#     rospy.loginfo("Executing Test Case 1")
    
#     pose1 = np.array([0.400, 0.000, 0.387, -3.118, -0.023, -0.752])
#     pose_one = pipeline.get_pose(pose1[0],pose1[1],pose1[2],pose1[3],pose1[4],pose1[5])

#     # Move robot to pose1
#     pipeline.go_to_goal(pose_one)

#     #Update current position 

#     curr_pose = np.array([pose1[0],pose1[1],pose1[2],pose1[3],pose1[4],pose1[5]])

#     ########################### Desired Configuration ########################################
    
#     #Desired Finger Angles
#     theta_desired = np.array([50, 50, 50])
 
#     #Desired Leg Lengths
#     lg = np.sqrt(L1i**2 + L2i**2 - 2*L1i*L2i*np.cos(theta_desired*np.pi/180))

#     #Assumed Pose of the parallel manipulator 

#     pose_m = np.array([0,0,80,0,0,0])

#     dl = 100

#     vel = [0,0,0,0,0,0]

#     #Iteravtive Jacobian

#     while dl> 0.01:

#         l,n,R,s,T = pipeline.inverse_kinematics(pose_m)
#         J = pipeline.Jacobian(n,R,s)

#         Dl = lg - l 
#         dl = np.linalg.norm(Dl)
#         velo = np.linalg.pinv(np.transpose(J).dot(T)).dot(Dl)

#         vel = vel + velo


#         #Update Hexapod Position
#         pose_m = np.array([pose_m[0]+velo[0],pose_m[1]+velo[1],pose_m[2]+velo[2],pose_m[3]+velo[3],pose_m[4]+velo[4],pose_m[5]+velo[5]])


#     #Update the Robot end effector position

#     new_quat = quaternion_from_euler(0,vel[4]*np.pi/180,0)

#     qm = pipeline.quaternion_multiply(new_quat,home_quat)

#     curr_pose = np.array([curr_pose[0]+vel[0]*0.001,curr_pose[1]+vel[1]*0.001,curr_pose[2]+vel[2]*0.001,curr_pose[3],curr_pose[4]+(60*np.pi/180),curr_pose[5]])

#     currp = pipeline.get_pose_new(curr_pose[0],curr_pose[1],curr_pose[2],qm[0],qm[1],qm[2],qm[3])

#     rospy.loginfo(vel)
#     rospy.loginfo(currp)
#     pipeline.go_to_goal(currp)


# rospy.loginfo("Completed")
