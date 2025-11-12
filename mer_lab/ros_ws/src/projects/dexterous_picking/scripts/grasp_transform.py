#!/usr/bin/env python3
import sys
import rospy
import numpy as np
import tf.transformations as tf

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, UInt32MultiArray
from cv_bridge import CvBridge

from geometry_msgs.msg import Pose

# x=12.1
# y=3.7cm
# z=2cm

# Camera frame Quaternion: x=0.012947, y=-0.697296, z=-0.012428, w=0.716555
# [INFO] [1698368897.282130]: Translation: x=0.703711, y=-0.081789, z=-0.556203

# [INFO] [1698369001.943552]: Quaternion: x=-0.012947, y=0.697297, z=0.012428, w=0.716554
# [INFO] [1698369001.945396]: Translation: x=0.533537, y=0.101871, z=0.718492

fx = 1063.443359375
fy = 1063.443359375
cx = 946.2454833984375
cy = 534.5155029296875

class GraspTransform:
    def __init__(self):
        rospy.init_node('grasp_transform_node')
        rospy.loginfo("Grasp Transform Node Started")
        self.depth_sub_left = rospy.Subscriber('/zed_camera/depth', Image, self.depth_callback)
        self.pixel_coordinates_sub = rospy.Subscriber('/selected_coordinates', Pose, self.pixel_coordinates_callback)
        self.coordinate_pub = rospy.Publisher('/grasp_coordinates', Float32MultiArray, queue_size=10)
        self.current_depth = None
        self.bridge = CvBridge()
        self.has_published = False

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.width = msg.width
        self.height = msg.height

    def pixel_coordinates_callback(self, msg):
        # if self.current_depth is not None:
        #     points = msg.data
        #     transformed_points = []

        #     for i in range(0, len(points), 2):
        #         u, v = int(points[i + 1]), int(points[i])
        #         z = self.current_depth[int(v), int(u)]
        #         x = (u - cx) * z / fx
        #         y = (v - cy) * z / fy
        #         transformed_point = self.transform_toBaseLink(x, y, z)
        #         transformed_points.extend(transformed_point)

        #     print(transformed_point)
        #     msg_to_publish = Float32MultiArray()
        #     msg_to_publish.data = transformed_points
        #     if(msg_to_publish.data[2]<0.1):
        #         msg_to_publish.data[2] = 0.1
        #     msg_to_publish.data[2] += 0.1
        #     self.coordinate_pub.publish(msg_to_publish)

        # else:
        #     rospy.logwarn("No Depth received yet.")
        u, v = msg.position.x, msg.position.y

        z = self.current_depth[int(v), int(u)]
        z = msg.position.z
        
        if self.current_depth is not None:

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            
            # rospy.loginfo("Pixel Coordinates (%s, %s) -> 3D Point (%s, %s, %s)", u, v, x, y, z)
            transformed_point = self.transform_toBaseLink(x, y, z)
            # print(transformed_point)
            # rospy.loginfo(f"Panda points: {transformed_point}")
            msg_to_publish = Float32MultiArray()
            msg_to_publish.data = transformed_point.flatten().tolist()
            msg_to_publish.data[2] += 0.16
            
            if(msg_to_publish.data[2]<0.41):
                msg_to_publish.data[2] = 0.41
            # if(msg_to_publish.data[2]<0.34):
            #     msg_to_publish.data[2] = 0.34

            msg_to_publish.data[3] = msg.orientation.z #orientation for simple pick
            self.coordinate_pub.publish(msg_to_publish)

        else:
            rospy.logwarn("No Depth received yet.")

    def transform_toBaseLink(self, x, y, z):
        quaternion = np.array([0.71187, -0.702085, -0.010759, -0.012378])
        homogeneous_matrix = tf.quaternion_matrix(quaternion)
        # translation = np.array([0.421937, 0.090651, 0.657990])
        translation = np.array([0.454021, 0.025, 0.8097])
        # translation = np.array([0.42768, 0.010267, 0.8097])

        homogeneous_matrix[:3, 3] = translation

        transform_point  = homogeneous_matrix@np.array([[x],[y],[z],[1]])
        return transform_point
    
if __name__ == '__main__':
    try:
        processor = GraspTransform()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass