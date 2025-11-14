#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import tf.transformations as tf
from std_msgs.msg import Float32MultiArray, UInt32MultiArray

# Define the camera intrinsics
K = np.array([[1063.443359375, 0, 946.2454833984375],
              [0, 1063.443359375, 534.5155029296875],
              [0, 0, 1]])

# Define the camera pose from quaternion and translation
quaternion = np.array([0.719762, -0.694085, 0.009759, -0.009378])
R = tf.quaternion_matrix(quaternion)[:3,:3]  # Rotation matrix
T = np.array([0.533481, 0.101814, 0.718489]).reshape(3, 1)  # Translation matrix

bridge = CvBridge()
depth_image = None

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    except CvBridgeError as e:
        print(e)

def coordinates_callback(msg):
    x = msg.x
    y = msg.y

    # Get depth at (x,y)
    Z = depth_image[int(y), int(x)]

    # Convert (x,y) to normalized image coordinates
    uv_homogeneous = np.array([x, y, 1]).reshape(3, 1)
    normalized_coords = np.linalg.inv(K) @ uv_homogeneous
    normalized_coords = normalized_coords * Z

    # Transform normalized coordinates to world coordinates
    world_coords = R @ normalized_coords + T

    # Publish to grasp_coordinates topic
    grasp_pub = rospy.Publisher('/grasp_coordinates', Float32MultiArray, queue_size=10)
    # header = Header(stamp=rospy.Time.now(), frame_id="world")
    grasp_point = world_coords
    msg_to_publish = Float32MultiArray()
    msg_to_publish.data = grasp_point.flatten().tolist()
    grasp_pub.publish(grasp_point)

def main():
    rospy.init_node('coordinates_converter', anonymous=True)
    
    rospy.Subscriber('/coordinates', Point, coordinates_callback)
    rospy.Subscriber('/zed_camera/depth', Image, depth_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
