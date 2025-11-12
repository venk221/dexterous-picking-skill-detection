#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import pandas as pd
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)

def callback(data):
    rospy.loginfo("Received Depth Image")
    np_arr = np.asarray(bytearray(data.data), dtype = "uint8")
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)
    height, width = image_np.shape
    SCALE = 1.3631
    cx = 320
    cy = 240
    fx = 1443.4268111030583,
    fy = 1443.4268111030583,
    rows, cols= image_np.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = (image_np >= 0) & (image_np <= 255)
    z = np.where(valid, SCALE * image_np/ 256.0, np.nan)
    x = np.where(valid, z * (c - cx) /fx, 0)
    y = np.where(valid, z * (r - cy) /fy, 0)
    w = np.ones([1,307200])
    tf_matrix = np.array([[0,1,0,0.18],[0,0,1,-2],[1,0,0,0.03],[0,0,0,1]])
    tf_matrix_2 = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]])
    tf_matrix_3 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
    points = np.dstack((x, y, z)).transpose(2,0,1).reshape(3,-1)
    points = np.vstack((points,w))
    points_in_world_frame = np.matmul(tf_matrix, points)
    points_in_RHS = points_in_world_frame
    points_in_RHS[1,:] = -1 * points_in_RHS[1,:]
    points_in_ROS_frame = np.matmul(tf_matrix_2, points_in_RHS)
    points_in_ROS_frame[2,:] = -1 * points_in_ROS_frame[2,:]
    points_in_ROS_frame = np.matmul(tf_matrix_3, points_in_ROS_frame)
    pointsXYZ = points_in_ROS_frame
    pointsarray = pointsXYZ.T
    pd.DataFrame(pointsarray.T).to_csv("points_array.csv")
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()

    pc2 = point_cloud2.create_cloud(header, fields, pointsarray)
    print ("Published Point cloud")
    pub.publish(pc2)

def depthListener():
    rospy.init_node("depthListener", anonymous=True)
    rospy.Subscriber("unity_camera/depth/image_raw/compressed", CompressedImage, callback, queue_size = 1)
    rospy.spin()

if __name__ == "__main__":
    pub = rospy.Publisher('/unity/pointCloud', PointCloud2, queue_size= 1)
    depthListener()