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
from ur3_moveit.srv import getPointCloud, getPointCloudRequest, getPointCloudResponse
from ur3_moveit.srv import GetDepthMap, GetDepthMapRequest, GetDepthMapResponse
np.set_printoptions(threshold=sys.maxsize)

def pixel_coord_np(width, height):
    """
    Pixel in homogenous coordinate
    Returns:
        Pixel coordinate:       [3, width * height]
    """
    x = np.linspace(0, width - 1, width).astype(np.int)
    y = np.linspace(0, height - 1, height).astype(np.int)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))

def intrinsic_from_fov(height, width, fov=25):
    """
    Basic Pinhole Camera Model
    intrinsic params from fov and sensor width and height in pixels
    Returns:
        K:      [4, 4]
    """
    px, py = (width / 2, height / 2)
    hfov = fov / 360. * 2. * np.pi
    fx = width / (2. * np.tan(hfov / 2.))

    vfov = 2. * np.arctan(np.tan(hfov / 2) * height / width)
    fy = height / (2. * np.tan(vfov / 2.))

    k = np.array([[fx, 0, px, 0.],
                     [0, fy, py, 0.],
                     [0, 0, 1., 0.],
                     [0., 0., 0., 1.]])
    print (k)
    return k

def Callback(req):
    print("Received req")
    getdepth = rospy.ServiceProxy("/ur3_moveit/getdepthmap", GetDepthMap)
    print("sending request")
    depth_req = GetDepthMapRequest()
    depth_req.send = True
    response_cloud = getdepth(depth_req)
    data = response_cloud.depthmap
    rospy.loginfo("Received Depth Image")
    # np_arr = np.fromstring(data.data, np.uint8)
    # print (data.data)
    # np_arr = np.frombuffer(data.data, np.uint8)
    np_arr = np.asarray(bytearray(data.data), dtype = "uint8")
    # print(np_arr)
    # image_np = cv2.imdecode(np_arr, 0)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)
    # print(image_np)
    print(image_np.shape)

    # image_single = image_np[:,:,0]
    # print(image_single.shape)
    # print(image_n p[:,:,0])
    # print("-------------------------------------------------")
    # print(image_np[:,:,1])
    # print("-------------------------------------------------")
    # print(image_np[:,:,2])
    # print("-------------------------------------------------")
    
    height, width = image_np.shape
    SCALE = 1.3631
    # SCALE = 1.443
    # k = intrinsic_from_fov(height, width, 25)
    # k_inv = np.linalg.inv(k)

    # print (k_inv)

    # pixel_coords = pixel_coord_np(width, height) 
    # cam_coords = k_inv[:3, :3] @ pixel_coords * image_np.flatten()


    # cam_coords[2,:] = cam_coords[2,:]/256.0 * SCALE
    # cam_coords[0,:] = cam_coords[0,:] * cam_coords[2,:] * SCALE
    
    # print(cam_coords.shape)
    # pd.DataFrame(image_np).to_csv("points_image.csv")
    # pd.DataFrame(cam_coords.T).to_csv("points_camera.csv")
    cx = 320
    cy = 240
    fx = 1443.4268111030583,
    fy = 1443.4268111030583,
    rows, cols= image_np.shape
    # print(image_single)
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    # print (c)
    # print (r)
    valid = (image_np >= 0) & (image_np <= 255)
    z = np.where(valid, SCALE * image_np/ 256.0, np.nan)
    # print (z.shape)
    x = np.where(valid, z * (c - cx) /fx, 0)
    # # print (x.shape)
    y = np.where(valid, z * (r - cy) /fy, 0)
    # # print (y.shape)
    w = np.ones([1,307200])
    # # tf_matrix = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,2],[0,0,0,1]])
    # tf_matrix = np.array([[1,0,0,0],[0,0,-1,2],[0,1,0,0],[0,0,0,1]])
    # tf_matrix_2 = np.array([[1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,1]])
    tf_matrix = np.array([[0,1,0,0.18],[0,0,1,-2],[1,0,0,0.03],[0,0,0,1]])
    tf_matrix_2 = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]])
    tf_matrix_3 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])

    # print(y)
    points = np.dstack((x, y, z)).transpose(2,0,1).reshape(3,-1)
    points = np.vstack((points,w))
    pd.DataFrame(points.T).to_csv("points.csv")
    # pd.DataFrame(points[:,:,1]).to_csv("y_camera.csv")
    # pd.DataFrame(points[:,:,2]).to_csv("z_camera.csv")
    # points = np.dstack((x, y, z, w)).reshape(480,4,640)
    # print(points.shape)
    # print(tf_matrix.shape)
    points_in_world_frame = np.matmul(tf_matrix, points)
    points_in_RHS = points_in_world_frame
    # points_in_RHS[1,:] = -1 * points_in_RHS[1,:]
    pd.DataFrame(points_in_RHS.T).to_csv("points_in_RHS.csv")

    points_in_ROS_frame = np.matmul(tf_matrix_2, points_in_RHS)
    # points_in_ROS_frame[2,:] = -1 * points_in_ROS_frame[2,:]
    points_in_ROS_frame = np.matmul(tf_matrix_3, points_in_ROS_frame)
    pd.DataFrame(points_in_ROS_frame.T).to_csv("points_in_ROS_frame.csv")
    # print(points_in_world_frame.shape)
    pointsXYZ = points_in_ROS_frame
    # z_world_frame = points_in_world_frame[:,:,2]
    # print(pointsXYZ.shape)
    # # pd.DataFrame(points[:,:,0]).to_csv("x.csv")
    # # pd.DataFrame(points[:,:,1]).to_csv("y.csv")
    # pd.DataFrame(points_in_world_frame.T).to_csv("world_2.csv")
    # pd.DataFrame(points_in_world_frame[:,:,1]).to_csv("y_world_2.csv")
    # pd.DataFrame(z_world_frame).to_csv("z_world_2.csv")
    # pd.DataFrame(points_in_world_frame[:,:,3]).to_csv("4thworld_2.csv")

    pointsarray = pointsXYZ.T
    # # print("sfs")
    # print(pointsarray.shape)
    pd.DataFrame(pointsarray.T).to_csv("points_array.csv")
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()

    pc2 = point_cloud2.create_cloud(header, fields, pointsarray)
    # print(pc2)
    response = getPointCloudResponse()
    response.cloud = pc2
    return response
    # print ("Published Point cloud")
    # pub.publish(pc2)

def depthListener():
    rospy.init_node("depthListener", anonymous=True)
    # rospy.Subscriber("unity_camera/depth/image_raw/compressed", CompressedImage, callback, queue_size = 1)
    s = rospy.Service("/ur3_moveit/pointcloud",getPointCloud,Callback)
    rospy.spin()

if __name__ == "__main__":
    # pub = rospy.Publisher('/unity/pointCloud', PointCloud2, queue_size= 1)

    depthListener()