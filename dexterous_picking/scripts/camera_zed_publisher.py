#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import pyzed.sl as sl

def main() :
    rospy.init_node('zed_camera_publisher_node', anonymous=True)
    rospy.loginfo("ZED Camera Publisher Node Started")
    
    image_pub = rospy.Publisher('/zed_camera/image', Image, queue_size=1)
    depth_pub = rospy.Publisher('/zed_camera/depth', Image, queue_size=1)
    depth_image_pub = rospy.Publisher('/zed_camera/depth_image', Image, queue_size=1)
    
    bridge = CvBridge()
    zed = sl.Camera()

    input_type = sl.InputType()
    # init = sl.InitParameters()
    init_params = sl.InitParameters(input_t=input_type)
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL 
    init_params.coordinate_units = sl.UNIT.METER  
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.depth_minimum_distance = 0.2 
    runtime_parameters =sl.RuntimeParameters()
    runtime_parameters.enable_fill_mode = True
    # init_params.coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_configuration.resolution
    width = image_size.width
    height = image_size.height

    # res = sl.Resolution()
    # res.width = 1920
    # res.height = 1080

    # image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    # depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    image_zed = sl.Mat()
    depth_image_zed = sl.Mat()
    depth_zed = sl.Mat(width, height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)


    while not rospy.is_shutdown():
        err = zed.grab(runtime_parameters)
        if err == sl.ERROR_CODE.SUCCESS :
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU)
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH, sl.MEM.CPU)
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU)
           
            image_ocv = image_zed.get_data()
            depth_ocv = depth_zed.get_data()
            depth_image_ocv = depth_image_zed.get_data()
            # M = cv2.getRotationMatrix2D((960, 540), 180, 1)
            # rotated180 = cv2.warpAffine(depth_image_ocv, M, (1920, 1080))

            # cv2.circle(image_ocv, (800, 450), 5, (0, 254, 0), -1)  # Draw a red point on the specified coordinate
            # cv2.imshow("nsank", image_ocv)
            # cv2.waitKey(0)

            try:
                image_msg = bridge.cv2_to_imgmsg(image_ocv, encoding="bgra8")
                depth_msg = bridge.cv2_to_imgmsg(depth_ocv, encoding="32FC1")
                depth_image_msg = bridge.cv2_to_imgmsg(depth_image_ocv, encoding="bgra8")
                # cv2.imwrite("/home/merlab/mer_lab/ros_ws/src/projects/dexterous_picking/scripts/depth_image.jpg", depth_image_ocv)
                image_pub.publish(image_msg)
                depth_pub.publish(depth_msg)
                depth_image_pub.publish(depth_image_msg)
            except CvBridgeError as e:
                print(e)
    zed.close()


if __name__ == "__main__":
    main()
