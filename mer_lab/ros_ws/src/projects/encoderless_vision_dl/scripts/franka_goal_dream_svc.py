#!/usr/bin/env python3
# license removed for brevity
import argparse
import os

import numpy as np
import sys
import rospy
import cv2
from PIL import Image as PILImage
from pyrr import Quaternion
from ruamel.yaml import YAML
import torch
import webcolors
import time
from std_msgs.msg import Bool, Float64MultiArray, Int64, Float64
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty
import tf2_ros as tf2
import tf
import copy
import dream
from encoderless_vision_dl.srv import dream_goal_img, dream_goal_imgResponse
from os.path import expanduser


# Declaring cvBridge for cv to ros conversion and vice versa

image_topic = "/camera/color/image_raw"

# ROS topic for listening to camera intrinsics
camera_info_topic = "/camera/color/camera_info"

# ROS service for sending request to capture frame
capture_frame_service_topic = "/dream/capture_frame"

# ROS service for sending request to clear buffer
clear_buffer_service_topic = "/dream/clear_buffer"

# ROS topics for outputs
topic_out_net_input_image = "/dream/net_input_image"
topic_out_keypoint_overlay = "/dream/keypoint_overlay"
topic_out_belief_maps = "/dream/belief_maps"
topic_out_keypoint_belief_overlay = "/dream/keypoint_belief_overlay"
topic_out_keypoint_names = "/dream/keypoint_names"
topic_out_keypoint_frame_overlay = "/dream/keypoint_frame_overlay"

# ROS frames for the output of DREAM
# tform_out_basename is now set by the user - previously was 'dream/base_frame'
tform_out_childname = "dream/camera_rgb_frame"

kf1 = mf.KalmanFilter(100, 10, 10, 7, 7)
kf2 = mf.KalmanFilter(100, 10, 10, 7, 7)
kf3 = mf.KalmanFilter(100, 10, 10, 7, 7)
kf4 = mf.KalmanFilter(100, 10, 10, 7, 7)
kf5 = mf.KalmanFilter(100, 10, 10, 7, 7)
kf6 = mf.KalmanFilter(100, 10, 10, 7, 7)
kf7 = mf.KalmanFilter(100, 10, 10, 7, 7)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class DreamInferenceROS:
    def __init__(self, args, single_frame_mode=True, compute_2d_to_3d_transform=False):
        """Initialize inference engine.

            single_frame_mode:  Set this to True.  (Appears to be some sort of future-proofing by Tim.)
        """
        self.cv_image = None
        self.camera_K = None
        self.j = 0
        self.i = 0
        self.k = 1
        self.control_flag = False
        self.marker_flag = [None, None, None, None, None, None, None]
        self.start_flag = False
        self.cx1_list = [0.0]
        self.cy1_list = [0.0]
        self.cx2_list = [0.0]
        self.cy2_list = [0.0]
        self.cx3_list = [0.0]
        self.cy3_list = [0.0]
        self.cx4_list = [0.0]
        self.cy4_list = [0.0]
        self.cx5_list = [0.0]
        self.cy5_list = [0.0]
        self.cx6_list = [0.0]
        self.cy6_list = [0.0]
        self.cx7_list = [0.0]
        self.cy7_list = [0.0]
        self.measured1_list = []
        self.measured2_list = []
        self.measured3_list = []
        self.measured4_list = []
        self.measured5_list = []
        self.measured6_list = []
        self.measured7_list = []
        self.updated1 = None
        self.updated2 = None
        self.updated3 = None
        self.updated4 = None
        self.updated5 = None
        self.updated6 = None
        self.updated7 = None
        self.track_window = None
        self.termination = None
        self.executed = False
        self.corrected_x = None
        


        # TODO: -- continuous mode produces a TF at each frame
        # not continuous mode allows for several frames before producing an estimate
        self.single_frame_mode = single_frame_mode
        self.capture_frame_srv = rospy.Service(
            capture_frame_service_topic, Empty, self.on_capture_frame
        )
        self.clear_buffer_srv = rospy.Service(
            clear_buffer_service_topic, Empty, self.on_clear_buffer
        )
        self.kp_projs_raw_buffer = np.array([])
        self.kp_positions_buffer = np.array([])
        self.pnp_solution_found = False
        self.capture_frame_max_kps = True

        self.compute_2d_to_3d_transform = compute_2d_to_3d_transform

        # Create subscribers
        self.image_sub = rospy.Subscriber(
            image_topic, Image, self.kp_generator_img, queue_size=1
        )
        self.bridge = CvBridge()
        self.dream_img_service = rospy.Service("dream_img_service_response", dream_goal_img, self.dream_img_service)

        # Input argument handling
        assert os.path.exists(
            args.input_params_path
        ), 'Expected input_params_path "{}" to exist, but it does not.'.format(
            args.input_params_path
        )

        if args.input_config_path:
            input_config_path = args.input_config_path
        else:
            # Use params filepath to infer the config filepath
            input_config_path = os.path.splitext(args.input_params_path)[0] + ".yaml"

        assert os.path.exists(
            input_config_path
        ), 'Expected input_config_path "{}" to exist, but it does not.'.format(
            input_config_path
        )

        # Create parser
        print("# Opening config file:  {} ...".format(input_config_path))
        data_parser = YAML(typ="safe")

        with open(input_config_path, "r") as f:
            network_config = data_parser.load(f)

        # Overwrite GPU
        # If nothing is specified at the command line, None is the default, which uses all GPUs
        # TBD - think about a better way of doing this
        network_config["training"]["platform"]["gpu_ids"] = args.gpu_ids

        # Load network
        print("# Creating network...")
        self.dream_network = dream.create_network_from_config_data(network_config)

        print(
            "Loading network with weights from:  {} ...".format(args.input_params_path)
        )
        self.dream_network.model.load_state_dict(torch.load(args.input_params_path))
        self.dream_network.enable_evaluation()

        # Use image preprocessing specified by config by default, unless user specifies otherwise
        if args.image_preproc_override:
            self.image_preprocessing = args.image_preproc_override
        else:
            self.image_preprocessing = self.dream_network.image_preprocessing()

        # Output names used to look up keypoints in TF tree
        self.keypoint_tf_frames = self.dream_network.ros_keypoint_frames
        print("ROS keypoint frames: {}".format(self.keypoint_tf_frames))

        # Define publishers
        self.net_input_image_pub = rospy.Publisher(
            topic_out_net_input_image, Image, queue_size=1
        )
        self.image_overlay_pub = rospy.Publisher(
            topic_out_keypoint_overlay, Image, queue_size=1
        )
        self.belief_image_pub = rospy.Publisher(
            topic_out_belief_maps, Image, queue_size=1
        )
        self.kp_belief_overlay_pub = rospy.Publisher(
            topic_out_keypoint_belief_overlay, Image, queue_size=1
        )
        self.kp_frame_overlay_pub = rospy.Publisher(
            topic_out_keypoint_frame_overlay, Image, queue_size=1
        )

        # Store the base frame for the TF lookup
        self.base_tf_frame = args.base_frame

        # Define TFs
        self.tfBuffer = tf2.Buffer()
        self.tf_broadcaster = tf2.TransformBroadcaster()
        self.listener = tf2.TransformListener(self.tfBuffer)
        self.camera_pose_tform = TransformStamped()

        self.camera_pose_tform.header.frame_id = self.base_tf_frame
        self.camera_pose_tform.child_frame_id = tform_out_childname

        # Subscriber for camera intrinsics topic
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic, CameraInfo, self.on_camera_info, queue_size=1
        )

        # Verbose mode
        self.verbose = args.verbose

    def on_capture_frame(self, req):
        print("Capturing frame.")
        found_kp_projs_net_input = dream_ros.process_image()
        print(found_kp_projs_net_input)
        (
            kp_projs_raw_good_sample,
            kp_positions_good_sample,
        ) = dream_ros.keypoint_correspondences(found_kp_projs_net_input)
        if self.capture_frame_max_kps and kp_projs_raw_good_sample is not None:
            n_found_keypoints = kp_projs_raw_good_sample.shape[0]
            if n_found_keypoints != self.dream_network.n_keypoints:
                print(
                    "Only found {} keypoints -- not continuing. Try again.".format(
                        n_found_keypoints
                    )
                )
                return []
        if (
            kp_projs_raw_good_sample is not None
            and kp_positions_good_sample is not None
        ):
            dream_ros.solve_pnp_buffer(
                kp_projs_raw_good_sample, kp_positions_good_sample
            )
        return []

    def on_clear_buffer(self, req):
        print("Clearing frame buffer.")
        self.kp_projs_raw_buffer = np.array([])
        self.kp_positions_buffer = np.array([])
        self.pnp_solution_found = False
        return []

    def on_image(self, image):
        self.cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")       

    def on_camera_info(self, camera_info):
        # Create camera intrinsics matrix
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        self.camera_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])



    def process_image(self):
        """Performs inference on the image most recently captured.

        Input (none)
        self.cv_image:  Holds the image to be processed.

        Returns
        detected_keypoints:  Array of 2D keypoint coords wrt original image, possibly including missing keypoints
        """

        if self.cv_image is None:
            return

        # Determine if we need debug content from single-image inference based on subscribers
        num_connect_belief_image_pub = self.belief_image_pub.get_num_connections()
        num_connect_net_input_image_pub = self.net_input_image_pub.get_num_connections()
        num_connect_image_overlay_pub = self.image_overlay_pub.get_num_connections()
        num_connect_kp_belief_overlay_pub = (
            self.kp_belief_overlay_pub.get_num_connections()
        )

        if (
            num_connect_belief_image_pub > 0
            or num_connect_net_input_image_pub > 0
            or num_connect_image_overlay_pub > 0
            or num_connect_kp_belief_overlay_pub > 0
        ):
            debug_inference = True
        else:
            debug_inference = False

        # Detect keypoints from single-image inference
        image_raw = PILImage.fromarray(self.cv_image)
        detection_result = self.dream_network.keypoints_from_image(
            image_raw,
            image_preprocessing_override=self.image_preprocessing,
            debug=debug_inference,
        )
        detected_keypoints = detection_result["detected_keypoints"]

        # Publish debug topics - only do this if something is subscribed
        # TODO: clean up so some of the intermediate processing isn't duplicated
        if num_connect_belief_image_pub > 0:
            belief_maps = detection_result["belief_maps"]
            belief_images = dream.image_proc.images_from_belief_maps(
                belief_maps, normalization_method=6
            )
            belief_image_mosaic = dream.image_proc.mosaic_images(
                belief_images, rows=1, cols=len(belief_images), inner_padding_px=5
            )
            cv_belief_image = np.array(belief_image_mosaic)
            cv_belief_image = cv_belief_image[:, :, ::-1].copy()
            belief_msg = self.bridge.cv2_to_imgmsg(cv_belief_image, encoding="bgr8")
            self.belief_image_pub.publish(belief_msg)

        if num_connect_net_input_image_pub > 0:
            net_input_image = detection_result["image_rgb_net_input"]
            cv_input_image = np.array(net_input_image)
            cv_input_image = cv_input_image[:, :, ::-1].copy()
            net_input_image_msg = self.bridge.cv2_to_imgmsg(
                cv_input_image, encoding="bgr8"
            )
            self.net_input_image_pub.publish(net_input_image_msg)

        if num_connect_image_overlay_pub > 0:
            # TODO: fix color cycle
            if self.dream_network.n_keypoints == 7:
                point_colors = [
                    "red",
                    "blue",
                    "green",
                    "yellow",
                    "black",
                    "cyan",
                    "white",
                ]
            else:
                point_colors = "red"

            image_overlay = dream.image_proc.overlay_points_on_image(
                image_raw,
                detected_keypoints,
                self.dream_network.friendly_keypoint_names,
                annotation_color_dot=point_colors,
                annotation_color_text=point_colors,
            )
            cv_image_overlay = np.array(image_overlay)
            cv_image_overlay = cv_image_overlay[:, :, ::-1].copy()
            image_overlay_msg = self.bridge.cv2_to_imgmsg(
                cv_image_overlay, encoding="bgr8"
            )
            self.image_overlay_pub.publish(image_overlay_msg)

        if num_connect_kp_belief_overlay_pub > 0:
            image_raw_resolution = (self.cv_image.shape[1], self.cv_image.shape[0])
            net_input_resolution = detection_result["image_rgb_net_input"].size
            belief_maps = detection_result["belief_maps"]
            flattened_belief_tensor = belief_maps.sum(dim=0)
            flattened_belief_image = dream.image_proc.image_from_belief_map(
                flattened_belief_tensor, colormap="hot", normalization_method=6
            )
            flattened_belief_image_netin = dream.image_proc.convert_image_to_netin_from_netout(
                flattened_belief_image, net_input_resolution
            )
            flattened_belief_image_raw = dream.image_proc.inverse_preprocess_image(
                flattened_belief_image_netin,
                image_raw_resolution,
                self.image_preprocessing,
            )
            flattened_belief_image_raw_blend = PILImage.blend(
                image_raw, flattened_belief_image_raw, alpha=0.5
            )

            # Overlay keypoints
            # TODO: fix color cycle
            if self.dream_network.n_keypoints == 7:
                point_colors = [
                    "red",
                    "blue",
                    "green",
                    "yellow",
                    "black",
                    "cyan",
                    "white",
                ]
            else:
                point_colors = "red"

            kp_belief_overlay = dream.image_proc.overlay_points_on_image(
                flattened_belief_image_raw_blend,
                detected_keypoints,
                self.dream_network.friendly_keypoint_names,
                annotation_color_dot=point_colors,
                annotation_color_text=point_colors,
            )
            cv_kp_belief_overlay = np.array(kp_belief_overlay)
            cv_kp_belief_overlay = cv_kp_belief_overlay[:, :, ::-1].copy()
            kp_belief_overlay_msg = self.bridge.cv2_to_imgmsg(
                cv_kp_belief_overlay, encoding="bgr8"
            )
            self.kp_belief_overlay_pub.publish(kp_belief_overlay_msg)

        return detected_keypoints

    def keypoint_correspondences(self, detected_kp_projs):
        """Convert 2D keypoint coords to (2D, 3D) pairs of correspondences.

        Input:
        detected_kp_projs:  Array of 2D keypoint coords wrt original image, possibly including missing keypoints

        Returns:
        kp_projs_raw_good_sample:  Array of 2D keypoint coords wrt original image
        kp_positions_good_sample:  Array of 3D keypoint coords
        """

        if not self.compute_2d_to_3d_transform:
            return None, None

        all_kp_positions = []

        for i in range(len(self.keypoint_tf_frames)):
            keypoint_tf_frame = self.keypoint_tf_frames[i]
            if self.verbose:
                print(
                    "Attempting transform lookup between {} and {}...".format(
                        self.base_tf_frame, keypoint_tf_frame
                    )
                )
            try:
                tform = self.tfBuffer.lookup_transform(
                    self.base_tf_frame, keypoint_tf_frame, rospy.Time()
                )
                this_pos = np.array(
                    [
                        tform.transform.translation.x,
                        tform.transform.translation.y,
                        tform.transform.translation.z,
                    ]
                )
                all_kp_positions.append(this_pos)

            except tf2.TransformException as e:
                print("TF Exception: {}".format(e))
                return None, None

        # Now determine which to keep
        kp_projs_raw_good_sample = []
        kp_positions_good_sample = []
        for this_kp_proj_est, this_kp_position in zip(
            detected_kp_projs, all_kp_positions
        ):
            if (
                this_kp_proj_est is not None
                and this_kp_proj_est[0]
                and this_kp_proj_est[1]
                and not (this_kp_proj_est[0] < -999.0 and this_kp_proj_est[1] < 0.999)
            ):
                # This keypoint is defined to exist within the image frame, so we keep it
                kp_projs_raw_good_sample.append(this_kp_proj_est)
                kp_positions_good_sample.append(this_kp_position)

        kp_projs_raw_good_sample = np.array(kp_projs_raw_good_sample)
        kp_positions_good_sample = np.array(kp_positions_good_sample)

        return kp_projs_raw_good_sample, kp_positions_good_sample

    def solve_pnp_buffer(self, candidate_kp_projs_raw, candidate_kp_positions):

        if self.camera_K is None:
            self.pnp_solution_found = False
            return

        kp_projs_raw_to_try = np.array(
            self.kp_projs_raw_buffer.tolist() + candidate_kp_projs_raw.tolist()
        )
        
        self.key_points = kp_projs_raw_to_try
        
        kp_positions_to_try = np.array(
            self.kp_positions_buffer.tolist() + candidate_kp_positions.tolist()
        )

        if self.verbose:
            print("\nSolving for PNP... ~~~~~~~~~~~~~~~~~~~~")
            print("2D Detected KP projections in raw:")
            print(kp_projs_raw_to_try)

            print("3D KP positions:")
            print(kp_positions_to_try)

        pnp_retval, tvec, quat = dream.geometric_vision.solve_pnp(
            kp_positions_to_try, kp_projs_raw_to_try, self.camera_K
        )

        if pnp_retval:
            self.pnp_solution_found = True

            if self.verbose:
                print("Camera-from-robot pose, found by PNP:")
                print(tvec)
                print(quat)

            # Update transform
            T_cam_from_robot = tf.transformations.quaternion_matrix(quat)
            T_cam_from_robot[:3, -1] = tvec
            T_robot_from_cam = tf.transformations.inverse_matrix(T_cam_from_robot)

            robot_from_cam_pos = T_robot_from_cam[:3, -1]

            R_robot_from_cam = T_robot_from_cam[:3, :3]
            temp = tf.transformations.identity_matrix()
            temp[:3, :3] = R_robot_from_cam
            robot_from_cam_quat = tf.transformations.quaternion_from_matrix(temp)

            # Update transform msg for publication
            self.camera_pose_tform.transform.translation.x = robot_from_cam_pos[0]
            self.camera_pose_tform.transform.translation.y = robot_from_cam_pos[1]
            self.camera_pose_tform.transform.translation.z = robot_from_cam_pos[2]

            self.camera_pose_tform.transform.rotation.x = robot_from_cam_quat[0]
            self.camera_pose_tform.transform.rotation.y = robot_from_cam_quat[1]
            self.camera_pose_tform.transform.rotation.z = robot_from_cam_quat[2]
            self.camera_pose_tform.transform.rotation.w = robot_from_cam_quat[3]

            # Save buffer since we got a PNP solution if we're not in single-frame mode
            if not self.single_frame_mode:
                self.kp_projs_raw_buffer = kp_projs_raw_to_try
                self.kp_positions_buffer = kp_positions_to_try
                print(
                    "Adding to buffer! New buffer size: {}".format(
                        self.kp_positions_buffer.shape[0]
                    )
                )

        else:
            print("PnP failed to provide a solution.")
            self.pnp_solution_found = False

    def dream_kalman_estimate(self, img, feature):
        yellow = (0, 255, 255)
        green = (0, 255, 0)
        red = (0, 0, 255)
        blue = (255, 0, 0)

        # # Detect feature points
        # joint1
        cx1 = feature[0][0]
        cy1 = feature[0][1]

        self.cx1_list = np.append(cx1, self.cx1_list)
        self.cy1_list = np.append(cy1, self.cy1_list)

        dx1 = self.cx1_list[0]-self.cx1_list[1]
        dy1 = self.cy1_list[0]-self.cy1_list[1]

        vx1 = dx1
        vy1 = dy1

        measured1 = np.array([[np.float32(cx1)], [np.float32(cy1)], [np.float32(vx1)], [np.float32(vy1)]])

        predicted1 = kf1.predict()

        if self.marker_flag[0] == True:
            self.updated1 = np.asarray(kf1.update(measured1))
            # print("Updated when True", updated)
            self.measured1_list = np.append(self.updated1, self.measured1_list) 
        elif self.marker_flag[0] == False:
            # rospy.sleep(2)
            new_measured1 = np.array([[np.float32(self.measured1_list[0])], [np.float32(self.measured1_list[1])], [np.float32(self.measured1_list[2])], [np.float32(self.measured1_list[3])]])
            self.updated1 = np.asarray(kf1.update(new_measured1))
            self.measured1_list = np.append(self.updated1, self.measured1_list)
            # self.cx1_list = np.append(self.updated1[0], self.cx1_list)
            # self.cy1_list = np.append(self.updated1[1], self.cy1_list)

        # cv2.circle(img, (int(cx1), int(cy1)), 5, green, -1)

        # joint2
        cx2 = feature[1][0]
        cy2 = feature[1][1]

        self.cx2_list = np.append(cx2, self.cx2_list)
        self.cy2_list = np.append(cy2, self.cy2_list)

        dx2 = self.cx2_list[0]-self.cx2_list[1]
        dy2 = self.cy2_list[0]-self.cy2_list[1]

        vx2 = dx2
        vy2 = dy2

        measured2 = np.array([[np.float32(cx2)], [np.float32(cy2)], [np.float32(vx2)], [np.float32(vy2)]])

        predicted2 = kf2.predict()

        if self.marker_flag[1] == True:
            self.updated2 = np.asarray(kf2.update(measured2))
            # print("Updated when True", updated)
            self.measured2_list = np.append(self.updated2, self.measured2_list) 
        elif self.marker_flag[1] == False:
            # rospy.sleep(2)
            new_measured2 = np.array([[np.float32(self.measured2_list[0])], [np.float32(self.measured2_list[1])], [np.float32(self.measured2_list[2])], [np.float32(self.measured2_list[3])]])
            self.updated2 = np.asarray(kf2.update(new_measured2))
            self.measured2_list = np.append(self.updated2, self.measured2_list)
            # self.cx2_list = np.append(self.updated2[0], self.cx2_list)
            # self.cy2_list = np.append(self.updated2[1], self.cy2_list)

        

        #joint3
        cx3 = feature[2][0]
        cy3 = feature[2][1] 

        self.cx3_list = np.append(cx3, self.cx3_list)
        self.cy3_list = np.append(cy3, self.cy3_list)

        dx3 = self.cx3_list[0]-self.cx3_list[1]
        dy3 = self.cy3_list[0]-self.cy3_list[1]

        vx3 = dx3
        vy3 = dy3

        measured3 = np.array([[np.float32(cx3)], [np.float32(cy3)], [np.float32(vx3)], [np.float32(vy3)]])

        predicted3 = kf3.predict()

        if self.marker_flag[2] == True:
            self.updated3 = np.asarray(kf3.update(measured3))
            # print("Updated when True", updated)
            self.measured3_list = np.append(self.updated3, self.measured3_list) 
        elif self.marker_flag[2] == False:
            # rospy.sleep(2)
            new_measured3 = np.array([[np.float32(self.measured3_list[0])], [np.float32(self.measured3_list[1])], [np.float32(self.measured3_list[2])], [np.float32(self.measured3_list[3])]])
            self.updated3 = np.asarray(kf3.update(new_measured3))
            self.measured3_list = np.append(self.updated3, self.measured3_list)
            # self.cx3_list = np.append(self.updated3[0], self.cx3_list)
            # self.cy3_list = np.append(self.updated3[1], self.cy3_list)
       

        pred_x1, pred_y1 = int(predicted1[0][0]), int(predicted1[1][0])   
        upd_x1, upd_y1 = int(self.updated1[0]), int(self.updated1[1])

        pred_x2, pred_y2 = int(predicted2[0][0]), int(predicted2[1][0])   
        upd_x2, upd_y2 = int(self.updated2[0]), int(self.updated2[1])

        pred_x3, pred_y3 = int(predicted3[0][0]), int(predicted3[1][0])   
        upd_x3, upd_y3 = int(self.updated3[0]), int(self.updated3[1])


        # joint4
        cx4 = feature[3][0]
        cy4 = feature[3][1]

        self.cx4_list = np.append(cx4, self.cx4_list)
        self.cy4_list = np.append(cy4, self.cy4_list)

        dx4 = self.cx4_list[0]-self.cx4_list[1]
        dy4 = self.cy4_list[0]-self.cy4_list[1]

        vx4 = dx4
        vy4 = dy4

        measured4 = np.array([[np.float32(cx4)], [np.float32(cy4)], [np.float32(vx4)], [np.float32(vy4)]])

        predicted4 = kf4.predict()

        if self.marker_flag[3] == True:
            self.updated4 = np.asarray(kf4.update(measured4))
            # print("Updated when True", updated)
            self.measured4_list = np.append(self.updated4, self.measured4_list) 
        elif self.marker_flag[3] == False:
            # rospy.sleep(2)
            new_measured4 = np.array([[np.float32(self.measured4_list[0])], [np.float32(self.measured4_list[1])], [np.float32(self.measured4_list[2])], [np.float32(self.measured4_list[3])]])
            self.updated4 = np.asarray(kf4.update(new_measured4))
            self.measured4_list = np.append(self.updated4, self.measured4_list)
            # self.cx4_list = np.append(self.updated4[0], self.cx4_list)
            # self.cy4_list = np.append(self.updated4[1], self.cy4_list)

        # joint5
        cx5 = feature[4][0]
        cy5 = feature[4][1]

        self.cx5_list = np.append(cx5, self.cx5_list)
        self.cy5_list = np.append(cy5, self.cy5_list)

        dx5 = self.cx5_list[0]-self.cx5_list[1]
        dy5 = self.cy5_list[0]-self.cy5_list[1]

        vx5 = dx5
        vy5 = dy5

        measured5 = np.array([[np.float32(cx5)], [np.float32(cy5)], [np.float32(vx5)], [np.float32(vy5)]])

        predicted5 = kf5.predict()

        if self.marker_flag[4] == True:
            self.updated5 = np.asarray(kf5.update(measured5))
            # print("Updated when True", updated)
            self.measured5_list = np.append(self.updated5, self.measured5_list) 
        elif self.marker_flag[4] == False:
            print("is update function getting called for 5th point")
            print("update output before False", self.measured5_list[0], self.measured5_list[1], self.measured5_list[2], self.measured5_list[3])
            # rospy.sleep(2)
            new_measured5 = np.array([[np.float32(self.measured5_list[0])], [np.float32(self.measured5_list[1])], [np.float32(self.measured5_list[2])], [np.float32(self.measured5_list[3])]])
            self.updated5 = np.asarray(kf5.update(new_measured5))
            self.measured5_list = np.append(self.updated5, self.measured5_list)
            # self.cx5_list = np.append(self.updated5[0], self.cx5_list)
            # self.cy5_list = np.append(self.updated5[1], self.cy5_list)

        # joint6
        cx6 = feature[5][0]
        cy6 = feature[5][1]

        self.cx6_list = np.append(cx6, self.cx6_list)
        self.cy6_list = np.append(cy6, self.cy6_list)

        dx6 = self.cx6_list[0]-self.cx6_list[1]
        dy6 = self.cy6_list[0]-self.cy6_list[1]

        vx6 = dx6
        vy6 = dy6

        measured6 = np.array([[np.float32(cx6)], [np.float32(cy6)], [np.float32(vx6)], [np.float32(vy6)]])

        predicted6 = kf6.predict()

        if self.marker_flag[5] == True:            
            self.updated6 = np.asarray(kf6.update(measured6))
            # print("Updated when True", updated)
            self.measured6_list = np.append(self.updated6, self.measured6_list) 
        elif self.marker_flag[5] == False:
            print("is update function getting called for 6th point")
            print("update output before False", self.measured6_list[0], self.measured6_list[1], self.measured6_list[2], self.measured6_list[3])
            # rospy.sleep(2)
            new_measured6 = np.array([[np.float32(self.measured6_list[0])], [np.float32(self.measured6_list[1])], [np.float32(self.measured6_list[2])], [np.float32(self.measured6_list[3])]])
            print("new measured 6", new_measured6)
            self.updated6 = np.asarray(kf6.update(new_measured6))
            self.measured6_list = np.append(self.updated6, self.measured6_list)
            # self.cx6_list = np.append(self.updated6[0], self.cx6_list)
            # self.cy6_list = np.append(self.updated6[1], self.cy6_list)

        pred_x4, pred_y4 = int(predicted4[0][0]), int(predicted4[1][0])   
        upd_x4, upd_y4 = int(self.updated4[0]), int(self.updated4[1])

        pred_x5, pred_y5 = int(predicted5[0][0]), int(predicted5[1][0])   
        upd_x5, upd_y5 = int(self.updated5[0]), int(self.updated5[1])

        pred_x6, pred_y6 = int(predicted6[0][0]), int(predicted6[1][0])   
        upd_x6, upd_y6 = int(self.updated6[0]), int(self.updated6[1])


        # joint7
        cx7 = feature[6][0]
        cy7 = feature[6][1]

        self.cx7_list = np.append(cx7, self.cx7_list)
        self.cy7_list = np.append(cy7, self.cy7_list)

        dx7 = self.cx7_list[0]-self.cx7_list[1]
        dy7 = self.cy7_list[0]-self.cy7_list[1]

        vx7 = dx7
        vy7 = dy7

        measured7 = np.array([[np.float32(cx7)], [np.float32(cy7)], [np.float32(vx7)], [np.float32(vy7)]])

        predicted7 = kf7.predict()


        if self.marker_flag[6] == True:
            self.updated7 = np.asarray(kf7.update(measured7))
            # print("Updated when True", updated)
            self.measured7_list = np.append(self.updated7, self.measured7_list) 
        elif self.marker_flag[6] == False:
            print("is update function getting called for 7th point")
            print("update output before False", self.measured7_list[0], self.measured7_list[1], self.measured7_list[2], self.measured7_list[3])
            # rospy.sleep(2)
            new_measured7 = np.array([[np.float32(self.measured7_list[0])], [np.float32(self.measured7_list[1])], [np.float32(self.measured7_list[2])], [np.float32(self.measured7_list[3])]])
            self.updated7 = np.asarray(kf7.update(new_measured7))
            self.measured7_list = np.append(self.updated7, self.measured7_list)
            # self.cx7_list = np.append(self.updated7[0], self.cx7_list)
            # self.cy7_list = np.append(self.updated7[1], self.cy7_list)

        pred_x7, pred_y7 = int(predicted7[0][0]), int(predicted7[1][0])   
        upd_x7, upd_y7 = int(self.updated7[0]), int(self.updated7[1])

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.4
        org1 = (10, 20)
        org2 = (10,80)
        org3 = (10, 100)
        org4 = (10,140)
        org5 = (10, 160)
        org6 = (10, 180)

        cv2.circle(img, (int(cx1), int(cy1)), 5, blue, -1)        
        cv2.circle(img, (int(cx2), int(cy2)), 5, blue, -1)
        cv2.circle(img, (int(cx3), int(cy3)), 5, blue, -1)
        cv2.circle(img, (int(cx4), int(cy4)), 5, blue, -1)        
        cv2.circle(img, (int(cx5), int(cy5)), 5, blue, -1)
        cv2.circle(img, (int(cx6), int(cy6)), 5, blue, -1)
        cv2.circle(img, (int(cx7), int(cy7)), 5, blue, -1)


        # cv2.circle(img, (pred_x1, pred_y1), 10, yellow, 4)
        cv2.circle(img, (upd_x1, upd_y1), 5, red, 4)

        # cv2.circle(img, (pred_x2, pred_y2), 10, yellow, 4)
        cv2.circle(img, (upd_x2, upd_y2), 5, red, 4)

        # cv2.circle(img, (pred_x3, pred_y3), 10, yellow, 4)
        cv2.circle(img, (upd_x3, upd_y3), 5, red, 4)


        # cv2.circle(img, (pred_x4, pred_y4), 10, yellow, 4)
        cv2.circle(img, (upd_x4, upd_y4), 5, red, 4)

        # cv2.circle(img, (pred_x5, pred_y5), 10, yellow, 4)
        cv2.circle(img, (upd_x5, upd_y5), 5, red, 4)

        # cv2.circle(img, (pred_x6, pred_y6), 10, yellow, 4)
        cv2.circle(img, (upd_x6, upd_y6), 5, red, 4)

        # cv2.circle(img, (pred_x7, pred_y7), 10, yellow, 4)
        cv2.circle(img, (upd_x7, upd_y7), 5, red, 4)

        # # cv2.circle(img, (int(cx1), int(cy1)), 5, green, -1)
        # cv2.putText(img, str(predicted5[0:4]), org2, font, fontScale, yellow, 1, cv2.LINE_AA)
        # cv2.putText(img, str(predicted5[4:]), org3, font, fontScale, yellow, 1, cv2.LINE_AA)
        # # cv2.circle(img, (pred_x, pred_y), 10, yellow, 4)
        # cv2.putText(img, str(self.updated5[0:4]), org4, font, fontScale, red, 1, cv2.LINE_AA)
        # cv2.putText(img, str(self.updated5[4:]), org5, font, fontScale, red, 1, cv2.LINE_AA)
        # cv2.circle(img, (upd_x, upd_y), 5, red, 4)

        print("next frame"+str(self.j))

        cv2.imwrite("/home/merlab/Pictures/kalman_images/"+str(self.j)+".jpg", img)                 

        self.j= self.j + 1

        latest_corrected_x = np.array([[self.updated1[0][0], self.updated1[1][0]], [self.updated2[0][0], self.updated2[1][0]], [self.updated3[0][0], self.updated3[1][0]], 
                                        [self.updated4[0][0], self.updated4[1][0]], [self.updated5[0][0], self.updated5[1][0]], [self.updated6[0][0], self.updated6[1][0]], 
                                        [self.updated7[0][0], self.updated7[1][0]]])

        return latest_corrected_x
    
    # runs only once, when all 7 joints are identified for the first time
    def first_input_estimate(self, img, key_points):
        print("first filter is called")
        blue = (255,0,0)
        d_img = self.cv_depth_image
        # print("Actual first first key points", key_points)
        self.marker_flag = [True, True, True, True, True, True, True]
        first_corrected_x = dream_ros.dream_kalman_estimate(img, key_points)       
        for i in range(len(key_points)):
                    x = np.int64(key_points[i][0])
                    y = np.int64(key_points[i][1])
                    cv2.circle(d_img,(x, y),10,blue,-1)    
        # print("First Corrected X", first_corrected_x)
        return first_corrected_x        
    
    # this is the function where the missing joints are identified for 
    def input_estimation(self, img, key_points, corrected_x):
        print("filters are called")        
        blue = (255,0,0)
        d_img = self.cv_depth_image
        # print("key points input after the first iteration", key_points)
        # print("corredtec_x after first iteration", corrected_x)

        latest_x = np.array([[-1, -1], [-1,-1], [-1, -1], 
                     [-1, -1], [-1, -1], [-1, -1], [-1, -1]])        
        if len(key_points) == 7:
            self.marker_flag = [True, True, True, True, True, True, True]
            latest_corrected_x = dream_ros.dream_kalman_estimate(img, key_points)
            for i in range(len(key_points)):
                    x = np.int64(key_points[i][0])
                    y = np.int64(key_points[i][1])
                    cv2.circle(d_img,(x, y),10,blue,-1)

        else:
            for i in range(len(key_points)):
                distances = np.sqrt((corrected_x[:, 0] - key_points[i][0])**2 + (corrected_x[:, 1] - key_points[i][1])**2)
                nearest_index = np.argmin(distances)
                latest_x[nearest_index] = key_points[i]
                # print("input after the first estimation", latest_x)

            for i in range(len(latest_x)):
                if ([-1,-1] == latest_x[i]).all():
                    # print(([-1,-1] == latest_x[i]).all())
                    self.marker_flag[i] = False
                    # print("marker_flag array", self.marker_flag)
            latest_corrected_x = dream_ros.dream_kalman_estimate(img, latest_x)

            for i in range(len(latest_corrected_x)):
                    x = np.int64(latest_corrected_x[i][0])
                    y = np.int64(latest_corrected_x[i][1])
                    cv2.circle(d_img,(x, y),10,blue,-1)
            
        cv2.imwrite("/home/merlab/Pictures/dream_depth_images/"+str(self.k)+".jpg", d_img)    

            # print("marker_flag array when length less than 7", self.marker_flag)

        # print("latest corrected X in input estimation", latest_corrected_x)
        # rospy.sleep(3)
        self.k = self.k + 1
        return latest_corrected_x
            
    def dream_img_service(self, msg):
        print("Service Started")
        img = copy.deepcopy(self.cv_image)
        key_points = self.key_points

        print(key_points)

        kp_x = []
        kp_y = []
        for i in range(len(key_points)):
            x = np.int64(key_points[i][0])
            y = np.int64(key_points[i][1])
            kp_x.append(x)
            kp_y.append(y)
            cv2.circle(img,(x, y),10,(0,255,255),-1)    

        # This part is simply to control the robot using DREAM key points
        kp = []
        for i in range(len(kp_x)):
           kp.append(kp_x[i]) 
           kp.append(kp_y[i])
        kp = np.reshape(np.array(kp), (-1, 2))
        # print("length of keypoints", len(kp))
        cv2.imwrite("/home/merlab/Pictures/dream_kalman_images/"+str(self.i)+".jpg", img)
        if len(kp) == 7 and not self.executed:
            # print("first key points input", kp)
            self.corrected_x = dream_ros.first_input_estimate(img, kp)
            self.executed = True   
            # print("Corrected_X", self.corrected_x)                       
        
        elif self.executed:
            # print("Executed?", self.executed)               
            self.corrected_x = dream_ros.input_estimation(img, kp, self.corrected_x)         

        # l = len(kp) - no_of_features
        # if len(kp)>no_of_features:
        #     kp = kp[l:]
        
        # print("Latest kp", kp)

        pix = Float64MultiArray()
        pix.data = self.corrected_x

        ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")

        return dream_goal_imgResponse(ros_img, pix)

    def publish_pose(self):
        self.camera_pose_tform.header.stamp = rospy.Time().now()
        self.tf_broadcaster.sendTransform(self.camera_pose_tform)

        # Generate and publish keypoint frame overlay
        if self.kp_frame_overlay_pub.get_num_connections() > 0:

            if self.cv_image is None or self.camera_K is None:
                return

            all_kp_transforms = []
            for i in range(len(self.keypoint_tf_frames)):
                keypoint_tf_frame = self.keypoint_tf_frames[i]
                # Lookup transform between dream published frame and keypoint frames
                try:
                    tform = self.tfBuffer.lookup_transform(
                        self.camera_pose_tform.child_frame_id,
                        keypoint_tf_frame,
                        rospy.Time(),
                    )
                    all_kp_transforms.append(tform)

                except tf2.TransformException as e:
                    print("TF Exception: {}".format(e))
                    return None, None

            cv_image_overlay = self.cv_image.copy()

            frame_len = 0.1
            frame_thickness = 3
            frame_triad_pts = np.array(
                [
                    [0.0, 0.0, 0.0, 1.0],
                    [frame_len, 0.0, 0.0, 1.0],
                    [0.0, frame_len, 0.0, 1.0],
                    [0.0, 0.0, frame_len, 1.0],
                ]
            )
            shift = 4
            factor = 1 << shift
            point_radius = 4.0

            for kp_tform in all_kp_transforms:
                pos = [
                    kp_tform.transform.translation.x,
                    kp_tform.transform.translation.y,
                    kp_tform.transform.translation.z,
                ]
                quat = [
                    kp_tform.transform.rotation.x,
                    kp_tform.transform.rotation.y,
                    kp_tform.transform.rotation.z,
                    kp_tform.transform.rotation.w,
                ]
                T = tf.transformations.quaternion_matrix(quat)
                T[:3, -1] = pos

                frame_triad_positions_homog = np.transpose(
                    np.matmul(T, np.transpose(frame_triad_pts))
                )
                frame_triad_positions = [
                    dream.geometric_vision.hnormalized(v).tolist()
                    for v in frame_triad_positions_homog
                ]
                frame_triad_projs = dream.geometric_vision.point_projection_from_3d(
                    self.camera_K, frame_triad_positions
                )

                # Overlay line on image
                point0_fixedpt = (
                    int(frame_triad_projs[0][0] * factor),
                    int(frame_triad_projs[0][1] * factor),
                )
                point1_fixedpt = (
                    int(frame_triad_projs[1][0] * factor),
                    int(frame_triad_projs[1][1] * factor),
                )
                point2_fixedpt = (
                    int(frame_triad_projs[2][0] * factor),
                    int(frame_triad_projs[2][1] * factor),
                )
                point3_fixedpt = (
                    int(frame_triad_projs[3][0] * factor),
                    int(frame_triad_projs[3][1] * factor),
                )

                # x-axis
                cv_image_overlay = cv2.line(
                    cv_image_overlay,
                    point0_fixedpt,
                    point1_fixedpt,
                    webcolors.name_to_rgb("red"),
                    thickness=frame_thickness,
                    shift=shift,
                )
                # y-axis
                cv_image_overlay = cv2.line(
                    cv_image_overlay,
                    point0_fixedpt,
                    point2_fixedpt,
                    webcolors.name_to_rgb("green"),
                    thickness=frame_thickness,
                    shift=shift,
                )
                # z-axis
                cv_image_overlay = cv2.line(
                    cv_image_overlay,
                    point0_fixedpt,
                    point3_fixedpt,
                    webcolors.name_to_rgb("blue"),
                    thickness=frame_thickness,
                    shift=shift,
                )
                # center of frame triad
                radius_fixedpt = int(point_radius * factor)
                cv_image_overlay = cv2.circle(
                    cv_image_overlay,
                    point0_fixedpt,
                    radius_fixedpt,
                    webcolors.name_to_rgb("black"),
                    thickness=-1,
                    shift=shift,
                )

            cv_image_overlay = cv_image_overlay[:, :, ::-1]
            image_overlay_msg = self.bridge.cv2_to_imgmsg(
                cv_image_overlay, encoding="bgr8"
            )
            self.kp_frame_overlay_pub.publish(image_overlay_msg)


if __name__ == "__main__":

    # Parse input arguments
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-i",
        "--input-params-path",
        required=True,
        help="Path to network parameters file.",
    )
    parser.add_argument(
        "-c",
        "--input-config-path",
        default=None,
        help="Path to network configuration file. If nothing is specified, the script will search for a config file by the same name as the network parameters file.",
    )
    parser.add_argument(
        "-b",
        "--base-frame",
        required=True,
        help="The ROS TF name for the base frame of the robot, which serves as the canonical frame for PnP.",
    )
    parser.add_argument(
        "-r",
        "--node-rate",
        type=float,
        default=10.0,
        help="The rate in Hz for this node to run.",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        default=False,
        help="Outputs all diagnostic information to the screen.",
    )
    parser.add_argument(
        "-p",
        "--image-preproc-override",
        default=None,
        help="Overrides the image preprocessing specified by the network. (Debug argument.)",
    )
    parser.add_argument(
        "-g",
        "--gpu-ids",
        nargs="+",
        type=int,
        default=None,
        help="The GPU IDs on which to conduct network inference. Nothing specified means all GPUs will be utilized. Does not affect results, only how quickly the results are obtained.",
    )
    args, unknown = parser.parse_known_args()

    # Initialize ROS node
    rospy.init_node("dream")

    # Create DREAM inference engine
    single_frame_mode = True
    mode_str = "single-frame mode" if single_frame_mode else "multi-frame mode"
    dream_ros = DreamInferenceROS(
        args, single_frame_mode, compute_2d_to_3d_transform=True
    )
    print("DREAM ~ online ~ " + mode_str)

    # Main loop
    rate = rospy.Rate(args.node_rate)
    while not rospy.is_shutdown():

        # Find keypoints in image
        found_kp_projs_net_input = dream_ros.process_image()

        # Solve PNP (if in single frame mode)
        if single_frame_mode and found_kp_projs_net_input is not None:
            (
                kp_projs_raw_good_sample,
                kp_positions_good_sample,
            ) = dream_ros.keypoint_correspondences(found_kp_projs_net_input)
            if (
                kp_projs_raw_good_sample is not None
                and kp_positions_good_sample is not None
            ):
                dream_ros.solve_pnp_buffer(
                    kp_projs_raw_good_sample, kp_positions_good_sample
                )

        # Publish pose (if found)
        if dream_ros.pnp_solution_found:
            dream_ros.publish_pose()

        rate.sleep()


# def main():
#     # Initialize the node
#     rospy.init_node('franka_goal_feature_node')
    
#     # subscriber for rgb image to detect markers
#     image_rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_callback, queue_size=1)

#     # subscriber for depth image for binarization
#     image_depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1)

#     # service declaration to receive the binary image
#     bin_img_service = rospy.Service("binary_image_service_response", franka_bin_img, franka_binary_image_service )

#     # wait for control points service to be up
#     # rospy.wait_for_service('franka_control_service')   
    
    
#     # try:
#     #     update_goal_pose() # publishes joint positions to position controller
#     # except rospy.ROSInterruptException:
#     #     pass

#     rospy.spin()


# if __name__ == '__main__':
#     main()
    
