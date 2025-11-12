#!/usr/bin/env python
import queue
import sys
import threading

import rospy
from detectron2.config import get_cfg
from cv_bridge import CvBridge
# import some common detectron2 utilities
from detectron2.engine import DefaultPredictor
from detectron2.utils.logger import setup_logger
from detectron2.utils.visualizer import Visualizer
from dexterous_picking.srv import GetGrasp, GetGraspResponse
from sensor_msgs.msg import Image
import rospkg
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
import cv2
from datetime import datetime


from std_msgs.msg import Float32MultiArray, UInt32MultiArray

class Detectron2(object):
    def __init__(self):
        setup_logger()

        self.bridge = CvBridge()
        self.last_msg = None
        self.last_depth_msg = None
        self.msg_lock = threading.Lock()

        rospack = rospkg.RosPack()
        pkgdir = rospack.get_path('dexterous_picking')
        self.cfg = get_cfg()
        self.cfg.merge_from_file(pkgdir + "/config/mask_rcnn_R_50_FPN_3x.yaml")  # Update with the path to the config file used for training
        self.cfg.MODEL.WEIGHTS = pkgdir + "/config/" + "model_rgb_final.pth"
        self.cfg.MODEL.DEVICE = "cuda"
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.6
        self.predictor = DefaultPredictor(self.cfg)

        self.visualization = True
        self.vis_pub = rospy.Publisher('grasp_detection/visualization', Image, queue_size=1)
        self.pub = rospy.Publisher('/selected_coordinates', Pose, queue_size=1)
        self.sub = rospy.Subscriber("/zed_camera/image", Image, self.callback_image, queue_size=1)
        self.depth_sub_left = rospy.Subscriber('/zed_camera/depth', Image, self.callback_depth_image, queue_size=1)
        
        self.service = rospy.Service("get_grasp_rgb", GetGrasp, self.get_object_labels) 
        # self.get_orientation_service = rospy.Service("get_orientation", GetGrasp, self.get_object_labels)
        self.sub_coord = rospy.Subscriber("/grasp_coordinates_list", Float32MultiArray, self.grasp_coordinates_callback, queue_size=1)
        # self.received_points = None

    def get_grasp(self, predictions, masks):
        grasp_id = GetGraspResponse.NO_OBJECT_FOUND
        success = False
        labels = predictions
        orientation = None
        
        rospy.loginfo("Detected objects: %s", labels)
        if len(labels) == GetGraspResponse.NO_OBJECT_FOUND:
            grasp_id = GetGraspResponse.NO_OBJECT_FOUND
            success = True
        elif labels[0] == 1:
            grasp_id = GetGraspResponse.FLIP
            success = True
        elif labels[0] == 2:
            grasp_id = GetGraspResponse.SLIDE_TO_EDGE
            success = True
        else:
            rospy.logerr("Unknown grasp type: %s", labels[0])
            success = False
        return grasp_id, success, orientation
    
    def associate_points_with_labels(self, points_tuple, masks, labels, depth_image):
        label_point_association = {}
        
        # for i, mask in enumerate(masks):
        for i, mask in reversed(list(enumerate(masks))):
            label = labels[i]
            for j in range(0, len(points_tuple), 2):
                x, y = int(points_tuple[j]), int(points_tuple[j + 1])  
                if mask[y, x]:
                    depth = np.min(depth_image[mask])
                    label_point_association[label] = (x, y, depth)
                    break   
        return label_point_association
    
    def polygon_from_mask(self, binary_mask):
        print(binary_mask.shape)
        # Assuming binary_mask is of shape (1, 1080, 1920)
        binary_mask_2d = binary_mask.reshape(1080, 1920)
        binary_mask_2d = binary_mask_2d.astype(np.uint8)
        contours, _ = cv2.findContours(binary_mask_2d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        points_list = []

        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            contour_points = [[point[0][0], point[0][1]] for point in approx]
            points_list.extend(contour_points)

        return np.array(points_list)
    
    def get_simple_pick_orientation(self, polygon_points):

        reference_yaw = -0.775
        rect = cv2.minAreaRect(polygon_points.astype(np.float32))
        angle_of_rotation = -rect[2]
        width = rect[1][0]
        height = rect[1][1]
        print(f"angle of rot: {angle_of_rotation}, width: {width}, height:{height}")

        if (width > 220 or height > 220) and abs(width - height) > 100 :
            if width > height:
                angle = angle_of_rotation
            else:
                angle =  90 + angle_of_rotation

            angle = round(angle / 5) * 5
            print(angle)
            theta_rad = np.deg2rad(max(-90, min(90, angle)))
            yaw = reference_yaw + theta_rad
            return yaw
            
        return reference_yaw

    def get_object_labels(self, req):
        while (not rospy.is_shutdown()) and self.msg_lock.acquire(False):
            img_msg = self.last_msg
            img_depth_msg = self.last_depth_msg
            self.msg_lock.release()
            break

        response = GetGraspResponse()
        np_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        current_depth = self.bridge.imgmsg_to_cv2(img_depth_msg, desired_encoding="passthrough")

        outputs = self.predictor(np_image)

        result = [value.item() for value in outputs["instances"].pred_classes]
        print("Predicted obj",result)

        instances = outputs["instances"].to("cpu")
        print(f"Instances: {instances}")

        masks = np.asarray(instances.pred_masks)
        
        labels = instances.pred_classes.tolist()

        label_point_association = self.associate_points_with_labels(self.received_points, masks, result, current_depth)
        print(f"Label Association: {label_point_association}")

        if "scores" in instances._fields:
            conf_scores = instances.scores.tolist()
        else:
            raise AttributeError("Unable to find 'scores' in the Instances object.")

        print(conf_scores)

        response.grasp_id, response.success, yaw = self.get_grasp(result, masks)
        print(f"Yaw: {yaw}")

        filtered_points = [point for grasp_id, point in label_point_association.items() if (6-grasp_id) == response.grasp_id]

        # print(filtered_points)
        # Z=yaw is the orientation for simple pick. Needs to be changed later
        # filtered_point_msgs = [Point(x=point[0], y=point[1], z=yaw) for point in filtered_points]

        filtered_point_msgs = []

        for point in filtered_points:
            quaternion = Quaternion(x=0.0, y=0.0, z=yaw, w=0)
            # Create a Point message
            position = Point(x=point[0], y=point[1], z=point[2])

            # Create a Pose message with the Point and Quaternion
            pose = Pose(position=position, orientation=quaternion)
            filtered_point_msgs.append(pose)
            print(f"Pose: {pose}")


        for point_msg in filtered_point_msgs:
            self.pub.publish(point_msg)

        # Visualize results
        if self.visualization:
            v = Visualizer(np_image[:, :, ::-1], scale=1.2)
            v = v.draw_instance_predictions(instances)
            img = v.get_image()[:, :, ::-1]
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.vis_pub.publish(image_msg)
            bridge = CvBridge()
            # img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            # save_dir = "/home/merlab/mer_lab/ros_ws/src/projects/dexterous_picking/scripts/detection_images"
            # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            # file_name = f"experiment_detection_{timestamp}.jpg"
            # file_path = save_dir + file_name
            # cv2.imwrite(file_path, img)
        return response

    def callback_image(self, msg):
        if self.msg_lock.acquire(False):
            self.last_msg = msg
            self.msg_lock.release()

    def callback_depth_image(self, msg):
        if self.msg_lock.acquire(False):
            self.last_depth_msg = msg
            self.msg_lock.release()

    def grasp_coordinates_callback(self, msg):
        self.received_points = msg.data
        # print(self.received_points)


def main(argv):
    rospy.init_node('grasp_detection_rgb_node')
    rospy.loginfo("Grasp Detection RGB Node Started")
    node = Detectron2()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)