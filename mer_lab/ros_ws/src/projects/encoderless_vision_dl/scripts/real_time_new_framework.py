#!/usr/bin/env python3.8

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo, JointState
# from forward import Kinematics
from cv_bridge import CvBridge
import torch
import torchvision
from torchvision.transforms import functional as F
import os
from PIL import Image as Img
from datetime import datetime
from os.path import expanduser
from utils import DataPrePro
from visualization import visualize
import csv

device = 'cuda' if torch.cuda.is_available() else 'cpu'
weights_path = rospy.get_param('vsbot/deeplearning/weights_path')
print("weights_path", weights_path)
model = torch.load(weights_path).to(device)
bridge = CvBridge()
i = 0
csvwriter = None

def image_callback(msg):
    global cv_img, model, CvBridge, i, csvwriter
    print("iterations", i)
    print("is image callback getting called")

    ros_img = msg

    if ros_img is not None:    
        cv_img = bridge.imgmsg_to_cv2(ros_img, "rgb8")

        image = Img.fromarray(cv_img)

        image = F.to_tensor(image).to(device)
        image.unsqueeze_(0)
        image = list(image)
        with torch.no_grad():
            model.to(device)
            model.eval()
            output = model(image)
        image = (image[0].permute(1,2,0).detach().cpu().numpy() * 255).astype(np.uint8)
        scores = output[0]['scores'].detach().cpu().numpy()
        high_scores_idxs = np.where(scores > 0.7)[0].tolist() # Indexes of boxes with scores > 0.7
        post_nms_idxs = torchvision.ops.nms(output[0]['boxes'][high_scores_idxs], \
            output[0]['scores'][high_scores_idxs], 0.3).cpu().numpy() # Indexes of boxes left after applying NMS (iou_threshold=0.3)
        # Below, in output[0]['keypoints'][high_scores_idxs][post_nms_idxs] and output[0]['boxes'][high_scores_idxs][post_nms_idxs]
        # Firstly, we choose only those objects, which have score above predefined threshold. This is done with choosing elements with [high_scores_idxs] indexes
        # Secondly, we choose only those objects, which are left after NMS is applied. This is done with choosing elements with [post_nms_idxs] indexes
        keypoints = []
        key_points = []
        for kps in output[0]['keypoints'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
            keypoints.append(list(map(int, kps[0,0:2])))
            # for kp in kps:
                # print(kp)
            key_points.append([list(map(int, kp[:2])) for kp in kps])
        # print(np.array(keypoints).shape)                
        # if len(keypoints) == 6:
        #     keypoints.pop(2)
        labels = []
        for label in output[0]['labels'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
                labels.append(label)
        keypoints_ = [x for _,x in sorted(zip(labels,keypoints))]
            
        # print(keypoints)
        print(len(keypoints_))
        bboxes = []
        for bbox in output[0]['boxes'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
            bboxes.append(list(map(int, bbox.tolist())))

        kp_x = []
        kp_y = []
        for j in range(len(keypoints_)):
            x = np.int64(keypoints_[j][0])
            y = np.int64(keypoints_[j][1])
            cv2.circle(cv_img,(x, y),10,(255,0,255),-1)
            kp_x.append(x)
            kp_y.append(y)
        kp = []
        for j in range(len(kp_x)-2):
           kp.append(kp_x[j+2]) 
           kp.append(kp_y[j+2])  

        print("control_points", kp)
        csvwriter.writerow(kp)
        
        # print("key points", keypoints)
        # img = visualize(image, bboxes, key_points)
        cv2.imwrite("/home/jc-merlab/Pictures/Data/real_time_planar_check/out_image_" + str(i) + ".jpg", cv_img)

    i = i+1

def main():
    global csvwriter
    # Initialize the node
    rospy.init_node('image_pix_gen')
    print("is main getting called")
    # subscriber for rgb image to detect markers
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1)
    filename = "/home/jc-merlab/Pictures/Data/plotting_data/keypoints_new.csv"
    col_name = ['kp1x', 'kp1y', 'kp2x', 'kp2y', 'kp3x', 'kp3y']

    f = open(filename, 'w')
    csvwriter = csv.writer(f)
    csvwriter.writerow(col_name)

    rospy.spin()

if __name__=='__main__':
    main()
