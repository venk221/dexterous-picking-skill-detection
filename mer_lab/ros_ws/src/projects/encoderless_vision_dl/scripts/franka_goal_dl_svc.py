#!/usr/bin/env python3.8

import numpy as np
import cv2
import rospy
from std_msgs.msg import Int32, Bool, Float64MultiArray
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
from encoderless_vision_dl.srv import dl_img, dl_imgResponse
from visualization import visualize

device = 'cuda' if torch.cuda.is_available() else 'cpu'
# weights_path = '/home/jc-merlab/Pictures/Data/trained_models/keypointsrcnn_weights_ld_b3_e25.pth'
weights_path = rospy.get_param('vsbot/deeplearning/weights_path')
model = torch.load(weights_path).to(device)
bridge = CvBridge()
i = 0
ros_img = None
kp = None
status_pub = rospy.Publisher("vsbot/status", Int32, queue_size=1)

def image_callback(msg):
    global ros_img

    ros_img = msg

def dl_image_service(img):
    global bridge, model, kp, ros_img

    print("svc ros image", type(ros_img))
    print("is keypoint service getting called")

    if ros_img is not None:
        status_msg = Int32()
        status_msg.data = 1
        status_pub.publish(status_msg)
        cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")

        inf_img = Img.fromarray(cv_img)

        inf_img = F.to_tensor(inf_img).to(device)
        inf_img.unsqueeze_(0)
        inf_img = list(inf_img)
        with torch.no_grad():
            model.to(device)
            model.eval()
            output = model(inf_img)
        inf_img = (inf_img[0].permute(1,2,0).detach().cpu().numpy() * 255).astype(np.uint8)
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

        print(keypoints_)
            
        # print(keypoints)
        print("no of keypoints", len(keypoints))
        bboxes = []
        for bbox in output[0]['boxes'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
            bboxes.append(list(map(int, bbox.tolist())))
        
        # print("key points", keypoints)
        # img = visualize(inf_img, bboxes, key_points)
        # cv2.imwrite("/home/jc-merlab/Pictures/Data/video_results_live_b1e25_v2/out_image_" + str(i) + ".jpg", img)

        # uncomment the next line for 4 feature points
        # indices = [0,1,2,3,4,5,6,8]
        # uncomment the next line for 3 feature points
        indices = [0,1,3,4,5]
        keypoints_ = [keypoints_[i] for i in indices]

        kp_x = []
        kp_y = []
        for i in range(len(keypoints_)):
            x = np.int64(keypoints_[i][0])
            y = np.int64(keypoints_[i][1])
            kp_x.append(x)
            kp_y.append(y)

        kp = []
        
        # Uncomment the next block for 3 features
        for i in range(len(kp_x)-2):
           kp.append(kp_x[i+2]) 
           kp.append(kp_y[i+2])

        # Uncomment the next block for 4 features
        # for i in range(len(kp_x)-3):
        #    kp.append(kp_x[i+3]) 
        #    kp.append(kp_y[i+3])

        kp_resp = Float64MultiArray()
        kp_resp.data = kp

        print("keypoints", kp_resp.data)
        print("type keypoints", type(kp_resp.data))

    # i = i+1
        return dl_imgResponse(ros_img, kp_resp)

    else:
        print("no image spawned")

  
def main():
    # Initialize the node
    rospy.init_node('goal_inference_gen')
    print("is main getting called")

    # Declaring the keypoints service
    kp_service = rospy.Service("franka_kp_dl_service", dl_img, dl_image_service)
    # subscriber for rgb image to detect markers
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback, queue_size=1)

    rospy.spin()

if __name__=='__main__':
    main()
