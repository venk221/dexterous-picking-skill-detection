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
# from scripts.test_nodes.utils_new import DataPrePro
from visualization import visualize
from kalmanfilter import KalmanFilter
import csv

device = 'cuda' if torch.cuda.is_available() else 'cpu'
weights_path = rospy.get_param('vsbot/deeplearning/weights_path')
model = torch.load(weights_path).to(device)
bridge = CvBridge()

kf1 = KalmanFilter(100, 7, 7, 7, 7)
kf2 = KalmanFilter(100, 7, 7, 7, 7)
kf3 = KalmanFilter(100, 7, 7, 7, 7)
kf4 = KalmanFilter(100, 7, 7, 7, 7)
kf5 = KalmanFilter(100, 7, 7, 7, 7)

class VideoInference:
    def __init__(self):
        self. kp_flag= [None, None, None, None, None]
        self.cv_image = None
        self.camera_K = None
        self.j = 0
        self.i = 0
        self.k = 1
        self.control_flag = False
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
        
        self.measured1_list = []
        self.measured2_list = []
        self.measured3_list = []
        self.measured4_list = []
        self.measured5_list = []

        self.updated1 = None
        self.updated2 = None
        self.updated3 = None
        self.updated4 = None
        self.updated5 = None

        self.executed = False
        self.corrected_x = None

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)

        self.filename = "/home/jc-merlab/Pictures/Data/plotting_data/keypoints.csv"
        self.col_name = ['kp1x', 'kp1y', 'kp2x', 'kp2y', 'kp3x', 'kp3y']
        f = open(self.filename, 'w')
        self.csvwriter = csv.writer(f)
        self.csvwriter.writerow(self.col_name)
# model.to(device)

# model.eval()

# print(type(model))
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name

    def kalman_estimate(self, img, feature):

            # print("input in kalman filter function", feature)       

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

            if self.kp_flag[0] == True:
                self.updated1 = np.asarray(kf1.update(measured1))
                # print("self.updated when True", self.updated)
                self.measured1_list = np.append(self.updated1, self.measured1_list) 
            elif self.kp_flag[0] == False:
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

            if self.kp_flag[1] == True:
                self.updated2 = np.asarray(kf2.update(measured2))
                # print("self.updated when True", self.updated)
                self.measured2_list = np.append(self.updated2, self.measured2_list) 
            elif self.kp_flag[1] == False:
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

            if self.kp_flag[2] == True:
                self.updated3 = np.asarray(kf3.update(measured3))
                # print("self.updated when True", self.updated)
                self.measured3_list = np.append(self.updated3, self.measured3_list) 
            elif self.kp_flag[2] == False:
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

            if self.kp_flag[3] == True:
                self.updated4 = np.asarray(kf4.update(measured4))
                # print("self.updated when True", self.updated)
                self.measured4_list = np.append(self.updated4, self.measured4_list) 
            elif self.kp_flag[3] == False:
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

            if self.kp_flag[4] == True:
                self.updated5 = np.asarray(kf5.update(measured5))
                # print("self.updated when True", self.updated)
                self.measured5_list = np.append(self.updated5, self.measured5_list) 
            elif self.kp_flag[4] == False:
                print("is update function getting called for 5th point")
                print("update output before False", self.measured5_list[0], self.measured5_list[1], self.measured5_list[2], self.measured5_list[3])
                # rospy.sleep(2)
                new_measured5 = np.array([[np.float32(self.measured5_list[0])], [np.float32(self.measured5_list[1])], [np.float32(self.measured5_list[2])], [np.float32(self.measured5_list[3])]])
                self.updated5 = np.asarray(kf5.update(new_measured5))
                self.measured5_list = np.append(self.updated5, self.measured5_list)
                # self.cx5_list = np.append(self.updated5[0], self.cx5_list)
                # self.cy5_list = np.append(self.updated5[1], self.cy5_list)

            # joint6
            # cx6 = feature[5][0]
            # cy6 = feature[5][1]

            # cx6_list = np.append(cx6, cx6_list)
            # cy6_list = np.append(cy6, cy6_list)

            # dx6 = cx6_list[0]-cx6_list[1]
            # dy6 = cy6_list[0]-cy6_list[1]

            # vx6 = dx6
            # vy6 = dy6

            # measured6 = np.array([[np.float32(cx6)], [np.float32(cy6)], [np.float32(vx6)], [np.float32(vy6)]])

            # predicted6 = kf6.predict()

            # if self.self.self.self.kp_flag[5] == True:            
            #     self.updated6 = np.asarray(kf6.update(measured6))
            #     # print("self.updated when True", self.updated)
            #     measured6_list = np.append(self.updated6, measured6_list) 
            # elif self.self.self.self.kp_flag[5] == False:
            #     print("is update function getting called for 6th point")
            #     print("update output before False", measured6_list[0], measured6_list[1], measured6_list[2], measured6_list[3])
            #     # rospy.sleep(2)
            #     new_measured6 = np.array([[np.float32(measured6_list[0])], [np.float32(measured6_list[1])], [np.float32(measured6_list[2])], [np.float32(measured6_list[3])]])
            #     print("new measured 6", new_measured6)
            #     self.updated6 = np.asarray(kf6.update(new_measured6))
            #     measured6_list = np.append(self.updated6, measured6_list)
            #     # cx6_list = np.append(self.updated6[0], cx6_list)
            #     # cy6_list = np.append(self.updated6[1], cy6_list)

            pred_x4, pred_y4 = int(predicted4[0][0]), int(predicted4[1][0])   
            upd_x4, upd_y4 = int(self.updated4[0]), int(self.updated4[1])

            pred_x5, pred_y5 = int(predicted5[0][0]), int(predicted5[1][0])   
            upd_x5, upd_y5 = int(self.updated5[0]), int(self.updated5[1])

            # pred_x6, pred_y6 = int(predicted6[0][0]), int(predicted6[1][0])   
            # upd_x6, upd_y6 = int(self.updated6[0]), int(self.updated6[1])


            # # joint7
            # cx7 = feature[6][0]
            # cy7 = feature[6][1]

            # cx7_list = np.append(cx7, cx7_list)
            # cy7_list = np.append(cy7, cy7_list)

            # dx7 = cx7_list[0]-cx7_list[1]
            # dy7 = cy7_list[0]-cy7_list[1]

            # vx7 = dx7
            # vy7 = dy7

            # measured7 = np.array([[np.float32(cx7)], [np.float32(cy7)], [np.float32(vx7)], [np.float32(vy7)]])

            # predicted7 = kf7.predict()


            # if self.self.self.self.kp_flag[6] == True:
            #     self.updated7 = np.asarray(kf7.update(measured7))
            #     # print("self.updated when True", self.updated)
            #     measured7_list = np.append(self.updated7, measured7_list) 
            # elif self.self.self.self.kp_flag[6] == False:
            #     print("is update function getting called for 7th point")
            #     print("update output before False", measured7_list[0], measured7_list[1], measured7_list[2], measured7_list[3])
            #     # rospy.sleep(2)
            #     new_measured7 = np.array([[np.float32(measured7_list[0])], [np.float32(measured7_list[1])], [np.float32(measured7_list[2])], [np.float32(measured7_list[3])]])
            #     self.updated7 = np.asarray(kf7.update(new_measured7))
            #     measured7_list = np.append(self.updated7, measured7_list)
            #     # cx7_list = np.append(self.updated7[0], cx7_list)
            #     # cy7_list = np.append(self.updated7[1], cy7_list)

            # pred_x7, pred_y7 = int(predicted7[0][0]), int(predicted7[1][0])   
            # upd_x7, upd_y7 = int(self.updated7[0]), int(self.updated7[1])

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
            # cv2.circle(img, (int(cx6), int(cy6)), 5, blue, -1)
            # cv2.circle(img, (int(cx7), int(cy7)), 5, blue, -1)


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
            # cv2.circle(img, (upd_x6, upd_y6), 5, red, 4)

            # cv2.circle(img, (pred_x7, pred_y7), 10, yellow, 4)
            # cv2.circle(img, (upd_x7, upd_y7), 5, red, 4)

            # cv2.putText(img, str(measured5), org1, font, fontScale, green, 1, cv2.LINE_AA)
            # cv2.putText(img, str(measured6), org2, font, fontScale, red, 1, cv2.LINE_AA)
            # cv2.putText(img, str(measured7), org3, font, fontScale, yellow, 1, cv2.LINE_AA)

            # cv2.putText(img, str(self.updated5), org4, font, fontScale, green, 2, cv2.LINE_AA)
            # cv2.putText(img, str(self.updated6), org5, font, fontScale, red, 2, cv2.LINE_AA)
            # cv2.putText(img, str(self.updated7), org6, font, fontScale, yellow, 2, cv2.LINE_AA)
            # # cv2.circle(img, (int(cx1), int(cy1)), 5, green, -1)
            # cv2.putText(img, str(predicted5[0:4]), org2, font, fontScale, yellow, 1, cv2.LINE_AA)
            # cv2.putText(img, str(predicted5[4:]), org3, font, fontScale, yellow, 1, cv2.LINE_AA)
            # # cv2.circle(img, (pred_x, pred_y), 10, yellow, 4)
            # cv2.putText(img, str(self.updated5[0:4]), org4, font, fontScale, red, 1, cv2.LINE_AA)
            # cv2.putText(img, str(self.updated5[4:]), org5, font, fontScale, red, 1, cv2.LINE_AA)
            # cv2.circle(img, (upd_x, upd_y), 5, red, 4)

            print("next frame"+str(self.j))

            # cv2.imwrite("/home/jc-merlab/Pictures/Data/video_results_live_b2e25_kalman/out_image_"+str(self.j)+".jpg", img)                 

            self.j= self.j + 1
            print(self.j)

            latest_corrected_x = np.array([[self.updated1[0][0], self.updated1[1][0]], [self.updated2[0][0], self.updated2[1][0]], [self.updated3[0][0], self.updated3[1][0]], 
                                            [self.updated4[0][0], self.updated4[1][0]], [self.updated5[0][0], self.updated5[1][0]]])

            return latest_corrected_x

        # runs only once, when all 5 joints are identified for the first time
    def first_input_estimate(self, img, key_points):
            print("first filter is called")
            # print("Actual first first key points", key_points)
            self.kp_flag= [True, True, True, True, True]
            first_corrected_x = self.kalman_estimate(img, key_points)        
            # print("First Corrected X", first_corrected_x)
            return first_corrected_x        

        # this is the function where the missing joints are identified for 
    def input_estimation(self, img, key_points, corrected_x):
            print("filters are called")        
            # print("key points input after the first iteration", key_points)
            # print("corredtec_x after first iteration", corrected_x)

            latest_x = np.array([[-1, -1], [-1,-1], [-1, -1], 
                         [-1, -1], [-1, -1]])
            # if len(key_points) == 7 and (155 < i < 150):        
            # if len(key_points) == 5 and (150 < self.i < 161):
            #     print("key point 5 but some missing")
            #     key_points[-1] = [-1, -1]
            #     key_points[-2] = [-1, -1]
            #     print(key_points)
            #     self.kp_flag= [True, True, True, False, False]
            #     latest_corrected_x = self.kalman_estimate(img, key_points)

            # elif len(key_points) == 5 and (180 < self.i < 191):
            #     print("key point 7 but some missing")
            #     key_points[-1] = [-1, -1]
            #     key_points[-2] = [-1, -1]
            #     self.kp_flag= [True, True, True, False, False]
            #     latest_corrected_x = self.kalman_estimate(img, key_points)

            if len(key_points) == 5:
                self.kp_flag= [True, True, True, True, True]
                latest_corrected_x = self.kalman_estimate(img, key_points)

            else:
                for i in range(len(key_points)):
                    distances = np.sqrt((corrected_x[:, 0] - key_points[i][0])**2 + (corrected_x[:, 1] - key_points[i][1])**2)
                    nearest_index = np.argmin(distances)
                    latest_x[nearest_index] = key_points[i]
                    # print("input after the first estimation", latest_x)

                for i in range(len(latest_x)):
                    if ([-1,-1] == latest_x[i]).all():
                        # print(([-1,-1] == latest_x[i]).all())
                        self.kp_flag[i] = False
                        # print("self.self.self.self.self.kp_flagarray", self.self.self.self.kp_flag)
                latest_corrected_x = self.kalman_estimate(img, latest_x)
                # print("kp_fla array when length less than 7", self.kp_flag)

            # print("latest corrected X in input estimation", latest_corrected_x)
            # rospy.sleep(3)
            # k = k + 1
            return latest_corrected_x


    def image_callback(self, msg):
        # global cv_img, model, CvBridge, i
        print("is image call back getting called")
        ros_img = msg

        if ros_img is not None:
            self.cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
            cv2.imwrite("/home/jc-merlab/Pictures/Data/plotting_data/raw_images_dl/keypoint_img_" + str(self.i) + ".png", self.cv_img)

            image = Img.fromarray(self.cv_img)

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

            indices = [0,1,3,4,5]
            keypoints_ = [keypoints_[i] for i in indices]

            # print(keypoints)
            print(len(keypoints))
            bboxes = []
            for bbox in output[0]['boxes'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
                bboxes.append(list(map(int, bbox.tolist())))

            # print("key points", keypoints)
            # img = visualize(cv_img, bboxes, key_points)
            # cv2.imwrite("/home/jc-merlab/Pictures/Data/video_results_live_b1e25_v3/out_image_" + str(i) + ".jpg", img)

            # kp = np.reshape(np.array(kp), (-1, 2))

            if len(keypoints_) == 5 and not self.executed:
                # print("first key points input", kp)
                    self.corrected_x = self.first_input_estimate(self.cv_img, keypoints_)
                    self.executed = True
                # print("Corrected_X", self.corrected_x)                       
            
            elif self.executed:
                # print("Executed?", self.executed)               
                self.corrected_x = self.input_estimation(self.cv_img, keypoints_, self.corrected_x)

        
        kp_x = []
        kp_y = []
        for i in range(len(self.corrected_x)):
            x = np.int64(self.corrected_x[i][0])
            y = np.int64(self.corrected_x[i][1])
            cv2.circle(self.cv_img,(x, y),10,(255,0,255),-1)
            kp_x.append(x)
            kp_y.append(y)
        kp = []
        for i in range(len(kp_x)-2):
           kp.append(kp_x[i+2]) 
           kp.append(kp_y[i+2])  
        
        self.csvwriter.writerow(kp)

        cv2.imwrite("/home/jc-merlab/Pictures/Data/video_results_full_b1e30/out_image_" + str(self.i) + ".jpg", self.cv_img)        

        self.i = self.i+1

def main():
    # Initialize the node
    rospy.init_node('live_inference_kp_detection')
    print("is main getting called")
    # subscriber for rgb image to detect markers
    # while not rospy.is_shutdown():
    kp_obj = VideoInference()
        # print("keep spinning")

    rospy.spin()

if __name__=='__main__':
    main()
