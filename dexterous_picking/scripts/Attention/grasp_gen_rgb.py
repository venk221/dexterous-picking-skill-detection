#!/usr/bin/env python

import glob
import os
import numpy as np
import torch
import sys
from PIL import Image as PILIMage
from skimage import io
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import transforms
import pyzed.sl as sl
from model import *
from util import SalObjDataset, ToTensorLab, RescaleT
from sensor_msgs.msg import Image
import rospy
import rospkg
import threading
import cv2
from datetime import datetime
from dexterous_picking.srv import GetGrasp, GetGraspResponse

from std_msgs.msg import Float32MultiArray, UInt32MultiArray

from cv_bridge import CvBridge
from geometry_msgs.msg import Point


def depth_to_bgra8(depth_image):
    depth_normalized = cv2.normalize(depth_image, None, 0, 1, cv2.NORM_MINMAX)
    depth_grayscale = (depth_normalized * 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(depth_grayscale, cv2.COLORMAP_JET)
    alpha_channel = np.ones(depth_colored.shape[:2], dtype=depth_colored.dtype) * 255
    bgra_image = cv2.merge((depth_colored, alpha_channel))

    return bgra_image

class U2net(object):
    def __init__(self):

        self.bridge = CvBridge()
        self.last_msg = None
        self.msg_lock = threading.Lock()
        self.predicting = False
        rospack = rospkg.RosPack()
        self.pkgdir = rospack.get_path('dexterous_picking')

        self.sub = rospy.Subscriber("/zed_camera/image", Image, self.callback_image, queue_size=1)
        self.pub = rospy.Publisher('/grasp_coordinates_list', Float32MultiArray, queue_size=10)
        self.pub_results = rospy.Publisher('/grasp_generation_results', Image, queue_size=10)

        self.service = rospy.Service("get_grasp_points_rgb", GetGrasp, self.get_grasp_points) 

        # self.service = rospy.Service("generate_grasp", GenerateGrasp, self.generate_grasp_point)
        self.init = sl.InitParameters()
        self.cam = sl.Camera()

    def get_grasp_points(self, request):

        while (not rospy.is_shutdown()) and self.msg_lock.acquire(False):
            img_msg = self.last_msg
            self.msg_lock.release()
            break
            
        image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgra8')
        self.process_image(image)
        response = GetGraspResponse()
        response.success = True
        return response
    
    def normPRED(self, d):
        ma = torch.max(d)
        mi = torch.min(d)
        dn = (d - mi) / (ma - mi)
        return dn

    def save_output(self, image_name, pred, d_dir, input_image):
        predict = pred
        predict = predict.squeeze()
        predict_np = predict.cpu().data.numpy()

        im = PILIMage.fromarray(predict_np * 255).convert('RGB')
        image = io.imread(image_name)
        # print("save output image shape", image.shape)
        imo = im.resize((image.shape[1], image.shape[0]), resample=PILIMage.BILINEAR)
        resized_image = cv2.resize(np.array(imo), (1920, 1080), interpolation=cv2.INTER_LINEAR)
        gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        gray_resized_image = cv2.resize(gray_image, (1920, 1080), interpolation=cv2.INTER_LINEAR)
        _, binary_image = cv2.threshold(gray_resized_image, 230, 255, cv2.THRESH_BINARY)


        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        coord_list = []

        for contour in contours:
            centroid = self.find_centroid(contour)
            coord_list.extend(centroid)
            # cv2.drawContours(input_image, [contour], 0, (255, 255, 255), cv2.FILLED)


        print(coord_list)
        msg = Float32MultiArray()
        msg.data = coord_list
        self.pub.publish(msg)

        red = [0,0,255]
        drawing_image = input_image.copy()

        for contour in contours:
            centroid = self.find_centroid(contour)
            cv2.circle(drawing_image, centroid, radius=5, color=red, thickness=-1)
            cv2.drawContours(drawing_image, [contour], 0, (255, 255, 255), cv2.FILLED)
        image_msg = self.bridge.cv2_to_imgmsg(drawing_image, encoding='bgra8')

        # save_dir = "/home/merlab/mer_lab/ros_ws/src/projects/dexterous_picking/scripts/detection_images"
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # file_name = f"experiment_generation_{timestamp}.jpg"
        # file_path = save_dir + file_name
        # cv2.imwrite(file_path, drawing_image)

        self.pub_results.publish(image_msg)

        rospy.sleep(0.1)

    def find_centroid(self, contour):
            M = cv2.moments(contour)
            if M["m00"] == 0:
                return None
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return (cX, cY)

    def process_image(self, image):

        # model_name = 'attu2netoutside_rgb_4.827609_tar_0.650952'
        model_name = 'attu2netoutside_bce_itr_144_train_4.420121_tar_0.626262'
        # model_name = 'attu2netoutside_bce_itr_108_train_4.442168_tar_0.629526'
        # model_name = 'attu2netoutside_bce_itr_90_train_4.456359_tar_0.631801'

        image_dir = os.path.join(self.pkgdir, 'scripts', 'Attention', 'test_data', 'test_images_DUTS-TE')
        prediction_dir = os.path.join(self.pkgdir, 'scripts', 'Attention', 'test_data', model_name + '_results' + os.sep)

        model_dir = os.path.join(self.pkgdir, 'scripts', 'Attention', 'saved_models', 'attu2netoutside', model_name + '.pth')

        img_name_list = []
        save_list =  glob.glob(image_dir + os.sep + '*')

        test_salobj_dataset = SalObjDataset(img_name_list=[image],
                                            lbl_name_list=[],
                                            transform=transforms.Compose([RescaleT(320), 
                                                                        ToTensorLab(flag=0)])
                                            )
        test_salobj_dataloader = DataLoader(test_salobj_dataset,
                                            batch_size=1,
                                            shuffle=False,
                                            num_workers=1)

        net = AttU2NetOutside(3, 1)

        if torch.cuda.is_available():
            checkpoint = torch.load(model_dir)
            net.load_state_dict(checkpoint['net_state_dict'])
            net.cuda()
        else:
            checkpoint = torch.load(model_dir, map_location='cpu')
            net.load_state_dict(checkpoint['net_state_dict'])

        net.eval()

        for i_test, data_test in enumerate(test_salobj_dataloader):

            print("inferencing:")

            inputs_test = data_test['image']
            inputs_test = inputs_test.type(torch.FloatTensor)

            if torch.cuda.is_available():
                inputs_test = Variable(inputs_test.cuda())
            else:
                inputs_test = Variable(inputs_test)

            d1, d2, d3, d4, d5, d6, d7 = net(inputs_test) 

            pred = d1[:, 0, :, :]
            pred = self.normPRED(pred)

            if not os.path.exists(prediction_dir):
                os.makedirs(prediction_dir, exist_ok=True)
            self.save_output(save_list[i_test], pred, prediction_dir, image)

            del d1, d2, d3, d4, d5, d6, d7

    def callback_image(self, msg):
        if self.msg_lock.acquire(False):
            self.last_msg = msg
            self.msg_lock.release()

def main(argv):
    rospy.init_node('grasp_generation_rgb_node')
    rospy.loginfo("Grasp Generation Node Started")
    node = U2net()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

