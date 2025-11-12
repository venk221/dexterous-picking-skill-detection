from os import listdir
import numpy as np
import cv2
import torch
import torchvision
import torchvision.transforms as transforms
from torchsummary import summary
from utils import DataPrePro
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import numpy as np
import glob
import time
from queue import Queue
from statistics import mean
from datetime import datetime
from PIL import Image
from torchvision.transforms import functional as F
from visualization import visualize
from arch_models import KpRcnn
import copy
from kalmanfilter import KalmanFilter


device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(device)

dpp = DataPrePro()

# hp = dpp.read_yaml()
weights_path = '/home/fearless/Pictures/Data/trained_models/keypointsrcnn_weights_ld_b3_e25.pth' # need to rewrite the path code
# weights_path = '/home/jc-merlab/Pictures/Data/trained_models/keypointsrcnn_weights_ld_b1_e25_v2.pth' # need to rewrite the path code
# model = KpRcnn()
# print(type(model))
# model = model.get_model(num_keypoints=hp['num_keypoints'], weights_path=weights_path).to(device)
# model.load_state_dict(torch.load('keypointsrcnn_weights.pth'))

model = torch.load(weights_path).to(device)
# model = model.load_state_dict(model).to(device)

keypoints = []

def vid_inf():
    global keypoints, frame
    # cap = cv2.VideoCapture('/home/jc-merlab/Pictures/Data/full_planar_output.avi')
    cap = cv2.VideoCapture('/home/fearless/Pictures/Data/test_data/videos/test101.avi')
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    i = 0
    while(cap.isOpened()):
      # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:        
    #         img = cv2.imread(frame)
            image = Image.fromarray(frame)

            image = F.to_tensor(image).to(device)
            image.unsqueeze_(0)
            image = list(image)

            with torch.no_grad():
                model.to(device)
                model.eval()
                output = model(image)

            print(type(output))
            image = (image[0].permute(1,2,0).detach().cpu().numpy() * 255).astype(np.uint8)
            scores = output[0]['scores'].detach().cpu().numpy()

            high_scores_idxs = np.where(scores > 0.7)[0].tolist() # Indexes of boxes with scores > 0.7
            print("high score index", high_scores_idxs)
            post_nms_idxs = torchvision.ops.nms(output[0]['boxes'][high_scores_idxs], \
                output[0]['scores'][high_scores_idxs], 0.3).cpu().numpy() # Indexes of boxes left after applying NMS (iou_threshold=0.3)

            # Below, in output[0]['keypoints'][high_scores_idxs][post_nms_idxs] and output[0]['boxes'][high_scores_idxs][post_nms_idxs]
            # Firstly, we choose only those objects, which have score above predefined threshold. This is done with choosing elements with [high_scores_idxs] indexes
            # Secondly, we choose only those objects,/home/jc-merlab/Pictures/Data/trained_models
            key_points = []
            for kps in output[0]['keypoints'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
                keypoints.append(list(map(int, kps[0,0:2])))
                # for kp in kps:
                    # print(kp)
                key_points.append([list(map(int, kp[:2])) for kp in kps])

            labels = []
            for label in output[0]['labels'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
                labels.append(label)

            # print(labels)
            # print(keypoints)

            keypoints_ = [x for _,x in sorted(zip(labels,keypoints))]
            # keypoints = sorted(keypoints, key=labels.index)
            # print(np.array(keypoints).shape)                

            # if len(keypoints) == 6:
            #     keypoints.pop(2)
            # print(keypoints_)
            # print(keypoints[0][0])

            bboxes = []
            for bbox in output[0]['boxes'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
                bboxes.append(list(map(int, bbox.tolist())))
            
            # print("key points", keypoints)
            img = visualize(image, bboxes, key_points)

            
            cv2.imwrite("/home/jc-merlab/Pictures/Data/video_results_01/out_image_" + str(i) + ".jpg", img)
            cv2.imwrite("/home/fearless/Pictures/Data/test_data/output/v100/out_image_" + str(i) + ".jpg", img)
            print(f"image {i} saved")
            # x1 = keypoints_[0][0]
            # y1 = keypoints_[0][1]

            # x2 = keypoints_[1][0]
            # y2 = keypoints_[1][1]

            # x3 = keypoints_[2][0]
            # y3 = keypoints_[2][1]

            # x4 = keypoints_[3][0]
            # y4 = keypoints_[3][1]

            # x5 = keypoints_[4][0]
            # y5 = keypoints_[4][1]


            # cv2.circle(frame, (int(x1), int(y1)), 5, (255,0,0),-1)
            # cv2.circle(frame, (int(x2), int(y2)), 5, (0,255,0),-1)
            # cv2.circle(frame, (int(x3), int(y3)), 5, (255,255,0),-1)
            # cv2.circle(frame, (int(x4), int(y4)), 5, (0,255,255),-1)
            # cv2.circle(frame, (int(x5), int(y5)), 5, (255,0,255),-1)

            # cv2.imwrite("/home/jc-merlab/Pictures/Data/video_results_02/out_image_" + str(i) + ".jpg", frame)

        i = i+1
    
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()

if __name__=='__main__':
    vid_inf()