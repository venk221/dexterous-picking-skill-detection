#!/usr/bin/env python3.8

import torch
import torch.nn as nn
import torchvision
from torchvision.models.detection.rpn import AnchorGenerator


class KpRcnn:
        def get_model(self, num_keypoints, weights_path=None):
            self.anchor_generator = AnchorGenerator(sizes=(32, 64, 128, 256, 512), aspect_ratios=(0.25, 0.5, 0.75, 1.0, 2.0, 3.0, 4.0))
            self.model = torchvision.models.detection.keypointrcnn_resnet50_fpn(pretrained=False,
                                                                   pretrained_backbone=True,
                                                                   num_keypoints=num_keypoints,
                                                                   num_classes = 2, # Background is the first class, object is the second class
                                                                   rpn_anchor_generator=self.anchor_generator)
            if weights_path:
                print("is get model getting called")
                state_dict = torch.load(weights_path)
                self.model.load_state_dict(state_dict)        
        
            return self.model

class Resnet152():
    def __init__(self):
        print("running resnet152")
        