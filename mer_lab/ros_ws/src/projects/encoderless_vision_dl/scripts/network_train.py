#!/usr/bin/env python3.8
import numpy as np
import torch
import torch.nn as nn
import torchvision
import utils
from arch_models import KpRcnn, Resnet152
from utils import DataPrePro, collate_fn
from datasets import KpRcnnDataset
from torch.utils.data import Dataset, DataLoader
from engine import train_one_epoch
from visualization import visualize

dpp = DataPrePro()

def train_kprcnn():
    hp = dpp.read_yaml()

    KEYPOINTS_FOLDER_TRAIN = dpp.curr_split('train') +"/train"
    KEYPOINTS_FOLDER_VAL = dpp.curr_split('val') +"/val"
    KEYPOINTS_FOLDER_TEST = dpp.curr_split('test') + "/test"
        
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

    print(device)

    dataset_train = KpRcnnDataset(KEYPOINTS_FOLDER_TRAIN, transform=dpp.train_transform(), demo=False)
    data_loader_train = DataLoader(dataset_train, batch_size=hp['batch_size_train'], shuffle=True, collate_fn=collate_fn)

    dataset_val = KpRcnnDataset(KEYPOINTS_FOLDER_VAL, transform=None, demo=False)
    data_loader_val = DataLoader(dataset_val, batch_size=1, shuffle=True, collate_fn=collate_fn)

    dataset_test = KpRcnnDataset(KEYPOINTS_FOLDER_TEST, transform=None, demo=False)
    data_loader_test = DataLoader(dataset_test, batch_size=1, shuffle=False, collate_fn=collate_fn)    

    model = KpRcnn()
    model = model.get_model(num_keypoints=hp['num_keypoints']).to(device)
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=hp['lr'], momentum=hp['momentum'], weight_decay=hp['weight_decay'])
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=hp['step_size'], gamma=hp['gamma'])
    
    for epoch in range(hp['num_epoch']):
        train_one_epoch(model, optimizer, data_loader_train, device, epoch, print_freq=hp['print_freq'])
        lr_scheduler.step()

    torch.save(model.state_dict(), dpp.parent_path + "trained_models/keypointsrcnn_weights" + '-' + dpp.day + '-' + dpp.month + '-' + dpp.year + '.pth')

    iterator = iter(data_loader_val)

    images, targets = next(iterator)

    images = list(image.to(device) for image in images)

    with torch.no_grad():
        model.eval()
        output = model(image)

    image = (images[0].permute(1,2,0).detach().cpu().numpy() * 255).astype(np.uint8)
    scores1 = output[0]['scores'].detach().cpu().numpy()

    high_scores_idxs = np.where(scores1 > 0.7)[0].tolist() # Indexes of boxes with scores > 0.7
    post_nms_idxs = torchvision.ops.nms(output[0]['boxes'][high_scores_idxs], output[0]['scores'][high_scores_idxs], 0.3).cpu().numpy() # Indexes of boxes left after applying NMS (iou_threshold=0.3)

    # Below, in output[0]['keypoints'][high_scores_idxs][post_nms_idxs] and output[0]['boxes'][high_scores_idxs][post_nms_idxs]
    # Firstly, we choose only those objects, which have score above predefined threshold. This is done with choosing elements with [high_scores_idxs] indexes
    # Secondly, we choose only those objects, which are left after NMS is applied. This is done with choosing elements with [post_nms_idxs] indexes

    keypoints = []
    for kps in output[0]['keypoints'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
        keypoints.append([list(map(int, kp[:2])) for kp in kps])

    bboxes = []
    for bbox in output[0]['boxes'][high_scores_idxs][post_nms_idxs].detach().cpu().numpy():
        bboxes.append(list(map(int, bbox.tolist())))

    visualize(image, bboxes, keypoints)

def train_resnet():
    model = Resnet152()



# if __name__ == "__main__":
#     train()
    


