#!/usr/bin/env python3.8

from datetime import datetime
import os
import albumentations as A
from os.path import expanduser
import splitfolders
import shutil
import glob

class DataPrePro:
    def __init__(self):

        # to generalize home directory. User can change their parent path without entering their home directory
        self.home = expanduser("~")

        # specifying variables to save dataset in folders according to date and time to keep track
        today = datetime.datetime.now()
        self.year = str(today.strftime('%Y'))
        self.month = str(today.strftime('%m'))
        self.day = str(today.strftime('%d'))
        self.h = str(today.hour)
        self.m = str(today.minute)       
        self.parent_path = self.home + "/Pictures/" + "Data/"
        self.root_dir = self.parent_path + self.year + "-" + self.month + "-" + self.day + "/"

    def train_transform(self):
        return A.Compose([
            A.Sequential([
                A.RandomRotate90(p=1), # Random rotation of an image by 90 degrees zero or more times
                A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, brightness_by_max=True, always_apply=False, p=1), # Random change of brightness & contrast
            ], p=1)
        ],
        keypoint_params=A.KeypointParams(format='xy'), # More about keypoint formats used in albumentations library read at https://albumentations.ai/docs/getting_started/keypoints_augmentation/
        bbox_params=A.BboxParams(format='pascal_voc', label_fields=['bboxes_labels']) # Bboxes should have labels, read more at https://albumentations.ai/docs/getting_started/bounding_boxes_augmentation/
    )
 
    def train_test_split(self, src_dir):
        dst_dir_img = src_dir + "images"
        dst_dir_anno = src_dir + "annotations"
 
        if os.path.exists(dst_dir_img) and os.path.exists(dst_dir_anno):
            print("folders exist")
        else:
            os.mkdir(dst_dir_img)
            os.mkdir(dst_dir_anno)
 
        for jpgfile in glob.iglob(os.path.join(src_dir, "*.jpg")):
            shutil.copy(jpgfile, dst_dir_img)
 
        for jsonfile in glob.iglob(os.path.join(src_dir, "*.json")):
            shutil.copy(jsonfile, dst_dir_anno)
 
        output = self.parent_path + "split_folder_output" + "-" + self.year + "-" + self.month + "-" + self.day 
 
        print(type(output))
 
        splitfolders.ratio(src_dir, # The location of dataset
                       output=output, # The output location
                       seed=42, # The number of seed
                       ratio=(.7, .2, .1), # The ratio of split dataset
                       group_prefix=None, # If your dataset contains more than one file like ".jpg", ".pdf", etc
                       move=False # If you choose to move, turn this into True
                       )
 
        shutil.rmtree(dst_dir_img)
        shutil.rmtree(dst_dir_anno)
 
        return output
 
 
    def read_yaml():
        path = Path('config/hyperparams.yaml')
        yaml = YAML(typ='safe')
        data = yaml.load(path)
# 
        return data
    # 
# 

# # Latest Changes