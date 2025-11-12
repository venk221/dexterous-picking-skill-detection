import cv2
import json
import os
from datetime import datetime
from os.path import expanduser
from utils import DataPrePro

hp = DataPrePro()


def main():
    global new_dir, file_path

    # Specifying path to store generated data for future use. User should be able to change the folder names as per requirement
    # parent_path =  hp.home + "/Pictures/" + "Data/"

    # this is the day's dataset which needs to be tested for correct keypoints on image 
    # new_path = hp.year + "-" + hp.month + "-" + hp.day + "/"

    # folder for main dataset
    # file_path = os.path.join(parent_path, new_path)

    file_path = '/home/jc-merlab/Pictures/Data/2023-01-12/'

    # print(file_path)

    # # this is where the tested images saved
    # new_dir = parent_path + "test_new_data" + hp.year + "-" + hp.month + "-" + hp.day + "-" + hp.h + "-" + hp.m + "/"
    # os.mkdir(new_dir)

    files = sorted(os.listdir(file_path))    

    # for file in files:
    #     if (file.endswith(".json")):
    #         # print(file)
    #         with open(file_path+file, encoding='utf-8', mode='r') as f:
    #             data = json.load(f)
    #             del data["keypoints"][2]
    #             del data["bboxes"][2]
    #             json_obj = json.dumps(data, indent=4)
    #             filename = file_path + file
    #             with open(filename, "w") as outfile:
    #                     outfile.write(json_obj)
    for file in files:
        if (file.endswith(".json")):
            # print(file)
            with open(file_path+file, encoding='utf-8', mode='r') as f:
                data = json.load(f)
                print(data["keypoints"][0][0])
                data["keypoints"][0][0][3]=1
                data["keypoints"][1][0][3]=1
                data["keypoints"][2][0][3]=1
                data["keypoints"][3][0][3]=1
                data["keypoints"][4][0][3]=1
                data["keypoints"][0][0][2]=10
                data["keypoints"][1][0][2]=11
                data["keypoints"][2][0][2]=12
                data["keypoints"][3][0][2]=13
                data["keypoints"][4][0][2]=14
                print(data["keypoints"])
                json_obj = json.dumps(data, indent=4)
                filename = file_path + file
                with open(filename, "w") as outfile:
                        outfile.write(json_obj)
                
    

# latest changes
    

if __name__ == '__main__':
    main()