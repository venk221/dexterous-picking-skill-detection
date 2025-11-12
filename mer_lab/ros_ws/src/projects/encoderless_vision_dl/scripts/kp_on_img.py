import cv2
import json
import os
from datetime import datetime
from os.path import expanduser
from utils import DataPrePro

hp = DataPrePro()

def kp_on_img(img, jfile):
    inp_img = cv2.imread(file_path + img)
    j = jfile.split('.')[0]
    kp = []
    with open(file_path+jfile, encoding='utf-8', mode='r') as f:
        data = json.load(f)
        # kp1=[data["keypoints"][0][0][0], data["keypoints"][0][0][1]]
        # kp2=[data["keypoints"][1][0][0], data["keypoints"][1][0][1]]
        # kp3=[data["keypoints"][2][0][0], data["keypoints"][2][0][1]]
        # kp4=[data["keypoints"][3][0][0], data["keypoints"][3][0][1]]
        # kp5=[data["keypoints"][4][0][0], data["keypoints"][4][0][1]]
        # kp6=[data["keypoints"][5][0][0], data["keypoints"][5][0][1]]
        for i in range(len(data["keypoints"])):
            kp.append([data["keypoints"][i][0][0], data["keypoints"][i][0][1]])     

    # kp = [kp1, kp2, kp3, kp4, kp5]   

    for k in range(len(kp)):
            x = int(kp[k][0])
            y = int(kp[k][1])
            cv2.circle(inp_img,(x, y),5,(0,0,255),-1)

    cv2.imwrite(new_dir + j + ".jpg", inp_img)   


def main():
    global new_dir, file_path

    # Specifying path to store generated data for future use. User should be able to change the folder names as per requirement
    parent_path =  hp.home + "/Pictures/" + "Data/"

    # this is the day's dataset which needs to be tested for correct keypoints on image 
    # new_path = hp.year + "-" + hp.month + "-" + hp.day + "/"

    new_path = "/home/jc-merlab/Pictures/Data/data_new/data_new_v1/"

    # folder for main dataset
    file_path = os.path.join(parent_path, new_path)

    # this is where the tested images saved
    new_dir = parent_path + "test_new_data" + hp.year + "-" + hp.month + "-" + hp.day + "-" + hp.h + "-" + hp.m + "/"
    os.mkdir(new_dir)

    print(new_dir)

    files = sorted(os.listdir(file_path))

    img_lst = []
    json_lst = []

    for file in files:
        if (file.endswith(".jpg")):
            img_lst.append(file)
    for file in files:
        if (file.endswith(".json")):
            json_lst.append(file)

    for count in range(len(img_lst)):
        kp_on_img(img_lst[count], json_lst[count])


# latest changes
    

if __name__ == '__main__':
    main()