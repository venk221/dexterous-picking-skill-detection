
import cv2
import matplotlib.pyplot as plt
import os
from os import listdir
import matplotlib.image as img
import json


def visualize(file_number,image, keypoints, output_folder,show=False):
    fontsize = 18 

    colors = [(150,150,150),(153, 51, 102),(255,0,0),(0,255,0),(0,0,255),(255,255,0),(0,255,255),(0,128,128),(0,128,128),(0,128,128),(0,128,128)]
    i = 0
    for kps in keypoints:
        image = cv2.circle(image.copy(), tuple(kps), 4, colors[i], -1)
        cv2.imwrite(os.path.join(output_folder, f"img_{file_number}.jpg"), image)
        i+=1

    if show:
        plt.imshow(image)
        plt.show()

    return image

def read_show_image(file):
    testImage = img.imread(file)
    plt.imshow(testImage)
    plt.show()

def get_keypoints(folder_name):
    
    with open(folder_name) as f:
        data = json.load(f)
        keypoints_original = data['keypoints']
        keypoints = [[round(keypoints_original[i][0][0]),round(keypoints_original[i][0][1])] for i in range(len(keypoints_original))]
    return keypoints


def main():

    folder_name = '/home/fearless/Pictures/Data/2023-04-14'
    images_list = []
    output_folder = f'{folder_name}_output'
    show = True

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for file in os.listdir(folder_name):
        if (file.endswith(".jpg")):
            images_list.append(file)

    images_list.sort()
    
    for i in images_list:
        print(f"{i}")
        image = img.imread(folder_name + '/'+ i)
        file_number = i.split('.')[0]
        keypoints = get_keypoints(folder_name + '/' + file_number + '.json')
        visualize(file_number,image, keypoints,output_folder,show=False)

if __name__ == '__main__':
    main()
