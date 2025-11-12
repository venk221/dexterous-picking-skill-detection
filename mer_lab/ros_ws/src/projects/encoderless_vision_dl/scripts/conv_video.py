import cv2
import numpy as np
import glob
import shutil
import os


# Define the parameters for the video
fps = 15
frame_width = 640
frame_height = 480

# Define the output file name and format
out_file = '/home/fearless/Pictures/Data/better2/output.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')

# Get the list of images in the folder
img_folder = '/home/fearless/Pictures/Data/better2/2023-04-11_output'
img_files = [f for f in sorted(os.listdir(img_folder)) if f.endswith('.jpg')]
# img_files.sort(key=lambda f: int(f[:-4]))
# img_files.sort(key=lambda x: int(x.split('_')[2].split('.')[0]))
img_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))
print("img_files",img_files)

# Create the video writer object
out = cv2.VideoWriter(out_file, fourcc, fps, (frame_width, frame_height))

# Read each image and write it to the video
for img_file in img_files:
    img = cv2.imread(os.path.join(img_folder, img_file))
    out.write(img)

# Release the video writer object
out.release()

# def copy_files(src_dir, dst_dir):
#     # To copy from one folder to another (working tested ok)
#     # src_dir = "2D_proj_ds"
#     # dst_dir = "test"
#     for jpgfile in glob.iglob(os.path.join(src_dir, "*rgb.jpg")):
#         shutil.copy(jpgfile, dst_dir)

#     print("copied all files")


# def create_video(folder_name, extension, video_name):
#     # folder_name = '/home/fearless/Pictures/Data/oct21_v1/images'
#     # extension = '.jpg'
#     # video_name = 'oct21_v1.avi'
    
#     pictures = glob.glob(folder_name + '/out_image_*' + extension)

#     img_array = []

#     # To make an array of all images in ascending order
#     # for i in range(30, len(pictures)):
#     for i in range(len(pictures)):
#         # imge = cv2.imread(pictures[pictures.index(
#             # folder_name+'/out_image_'+(str(i)).zfill(3) + '.jpg')])
        
#         imge = cv2.imread(pictures[pictures.index(
#             folder_name+'/out_image_'+(str(i)) + '.jpg')])

#         img_array.append(imge)

#     # To get size of the array
#     img = cv2.imread(pictures[0])
#     height, width, layers = img.shape
#     size = (width, height)

#     # To create a video

#     out = cv2.VideoWriter(
#         video_name, cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

#     for i in range(len(img_array)):
#         out.write(img_array[i])
#     out.release()

#     print("Video Created")


# def create_video_2(folder_name, video_name):
#     # folder_name = '/home/fearless/Pictures/Data/oct21_v1/images'
#     # extension = '.jpg'
#     # video_name = 'oct21_v1.avi'

#     # pictures = glob.glob(folder_name + '/*.rgb' + extension)
#     pictures = glob.glob(folder_name)

#     print(pictures)

#     img_array = []

#     # To make an array of all images in ascending order
#     for i in range(len(pictures)):
#         imge = cv2.imread(pictures[pictures.index(folder_name+'image_' + str(i) + '.jpg')])

#         img_array.append(imge)

#     # To get size of the array
#     img = cv2.imread(pictures[0])
#     height, width, layers = img.shape
#     size = (width, height)

#     # To create a video

#     out = cv2.VideoWriter(
#         video_name, cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

#     for i in range(len(img_array)):
#         out.write(img_array[i])
#     out.release()

#     print("Video Created")


# def rename_align(src_dir, ext, name, str_no):

#     # takes input as a folder name and extention to rename, and rename starting point
#     # make a list of all files
#     # arranges all files in order
#     # finds the starting files name of that extention
#     # renames that file from given start number

#     # files = glob.glob(src_dir+'/*'+ext)
#     files = glob.glob(src_dir+'/image*'+ext)
#     print(files[0])
#     print("len(files): ", len(files))

#     first_img_no = 0
#     while True:
#         if os.path.isfile(src_dir+'/image'+str(first_img_no) + ext):
#             break
#         if first_img_no > 100000:
#             return print("Files does not exist")
#         first_img_no += 1

#     print("starting_point: ", first_img_no)

#     for i in range(first_img_no, 1000):  # first_img_no+len(files)):
#         try:
#             old_name = files[files.index(src_dir+'/image'+str(i) + ext)]
#             # new_name = src_dir+'/' + name + str(str_no) + ext
#             new_name = src_dir+'/' + name + str(str_no) + ext
#             os.rename(old_name, new_name)
#             print(old_name)
#             str_no += 1

#         except:
#             print(f"file {i} does not exist")


# if __name__ == "__main__":

#     # src_dir = "/home/fearless/Pictures/Data/nov1_v1/images"
#     # name = "image"
#     # ext = '.jpg'
#     # # ext = '.json'
#     # # dst_dir = "/home/fearless/Pictures/Data/oct21_v1/images3"
#     # str_no = 1
#     # rename_align(src_dir, ext, name, str_no)

#     folder_name = '/home/jc-merlab/Pictures/Data/video_images/rgb/'
#     extension = '.jpg'
#     video_name = '/home/jc-merlab/Pictures/Data/inference_data/b1e25_v2.avi'

#     create_video_2(folder_name, video_name)

#     # folder_name = '/home/fearless/VBM_DR/Abhay_Inference_req/2022-12-12'
#     # extension = '.jpg'
#     # video_name = '/home/fearless/VBM_DR/Abhay_Inference_req/test101.avi'

#     # create_video_2(folder_name, extension, video_name)

#     # src_dir = "2D_proj_ds"
#     # dst_dir = "test"
#     # copy_files(src_dir , dst_dir)
