import os, shutil
from os.path import expanduser
import fnmatch
from datetime import datetime

home = expanduser("~")
dir_path = home + "/Pictures/Data/"
dir_list = os.listdir(dir_path)

today = datetime.now()
year = str(today.strftime('%Y'))
month = str(today.strftime('%m'))
day = str(today.strftime('%d'))
h = str(today.hour)
m = str(today.minute)
parent_path =  home + "/Pictures/" + "Data/"
new_path = "merge_folder" + "-" + year + "-" + month + "-" + day + "-" + h + "-" + m + "/"

# # This block is to create a merge folder
# merge_folder = os.path.join(parent_path, new_path)

# os.mkdir(merge_folder)

# # We are creating more than one folder and the files in each folders starts with index 0
# # In the following block we are changing the indices of the folders other than the first folder from the file index after the last file in the first folder

# # take the number of files in first folder
# file_count = int((len(fnmatch.filter(os.listdir(dir_path+dir_list[0]), '*.*'))/2))

# # take the number of folders in the parent directory
# dir_count = len(dir_list)

# # we are taking the .jpg files and changing the names as indexed after the first folder for the rest of the folders
# for i in range(dir_count-1):
#     for file_name in sorted(os.listdir(dir_path+dir_list[i+1])):
#         if file_name.endswith('jpg'):
#             source = dir_path+dir_list[i+1] + "/" + file_name
#             destination =  merge_folder + "/" + str(file_count) + ".jpg"
#             os.rename(source, destination)        
#             file_count = file_count + 1
            
# # take the number of files in first folder
# file_count = int((len(fnmatch.filter(os.listdir(dir_path+dir_list[0]), '*.*'))/2))

# # we are taking the .jpg files and changing the names as indexed after the first folder for the rest of the folders
# for i in range(dir_count-1):
#     for file_name in sorted(os.listdir(dir_path+dir_list[i+1])):
#         if file_name.endswith('json'):
#             source = dir_path+dir_list[i+1] + "/" + file_name
#             destination = merge_folder + "/" + str(file_count) + ".json"
#             os.rename(source, destination)
#             file_count = file_count+1

# # list the new numbers of directory
# dir_list = os.listdir(dir_path)

# dir_count = len(dir_list)

# # delete the empty directories
# for i in range(dir_count):
#     if len(os.listdir(dir_path+dir_list[i])) == 0:
#         os.rmdir(dir_path+dir_list[i])

# # # new list of directories without empty ones
# new_dir_list = os.listdir(dir_path)

# # # enumerate on new_dir_list to get the 
# # content of all the folders and store it in a dictionary
# content_list = {}
# for index, val in enumerate(new_dir_list):
#     path = os.path.join(dir_path, val)
#     content_list[new_dir_list[index] ] = os.listdir(path)

# # # merge two folder by moving files from one folder to another
# source = dir_path + new_dir_list[0]
# destination = dir_path + new_dir_list[1]

# # gather all files
# allfiles = os.listdir(source)
 
# # iterate on all files to move them to destination folder
# for f in allfiles:
#     src_path = os.path.join(source, f)
#     dst_path = os.path.join(destination, f)
#     shutil.move(src_path, dst_path)

# dir_count = len(new_dir_list)

# # # remove the remaining empty directory
# for i in range(dir_count):
#     if len(os.listdir(dir_path+new_dir_list[i])) == 0:
#         os.rmdir(dir_path+new_dir_list[i])

# # now you have the dataset to train for the day 


folder1 = "/home/jc-merlab/Pictures/Data/data_new_v2/"
folder2 = "/home/jc-merlab/Pictures/Data/data_new_v1/"

# Get the maximum file index in folder1
file_index = 0
for filename in os.listdir(folder1):
    if filename.endswith(".json"):
        file_index = max(file_index, int(filename.split(".")[0]))

# Loop through each file in folder2 and copy it to folder1
for filename in os.listdir(folder2):
    if filename.endswith(".json"):
        file_index += 1
        src_file = os.path.join(folder2, filename)
        dst_file = os.path.join(folder1, f"{str(file_index).zfill(6)}.json")
        shutil.copy(src_file, dst_file)
        src_img = os.path.join(folder2, filename.replace(".json", ".rgb.jpg"))
        dst_img = os.path.join(folder1, f"{str(file_index).zfill(6)}.rgb.jpg")
        shutil.copy(src_img, dst_img)