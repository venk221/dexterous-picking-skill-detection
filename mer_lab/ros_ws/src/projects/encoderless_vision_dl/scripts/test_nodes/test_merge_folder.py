# Python 3 code to rename multiple
# files in a directory or folder

# importing os module
import os

file_path = '/home/jc-merlab/Pictures/Data/data_new_v1/'

# listing the file names in the desired folder in order
files = sorted(os.listdir(file_path))

int_stream = '000000'
i = 904
j = 904

for file_name in files:
    # print(file_name)
    if file_name.endswith('.jpg'):
    # Construct old file name
        source = file_path + file_name
        print(source)
        # print(len(str(i)))
        new_stream = int_stream[0:-len(str(i))]
        # print(new_stream)
        image_file = new_stream + str(i)
        # print(image_file)

        destination = file_path + image_file + '.rgb.jpg'
        print(destination)

        # rename all the files
        os.rename(source, destination)

        i = i + 1

    if file_name.endswith('.json'):
    # Construct old file name
        source = file_path + file_name
        # print(len(str(j)))
        new_stream = int_stream[0:-len(str(j))]
        # print(new_stream)
        image_file = new_stream + str(j)
        # print(image_file)

        destination = file_path + image_file + '.json'

        os.rename(source, destination)

        j = j + 1

    
    # Adding the count to the new file name and extension
        # destination = folder_dir + str(count) + ".jpg"

    # Renaming the file
        # os.rename(source, destination)
        # count = count + 1
# # Function to rename multiple files
# def main():

# 	folder = "xyz"
# 	for count, filename in enumerate(os.listdir(folder)):
# 		dst = f"Hostel {str(count)}.jpg"
# 		src =f"{folder}/{filename}" # foldername/filename, if .py file is outside folder
# 		dst =f"{folder}/{dst}"
		
# 		# rename() function will
# 		# rename all the files
# 		os.rename(src, dst)

# # Driver Code
# if __name__ == '__main__':
	
# 	# Calling main() function
# 	main()
