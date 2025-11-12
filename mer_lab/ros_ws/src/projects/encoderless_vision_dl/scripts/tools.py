import os 
import datetime
import shutil




def rename_files(folder1, str_no):

	file_names_jpg = []
	file_names_json = []
	for filename in os.listdir(folder1):
		if filename.endswith(".json"):
			file_names_json.append(filename)

		if filename.endswith(".jpg"):
			file_names_jpg.append(filename)

	file_names_jpg.sort()
	file_names_json.sort()

	print(f"sorted files start no {str_no} and {folder1}")


	for inx in range(len(file_names_jpg)):
		src = f"{folder1}/{file_names_jpg[inx]}"
		dst = f"{folder1}/{str(str_no + inx).zfill(6)}.rgb.jpg"

		# print(inx,src,dst)
		os.rename(src, dst)
		inx += 1
		

	for inx in range(len(file_names_json)):
		src = f"{folder1}/{file_names_json[inx]}"
		dst = f"{folder1}/{str(str_no + inx).zfill(6)}.json"

		# print(inx,src,dst)
		os.rename(src, dst)
		inx += 1

	# print(os.listdir(folder1))
	print(f"renamed start no {str_no} and {folder1}")

	return str_no + inx


def move_files(src_folder, dst_folder):

	files = os.listdir(src_folder)
	
	for file in files:
		src_path = os.path.join(src_folder, file)
		dst_path = os.path.join(dst_folder, file)
		shutil.copy2(src_path, dst_path)
	
	print(f"File moved from : {src_folder} to: {dst_folder}")


def main():
	now = datetime.datetime.now()


	folders =["/home/fearless/Pictures/Data/Dataset_EE _v2/Combined_dataset_16-02-2023 20:48:15",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v3",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v4",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v5",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v6",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v7",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v8",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v9",
	"/home/fearless/Pictures/Data/Dataset_EE _v2/data_v10"]
	
	path = os.path.expanduser("~/Pictures/Data")

	name_of_file = "Combined_dataset_"
	timestamp = now.strftime("%d-%m-%Y %H:%M:%S")

	output_folder = name_of_file + timestamp

	output_folder = os.path.join(path, output_folder)

	if not os.path.exists(output_folder):
		os.makedirs(output_folder)



	str_no = 0	
	for f_no in range(len(folders)):
		end_number = rename_files(folders[f_no],str_no)

		move_files(folders[f_no], output_folder)

		str_no = end_number




	# Loop through the files and move them to the destination folder
	

if __name__ == "__main__":
	main()


















	# folder1 = "2023-02-08"

	# file_names_jpg = []
	# file_names_json = []
	# for filename in os.listdir(folder1):
	# 	if filename.endswith(".json"):
	# 		file_names_json.append(filename)

	# 	if filename.endswith(".jpg"):
	# 		file_names_jpg.append(filename)

	# file_names_jpg.sort()
	# file_names_json.sort()


	# for inx in range(len(file_names_jpg)):
	# 	src = f"{folder1}/{file_names_jpg[inx]}"
	# 	dst = f"{folder1}/{str(inx).zfill(6)}.rgb.jpg"

	# 	print(inx,src,dst)
	# 	os.rename(src, dst)
	# 	inx += 1
		

	# for inx in range(len(file_names_json)):
	# 	src = f"{folder1}/{file_names_json[inx]}"
	# 	dst = f"{folder1}/{str(inx).zfill(6)}.json"

	# 	print(inx,src,dst)
	# 	os.rename(src, dst)
	# 	inx += 1

	# print(os.listdir(folder1))