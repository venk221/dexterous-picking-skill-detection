import os
from datetime import datetime
from os.path import expanduser
import splitfolders
import shutil
import glob
from utils import DataPrePro

dpp = DataPrePro()

def train_test_split(src_dir):
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

        output = dpp.parent_path + "split_folder_latest" 

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

def main():
    path = dpp.parent_path + dpp.year + "-" + dpp.month + "-" + dpp.day + "/"
    train_test_split(path)



if __name__ == '__main__':
    main()