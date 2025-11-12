#!/usr/bin/env python3.8

import torch
import torch.nn as nn
from ros_ws.src.projects.encoderless_vision_dl.scripts.preprocess import Hyperparams

hp = Hyperparams()

def curr_split():

    latest_split_folder = hp.parent_path + "split_folder_output" + "-" + hp.year + "-" + hp.month + "-" + hp.day

    print(latest_split_folder)

if __name__ == '__main__()':
    curr_split()