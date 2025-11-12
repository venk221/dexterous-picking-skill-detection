#!/usr/bin/env python3.8

from datetime import datetime
# import rospy
import os
from os.path import expanduser

class Def_Path:
    def __init__(self):

        # to generalize home directory. User can change their parent path without entering their home directory
        self.home = expanduser("~")

        # specifying variables to save dataset in folders according to date and time to keep track
        today = datetime.now()
        self.year = str(today.strftime('%Y'))
        self.month = str(today.strftime('%m'))
        self.day = str(today.strftime('%d'))
        self.h = str(today.hour)
        self.m = str(today.minute)


# Latest Changes