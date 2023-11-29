#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import roslaunch, os
import numpy as np
from pathlib import Path

class Basement:
    def __init__(self, name0:str = "release"):
        self.name = name0
        self.bottom_height = 32
        self.__bgr_full = np.zeros((256,256,3), np.uint8)
        self.__bgr_bottom = np.zeros((self.bottom_height,256,3), np.uint8)
        self.img_h, self.img_s, self.img_v = np.zeros((self.bottom_height,256), np.uint8),\
            np.zeros((self.bottom_height,256), np.uint8),\
                np.zeros((self.bottom_height,256), np.uint8)
        self.taskmodules = dict()
        self.tick = 0
        self.real_speed_x = 0.0
        self.real_speed_z = 0.0
        self.launch = dict()

    def roslaunch(self, filename):
        if filename in self.launch:
            return
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        path = Path(os.path.abspath(__file__)).parent.parent.joinpath("launch/%s"%filename)
        self.launch[filename] = roslaunch.parent.ROSLaunchParent(uuid, [str(path)])
        self.launch[filename].start()

    def set_bgr(self, full, bottom):
        self.__bgr_full = full
        self.__bgr_bottom = bottom

    def set_hsv(self, origin):
        self.img_h, self.img_s, self.img_v = origin[:, :, 0], origin[:, :, 1], origin[:, :, 2]

    def get_bgr_full(self) -> np.ndarray:
        return self.__bgr_full.copy()
    
    def get_bgr_bottom(self) -> np.ndarray:
        return self.__bgr_bottom.copy()