#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import cv2

class Basement:
    def __init__(self, name0:str = "release"):
        self.name = name0
        self.bottom_height = 32
        self.__bgr_full = np.zeros((256,256,3), np.uint8)
        self.__bgr_bottom = np.zeros((self.bottom_height,256,3), np.uint8)
        self.img_h, self.img_s, self.img_v = np.zeros((self.bottom_height,256), np.uint8),\
            np.zeros((self.bottom_height,256), np.uint8),\
                np.zeros((self.bottom_height,256), np.uint8)
        self.global_tan = 0.0
        self.local_tan = 0.0
        self.local_tan_sqaured = 0.0
    def set_bgr(self, full, bottom):
        self.__bgr_full = full
        self.__bgr_bottom = bottom
        bottom = cv2.blur(bottom, (5, 5), anchor=(-1, -1), borderType=cv2.BORDER_DEFAULT)
        img_hsv = cv2.cvtColor(bottom, cv2.COLOR_BGR2HSV)
        self.img_h, self.img_s, self.img_v = img_hsv[:, :, 0], img_hsv[:, :, 1], img_hsv[:, :, 2]
    def get_bgr_full(self) -> np.ndarray:
        return self.__bgr_full.copy()
    def get_bgr_bottom(self) -> np.ndarray:
        return self.__bgr_bottom.copy()