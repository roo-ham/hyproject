#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from basement import Basement

class VisionImage:
    def __init__(self, base:Basement):
        self.basement = base
        print("I'm VisionImage")
    def get_yellow(self):
        under_yellow = self.basement.img_h < 15
        over_yellow = self.basement.img_h > 35
        return ~(under_yellow | over_yellow)
    def get_white(self):
        over_sat = self.basement.img_s > 230
        over_bri = self.basement.img_v > 230
        return (over_sat & over_bri)
    def get_black(self):
        over_sat = self.basement.img_s > 230
        over_bri = self.basement.img_v < 50
        return (over_sat & over_bri)
    def update(self):
        self.timeout -= 1
        if self.timeout < 0 :
            self.basement.restart()
            return
        img = self.basement.get_bgr_bottom()
        yellow = self.get_yellow()
        white = self.get_white()
        black = self.get_black()
        img[:, :, 0] = np.where(yellow, 20, img[:, :, 0])
        img[:, :, 1] = np.where(yellow, 255, img[:, :, 1])
        img[:, :, 2] = np.where(yellow, 255, img[:, :, 2])

        img[:, :, 1] = np.where(white, 0, img[:, :, 1])
        img[:, :, 2] = np.where(white, 255, img[:, :, 2])

        img[:, :, 1] = np.where(black, 0, img[:, :, 1])
        img[:, :, 2] = np.where(black, 0, img[:, :, 2])
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)

class VisionMarker:
    def __init__(self, base:Basement):
        self.basement = base
        print("I'm VisionMarker")