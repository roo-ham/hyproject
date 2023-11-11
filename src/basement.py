#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np

class Basement:
    def __init__(self, name0:str = "release"):
        self.name = name0
        self.__bgr_full = np.zeros((256,256,3), np.uint8)
        self.__bgr_bottom = np.zeros((128,256,3), np.uint8)
    def set_bgr(self, full:np.ndarray, bottom:np.ndarray):
        self.__bgr_full = full
        self.__bgr_bottom = bottom
    def get_bgr_full(self) -> np.ndarray:
        return self.__bgr_full.copy()
    def get_bgr_bottom(self) -> np.ndarray:
        return self.__bgr_bottom.copy()
    def restart(self):
        rospy.signal_shutdown("restarting hyproject...")
        rospy.init_node("hyproject_main")