#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

class Basement:
    def __init__(self, name0:str = "release"):
        self.name = name0
        self.bgr_top = np.zeros((128,256,3), np.uint8)
        self.bgr_bottom = np.zeros((128,256,3), np.uint8)