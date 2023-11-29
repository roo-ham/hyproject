#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

from basement import Basement
from module import TaskModule
    
class Wall(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Wall")

    def update(self, orthogonal_pos, polar_pos):
        self.weight_x = 0.0
        self.weight_z = 0.0
        self.x = 0.0
        self.z = 0.0

        in_range_pos = []
        for p1, p2 in zip(polar_pos, orthogonal_pos):
            if p1[0] > 0.35 or abs(p1[1]) > 0.05:
                continue
            in_range_pos.append(p2)

        if len(in_range_pos) == 0:
            return

        number_of_point = len(in_range_pos)
        mean_pos = sum(in_range_pos)/number_of_point
        sum_tangent = 0.0

        for p2 in in_range_pos:
            relative_point = p2 - mean_pos
            if relative_point[1] == 0:
                number_of_point -= 1
                continue
            sum_tangent += relative_point[0]/relative_point[1]
        
        if number_of_point == 0:
            return
        atan = np.arctan(sum_tangent / number_of_point)

        delta_x = -1.5
        delta_z = atan
        
        k = np.pi * 0.5
        #self.weight_x = 2.0
        #self.weight_z = 1 - np.sqrt(abs(atan / k))

        #self.x, self.z = delta_x, delta_z