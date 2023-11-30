#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

from rooham.mathtools import get_wall_angle
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
            if p1[0] > 0.3 or abs(p1[1]) > 0.1:
                continue
            in_range_pos.append(p2)
        if len(in_range_pos) < 5:
            return
        atan = get_wall_angle(in_range_pos)

        delta_x = 0.0
        delta_z = atan
        
        self.weight_x = 1000
        self.weight_z = 1

        self.x, self.z = delta_x, delta_z