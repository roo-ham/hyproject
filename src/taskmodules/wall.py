#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rooham.timer import *
from rooham.mathtools import get_wall_angle
from basement import Basement
from module import TaskModule
    
class Wall(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Wall")
        set_timer("wall/waiting_rotation", 5)

    def update(self, orthogonal_pos, polar_pos):
        self.weight_x = 0.0
        self.weight_z = 0.0
        self.x = 0.0
        self.z = 0.0

        in_range_pos = []
        for p1, p2 in zip(polar_pos, orthogonal_pos):
            if p1[0] > 0.4 or abs(p1[1]) > 0.1:
                continue
            in_range_pos.append(p2)
        if len(in_range_pos) < 5:
            set_timer("wall/obstacle_ignore", 0.2, True)
            set_timer("wall/waiting_rotation", 5, True)
            return
        elif is_timer_running("wall/obstacle_ignore"):
            return

        atan = get_wall_angle(in_range_pos)

        delta_x = 0.0
        delta_z = atan
        
        self.weight_x = 1000*atan
        self.weight_z = 1

        if is_timer_running("wall/waiting_rotation"):
            self.weight_z = 0

        self.x, self.z = delta_x, delta_z