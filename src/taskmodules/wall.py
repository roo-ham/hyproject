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

        left_points = []
        front_points = []
        right_points = []

        for p1, p2 in zip(polar_pos, orthogonal_pos):
            if p1[0] < 0.25 and p1[1] < -0.4:
                left_points.append(p2)
            if p1[0] < 0.4 and abs(p1[1]) < 0.1:
                front_points.append(p2)
            if p1[0] < 0.25 and p1[1] > 0.4:
                right_points.append(p2)

        if len(left_points) > 10:
            self.weight_z = 100
            self.z += -0.5

        if len(front_points) < 5:
            set_timer("wall/obstacle_ignore", 0.2, True)
            set_timer("wall/waiting_rotation", 5, True)
            return
        
        if is_timer_running("wall/obstacle_ignore"):
            return

        atan = get_wall_angle(front_points)

        delta_x = 0.0
        delta_z = atan
        
        self.weight_x = 1000*atan
        self.weight_z = 1.0

        if is_timer_running("wall/waiting_rotation"):
            self.weight_z = 0

        self.x, self.z = delta_x, delta_z