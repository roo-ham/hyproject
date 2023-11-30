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

    def do_front(self, front_points):
        if is_timer_running("wall/obstacle_ignore"):
            return

        atan = get_wall_angle(front_points)

        delta_x = 0.0
        delta_z = atan
        
        self.weight_x = 1000*atan
        self.weight_z = 1.0

        if is_timer_running("wall/waiting_rotation"):
            self.weight_z = 0
        elif is_timer_running("wall/side_blocked"):
            delta_x = -0.5

        self.x += delta_x
        self.z += delta_z

    def update(self, orthogonal_pos, polar_pos):
        self.weight_x = 0.0
        self.weight_z = 0.0
        self.x = 0.0
        self.z = 0.0

        left_points = []
        front_points = []
        right_points = []

        for p1, p2 in zip(polar_pos, orthogonal_pos):
            if p1[0] < 0.25 and p1[1] > 0.4:
                left_points.append(p2)
            if p1[0] < 0.4 and abs(p1[1]) < 0.1:
                front_points.append(p2)
            if p1[0] < 0.25 and p1[1] < -0.4:
                right_points.append(p2)

        side_blocked = [False, False]

        if len(left_points) > 10:
            self.weight_z = 0.5
            self.z += -0.5
            side_blocked[0] = True

        if len(right_points) > 10:
            self.weight_z = 0.5
            self.z += 0.5
            side_blocked[1] = True

        if side_blocked[0] == side_blocked[1]:
            set_timer("wall/side_blocked", 6, True)

        if not is_timer_running("wall/side_blocked"):
            if side_blocked[0]:
                set_timer("lane/lane_exception/left", 1)
            elif side_blocked[1]:
                set_timer("lane/lane_exception/right", 1)

        if len(front_points) > 1:
            self.do_front(front_points)
        else:
            set_timer("wall/obstacle_ignore", 0.2, True)
            set_timer("wall/waiting_rotation", 5, True)