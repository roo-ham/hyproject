#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rooham.flag import *
from rooham.timer import *
from rooham.mathtools import get_wall_angle
from basement import Basement
from module import TaskModule
    
class Wall(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Wall")
        set_timer("wall/waiting_rotation", 8)

    def do_front(self, front_points):
        wall_angle = get_wall_angle(front_points)

        delta_x = 0.0
        delta_z = wall_angle
        
        self.weight_x = 1.0
        self.weight_z = 10.0

        if is_timer_on("wall/waiting_rotation"):
            self.weight_z = 0
        else:
            self.weight_z = 1.0
            delta_x = -0.5
            delta_z = -0.5
            set_timer("lane/ramp", 2)

        self.x = delta_x
        self.z += delta_z

    def update(self, orthogonal_pos, polar_pos):
        self.weight_x = 0.0
        self.weight_z = 0.0
        self.z = 0.0

        if is_flag("tpark") or is_timer_on("marker/stop/phase2"):
            set_timer("wall/obstacle_ignore", 0.2, True)
            set_timer("wall/waiting_rotation", 8, True)
            return

        left_points = []
        front_points = []
        right_points = []
        front_2_points = []

        left_distance = 0.0
        right_distance = 0.0

        SIDE_WALL_CONST = 0.4
        special_situation = is_timer_on("lane/front_blocked")

        for p1, p2 in zip(polar_pos, orthogonal_pos):
            radius, angle = p1
            if radius < SIDE_WALL_CONST and angle > 0.4:
                left_points.append(p2)
                left_distance += radius
            elif radius < SIDE_WALL_CONST and angle < -0.4:
                right_points.append(p2)
                right_distance += radius
            if (radius < 0.30 and abs(angle) < 0.5) \
                or (radius < 0.35 and 0.5 < abs(angle) and abs(angle) < 0.6):
                if special_situation:
                    pass
                else:
                    front_points.append(p2)
            if radius < 0.30 and abs(angle) < 0.5:
                front_2_points.append(p2)

        side_blocked = [False, False]

        if len(left_points) > 10:
            left_distance /= len(left_points)
            self.weight_z = 0.5
            self.z += left_distance - SIDE_WALL_CONST
            side_blocked[0] = True

        if len(right_points) > 10:
            right_distance /= len(right_points)
            self.weight_z = 0.5
            self.z += SIDE_WALL_CONST - right_distance
            side_blocked[1] = True

        if not (side_blocked[0] | side_blocked[1]):
            set_timer("wall/side_blocked", 7, True)

        if is_timer_on("lane/junction/do") \
                or is_timer_on("lane/front_blocked"):
            self.weight_z = 0
            self.z = 0

        if is_flag("lane/curve"):
            pass
        elif is_timer_off("lane/junction/wait") \
                and (is_timer_on("lane/junction/rotation/left") or is_timer_on("lane/junction/rotation/right")):
            pass
        elif len(front_points) > 2:
            if is_timer_on("wall/obstacle_ignore"):
                return
            elif is_timer_off("wall/waiting_rotation") and len(front_2_points) <= 2:
                pass
            else:
                self.do_front(front_points)
                return

        set_timer("wall/obstacle_ignore", 0.2, True)
        set_timer("wall/waiting_rotation", 8, True)