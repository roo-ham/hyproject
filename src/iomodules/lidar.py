#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

from taskmodules import *
from basement import Basement
from module import IOModule

class Lidar(IOModule):
    def __init__(self, base:Basement):
        super().__init__(base, "Lidar")
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.wall_storage:Wall = base.taskmodules["Wall"]
        
    def callback(self, data):
        super().callback(data)
        orthogonal_pos = []
        polar_pos = []
        for n, radius in enumerate(data.ranges):
            if radius < data.range_min:
                continue
            angle = data.angle_min + data.angle_increment * n
            orthogonal_pos.append(radius * np.array((np.sin(angle), np.cos(angle))))
            polar_pos.append(np.array((radius, angle)))
        self.wall_storage.update(orthogonal_pos, polar_pos)