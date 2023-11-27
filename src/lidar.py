import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

import storage
from basement import Basement
from submodule import Submodule

class Lidar(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "Lidar")
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.wall_storage:storage.Lane = base.storages["wall"]
        self.orthogonal_pos = []
        self.polar_pos = []
        
    def callback(self, data):
        super().callback(data)
        print(".")
        self.orthogonal_pos = []
        self.polar_pos = []
        for n, radius in enumerate(data.ranges):
            if radius < data.range_min:
                continue
            angle = data.angle_min + data.angle_increment * n
            self.orthogonal_pos.append(np.array((radius, angle)))
            self.polar_pos.append(radius * np.array((np.sin(angle), np.cos(angle))))
        self.wall_storage.update(self.orthogonal_pos, self.polar_pos)