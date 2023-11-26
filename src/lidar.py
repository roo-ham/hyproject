import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

from basement import Basement
from submodule import Submodule

class Lidar(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "Lidar")
        rospy.Subscriber('/scan', LaserScan, self.callback)
        
    def callback(self, data):
        super().callback(data)