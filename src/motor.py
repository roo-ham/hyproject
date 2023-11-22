import rospy
import numpy as np
from geometry_msgs.msg import Twist

from storage import Storage
from basement import Basement
from submodule import Submodule

class Motor(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "Motor")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()

    def update(self):
        super().update()

        self.drive_data.linear.x = 0.2
        self.drive_data.angular.z = 0.5
        
        x = 0.0
        z = 0.0
        weight_x = 0.01
        weight_z = 0.01
        for s in self.basement.storages.values():
            x0, z0, weight_x0, weight_z0 = Storage.getDataset(s)
            x += x0 * weight_x0
            z += z0 * weight_z0
            weight_x += weight_x0
            weight_z += weight_z0
        x /= weight_x
        z /= weight_z
        
        self.basement.real_speed_x = x
        self.basement.real_speed_z = z
        
        self.drive_data.linear.x *= x
        self.drive_data.angular.z *= z
        self.pub.publish(self.drive_data)