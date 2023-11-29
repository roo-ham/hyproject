import rospy
import numpy as np
from geometry_msgs.msg import Twist

from ..basement import Basement
from ..module import IOModule

class Motor(IOModule):
    def __init__(self, base:Basement):
        super().__init__(base, "Motor")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()

    def update(self):
        super().update()

        self.drive_data.linear.x = 0.2
        self.drive_data.angular.z = 1.0
        
        x = 0.0
        z = 0.0
        weight_x = 0.01
        weight_z = 0.01
        for s in self.basement.taskmodules.values():
            x0, z0, weight_x0, weight_z0 = s.getDataset()
            x += x0 * weight_x0
            z += z0 * weight_z0
            weight_x += weight_x0
            weight_z += weight_z0
        x /= weight_x
        z /= weight_z
        
        # 새 속도는 바로 적용되는 것이 아니라 이전 속도를 반영함
        # 주행이 부드러워지는 효과를 낼 수 있음
        x = self.smoothFunction(self.basement.real_speed_x, x)
        z = self.smoothFunction(self.basement.real_speed_z, z)
        
        self.basement.real_speed_x = x
        self.basement.real_speed_z = z
        
        self.drive_data.linear.x *= x
        self.drive_data.angular.z *= z
        self.pub.publish(self.drive_data)

    def smoothFunction(self, a, b):
        return ((a*4) + b)/5