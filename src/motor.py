import rospy
import numpy as np
from geometry_msgs.msg import Twist

from basement import Basement
from submodule import Submodule

class Motor(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "Motor")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.OFFSET_CONSTANT = 0

    def update(self):
        super().update()

        self.drive_data.linear.x = 0.1
        self.drive_data.angular.z = 0.0
        
        gtan = self.basement.global_tan
        ltan = self.basement.local_tan
        ltan2 = self.basement.local_tan_sqaured
        
        self.drive_data.linear.x *= ltan2 + 0.5
        self.drive_data.angular.z += gtan / ((ltan**2) + 0.1) + self.OFFSET_CONSTANT
        self.pub.publish(self.drive_data)