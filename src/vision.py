import rospy
from cv_bridge import CvBridge
import numpy, cv2
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

from basement import Basement

class VisionImage:
    def __init__(self):
        self.bridge:CvBridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.img = None
        print("I'm VisionImage")
    def callback(self, data):
        print("VisionImage : I've got callback")
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img = cv2.resize(self.img, (128, 128))
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", self.img)

class VisionMarker:
    def __init__(self):
        print("I'm VisionMarker")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
    def callback(self, data):
        print("VisionMarker : I've got callback")


if __name__ == "__main__":
    VisionImage()
    VisionMarker()
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass