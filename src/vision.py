import rospy
from cv_bridge import CvBridge
import numpy, cv2
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

from basement import Basement

class VisionImage:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.img = None
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
    def callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img = cv2.resize(self.img, (128, 128))
        cv2.imshow("hyproject", self.image)
        cv2.waitKey(1)

class VisionMarker:
    def __init__(self):
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
    def callback(self):
        pass


if __name__ == "__main__":
    VisionImage()
    VisionMarker()
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass