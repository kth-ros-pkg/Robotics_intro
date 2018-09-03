from cv_bridge.core import CvBridge
import cv2
import tempfile

class RosImageTmpFile:
    def __init__(self, image_msg, desired_encoding="bgr8"):
        self.image_msg = image_msg
        self.desired_encoding = desired_encoding

    def __enter__(self):
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(self.image_msg, desired_encoding=self.desired_encoding)
        self.tmpfile = tempfile.NamedTemporaryFile(suffix=".jpg")
        cv2.imwrite(self.tmpfile.name, cv_img)
        return self.tmpfile.__enter__()

    def __exit__(self, type, value, traceback):
        return self.tmpfile.__exit__(type, value, traceback)

