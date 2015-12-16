#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class image_converter:

  def __init__(self):
    cv2.namedWindow("Depth Image", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/depth_registered/image",Image,self.callback)

  def callback(self,data):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(data, '32FC1')
      depth_array = np.array(depth_image, dtype=np.float32)
      cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
      cv2.imshow("Depth Image",depth_image)
      cv2.imwrite('capture_depth.png',depth_array*255)
    except CvBridgeError as e:
      print(e)

    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

