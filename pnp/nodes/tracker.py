#!/usr/bin/env python
import roslib
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

class Tracker:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e

        h = np.zeros((300,256,1))
        bins = np.arange(256).reshape(256,1)
        hist_item = cv2.calcHist([np.asarray(cv_image)],[0],None,[256],[0,255])
        cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.column_stack((bins,hist))
        cv2.polylines(h,[pts],False,(255,255,255))
        h=np.flipud(h)

        cv2.imshow("Histogram",h)
        cv.ShowImage("Image window", cv_image)
        # cv.WaitKey(1)
        cv2.waitKey(1)

def main(args):
    ic = Tracker()
    rospy.init_node('tracker')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)