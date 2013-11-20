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
import math

class Blobtracker:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.callback)
        self.grey = (200,200,200)
        self.tgrey = (127,127,127)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "passthrough")
        except CvBridgeError, e:
            print e

        blurred = cv2.GaussianBlur(np.asarray(cv_image), (5,5), 3)
        retval, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        imageColor = cv2.cvtColor(np.asarray(cv_image), cv.CV_GRAY2BGR)
        thresh2 = thresh.copy()
        contours,hierarchy = cv2.findContours(thresh2,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            max_area = 0
            best_cnt = contours[0]
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area
                    best_cnt = cnt
            M = cv2.moments(best_cnt)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            print cx, cy
            cv2.circle(imageColor,(cx,cy), 5, (255,0,0),-1)
            cv2.line(imageColor, (cx, 0), (cx, 480), self.tgrey)
            cv2.line(imageColor, (0, cy), (640, cy), self.tgrey)
            
        cv2.line(imageColor, (320, 0), (320, 480), self.grey)
        cv2.line(imageColor, (0, 240), (640, 240), self.grey)
        cv2.imshow("result", imageColor)
        cv2.waitKey(1)

def main(args):
    ic = Blobtracker()
    rospy.init_node('blobtracker')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

