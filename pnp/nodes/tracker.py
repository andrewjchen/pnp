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
        self.t1 = 0
        self.t2 = 0
        cv2.namedWindow("canny")
        cv.CreateTrackbar("t0", "canny", 0, 1000, self.t0change)
        cv.CreateTrackbar("t1", "canny", 0, 1000, self.t1change)

    def t0change(pos, data):
        t1 = pos


    def t1change(pos, data):
        t2 = pos


    def histogram(self, src):
        h = np.zeros((300,256,1))
        bins = np.arange(256).reshape(256,1)
        hist_item = cv2.calcHist([np.asarray(src)],[0],None,[256],[0,255])
        cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.column_stack((bins,hist))
        cv2.polylines(h,[pts],False,(255,255,255))
        h=np.flipud(h)
        return h

    def thresholded(self, image):
        return cv2.adaptiveThreshold(
            np.asarray(image),
            255,
            cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C,
            cv.CV_THRESH_BINARY,
            55,
            -10)

    def corners(self, image):
        corners = cv2.cornerHarris(np.asarray(image), 3, 7, .004)
        # corners =  corners * 10e6
        # corners /= 255.0
        return corners

    def do_sobel(self, image):
        grad_x = cv2.Sobel(np.asarray(image), cv2.CV_16S, 1, 0, ksize=3)
        grad_y = cv2.Sobel(np.asarray(image), cv2.CV_16S, 0, 1, ksize=3)

        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)

        grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        return grad

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "passthrough")
        except CvBridgeError, e:
            print e

        blurred = cv2.GaussianBlur(np.asarray(cv_image), (5,5), 3)
        # image = np.asarray(cv_image)
        cv.ShowImage("original", cv_image)
        # cv.ShowImage("blurrd", cv.fromarray(blurred))
        # circles = cv2.HoughCircles(
        #     np.asarray(cv_image),
        #     cv.CV_HOUGH_GRADIENT,
        #     1,
        #     1000)
        # print circles
        sobel = self.do_sobel(blurred)
        # sobel = cv2.Sobel(np.asarray(blurred), -1, 1, 1)
        thresSobel = cv2.adaptiveThreshold(
            np.asarray(sobel),
            255,
            cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C,
            cv.CV_THRESH_BINARY,
            55,
            -10)
        cv2.imshow("sobel", sobel)
        self.t1 = cv2.getTrackbarPos("t0", "canny")
        self.t2 = cv2.getTrackbarPos("t1", "canny")
        # canny = cv2.Canny(np.asarray(sobel), self.t1, self.t2)
        # cv2.imshow("blurred", blurred)
        # cv2.imshow("canny", canny)

        cv2.imshow("thressobel", thresSobel)
        # cv2.imshow("histogram", self.histogram(cv_image))
        # cv2.imshow("thresholded", self.thresholded(cv_image))
        # cv2.imshow("corners", self.corners(cv.fromarray(blurred)))
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

