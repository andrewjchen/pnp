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

class TemplateMatcher:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.callback)
        cv.NamedWindow("sobel")
        cv.SetMouseCallback("sobel", self.on_mouse, param = 'hi')
        self.tbounds = [(0,0), (100, 100)]
        self.template = cv2.imread('template.png', cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.waiting = False




    def on_mouse(self,event, x, y, flag, param):
        if(event == cv.CV_EVENT_LBUTTONDOWN):
           self.tbounds[0] = (x,y)
           self.waiting = True
        if(event == cv.CV_EVENT_MOUSEMOVE) and self.waiting:
            self.tbounds[1] = (x,y)
        if(event == cv.CV_EVENT_LBUTTONUP):
           self.waiting = False

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
        #cv.ShowImage("original", cv_image)
        sobel = self.do_sobel(blurred)
        sobel_color = cv2.cvtColor(sobel, cv.CV_GRAY2BGR)
        
        # if self.template is None:
        #     self.template = sobel[50:80, 170:230]
        tmpresult = cv2.matchTemplate(sobel, self.template, cv.CV_TM_SQDIFF_NORMED)

        if self.waiting:
            cv2.rectangle(sobel_color, self.tbounds[0], self.tbounds[1], (255,0,0))
        cv2.imshow("sobel", sobel_color)
        # result_color = cv2.cvtColor(tmpresult, cv.CV_GRAY2BGR)
        # matches = sobel_color.copy()
        matches = cv2.applyColorMap(np.asarray(tmpresult), cv2.COLORMAP_JET)
        # print result_color.shape
        cv2.imshow("match", matches)
        # cv2.imshow("template", self.template)
        if cv2.waitKey(1) == 10:
            print "say cheese!"
            # print self.tbounds
            b = self.tbounds
            self.template = sobel[b[0][1]:b[1][1],
                                  b[0][0]:b[1][0]]

def main(args):
    ic = TemplateMatcher()
    rospy.init_node('template_matcher')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


