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
import operator

import serial
import time

class Driver:
    def __init__(self):
        print('opening device...')
        self.port =  serial.Serial(port='/dev/ttyACM0',
                     baudrate=115200,
                     bytesize=serial.EIGHTBITS,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=0)
        self.port.open()
        time.sleep(3)
        self.send('g91')
        print('device initialized!')

    def close(self):
        self.port.close()

    def send(self, str):
        self.port.write(str + '\r\n')
        self.port.flush()

    def moveY(self, val):
        self.send('g0 y' + str(val))

    def moveX(self, val):
        self.send('g0 x' + str(val))

    def read(self):
        self.port.read(self.port.inWaiting())

class TemplateMatcher:

    def __init__(self, driver=None):
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

        self.driver = driver
        self.cx, self.cy = (320, 240)

        self.grey = (200,200,200)
        self.tgrey = (127,127,127)
        # self.original_video = cv2.VideoWriter()

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
        # colorsobel = cv2.applyColorMap(sobel, cv2.COLORMAP_JET)
        sobel_color = cv2.cvtColor(sobel, cv.CV_GRAY2BGR)
        
        # if self.template is None:
        #     self.template = sobel[50:80, 170:230]
        tmpresult = cv2.matchTemplate(sobel, self.template, cv.CV_TM_SQDIFF_NORMED)
        # result_mono = cv2.cvtColor(tmpresult, cv.CV_RGB2GRAY)
        if self.waiting:
            cv2.rectangle(sobel_color, self.tbounds[0], self.tbounds[1], (255,0,0))

        # result_color = cv2.cvtColor(tmpresult, cv.CV_GRAY2BGR)
        # matches = sobel_color.copy()
        matches = cv2.cvtColor(tmpresult, cv.CV_GRAY2BGR)
        # matches = cv2.applyColorMap(tmpresult, cv2.COLORMAP_JET)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(tmpresult)
        target = tuple(map(operator.add, (self.template.shape[1]/2, self.template.shape[0]/2), minLoc))
        # print target
        self.cx, self.cy = target
        cv2.line(sobel_color, (320, 0), (320, 480), self.grey)
        cv2.line(sobel_color, (0, 2409), (640, 240), self.grey)
        cv2.circle(sobel_color, target, 5, (0, 255, 0), -1)

        cv2.line(sobel_color, (self.cx, 0), (self.cx, 480), self.tgrey)
        cv2.line(sobel_color, (0, self.cy), (640, self.cy), self.tgrey)
        cv2.imshow("sobel", sobel_color)
        cv2.imshow("match", tmpresult)
        self.tick()
        key = cv2.waitKey(1)
        # print key
        if key == 1048586:
            print "say cheese!"
            # print self.tbounds
            b = self.tbounds
            self.template = sobel[b[0][1]:b[1][1],
                                  b[0][0]:b[1][0]]

    def tick(self):
        erry = 320 - self.cx
        errx = 240 - self.cy
        py = -.005
        px = .005
        outputy = py * erry
        outputx = px * errx

        if self.driver is not None:
            self.driver.moveX(outputx)
            self.driver.moveY(outputy)

def main(args):
    d = Driver()
    # d = None
    ic = TemplateMatcher(d)
    rospy.init_node('template_matcher')

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        #ic.tick()
        if d is not None:
            d.read()
        r.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


