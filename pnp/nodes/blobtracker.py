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
import serial
import time
import timeit
import thread
import showdata as sm

from pyqtgraph.Qt import QtGui, QtCore
from numpy import arange, array, ones, linalg
import numpy as np
import pyqtgraph as pg
import time
import random
from pylab import plot, show

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


class Blobtracker:

    def __init__(self, driver):
        self.bridge = CvBridge()


        self.grey = (200,200,200)
        self.tgrey = (127,127,127)
        self.driver = driver
        self.cx = 320
        self.cy = 240
        self.cx0 = 320
        self.cy0 = 240
        # self.analogData = sm.AnalogData(100)
        # self.analogPlot = sm.AnalogPlot(self.analogData)
        self.val = 0
        self.xcount = 0
        self.ycount = 0
        self.xs = []
        self.ys = []

        self.video = cv2.VideoWriter()
        self.video.open("bt-original.avi", cv.CV_FOURCC('F', 'M', 'P', '4'), 60, (640,480), True)

        self.uvc_video = cv2.VideoWriter()
        self.uvc_video.open("bt-uvc.avi", cv.CV_FOURCC('F', 'M', 'P', '4'), 60, (640,480), True)

        self.uvc_sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.uvc_callback)
        self.uvc_frame = None
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.callback)

    def uvc_callback(self, data):
        try:
            # print('uvc_callback')
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
            self.uvc_frame = np.asarray(cv_image)
        except CvBridgeError, e:
            print e

    def callback(self,data):
        # print('start')
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
            if M['m00'] > 3000:
                print M['m00']
                cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                cv2.circle(imageColor,(cx,cy), 5, (255,0,0),-1)
                cv2.line(imageColor, (cx, 0), (cx, 480), self.tgrey)
                cv2.line(imageColor, (0, cy), (640, cy), self.tgrey)
                self.cx0 = self.cx
                self.cy0 = self.cy
                self.cx = cx
                self.cy = cy
            
        cv2.line(imageColor, (320, 0), (320, 480), self.grey)
        cv2.line(imageColor, (0, 2409), (640, 240), self.grey)
        # cv2.imshow("result", imageColor)
#        cv2.imshow("THRESH", thresh)
        cv2.imshow("result", imageColor)
        # merged = cv2.merge(imageColor, self.uvc_frame)
        # cv2.imshow('merged', merged)
        cv2.imshow("uvc", self.uvc_frame)
        if self.uvc_frame is not None:
            self.video.write(imageColor)
            self.uvc_video.write(self.uvc_frame)
        # self.tick()

        # print('processing events...')
        # pg.QtGui.QApplication.processEvents()
        # print('processed events!')
        # print('cv.waitKey...')
        cv2.waitKey(1)
        # time.sleep(.01)
        # print('waitKey finished')
        # pos_curve.setData(x=np.array(xs), y=np.array(ys))

    def tick(self): 
        erry = 320 - self.cx
        errx = 240 - self.cy
        py = -.005
        px = .005
        outputy = py * erry
        outputx = px * errx

        diffx = self.cx - self.cx0
        diffy = self.cy - self.cy0
        nval = math.sqrt(diffx**2 + diffy**2)
        self.val = nval * .2 + self.val * .8
        # print errx, erry
        if self.driver is not None:
            self.driver.moveX(outputx)
            self.driver.moveY(outputy)
        # self.xcount+=outputx
        # self.ycount+=outputy
        # return self.xcount, self.ycount
        '''
        self.xs.append(self.xcount)
        self.ys.append(self.ycount)
        return self.xcount, self.ycount
        curve.setData(x=np.array(self.xs), y=np.array(self.ys))
        '''

        # self.analogData.add((self.xcount, self.ycount))
        # self.analogPlot.update(self.analogData)

def main():
    d = Driver()

    rospy.loginfo("starting blobtracker")
    print('starting blobtracker')
    rospy.init_node('blobtracker')
    # d = None
    ic = Blobtracker(d)

    # app = QtGui.QApplication([])
    # win = pg.GraphicsWindow(title="PNP")
    # win.resize(800,600)

    # pos_plot = win.addPlot(title="position")
    # pos_plot.enableAutoRange('xy', True)
    # global pos_curve
    # pos_curve = pos_plot.plot(pen=None,
    #                           symbol='o',
    #                           symbolPen=None,
    #                           symbolSize=10,
    #                           symbolBrush=(100, 100, 255, 50))


    r = rospy.Rate(60) # 10hz
    # xs = [1,3,5,7,9]
    # ys= [43, 2,33, 64, 2]

    while not rospy.is_shutdown():
        ic.tick()
    #     x, y = ic.tick()

    #     # xs.append(x)
    #     # ys.append(y)
    #     # 
        if d is not None:
            d.read()
    #     # time.sleep(.01)

    #     # pg.QtGui.QApplication.processEvents()
    #     cv2.waitKey(1)
        r.sleep()

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print "Shutting down"

    cv.DestroyAllWindows()
    if d is not None:
        d.close()

if __name__ == '__main__':
    main()


