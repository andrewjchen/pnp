#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import operator
import cv
import cv2
import numpy as np

import pnp_lib.pyteacup

class ClickNavigator:

    def __init__(self, driver=None):
        self.driver = driver
        self.bridge = CvBridge()
        self.init_homography()
        #self.center = (self.xmax / 2, self.ymax/2)
        self.center = (215, 163)
        self.pt = self.center
        cv.NamedWindow("rectified")
#        cv.SetMouseCallback("rectified", self.on_mouse, param = 'hi')
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.callback)

        cv.NamedWindow("sobel")
        cv.SetMouseCallback("sobel", self.on_mouse, param = 'hi')
        self.tbounds = [(0,0), (100, 100)]
        self.template = cv2.imread('template.png', cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.waiting = False
        self.uvc_frame = None

        self.driver = driver
        self.cx, self.cy = (320, 240)

        self.grey = (200,200,200)
        self.tgrey = (127,127,127)


    def init_homography(self):
        '''
        Initializes the homography matrix using hardcoded values obtained offline.
        Reads the data from a file media/check4.ppm.
        '''
        img = cv2.imread('media/check4.ppm')

        retval, corners =cv2.findChessboardCorners(img, (5, 4))

        pts = np.empty((5 * 4, 2), dtype=np.float32)
        i = 0
        for y in np.linspace(300, 85, 4):
            for x in np.linspace(524, 145, 5):
                pts[i][0] = x
                pts[i][1] = y
                i+=1

        corn2 = (corners.reshape(20,2))
        h, mask = cv2.findHomography(corn2, pts)
        # remove offsets
        h[1][2] = 0
        h[0][2] = 0
        res = h*np.matrix('640;0;1')
        xmax =int(res.item(0) / res.item(2))
        res = h*np.matrix('0;480;1')
        ymax = int(res.item(1) / res.item(2))
        self.h = h
        self.xmax = xmax
        self.ymax = ymax

    def on_mouse(self, event, x, y, flag, param):
#        print .event, x, y, flag, param)
        if(event == cv.CV_EVENT_LBUTTONDOWN):
           self.tbounds[0] = (x,y)
           self.waiting = True
        if(event == cv.CV_EVENT_MOUSEMOVE) and self.waiting:
            self.tbounds[1] = (x,y)
        if(event == cv.CV_EVENT_LBUTTONUP):
           print x,y
           self.waiting = False
#        if(event == cv.CV_EVENT_LBUTTONUP):
#            self.pt = (x,y)
#            print self.pt
#            cmd = self.pt[0] - self.center[0], -(self.pt[1] - self.center[1])
#            #print cmd
#            if self.driver is not None:
#                self.driver.cmd(cmd[0]/10,cmd[1]/10,0,0)

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
        original = np.asarray(cv_image)
        rectified = cv2.warpPerspective(original, self.h, (self.xmax, self.ymax))
#        rectified_color = cv2.cvtColor(rectified, cv.CV_GRAY2BGR)
#        cv2.circle(rectified_color, self.pt, 5, (0, 255, 0), -1)
#        cv2.line(rectified_color, (0, self.center[1]), (self.xmax, self.center[1]), (255, 0, 0))
#        cv2.line(rectified_color, (self.center[0], 0), (self.center[0], self.ymax), (255, 0, 0))
#        cv2.imshow("original", original)
#        cv2.imshow("rectified", rectified_color)
#        cv2.waitKey(1)


#        try:
#            cv_image = self.bridge.imgmsg_to_cv(data, "passthrough")
#        except CvBridgeError, e:
#            print e
#        original = np.asarray(cv_image)

        color_original = cv2.applyColorMap(rectified, cv2.COLORMAP_JET)
        blurred = cv2.GaussianBlur(rectified, (5,5), 3)
        #cv.ShowImage("original", cv_image)
        sobel = self.do_sobel(blurred)
        sobel_color = cv2.cvtColor(sobel, cv.CV_GRAY2BGR)

        # if self.template is None:
        #     self.template = sobel[50:80, 170:230]
        tmpresult = cv2.matchTemplate(sobel, self.template, cv.CV_TM_SQDIFF_NORMED)
        # result_mono = cv2.cvtColor(tmpresult, cv.CV_RGB2GRAY)
#        if True:
        if self.waiting:
            cv2.rectangle(sobel_color, self.tbounds[0], self.tbounds[1], (255,0,0))

        # result_color = cv2.cvtColor(tmpresult, cv.CV_GRAY2BGR)
        # matches = sobel_color.copy()
        # print tmpresult
        tmpresult*=255.0
        result = tmpresult.astype(np.uint8)
        matches = cv2.applyColorMap(result, cv2.COLORMAP_JET)
        matches = cv2.resize(matches, (640, 480))
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(tmpresult)
        target = tuple(map(operator.add, (self.template.shape[1]/2, self.template.shape[0]/2), minLoc))
        # print target
        self.cx, self.cy = target
#        cv2.line(sobel_color, (320, 0), (320, 480), self.grey)
#        cv2.line(sobel_color, (0, 240), (640, 240), self.grey)
        cv2.circle(sobel_color, target, 5, (0, 255, 0), -1)

        cv2.line(sobel_color, (self.cx, 0), (self.cx, 480), self.tgrey)
        cv2.line(sobel_color, (0, self.cy), (640, self.cy), self.tgrey)

        cv2.line(sobel_color, (0, self.center[1]), (self.xmax, self.center[1]), (255, 0, 0))
        cv2.line(sobel_color, (self.center[0], 0), (self.center[0], self.ymax), (255, 0, 0))
        cv2.imshow("sobel", sobel_color)
        cv2.imshow("match", matches)
#        cv2.imshow("uvc", self.uvc_frame)
#        if self.uvc_frame is not None:
#            self.video.write(sobel_color)
#            self.uvc_video.write(self.uvc_frame)
#            self.conv_video.write(matches)
#            self.original_video.write(color_original)
        # self.tick()
        key = cv2.waitKey(1)
#        print key
        if key ==10:
            print "say cheese!"
            # print self.tbounds
            b = self.tbounds
            self.template = sobel[b[0][1]:b[1][1],
                                  b[0][0]:b[1][0]]
def main(args):
    rospy.init_node('click_navigation')
    ptc = None
#    ptc = pnp_lib.pyteacup.PyTeacup()
    ic = ClickNavigator(ptc)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


