#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


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
        self.center = (231, 368)
        self.pt = self.center
        cv.NamedWindow("rectified")
        cv.SetMouseCallback("rectified", self.on_mouse, param = 'hi')
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.callback)


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

    def on_mouse(self,event, x, y, flag, param):
        if(event == cv.CV_EVENT_LBUTTONUP):
            self.pt = (x,y)
            print self.pt
            cmd = self.pt[0] - self.center[0], -(self.pt[1] - self.center[1])
            #print cmd
            if self.driver is not None:
                self.driver.cmd(cmd[0]/10,cmd[1]/10,0,0)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "passthrough")
        except CvBridgeError, e:
            print e
        original = np.asarray(cv_image)
        rectified = cv2.warpPerspective(original, self.h, (self.xmax, self.ymax))
        rectified_color = cv2.cvtColor(rectified, cv.CV_GRAY2BGR)
        cv2.circle(rectified_color, self.pt, 5, (0, 255, 0), -1)
        cv2.line(rectified_color, (0, self.center[1]), (self.xmax, self.center[1]), (255, 0, 0))
        cv2.line(rectified_color, (self.center[0], 0), (self.center[0], self.ymax), (255, 0, 0))
        cv2.imshow("original", original)
        cv2.imshow("rectified", rectified_color)
        cv2.waitKey(1)

def main(args):
    rospy.init_node('click_navigation')
    ptc = None
    ptc = pnp_lib.pyteacup.PyTeacup()
    ic = ClickNavigator(ptc)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


