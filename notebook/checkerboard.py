# -*- coding: utf-8 -*-
"""
Rectifies a single checkerboard image.
Hand-tuned.
"""

import cv
import cv2
import numpy as np

import matplotlib.pyplot as plt

img = cv2.imread('media/check4.ppm')

retval, corners =cv2.findChessboardCorners(img, (5, 4))
img2 = img.copy()

cv2.drawChessboardCorners(img2, (5,4), corners, True)
plt.imshow(img2)

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


dst = cv2.warpPerspective(img, h, (xmax, ymax))
plt.imshow(dst)
plt.show()
