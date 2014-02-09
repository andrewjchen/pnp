#!/usr/bin/env python

import serial
import time
import pygame
import math
import rtfdriver

if __name__ == '__main__':
    print 'starting joystick_control.py ...'
    d = rtfdriver.RTFDriver()

    pygame.init()
    j=pygame.joystick.Joystick(0)
    j.init()

    while True:
        pygame.event.pump()
        d.read()

        x,y,z,t = 0,0,0,0
        xa = j.get_axis(0)
        if math.fabs(xa) < .1:
            xa = 0
        ya = -j.get_axis(1)
        if math.fabs(ya) < .1:
            ya = 0
        za = j.get_axis(4)
        if math.fabs(za) < .1:
            za = 0
        ta = j.get_axis(3)
        if math.fabs(ta) < .1:
            ta = 0

        if j.get_button(0) != 0:
            z = -500

        if j.get_button(3) != 0:
            z = 500
        d.cmd(xa, ya, za, ta)

        time.sleep(.01)
    d.close()






#while 1:
#    print port.read(100)x