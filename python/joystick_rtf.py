#!/usr/bin/env python

import serial
import time
import pygame
import math
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
        # self.send('g91')
        self.send('g90')
        print('device initialized!')

    def close(self):
        self.port.close()

    def send(self, str):
        self.port.write(str + '\n')
        self.port.flush()

    def move_y(self, val):
        self.send('g0 y' + str(val))

    def move_x(self, val):
        self.send('g0 x' + str(val))

    def read(self):
        self.port.read(self.port.inWaiting())

class EventHandler:
    def __init__(self, joystick):
        self.j = joystick
        self.laststate = False

        self.press_listeners = []
        self.release_listeners = []

    def _notify_change(self, button, state):
        listeners = self.press_listeners if state else self.release_listeners
        for l in listeners:
            l(button, state)

    def tick(self):
        pygame.event.pump()
        state = self.j.get_button(3)
        if self.laststate != state:
            self._notify_change(3, state)
        self.laststate = state

class Controller:
    def __init__(self, driver):
        self.driver = driver

    def press(self, button, state):
        cmd = '500, 0, 0, 0'
        print('sending cmd=' + str(cmd))        
        d.send(cmd)

    def rel(self, button, state):
        cmd = '0, 0, 0, 0'
        print('sending cmd=' + str(cmd))
        d.send(cmd)


if __name__ == '__main__':
    print 'starting joystick_control.py ...'
    d = Driver()

    pygame.init()
    j=pygame.joystick.Joystick(0)
    j.init()


    # eh = EventHandler(j)
    # c = Controller(d)
    # eh.press_listeners.append(c.press)
    # eh.release_listeners.append(c.rel)

    while True:
        pygame.event.pump()
        d.read()


        x,y,z,t = 0,0,0,0
        xa = j.get_axis(1)
        if math.fabs(xa) < .1:
            xa = 0
        ya = j.get_axis(0)
        if math.fabs(ya) < .1:
            ya = 0
        za = j.get_axis(4)
        if math.fabs(za) < .1:
            za = 0
        ta = j.get_axis(3)
        if math.fabs(ta) < .1:
            ta = 0
        print j.get_axis(1)

        val = int(500 / (xa + .00001))
        if math.fabs(val) > 5000:
            val = 0
        x = val

        val = int(500 / (-ya + .00001))
        if math.fabs(val) > 5000:
            val = 0
        y = val

        # val = int(500 / (-za + .00001))
        # if math.fabs(val) > 5000:
        #     val = 0
        # z = val

        val = int(500 / (-ta+ .00001))
        if math.fabs(val) > 5000:
            val = 0
        t = val





        if j.get_button(0) != 0:
            z = -500
        
        if j.get_button(3) != 0:
            z = 500



        # x = -10 * j.get_axis(1)
        # y = 10 * j.get_ax)is(0)
        # x = 1000/(x + .0001)
        # y = 1000/(y + .0001)
        # print('pos= ' + str(x) + '\t' + str(y))
        # d.send('g0 y{0} x{1}'.format(x,y))
        cmd = '{0},{1},{2},{3}'.format(x,y,z,t)
        print cmd
        d.send(cmd)
        # d.move_x(100 * j.get_axis(0))
        # d.move_y(-100 * j.get_axis(1))
        time.sleep(.01)
    d.close()






#while 1:
#    print port.read(100)x