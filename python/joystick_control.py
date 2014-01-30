#!/usr/bin/env python

import serial
import time
import pygame

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
        self.port.write(str + '\r\n')
        self.port.flush()

    def move_y(self, val):
        self.send('g0 y' + str(val))

    def move_x(self, val):
        self.send('g0 x' + str(val))

    def read(self):
        self.port.read(self.port.inWaiting())


if __name__ == '__main__':
    print 'starting joystick_control.py ...'
    d = Driver()
    # d.move_x(10)
    # d.move_y(10)
    # d.move_x(-10)
    # d.move_y(-10)

    pygame.init()
    j=pygame.joystick.Joystick(0)
    j.init()

    while True:
        pygame.event.pump()
        # for i in range(j.get_numaxes()):
        #     print('axis= '+ str(i) + ', val=' + str(j.get_axis(i)))
        # d.move_x()
        x = 100 * j.get_axis(0)
        y = 100 * j.get_axis(1)
        print('pos= ' + str(x) + '\t' + str(y))
        d.send('g0 y{0} x{1}'.format(x,y))
        # d.move_x(100 * j.get_axis(0))
        # d.move_y(-100 * j.get_axis(1))
        time.sleep(.1)
    d.close()






#while 1:
#    print port.read(100)x
