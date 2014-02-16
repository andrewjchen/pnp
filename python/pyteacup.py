# -*- coding: utf-8 -*-

import serial
import time

class PyTeacup:
    def __init__(self):
        print('opening device...')
        self.port =  serial.Serial(port='/dev/ttyACM0',
                     baudrate=115200,
                     bytesize=serial.EIGHTBITS,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=0.1)
        self.port.open()
        time.sleep(3)
        self.send('g91')
        print('device initialized!')

    def send(self, str):
        self.port.write(str + '\r\n')
        self.port.flush()

    def cmd(self, x, y, z, t):
        self.send('g0x{0}y{1}z{2}e{3}'.format(x,y,z,t))