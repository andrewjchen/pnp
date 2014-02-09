# -*- coding: utf-8 -*-

import serial
import time
import math

class RTFDriver:
    def __init__(self):
        print('opening device...')
        self.port =  serial.Serial(port='/dev/ttyACM0',
                     baudrate=115200,
                     bytesize=serial.EIGHTBITS,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=0)
        self.port.open()
        time.sleep(3)

    def close(self):
        self.port.close()

    def _send(self, str):
        self.port.write(str + '\n')
        self.port.flush()

    def read(self):
        self.port.read(self.port.inWaiting())

    def cmd(self, x,y,z,t):
        val = int(50 / (-y + .00001))
        if math.fabs(val) > 5000:
            val = 0
        xa = val

        val = int(50 / (-x + .00001))
        if math.fabs(val) > 5000:
            val = 0
        ya = val
        val = int(200 / (z + .00001))
        if math.fabs(val) > 5000:
            val = 0
        za = val
        val = int(500 / (t + .00001))
        if math.fabs(val) > 5000:
            val = 0
        ta = val
        cmd = '{0},{1},{2},{3}'.format(xa,ya,za,ta)
        self._send(cmd)