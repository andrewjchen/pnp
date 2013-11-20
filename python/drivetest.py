#!/usr/bin/env python

import serial
import time

print("Running drivetest.py")
port = serial.Serial(port='/dev/ttyACM0',
                     baudrate=115200,
                     bytesize=serial.EIGHTBITS,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=0.1)

port.open()

def send(str):
    print("command=" + str)
    port.write(str + '\r\n')
    port.flush()
    time.sleep(1)
    #time.sleep(.01)


time.sleep(2)

send('g91')
send('g0x10')
send('g0y10')
send('g0x-10')
send('g0y-10')
port.close()



#while 1:
#    print port.read(100)x
