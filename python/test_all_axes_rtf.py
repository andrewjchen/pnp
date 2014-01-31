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


time.sleep(2)

send('0,0,0,0')
send('500,0,0,0')
send('0,0,0,0')
send('-500,0,0,0')
send('0,0,0,0')

send('0,0,0,0')
send('0,500,0,0')
send('0,0,0,0')
send('0,-500,0,0')
send('0,0,0,0')

send('0,0,0,0')
send('0,0,500,0')
send('0,0,0,0')
send('0,0,-500,0')
send('0,0,0,0')

send('0,0,0,0')
send('0,0,0,500')
send('0,0,0,0')
send('0,0,0,-500')
send('0,0,0,0')
port.close()