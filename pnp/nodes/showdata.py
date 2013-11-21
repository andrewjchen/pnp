################################################################################
# showdata.py
#
# Display analog data from Arduino using Python (matplotlib)
# 
# electronut.in
#
################################################################################

import sys, serial
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt
import random

# class that holds analog data for N samples
class AnalogData:
  # constr
  def __init__(self, maxLen):
    self.ax = deque([0.0]*maxLen)
    self.ay = deque([0.0]*maxLen)
    self.maxLen = maxLen

  # ring buffer
  def addToBuf(self, buf, val):
    if len(buf) < self.maxLen:
      buf.append(val)
    else:
      buf.pop()
      buf.appendleft(val)

  # add data
  def add(self, data):
    assert(len(data) == 2)
    self.addToBuf(self.ax, data[0])
    self.addToBuf(self.ay, data[1])
    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, analogData):
    # set plot to animated
    plt.ion()
    self.axline, = plt.plot(analogData.ax)
    self.ayline, = plt.plot(analogData.ay)
    plt.ylim([0, 100])

  # update plot
  def update(self, analogData):
    self.axline.set_ydata(analogData.ax)
    self.ayline.set_ydata(analogData.ay)
    plt.draw()

# main() function
def main():
  # plot parameters
  analogData = AnalogData(100)
  analogPlot = AnalogPlot(analogData)

  print 'plotting data...'

  val1 = 0

  while True:
    try:
      val = random.random() * 500
      val1 = .1 * val + .9 * val1
      data = val1, val
      sleep(.01)
      #print data
      if(len(data) == 2):
        analogData.add(data)
        analogPlot.update(analogData)
    except KeyboardInterrupt:
      print 'exiting'
      break
  # close serial
# call main
if __name__ == '__main__':
  main()