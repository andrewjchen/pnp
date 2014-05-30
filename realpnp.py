# -*- coding: utf-8 -*-
"""
Created on Tue May 20 20:20:31 2014

@author: ajc
"""

import pyteacup as ptc
import pygame
import numpy as np
import time
import math

def sumClip(arr, lower, upper):
    mag=math.sqrt(arr[0]**2 + arr[1]**2)
    theta=math.atan2(arr[1], arr[0])
    mag=max(min(upper, mag), lower)
    return np.array((mag*math.cos(theta), mag*math.sin(theta)), dtype=np.float32)

def real(pixels):
    return np.array((50*pixels[0]/640.0,
                     (50*(640 - pixels[1])/640.0)))
def pixel(pixels):
    return np.array((pixels[0]*640.0/50.0,640 -pixels[1]*640.0/50.0))

class pnp_head():
    def __init__(self):
        self.cnc = ptc.PyTeacup()
        self.pos = np.array([10,20], dtype=np.float32)
        self.vel = np.array([1,1], dtype=np.float32)
        self.command = np.array([0,0], dtype=np.float32)
        self.part_picked = False

    def tick(self, dt):
        dx=self.vel[0]*dt
        dy=self.vel[1]*dt
        self.pos[0]+=dx
        self.pos[1]+=dy
        self.command[0]-=dx
        self.command[1]-=dy
        self.vel=sumClip(10*self.command, -70, 70)
        
    def cmd(self, x, y):
        self.command=np.array((x,y), dtype=np.float32)
        self.cnc.cmd(x, y, 0, 0)
    def set_target(self, target):
        rel=target-self.pos
        print rel
        self.cmd(rel[0], rel[1])

pygame.init()
screen = pygame.display.set_mode((640, 640))
head=pnp_head()

prev_time=time.time()

while True:
    cur_time=time.time()
    dt=cur_time - prev_time
    
    screen.fill((255,255,255))
    ###################### DRAWS THE LINES #################################
    myfont = pygame.font.SysFont("monospace", 10)
    for x in range (0, 640, 30):
        xp = int(real(np.array((x, 0)))[0])
        label=myfont.render("{0}".format(xp), 2, (127,127,127))
        screen.blit(label, (x, 630, 10, 10))
        
    for y in range (0, 640, 30):
        yp = int(real(np.array((0, y)))[1])
        label=myfont.render("{0}".format(yp), 2, (127,127,127))
        screen.blit(label, (0, y, 10, 10))
    ########################################################################
    
    ####################### EVENT LOOP #####################################
    ev = pygame.event.get()
    for event in ev:
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            if event.button == 1:
                realp = real(np.array(pos))
                print pos, realp
                head.set_target(realp)
    

    # head tick
    head.tick(dt)    
    
    # draws the head
    head_pos = pixel(head.pos)
    pygame.draw.circle(screen, (0,0,0), (int(head_pos[0]), int(head_pos[1])), 10)
    

    prev_time=cur_time
    pygame.display.update()
    time.sleep(.01)