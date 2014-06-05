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
    

scale=640./50.
to_real=np.matrix([[1.,0,0],
                [0,-1.,640.],
                [0,0,scale]])
to_pixel=np.linalg.inv(to_real)

cam_scale=640./15.
to_cam_real=np.matrix([[1., 0, -(640.+320.)],
                    [0, -1., 240*1.],
                    [0,0,cam_scale]])
to_cam_pixel=np.linalg.inv(to_cam_real)
def real(pixels):
    ''' given pixel coordinates in the window, returns the position in 
    real space.
    '''
    coord=np.matrix([[pixels[0]],
                        [pixels[1]],
                        [1]])
    ret=(to_real*coord).A1
    #TODO better way to normalize matricies
    ret[0]/=ret[2]
    ret[1]/=ret[2]
    ret[2]=1.
    return ret

def pixel(rcoords):
    ''' given coordinates in real space, returns the drawn position in pixels'''
    coord=np.matrix([[rcoords[0]],
                        [rcoords[1]],
                        [1]])
    ret=(to_pixel*coord).A1
    #TODO better way to normalize matricies
    ret[0]/=ret[2]
    ret[1]/=ret[2]
    ret[2]=1.
    return ret
    
def cam_real(pixels):
    ''' given camera pixel coordinates, returns the position in real space.'''
    coord=np.matrix([[pixels[0]],
                        [pixels[1]],
                        [1]])
    ret=(to_cam_real*coord).A1
    #TODO better way to normalize matricies
    ret[0]/=ret[2]
    ret[1]/=ret[2]
    ret[2]=1.
    return ret
    
def cam_pixel(rcoords):
    ''' given real coordinates, returns the pixel coordinates in the camera frame'''
    coord=np.matrix([[rcoords[0]],
                     [rcoords[1]],
                     [1]])
    ret=(to_cam_pixel*coord).A1
    #TODO better way to normalize matricies
    ret[0]/=ret[2]
    ret[1]/=ret[2]
    ret[2]=1.
    return ret

def is_workspace(point):
    return point[0] < 640

class pnp_head():
    def __init__(self):
        self.cnc = None
#        self.cnc = ptc.PyTeacup()
        self.pos = np.array([10,20, 1], dtype=np.float32)
        self.vel = np.array([1,1,1], dtype=np.float32)
        self.command = np.array([0,0,1], dtype=np.float32)
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
        if self.cnc is not None:
            self.cnc.cmd(x, y, 0, 0)
    def set_target(self, target):
        rel=target-self.pos
#        print rel
        self.cmd(rel[0], rel[1])

pygame.init()
screen = pygame.display.set_mode((640+640, 640))
#camera_screen = pygame.display.set_mode((640, 480))
head=pnp_head()

prev_time=time.time()

points = []
print type(points)
while True:
    cur_time=time.time()
    dt=cur_time - prev_time
    
    screen.fill((255,255,255))
    pygame.draw.rect(screen, (0,0,0), (640, 480, 640, 160), 0)
    pygame.draw.line(screen, (127,127,127), (640, 0), (640, 640), 1)
    ###################### DRAWS STAGE REAL FRAME LINES ####################
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
    
    ###################### DRAWS CAMERA FRAME LINES ########################
    myfont = pygame.font.SysFont("monospace", 10)
    for x in range (640, 1280, 30):
        xp = int(cam_real(np.array((x, 0)))[0])
        label=myfont.render("{0}".format(xp), 2, (255,127,127))
        screen.blit(label, (x, 240, 10, 10))
        
    for y in range (0, 480, 30):
        yp = int(cam_real(np.array((0, y)))[1])
        label=myfont.render("{0}".format(yp), 2, (255,127,127))
        screen.blit(label, (640+320, y, 10, 10))
    ########################################################################
    
    ###################### DRAWS CAMERA REAL FRAME LINES ###################
    myfont = pygame.font.SysFont("monospace", 10)
    for x in range (640, 1280, 30):
        xp = int((cam_real(np.array((x, 0))) + head.pos)[0])
        label=myfont.render("{0}".format(xp), 2, (127,127,127))
        screen.blit(label, (x, 230, 10, 10))
        
    for y in range (0, 480, 30):
        yp = int((cam_real(np.array((0, y))) + head.pos)[1])
        label=myfont.render("{0}".format(yp), 2, (127,127,127))
        screen.blit(label, (640+320+10, y, 10, 10))
    ########################################################################
    
    ####################### EVENT LOOP #####################################
    ev = pygame.event.get()
    for event in ev:
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            if is_workspace(pos):
                if event.button == 1:
                    realp = real(np.array(pos))
                    print pos, realp
                    head.set_target(realp)
            else:
                if event.button == 1:
                    cam_realp = cam_real(np.array(pos))
                    realp=head.pos + cam_realp
                    print realp
    
    # head tick
    head.tick(dt)    
    
    # draws the head
    head_pos = pixel(head.pos)
    pygame.draw.circle(screen, (0,0,0), (int(head_pos[0]), int(head_pos[1])), 10)
    
    #draws the camera frame around head
    '''
    a               b 
            p
    c               d
    '''
    # points in real space. add to camera head space.
    a=cam_real(np.array((640, 0)))
#    print head_pos , a
    a=pixel(head.pos+cam_real(np.array((640, 0))))
    b=pixel(head.pos+cam_real(np.array((640+640, 0))))
    c=pixel(head.pos+cam_real(np.array((640+640, 480))))
    d=pixel(head.pos+cam_real(np.array((640, 480))))
    
    ap=(int(a[0]), int(a[1]))
    bp=(int(b[0]), int(b[1]))
    cp=(int(c[0]), int(c[1]))
    dp=(int(d[0]), int(d[1]))
    points=(ap,bp,cp,dp)
#    print points
    pygame.draw.polygon(screen, (200,200,200),(ap,bp,cp,dp), 1)
    
#    pygame.draw.rect(screen, (200, 200, 200), (int(head_pos[0]-80), int(head_pos[1]-60), 160, 120), 1)
    

    prev_time=cur_time
    pygame.display.update()
    time.sleep(.01)