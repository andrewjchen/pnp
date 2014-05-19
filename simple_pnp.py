import sys, pygame, os
import numpy as np
import math
  
def clamp(val, low, high):
    return min(max(low, val), high)
  
class pads_set():
    def __init__(self):
        self.pads_pos = 400*np.random.random((10,2))
        self.screen_size = (640, 480)
 
class parts_set():
    def __init__(self):
        self.parts_pos = 400*np.random.random((10,2))
          
class pnp_head():
    def __init__(self):
        self.pos = np.array([0,0])
        self.vel = np.array([0,0])
        self.set_target(np.array([300, 300]))
        self.part_picked = False
 
    def tick(self):
        self.pos[0]+=self.vel[0]
        self.pos[1]+=self.vel[1]
        self.vel[0] = clamp((self.target[0]-self.pos[0]), -2, 2)
        self.vel[1] = clamp((self.target[1]-self.pos[1]), -2, 2)
   
    def get_current_pos(self):
#        self.pos = self.pos + 1*np.random.random((2,))
        return self.pos
  
    def set_target(self, target):
        self.target=target
        
    def pick(self):
        self.part_picked = True
        
    def place(self,parts):
        parts.parts_pos = np.vstack((parts.parts_pos, self.pos))
        
def get_distance(c1, c2):
    return math.sqrt((c2[0]-c1[0])**2 + (c2[1]-c1[1])**2)
    
  
if __name__ == "__main__":
  
    pygame.init()
    BLACK = 0, 0, 0
    WHITE = 255, 255, 255
  
    #Set Display
    pads = pads_set()
    parts = parts_set()
    head = pnp_head()
  
    screen = pygame.display.set_mode(pads.screen_size) #Same as camera width
    pygame.display.set_caption("pnp display current position")
  
    #Initialize Clock
    clock = pygame.time.Clock()
  
    #Game Loop
    current_select = -1
    while 1:
  
        #time.sleep(.001)
        clock.tick(50) #lock the frame rate to 50 FPS
        

         
  
        #Display
        screen.fill(WHITE)
  
        #Plot markers
        for pad in np.arange(pads.pads_pos.shape[0]):
            pygame.draw.circle(screen, (255,0,0), (int(pads.pads_pos[pad][0]),int(pads.pads_pos[pad][1])), 20)
         
        for par in np.arange(parts.parts_pos.shape[0]):
            pygame.draw.circle(screen, (0,255,0), (int(parts.parts_pos[par][0]),int(parts.parts_pos[par][1])), 20)
         
          
        #Check for clicks:
        ev = pygame.event.get()
#        pos=(0,0)
        for event in ev:
#            print event
            if event.type == pygame.MOUSEBUTTONUP:
                
                pos = pygame.mouse.get_pos()
                if event.button == 3:
                    head.set_target(pos)
                if event.button == 1:
                    best_dist = 9999
                    best_part = None
                    for i,p in enumerate(parts.parts_pos):
                        part_pos = ()
                        dist =get_distance(pos, (p[0], p[1]))
                        if dist < best_dist:
                            best_part = i
                            best_dist = dist
                    if best_dist < 100:
                        current_select = best_part
            if event.type == pygame.KEYUP:
#                print event.key
                if event.key == pygame.K_SPACE:
                    parts.parts_pos = np.vstack((parts.parts_pos[0:current_select],
                                                 parts.parts_pos[current_select+1:]))
                    current_select = -1
                if event.key == pygame.K_RETURN:
                    head.place(parts)
                        
                        
  
        head.tick()
          
        #Get current position update
        head_pos = head.get_current_pos().astype(int)
  
        #Display current pt
        pygame.draw.circle(screen, (0,0,0), (head_pos[0], head_pos[1]), 10)
        
        if current_select != -1:
#            print current_select
            head.set_target(parts.parts_pos[current_select])
            pygame.draw.circle(screen,
                               (255,255,0),
                                (int(parts.parts_pos[current_select][0]),
                                 int(parts.parts_pos[current_select][1])),
                                25,
                                10)
          
        pygame.display.update()