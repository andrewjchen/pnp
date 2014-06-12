import sys, pygame, os
import numpy as np
import math
import cv, cv2
# from sklearn.cluster import Kmeans
# from sklearn.mixture import GMM
import scipy.spatial as scispat
 
def get_distance(c1, c2):
    return math.sqrt((c2[0]-c1[0])**2 + (c2[1]-c1[1])**2)

def clamp(val, low, high):
    return min(max(low, val), high)

def cv2array(im): 
    depth2dtype = { 
        cv.IPL_DEPTH_8U: 'uint8', 
        cv.IPL_DEPTH_8S: 'int8', 
        cv.IPL_DEPTH_16U: 'uint16', 
        cv.IPL_DEPTH_16S: 'int16', 
        cv.IPL_DEPTH_32S: 'int32', 
        cv.IPL_DEPTH_32F: 'float32', 
        cv.IPL_DEPTH_64F: 'float64', 
    } 

    arrdtype=im.depth 
    a = np.fromstring( im.tostring(), 
         dtype=depth2dtype[im.depth], 
         count=im.width*im.height*im.nChannels) 
    a.shape = (im.height,im.width,im.nChannels) 
    return a 


class pads_set():
    def __init__(self,edges):

        tmp = np.hstack(( (edges[1,0]-edges[0,0])*np.random.random((10,1)),
         (edges[2,1]-edges[3,1])*np.random.random((10,1)) )) 

        #Middle of Workspace, if workspace top left is [0,0]
        #mean_pt = np.array([ (edges[1,0] - edges[0,0])/2, (edges[2,1] - edges[3,1])/2 ]) + np.array([])
        mean_pt = np.array( [edges[0,0], edges[3,1]] )
        self.pads_pos = tmp + np.tile(mean_pt,(tmp.shape[0], 1))
        print 'edges',edges
        print 'tmp',tmp
        print 'mean_pt',mean_pt
        print 'pads',tmp + np.tile(mean_pt,(tmp.shape[0], 1))
        
class parts_set():
    def __init__(self,edges):
        self.parts_pos = 400*np.random.random((10,2))
          
class pnp_head():
    def __init__(self):
        self.pos = np.array([320,240])
        #self.pos = np.array([int(np.random.random(1)*640), int(np.random.random(1)*480)])
        #Convention: center of camera is self.pos

        self.vel = np.array([0,0])
        self.set_target(np.array([300, 300]))
        self.part_picked = False
        self.edges_to_detect = np.array([1,1,1,1]) #L, R, U, D directions
        self.edges = np.zeros((4,2))
        self.prev_edge_detected = [0, 0] #Flag to indicate if edge has been detected[0], and for how long[1]
        self.part_blob = cv2.imread('part.png')
        self.blob_thresh = 0.8 #Template matching threshold
        self.screen_size = (640, 480)
        self.stat_target = [0,0]

    def tick(self):
        self.pos[0]+=self.vel[0]
        self.pos[1]+=self.vel[1]
        self.vel[0] = clamp((self.target[0]-self.pos[0]), -20, 20)
        self.vel[1] = clamp((self.target[1]-self.pos[1]), -20, 20)
   
    def get_current_pos(self):
#        self.pos = self.pos + 1*np.random.random((2,))
        return self.pos
  
    def set_target(self, target):
        self.target=target
        
    def pick(self):
        self.part_picked = True
        
    def place(self,parts):
        parts.parts_pos = np.vstack((parts.parts_pos, self.pos))

    def scanning_field(self,find_edges_flag):
        '''
        function that allows head to start in random position
        - scan left, right, up down
        - senses edge when camera sees color "self.edges_detected"
        '''
        find_blobs_flag = 0
        #print self.edges_to_detect
        edge_detected = self.field_edge_detected()

        #Detect new edge: 
        if edge_detected and self.prev_edge_detected[0]==0:
            #Set first '1' to zero in self.edges to detect
            ind = np.nonzero(self.edges_to_detect==1)[0][0]
            self.edges_to_detect[ind] = 0
            self.edges[ind,:] = self.pos - [32, 24]
            self.prev_edge_detected[0] = 1
            self.prev_edge_detected[1] = 1

        #Retreat from previous edge
        elif edge_detected==0 and self.prev_edge_detected[0]:
            self.prev_edge_detected[0] = 0
            self.prev_edge_detected[1] = 0

        #Count how long it has been since change
        elif edge_detected == self.prev_edge_detected[0]:
            self.prev_edge_detected[1]+= 1

        if self.edges_to_detect[0]:
            self.target[0] = self.pos[0] - 10 #move in -x direction
            self.target[1] = self.pos[1]

        elif self.edges_to_detect[1]:
            self.target[0] = self.pos[0] + 10 #move in +x direction
            self.target[1] = self.pos[1] 

        elif self.edges_to_detect[2]:
            self.target[0] = self.pos[0] #move in +y direction
            self.target[1] = self.pos[1]  + 10

        elif self.edges_to_detect[3]:
            self.target[0] = self.pos[0] #move in -y direction
            self.target[1] = self.pos[1]  - 10
        else:
            find_edges_flag = 0
            find_blobs_flag = 1

        return find_edges_flag, find_blobs_flag

    def field_edge_detected(self):
        '''
        Assume input is image input is in 640x480x3 np.ndarray
        '''
        camera_image = self.image
        #Template matching for future
        #tmpresult = cv2.matchTemplate(camera_image, self.edge_blob, cv.CV_TM_SQDIFF_NORMED)
        
        #Super simple edge detection (if sum of pixels < certain value)
        x = np.reshape(camera_image, (camera_image.shape[0]*camera_image.shape[1]*camera_image.shape[2],))
        if np.mean(x/255.) < 0.2:
            edge_detected = True 
        else:
            edge_detected = False

        return edge_detected

    def scanning_for_blobs(self, find_blobs_flag):
        '''function to acquire camera image, look for blobs, 
           if find blob add it to self.pads

           first, go to top left corner of detected edge frame
           then scan until fully cover edges 
         
          '''
        print find_blobs_flag
        #pan to top left
        if find_blobs_flag == 1:
            self.target[0] = self.edges[0,0]
            self.target[1] = self.edges[3,1]

            if np.sum((self.pos - self.target)**2) < 10:
                find_blobs_flag = 2 #Code for sweeping right

        #wait to get to top-left corner
        if find_blobs_flag == 2:
            self.target[0] = self.pos[0] + 10
            self.target[1] = self.pos[1]

            if self.target[0] > self.edges[3,0]:
                find_blobs_flag = 3 #Code for moving down
                self.foo = 1
                self.prev_blobs_flag = 2

        if find_blobs_flag == 3:
            if self.foo:
                self.stat_target[0] = self.pos[0]
                self.stat_target[1] = self.pos[1] + 50
                self.foo = 0

            self.target[0] = self.stat_target[0]
            self.target[1] = self.stat_target[1] 

            if np.sum((self.pos - self.target)**2) < 10:
                if self.prev_blobs_flag == 2:
                    find_blobs_flag = 4
                elif self.prev_blobs_flag == 4:
                    find_blobs_flag = 2

        if find_blobs_flag == 4:
            self.target[0] = self.pos[0] - 10
            self.target[1] = self.pos[1]

            if self.target[0] < self.edges[0,0]:
                find_blobs_flag = 3 #Code for moving down
                self.foo = 1
                self.prev_blobs_flag = 4

        if self.pos[1] > self.edges[2,1]:
            find_blobs_flag = 0

        return find_blobs_flag

        #Now alternate between sweeping right, move down by .75 height of camera, sweep left


        #Extract_blobs
        self.extract_blobs()

    def extract_blobs(self):
        '''use blob tracking to find parts in field of view'''
        camera_image = self.image
        tmpresult = cv2.matchTemplate(camera_image, self.part_blob, cv.CV_TM_SQDIFF_NORMED)

        #Threshold tmpresult
        blob_pos = np.array([i for i,match in enumerate(tmpresult.flat) if match < self.blob_thresh])

        #Translate to camera (x,y) coordinates
        blob_pos_x = blob_pos/tmpresult.shape[1]
        blob_pos_y = blob_pos - (blob_pos_x*tmpresult.shape[1])

        # #Account for edges
        # blob_pos_x = blob_pos_x + int(0.5*self.part_blob.shape[0])
        # blob_pos_y = blob_pos_x + int(0.5*self.part_blob.shape[1])
        blob_comb_pos = np.vstack(( np.array([blob_pos_x]), np.array([blob_pos_y]) )).T

        #Assume size of part is about 20x20 pixels: 
        #Calculate distance matrix: 
        tmp = scispat.distance.pdist(blob_comb_pos)
        pt_dist = scispat.distance.squareform(tmp)

        for i,pt in enumerate(blob_comb_pos):
            if 'part_list' in locals():
                if np.all(pt_dist[i,part_list] > 20): 
                    part_list.extend([i])
            else:
                part_list = [i]

        #Translate to workspace coordinates: 
        parts = blob_comb_pos[part_list,:] 

        #Translate to workspace coordinates:
        #Frame (set (0,0) to be center)
        parts = parts - np.tile(np.array([480/2, 640/2]), (parts.shape[0],1) )
        parts = parts + np.tile(self.pos, (parts.shape[0],1) )
        self.parts = np.vstack((self.parts, parts))
  
if __name__ == "__main__":
  
    pygame.init()
    BLACK = 0, 0, 0
    WHITE = 255, 255, 255

    #Initialize faux pads / parts (replace with blob tracking later)
    # pads = pads_set()
    # parts = parts_set()

    #Initialize head
    head = pnp_head()

    #Set Display
    screen = pygame.display.set_mode(head.screen_size) #Same as camera width
    pygame.display.set_caption("pnp display current position")
  
    #Initialize Clock
    clock = pygame.time.Clock()

    #Start by scanning field:
    find_edges_flag = 1
  
    #Setup Camera Loop: 
    cv.NamedWindow("w1", cv.CV_WINDOW_AUTOSIZE)
    camera_index = 0
    capture = cv.CaptureFromCAM(camera_index)

    #Game Loop
    current_select = -1
    while 1:
  
        #time.sleep(.001)
        clock.tick(50) #lock the frame rate to 50 FPS
          
        #Display
        screen.fill(WHITE)

        #TODO: Get current camera image
        frame = cv.QueryFrame(capture)
        cv.ShowImage("w1", frame)

        head.image = cv2array(frame)
        #head.image = sppw.cv2array( cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    
        #Scanning procedure:
        if find_edges_flag:
            find_edges_flag, find_blobs_flag = head.scanning_field(find_edges_flag)
        
        #When done, display markers:
        else:
            if 'pads' not in locals():
                pads = pads_set(head.edges)
                parts = parts_set(head.edges)
        #Plot markers
            for pad in np.arange(pads.pads_pos.shape[0]):
                pygame.draw.circle(screen, (255,0,0), (int(pads.pads_pos[pad][0]),int(pads.pads_pos[pad][1])), 5)
             
            # for par in np.arange(parts.parts_pos.shape[0]):
            #     pygame.draw.circle(screen, (0,255,0), (int(parts.parts_pos[par][0]),int(parts.parts_pos[par][1])), 20)
             
        if find_blobs_flag:
            find_blobs_flag = head.scanning_for_blobs(find_blobs_flag)

            #Check for clicks:
            ev = pygame.event.get()
    #        pos=(0,0)
            for event in ev:
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
        pygame.draw.rect(screen, BLACK, (head_pos[0]-32, head_pos[1]-24, 64, 48), 1)

        #Draw edges
        pygame.draw.lines(screen,BLACK,True,[ (head.edges[0,0], head.edges[2,1]), 
                                              (head.edges[0,0], head.edges[3,1]),
                                              (head.edges[1,0], head.edges[3,1]),
                                              (head.edges[1,0], head.edges[2,1])], 1)

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