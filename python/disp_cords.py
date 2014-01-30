
import sys, pygame, os
import numpy as np
import sim_data
import time
import cv, cv2

def get_new_marks(marker_pos, input,cnt): 
    a = np.array((1,cnt)) # Change this to input
    marker_pos = np.vstack((marker_pos,a))
    cnt = cnt +1
    return marker_pos, cnt

def coord_translate(data,params):
    data.current_pt_trans[0,0] = data.current_pt[0,0] - data.left + params.margin[0]
    data.current_pt_trans[0,1] = data.top + params.margin[1] -  data.current_pt[0,1]
    return data

def update_rect(data,params,bounds,edge_flag):
    #Edge flag is 2, 3, 4, 0
    if edge_flag == 2:
        data.rect[0,0] = bounds[0] - data.left + params.margin[0]
        data.rect[1,0] = bounds[0] - data.left + params.margin[0]
        data.rect[4,0] = bounds[0] - data.left + params.margin[0]
    elif edge_flag == 3:
        data.rect[2,0] = bounds[1] - data.left + params.margin[0]
        data.rect[3,0] = bounds[1] - data.left + params.margin[0]
    elif edge_flag == 4:
        data.rect[1,1] = data.top + params.margin[1] - bounds[2]
        data.rect[2,1] = data.top + params.margin[1] - bounds[2]
    elif edge_flag == 0:
        data.rect[3,1] = data.top + params.margin[1] - bounds[3]
        data.rect[0,1] = data.top + params.margin[1] - bounds[3]
        data.rect[4,1] = data.top + params.margin[1] - bounds[3]
    return data

def detect_edge(data,params,bounds,edge_flag):
    #Left ; Right ; Top ; Bottom
    #Find current edge searching for: 
    marker_flag = 0
    i = 0
    while (i<3) and not np.isnan(bounds[i]):
        i += 1

    if sum(data.velocity[0]) == 0:
        data.stationary_cnt += 1

    if data.stationary_cnt > data.edge_cnt:
        data.stationary_cnt = 0 #Reset counter, bound has been found
        edge_flag += 1

        if i==0 or i==1:
            bounds[i] = data.current_pt[0,0] #Take x coord

        else:
            bounds[i] = data.current_pt[0,1] #Take y coord

            if i ==3:
                edge_flag = 0 #Stop looking for edges

                #Initialize Image Acquisition and Marker Search
                marker_flag = 1;
                data.workspace = np.zeros([bounds[2]-bounds[3]+data.window[1],bounds[1]-bounds[0]+data.window[0],3])
                
        data = update_rect(data,params,bounds,edge_flag)

    return bounds, edge_flag, data, marker_flag

if __name__ == "__main__":

    data = sim_data.dataset()
    par = sim_data.params()

    pygame.init()
    BLACK = 0, 0, 0
    WHITE = 255, 255, 255

    #Set Display
    screen = pygame.display.set_mode(par.screen_size) #Same as camera width
    pygame.display.set_caption("pnp display current position")

    #Initialize Clock
    clock = pygame.time.Clock()

    #Initialize Markers / Edges Array: 
    marker_pos = np.array((0,0)) # m x 2 array of marker positions
    bounds = np.empty((4,1)) #Left ; Right ; Top ; Bottom
    bounds[:] = np.nan

    edge_flag = 1 #Flag that indicates when edges are being searched for
    marker_flag = 0
    marker_cnt = 0

    #Game Loop
    while 1:
        #time.sleep(.001) 
        clock.tick(50) #lock the frame rate to 50 FPS

        #Quit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        #Display
        screen.fill(WHITE)

        #Get inputs: Check for new markers
        #if marker_flag:
            #marker_pos, marker_cnt = get_new_marks(marker_pos,[],marker_cnt)

        #Do edge detection
        if edge_flag:
            if edge_flag == 1:
                data.step('left')
            elif edge_flag ==2:
                data.step('right')
            elif edge_flag ==3:
                data.step('up')
            elif edge_flag ==4:
                data.step('down')
            bounds, edge_flag, data, marker_flag = detect_edge(data,par,bounds,edge_flag)

        #Acquire Image: 
        if marker_flag == 1:
            x = data.current_pt[0,0] - data.left
            y = data.top - data.current_pt[0,1]
            dx = data.window[0]/2
            dy = data.window[1]/2
            data.workspace[y:y+2*dy,x:x+2*dx,:] = data.image[y:y+2*dy, x:x+2*dx, :]
            print np.max(data.image[y:y+2*dy, x:x+2*dx, :])
            #data.workspace = pygame.transform.smoothscale(data.workspace, data.workspace.shape)
            try:
                sweepup
            except:
                sweepup = 1

            if sweepup:
                if data.current_pt[0,1]<bounds[2]: #Sweep Up
                    data.step('up')
                    left_ind=0
                else: #Sweep left
                    for x in range(50):
                        data.step('left')
                    else: #Done going left
                        sweepup = 0
            else:
                if data.current_pt[0,1] > bounds[3]: 
                    data.step('down')
                else:
                    for x in range(50):
                        data.step('left')
                    else: #Done going left
                        sweepup = 1


        #Find markers in current image. 

            # image = data.image[x:x+2*dx, y:y+2*dy] #compensates for margin around image, and window size. 
            # img = image.astype('uint8')
            # template = data.marker
            # tmp = template.astype('uint8')
            # w, h = template.shape[0:2]
        #res = cv2.matchTemplate(img,tmp,5) #cv2.TM_CCOEFF_NORMED)


            # for pt in zip(*loc[::-1]):
            #     cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
            #     cv2.imwrite('res.png',img)

        #Display workspace, as acquired
        if marker_flag:
            print 'par.work: ',par.workspace.shape, 'data.work: ',data.workspace.shape, 'par margin:', par.margin
            foo = data.workspace[data.window[0]:data.workspace.shape[0]-data.window[0], data.window[1]:data.workspace.shape[1]-data.window[1]]
            foo1=np.zeros([foo.shape[1],foo.shape[0],3])
            foo1[:,:,0] = foo[:,:,0].T
            foo1[:,:,1] = foo[:,:,1].T
            foo1[:,:,2] = foo[:,:,2].T
            par.workspace[par.margin[0]:par.margin[0]+data.workspace.shape[1]-2*data.window[1],par.margin[1]:par.margin[1]+data.workspace.shape[0]-2*data.window[0],:] = foo1
            pygame.surfarray.blit_array(screen,par.workspace.astype(int)) 
            

        #Plot markers
        if marker_cnt > 0:
            for m in np.arange(marker_pos.shape[0]):
                pygame.draw.circle(screen, (255,0,0), (int(marker_pos[m][0]),int(marker_pos[m][1])), 20)
          
        # Draw bounds
        pygame.draw.lines(screen, (0,0,0), 0, data.rect, 2)

        #Get current position update
        data = coord_translate(data,par)

        #Display current pt 
        pygame.draw.circle(screen, (0,0,0), (data.current_pt_trans[0,0], data.current_pt_trans[0,1]), 10)
        
        #Display marker search box
        if not edge_flag:
            pygame.draw.rect(screen, (0,0,0),(data.current_pt_trans[0,0]-data.window[0]/2,data.current_pt_trans[0,1]-data.window[1]/2,data.window[0],data.window[1]),2)

        pygame.display.update()

