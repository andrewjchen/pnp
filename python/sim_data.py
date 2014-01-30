'''file to simululate income values of PnP '''
import numpy as np
import random 
import cv, cv2

class dataset():
	def __init__(self):
		self.current_pt = np.array([[0,0]])
		self.left = np.random.randint(-640,0) 
		self.right = self.left+640+np.random.randint(-10,10)
		self.bottom = np.random.randint(-480,0) 
		self.top = self.bottom+480+np.random.randint(-10,10)

		self.current_pt_trans = np.array([[self.left,self.top]])
		self.velocity = np.array([[0,0]])
		self.edge_cnt = 20
		self.stationary_cnt = 0
		self.rect = np.array([[-1000,1000],[-1000,-1000],[1000,-1000],[1000,1000], [-1000,1000]])

		self.window = [50, 50] #window of view (depends on z axis)
		im = cv2.imread('test.png') #full image
		self.image = np.zeros([im.shape[0]+1.5*self.window[0], im.shape[1]+1.5*self.window[1], 3])
		self.image[self.window[0]/2:im.shape[0]+self.window[0]/2, self.window[1]/2:im.shape[1]+self.window[1]/2, :] = im
		
		self.marker = self.image[425:455, 25:100]

	def step(self,direction,step_size):

		old_pt = self.current_pt.copy()
		dt = 1 #Insert update rate

		if direction == 'left':
			self.current_pt[0,0] = self.current_pt[0,0] - step_size

		elif direction == 'right':
			self.current_pt[0,0] = self.current_pt[0,0] + step_size

		elif direction == 'up':
			self.current_pt[0,1] = self.current_pt[0,1] + step_size

		elif direction == 'down':
			self.current_pt[0,1] = self.current_pt[0,1] - step_size

		if self.current_pt[0,0] < self.left:
			self.current_pt[0,0] = self.left
		if self.current_pt[0,0] > self.right:
			self.current_pt[0,0] = self.right
		if self.current_pt[0,1] > self.top:
			self.current_pt[0,1] = self.top
		if self.current_pt[0,1] < self.bottom:
			self.current_pt[0,1] = self.bottom

		self.velocity = (self.current_pt - old_pt)/dt
		
	#def set_markers(self):
		
class params():
	def __init__(self):
		self.screen_buffer = 1.3
		self.screen_size = (int(self.screen_buffer*640), int(self.screen_buffer*480))
		sz = (self.screen_size[0], self.screen_size[1], 3)
		self.workspace = np.zeros(sz) + 255
		self.margin = [int(.5*(int(self.screen_buffer*640) - 640)), int(.5*(int(self.screen_buffer*480) - 480))]


