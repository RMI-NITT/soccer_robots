#!/usr/bin/env python
import cv2
import numpy as np
import math
import time
from copy import deepcopy

FULL_WIDTH = 1920    #actual size of image
FULL_HEIGHT = 1080   #actual size of image

FINAL_WIDTH  = 1100  #final size
FINAL_HEIGHT = 620   #final size


"""
For perspective transform

"""
LEFT_TOP = [165,5]#[148,12]
RIGHT_TOP = [1800,13]#[1571,22]
RIGHT_BOTTOM = [1815,960]#[1583,830]
LEFT_BOTTOM = [200,1037]#[171,900]


mat_size_1 = 14  # size of the grid matrix
mat_size_2 = 21  # size of the grid matrix
MAT = np.array((mat_size_1, mat_size_2), dtype=np.uint64)
class IP(object):

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3,FULL_WIDTH) #3 - WIDTH
        self.cap.set(4,FULL_HEIGHT)  #4 - HEIGHT


    def get_image(self):
        ret,image = self.cap.read()
        while not np.size(image,0) == FULL_HEIGHT or not np.size(image,1) == FULL_WIDTH:
            print "trying"
            ret,image = self.cap.read()
        return image
        # print image

    def perspective_transform(self,image,pts1,pts2):
        return cv2.warpPerspective(image,cv2.getPerspectiveTransform(pts1,pts2),(FINAL_WIDTH,FINAL_HEIGHT))

    def rgb2gray(self,image):
        return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    def rgb2hsv(self,image):
        return cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    def threshold_image(self,image,threshold,max_val):
        return cv2.threshold(image,threshold,max_val,cv2.THRESH_BINARY)

    def display_image(self,image,window_name = "Image"):
        cv2.imshow(window_name,image)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            return 0
        return 1

    def find_contours(self,image):
        major,_,_ = cv2.__version__.split('.')
        if major == '3':
            _,contour,hierarchy = cv2.findContours(image, cv2.RETR_TREE, 2)
        else:
            contour,hierarchy = cv2.findContours(image, cv2.RETR_TREE, 2)
        return contour,hierarchy

    def find_area(self,contour):
        return cv2.contourArea(contour)

    def find_moments(self,contour):
        return cv2.moments(contour)

    def crop_image(self,image,center,amount):
        if int(center[1])-amount >= 0 and int(center[0])-amount >= 0:
            if int(center[1])+amount <= FINAL_HEIGHT and int(center[0])+amount <= FINAL_WIDTH:
                img = image[(int(center[1])-amount):(int(center[1])+amount),(int(center[0])-amount):(int(center[0])+amount)]
                return img
            else:
                return -1
        else:
            return -1

    def get_hsv_mask(self,image,lower,upper):
        return cv2.inRange(image, lower, upper)

    def end_all(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def draw_grid(self,im2,x_grid_num,y_grid_num):
        '''
        this function is to draw rectangular grid on the soccer arena to get discrete states for the A* search algo
        size of one rectangle is mat_size_1 x mat_size_2
        x_grid_num and y_grid_num are the location(coordinates) of the centre of the obstacle in the grid.
        '''

    	row = [0 for i in range(18)]
    	mat = np.zeros((mat_size_1, mat_size_2), dtype=np.uint64)
        X = im2.shape[1]        # size of the image obtained from the camera
        Y = im2.shape[0]        # size of the image obtained from the camera
    	start_x = 0             # start coordinate in the grid
    	start_y = 0             # start coordinate in the grid
    	end_x = X/mat_size_2    # end coordinate in the grid
    	end_y = Y/mat_size_1    # end coordinate in the grid
        """
        For some rect. blocks around the obstacle we draw color filled rectangles and update the corresponding locations in the matrix as 1.
        And for locations without obstacles unfilled rectangles are drawn and the matrix is updated as 0.
        """
    	for i in range(mat_size_2):
    		for j in range(mat_size_1):
    			if i not in range(x_grid_num-2, x_grid_num+3) or j not in range(y_grid_num-2, y_grid_num+3) :
    	   			im2 = cv2.rectangle(im2,(start_x+(X/mat_size_2)*i,start_y+(Y/mat_size_1)*j),(end_x+(X/mat_size_2)*i,end_y+(Y/mat_size_1)*j),(0,0,255))
    				mat[j][i] = 0
    			else :
    	   			im2 = cv2.rectangle(im2,(start_x+(X/mat_size_2)*i,start_y+(Y/mat_size_1)*j),(end_x+(X/mat_size_2)*i,end_y+(Y/mat_size_1)*j),(0,0,255),-1) 
    				mat[j][i] = 1
    				#print "HI", mat[j][i]
    	
    	MAT = np.copy(mat)  #dont remember why we copied the matrix
    	
    	return im2,MAT
    
class detectRobot(IP):

    def __init__(self):

        self.pts_in_img = np.float32([LEFT_TOP,RIGHT_TOP,RIGHT_BOTTOM,LEFT_BOTTOM])
        self.pts_reqd = np.float32([[0,0],[FINAL_WIDTH,0],[FINAL_WIDTH,FINAL_HEIGHT],[0,FINAL_HEIGHT]])
        self.cx_bot_old = 0
	self.cy_bot_old = 0
        self.cx_bot = 0
        self.cy_bot = 0
        self.vx_bot_pixel = 0;  self.vy_bot_pixel = 0
        super(detectRobot,self).__init__()

    # Input: image(RGB),threshold
    # Output: Contours and hierarchy in the same order
    def preprocessing(self,image,threshold,displayImage=False):
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,threshold,255,cv2.THRESH_BINARY)
        if displayImage:
            self.display_image(thresh)
        contour,hierarchy = self.find_contours(thresh)
        return ret,thresh,contour,hierarchy

    def get_center(self,contour):
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return -1
        else:
            return M["m10"]/M["m00"],M["m01"]/M["m00"]

    def get_yaw_angle(self,cX,cY,yX,yY):
        return math.atan2(cY-yY,cX-yX)*180/math.pi

    def update_bot_state(self,cx_bot,cy_bot):

        self.cx_bot_old = self.cx_bot
        self.cy_bot_old = self.cy_bot
        self.cx_bot = cx_bot
        self.cy_bot = cy_bot

        self.get_velocity_bot()

    def get_velocity_bot(self):
        self.vx_bot_pixel = int((self.cx_bot-self.cx_bot_old)*self.fps)
        self.vy_bot_pixel = int((self.cy_bot-self.cy_bot_old)*self.fps)
        if self.vx_bot_pixel < 0:
            self.dir = 0
        else:
            self.dir = 1
        return int(self.vx_bot_pixel),int(self.vy_bot_pixel)

class Ball(detectRobot):

    def __init__(self):
        super(Ball,self).__init__()
        self.fps = 28
        #self.update_fps(60)

        self.lower_ball = np.array([20,52,230])
        self.upper_ball = np.array([40,180,256])

        self.vx_pixel = 0;  self.vy_pixel = 0

        self.cx_old = 0;    self.cy_old = 0;
        self.cx = 0;        self.cy = 0
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.sides = [[(100,0),(100,1022)],[(1000,0),(1000,1022)]]

        self.dir = 0;       self.destination = (100,310)

    def update_fps(self,num_frames=120):
        print "Checking fps..."
        print "Capturing {0} frames".format(num_frames)
        # Start time
        start = time.time()
        # Grab a few frames
        for i in xrange(0, num_frames) :
            ret, frame = self.cap.read()
        # End time
        end = time.time()
        # Time elapsed
        seconds = end - start
        print "Time taken : {0} seconds".format(seconds)
        # Calculate frames per second
        fps  = num_frames / seconds;
        print "Estimated frames per second : {0}".format(fps);
        self.fps = fps

    def update_state(self,cx,cy):
        self.cx_old = self.cx
        self.cy_old = self.cy
        self.cx = cx
        self.cy = cy
        self.get_velocity()

    def get_velocity(self):
        self.vx_pixel = int((self.cx-self.cx_old)*self.fps)
        self.vy_pixel = int((self.cy-self.cy_old)*self.fps)
        if self.vx_pixel < 0:
            self.dir = 0
        else:
            self.dir = 1
        return int(self.vx_pixel),int(self.vy_pixel)

    def vector_length(self):
        return int(self.cx+self.vx_pixel), int(self.cy+self.vy_pixel)

    def draw_arrow(self,img):
        cv2.line(img,(int(self.cx),int(self.cy)),self.vector_length() ,(255,255,0),5)
        cv2.putText(img,str((int(self.vx_pixel),int(self.vy_pixel))),(int(self.cx),int(self.cy)), self.font, 0.5,(255,255,0),2,cv2.LINE_AA)

    def prediction_lines(self,img):
        cv2.line(img,self.sides[0][0],self.sides[0][1],(255,0,0),5)
        cv2.line(img,self.sides[1][0],self.sides[1][1],(255,0,0),5)

    def get_prediction(self,image):
        if self.vx_pixel:
            time_to_line = (self.sides[self.dir][0][0] - self.cx)/self.vx_pixel
            dest_y = self.cy + time_to_line*self.vy_pixel
            if dest_y < 0 or dest_y > FINAL_WIDTH:
                return -1
            # cv2.circle(image,(int(self.sides[self.dir][0][0]),int(dest_y)), 5, (0,0,255), -1)
            self.destination = self.sides[self.dir][0][0],dest_y
            # cv2.putText(image,str(self.destination),(int(self.destination[0]),int(self.destination[1])), self.font, 0.5,(255,255,0),2,cv2.LINE_AA)
        return self.destination

    def abs_vel(self):
        return math.sqrt(self.vx_pixel**2+self.vy_pixel**2)
if __name__=="__main__" :
    test = Ball()
