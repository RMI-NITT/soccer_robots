#!/usr/bin/env python
from __future__ import division
import math
import XBee
import socket
import rospy
import serial
from image_processing.msg import bot_state
from time import sleep

#Required constants
WHEEL_RADIUS = 5   #cm
BOT_RADIUS = 13.5  #cm
MIN_VEL = 30
MIN_VEL_GTG = 85
bot_1=0001
MAX_WH_VEL = 11
#ser = serial.Serial('/dev/ttyUSB0',115200)
#x_dot robot moves up
#y_dot robot moves left

class robot:

    def __init__(self,init_state=[0,0,0], number=0,ip=0,port=0):

        self.state = init_state   #providefrom IP
        self.bot_number = number   #give numbers to each robot T1_1 T2_1 etc..
        self.wheel_radius = WHEEL_RADIUS
        self.bot_radius = BOT_RADIUS
	#        self.tcp_ip = ip;         self.tcp_port = port   #ip address of each robot
        #self.buffer_size = 1024

    #Sends message to the ip address mentioned.
    #Returns 1 if success else returns 0

    def send(self,message):
	'''
	#ser.write(message)
	#print "done \n"
	print "I'm here"
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.connect((self.tcp_ip, self.tcp_port))
	print "b4"
        self.sock.send(message)
        data = self.sock.recv(self.buffer_size)
        self.sock.close()
	print "after"
        if(data != 'H'):
           print "Error: Incorrect acknowledgement from robot ",self.bot_number
           return 0
        else:
	   print "ack "
           return 1
	'''
	  # Your serial port name here
        xbee = XBee.XBee("/dev/ttyACM0",115200)
	#print "b4 send"
        xbee.SendStr(message,0x0001)
        sleep(0.25)
        #xbee.SendStr(message,0x0002)
	#print "after"

    #Finds wheel velocities for given (v_x,v_y,w)
    #Returns 1 if data is sent to robot else returns 0
    '''
    def move(self,x_dot,y_dot,w,solenoid,dribbler):
        # print "velocities:",(x_dot,y_dot,w)
        vel_w_1 = ((-1*math.sin((30+self.state[2])*math.pi/180)*x_dot) + math.cos((30+self.state[2])*math.pi/180)*y_dot + self.bot_radius*w)/self.wheel_radius;
        vel_w_2 = ((-1*math.sin((-90+self.state[2])*math.pi/180)*x_dot) + math.cos((-90+self.state[2])*math.pi/180)*y_dot + self.bot_radius*w)/self.wheel_radius;
        vel_w_3 = ((-1*math.sin((150+self.state[2])*math.pi/180)*x_dot) + math.cos((150+self.state[2])*math.pi/180)*y_dot + self.bot_radius*w)/self.wheel_radius;
        max_val = max(abs(vel_w_1),abs(vel_w_2),abs(vel_w_3))
        if max_val <= 1:
            vel_w_1 = 0
            vel_w_2 = 0
            vel_w_3 = 0
        elif round(max_val,0) == abs(round(vel_w_1,0)) and round(max_val,0) == abs(round(vel_w_2,0)) and round(max_val,0) == abs(round(vel_w_3,0)):
            # print "I'm here"
            vel_w_1 = (vel_w_1)*(255-60)/10.0
            vel_w_2 = (vel_w_2)*(255-60)/10.0
            vel_w_3 = (vel_w_3)*(255-60)/10.0
            # print "wheel_velocities: ",vel_w_1,vel_w_2,vel_w_3
            if vel_w_1 > 0:
                vel_w_1 += 60
            else:
                vel_w_1 -= 60
            if vel_w_2 > 0:
                vel_w_2 += 60
            else:
                vel_w_2 -= 60
            if vel_w_3 > 0 :
                vel_w_3 += 60
            else:
                vel_w_3 -= 60
        else:

            vel_w_1 /= max_val; vel_w_2 /= max_val; vel_w_3 /= max_val;
            vel_w_1 *= (200-MIN_VEL)
            if vel_w_1 > 0:
                vel_w_1 += MIN_VEL
            else:
                vel_w_1 -= MIN_VEL
            vel_w_2 *= (200-MIN_VEL)
            if vel_w_2 > 0:
                vel_w_2 += MIN_VEL
            else:
                vel_w_2 -= MIN_VEL
            vel_w_3 *= (200-MIN_VEL)
            if vel_w_3 > 0 :
                vel_w_3 += MIN_VEL
            else:
                vel_w_3 -= MIN_VEL
        message = str(int(vel_w_1))+":"+str(int(vel_w_2))+":"+str(int(vel_w_3))+":"+str(solenoid)+":"+str(dribbler)+":"
	print "I'm here too"
        print message
        return self.send(message)
    '''

    def update_state(self,given_state):
        self.state = given_state;
        #print "State updated"
        #print "State : " , self.state
#Maxvelocity = 44cm/s
#MaxValue = 8.5
    def kinematic_model(self,x_dot=20, y_dot=0, w=0,solenoid=0,dribbler=0):
        #print "kinematic model called:", x_dot, y_dot, w
        #print "pose angle:", self.state[2]
        vel_w_1 = (((-1*math.sin((30+self.state[2])*math.pi/180)*y_dot) + math.cos((30+self.state[2])*math.pi/180)*x_dot + self.bot_radius*w)/self.wheel_radius); # right_wheel  wrt dribbler
        vel_w_2 = (((-1*math.sin((-90+self.state[2])*math.pi/180)*y_dot) + math.cos((-90+self.state[2])*math.pi/180)*x_dot + self.bot_radius*w)/self.wheel_radius); # left_wheel
        vel_w_3 = (((-1*math.sin((150+self.state[2])*math.pi/180)*y_dot) + math.cos((150+self.state[2])*math.pi/180)*x_dot + self.bot_radius*w)/self.wheel_radius); # back_wheel

        print "Velocity_wheels b4 scaling  :",vel_w_1,vel_w_2,vel_w_3

        if(vel_w_1>0.001):
            vel_w_1 = (vel_w_1/MAX_WH_VEL)*(255 - MIN_VEL_GTG) + MIN_VEL_GTG  # changed factor from 8.5 to 125 (due to increase in max(x_dot),max(y_dot) after including Kp)
        elif(vel_w_1<-0.001):
            vel_w_1 = (vel_w_1/MAX_WH_VEL)*(255 - MIN_VEL_GTG) - MIN_VEL_GTG
        if(vel_w_2>0.001):
            vel_w_2 = (vel_w_2/MAX_WH_VEL)*(255 - MIN_VEL_GTG) + MIN_VEL_GTG
        elif(vel_w_2<-0.001):
            vel_w_2 = (vel_w_2/MAX_WH_VEL)*(255 - MIN_VEL_GTG) - MIN_VEL_GTG
        if(vel_w_3>0.001):
            vel_w_3 = (vel_w_3/MAX_WH_VEL)*(255 - MIN_VEL_GTG) + MIN_VEL_GTG
        elif(vel_w_3<-0.001):
            vel_w_3 = (vel_w_3/MAX_WH_VEL)*(255 - MIN_VEL_GTG) - MIN_VEL_GTG


        max_val = max(abs(vel_w_1),abs(vel_w_2),abs(vel_w_3))
        #print max_val

        if(max_val>255):
            if(vel_w_1>0.001):
                vel_w_1 = (vel_w_1/max_val)*(255 - MIN_VEL_GTG) + MIN_VEL_GTG
            elif(vel_w_1<-0.001):
                vel_w_1 = (vel_w_1/max_val)*(255 - MIN_VEL_GTG) - MIN_VEL_GTG
            if(vel_w_2>0.001):
                vel_w_2 = (vel_w_2/max_val)*(255 - MIN_VEL_GTG) + MIN_VEL_GTG
            elif(vel_w_2<-0.001):
                vel_w_2 = (vel_w_2/max_val)*(255 - MIN_VEL_GTG) - MIN_VEL_GTG
            if(vel_w_3>0.001):
                vel_w_3 = (vel_w_3/max_val)*(255 - MIN_VEL_GTG) + MIN_VEL_GTG
            elif(vel_w_3<-0.001):
                vel_w_3 = (vel_w_3/max_val)*(255 - MIN_VEL_GTG) - MIN_VEL_GTG

	#print "Marana Velocity_wheels:",vel_w_1,vel_w_2,vel_w_3


        #vel_w_1 = -130
        #vel_w_2 = 255
        #vel_w_3 = -130

        message = str(int((vel_w_1 - (vel_w_1>255)*(vel_w_1%255))+500))+":"+str(int((vel_w_2 - (vel_w_2>255)*(vel_w_2%255))+500))+":"+str(int((vel_w_3 - (vel_w_3>255)*(vel_w_3%255))+500))+":"+str(solenoid)+":"+str(dribbler)+":"
        print vel_w_1 , ":" , vel_w_2 , ":" ,vel_w_3 # , ":" ,solenoid, ":" , dribbler
        return self.send(message)
