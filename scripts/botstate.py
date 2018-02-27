#!/usr/bin/env python
import robot
import time
import rospy
import math
from image_processing.msg import bot_state

def callback_bot(msg):
    if msg.num_circles == 3:
        bot_dictionary[msg.num_circles].update_state((msg.pose.x,msg.pose.y,msg.pose.theta))

if __name__=="__main__":
    print "Fuck"
    rospy.init_node('robot',anonymous=True)        
    rospy.Subscriber('bot_states',bot_state,callback_bot)
    rospy.spin()
