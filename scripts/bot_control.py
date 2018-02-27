#!/usr/bin/env python
import robot
import time
import rospy
import math
#from image_processing.msg import ball
from image_processing.msg import bot_state
#from image_processing.msg import ball_predict

def callback_bot(msg):
    if msg.num_circles == 3:
        print "x:", msg.pose.x, "y:", msg.pose.y, "theta:", msg.pose.theta
        bot3.update_state((msg.pose.x,msg.pose.y,msg.pose.theta))


if __name__=="__main__":
    global destination
    destination = (800,310,0)
    try:
        bot3=robot.robot()
        rospy.init_node('track_bot',anonymous=True)
        rospy.Subscriber('bot_states',bot_state,callback_bot)
        print "Waiting..."
        kp_x = 1.0;     kp_y = 1.7;     kp_theta = 0.30  #kp_x = 1.0;     kp_y = 1.7;     kp_theta = 0.30
        kd_x = 0.125;   kd_y = 0.1;     kd_theta = 0
        ki_x = 0.00020; ki_y = 0.00020; ki_theta = 0.0 #ki_x = 0.00020 ki_y = 0.00020
        old_error_x = 0;    old_error_y = 0;    old_error_theta = 0;
        sum_x = 0;          sum_y = 0;          sum_theta = 0;

        while not rospy.is_shutdown():
            #print "inside while"
            if destination[0] == 100 or destination[0] == -1 or destination[1] > 460 or destination[1] < 160:
                destination = HOME
            error_x = int(destination[0]-bot3.state[0])
            error_y = int(destination[1]-bot3.state[1])
            error_theta = math.atan2(math.sin(math.pi*(destination[2]-bot3.state[2])/180),math.cos(math.pi*(destination[2]-bot3.state[2])/180))
            print "Errors: ",error_x,error_y,error_theta
            x_dot = kp_x*error_x + kd_x*(error_x-old_error_x) + ki_x*sum_x
            y_dot = kp_y*error_y + kd_y*(error_y-old_error_y) + ki_y*sum_y
            if(abs(error_x) > 30 or abs(error_y) > 30):
                # print "I'm here"
                theta_dot = 0
                old_error_x = error_x;  old_error_y = error_y;
                sum_x += error_x;   sum_y += error_y;
            else:
                x_dot = y_dot = 0
                if abs(error_theta) > (math.pi/4):
                    theta_dot = kp_theta*error_theta #math.atan2(math.sin(kp_theta*error_theta + kd_theta*(error_theta-old_error_theta) + ki_theta*sum_theta),math.cos(kp_theta*error_theta + kd_theta*(error_theta-old_error_theta) + ki_theta*sum_theta))
                    sum_theta += error_theta
                    old_error_theta = error_theta;
                else:
                    print error_theta*180
                    theta_dot = 0

            if x_dot >= 175:
                x_dot = 175
            if y_dot >= 175:
                y_dot = 175
            if x_dot <= -175:
                x_dot = -175
            if y_dot <= -175:
                y_dot = -175
            if theta_dot <= -1.57:
                theta_dot = -1.57
            if theta_dot >= 1.57:
                theta_dot = 1.57

            y_dot=0; x_dot=-200; theta_dot
            bot3.kinematic_model((x_dot/8.0), (y_dot/8.0), -(theta_dot/2.0)) #x_dot, y_dot, w
            #rospy.spin()

    except rospy.ROSInterruptException:
        pass
