#!/usr/bin/env python
import robot
import time
import rospy
import math
import numpy as np
import pathPlanning
from image_processing.msg import route
from image_processing.msg import bot_state
import matplotlib.pyplot as plt


path_x = np.array(50, dtype=np.uint64)
path_y = np.array(50, dtype=np.uint64)

end_x = 1100/21
end_y = 620/14
call = 0
time, x, y, x_dot, y_dot = 0,0,0,0,0
gen_traj_x = []
gen_traj_y = []
actual_traj_x = []
actual_traj_y = []

def traj_gen(path_x,path_y):
    global time, x, y, x_dot, y_dot, call
    global gen_traj_x, gen_traj_y, actual_traj_x, actual_traj_y
    # print "traj gen"
    #print "at path"
    #print "path_x : ",path_x, "path_y : ", path_y
    kp_x = 0.0006;     kp_y = 0.00029  ;     kp_theta = 1  #kp_x = 1.0;     kp_y = 1.7;     kp_theta = 0.30 ||| kp_x = 0.04 kp_y = 0.05
    kd_x = 0.00;   kd_y = 0.00;     kd_theta = 0
    ki_x = 0.000; ki_y = 0.000; ki_theta = 0.0 #ki_x = 0.00020 ki_y = 0.00020
    # kp_x = 0.000;     kp_y = 0.000;     kp_theta = 0.0  #kp_x = 1.0;     kp_y = 1.7;     kp_theta = 0.30 ||| kp_x = 0.04 kp_y = 0.05
    # kd_x = 0.0;   kd_y = 0.0;     kd_theta = 0.0
    # ki_x = 0.00; ki_y = 0.00; ki_theta = 0.0 #ki_x = 0.00026 ki_y = 0.00020
    old_error_x = 0.0;    old_error_y = 0.0;    old_error_theta = 0.0;
    sum_x = 0.0;          sum_y =  0.0;          sum_theta = 0.0;

    # print path_x
    # print path_y

    # if (call == 0):
    #    global call
    #    call = 1
            #print "call:", call

    '''
    Curve fitting is done to get a smooth path from the discrete path obtained using A*
    x, y are the x and y coordinates wrt time. x_dot, y_dot obtained by differentiating x and y
    '''
    time, x, y, x_dot, y_dot = pathPlanning.curve_fit((np.asarray(path_x)),(np.asarray(path_y)))
    destination_theta = -math.atan2( bot3.state[1] - y[-1]*end_y,bot3.state[0]- x[-1]*end_x)*180/math.pi
    if x[-1] > 10 :         # to account for the tilt in the camera
        error_x_pixel = 40
        error_y_pixel = 40
    else :
        error_x_pixel = 90
        error_y_pixel = 90

    error_theta = math.atan2(math.sin(math.pi*(bot3.state[2]-destination_theta)/180),math.cos(math.pi*(bot3.state[2]-destination_theta)/180))
    # print "y[-1]*end_y : ",y[-1]*end_y , "x[-1]*end_x : ",x[-1]*end_x
    # print "bot3.state[1] : ",bot3.state[1] , bot3.state[0]
    print "destination_theta" , destination_theta
    # print "X: ", x
    #print "traj received"
    x_actual = np.zeros(len(x))
    y_actual = np.zeros(len(y))

    time_period = (time[1]-time[0]) # /2
    rate = rospy.Rate(1/time_period)

    # x_dot = np.zeros(100)
    # y_dot = np.linspace(20,10,100)

# x_dot > 0 , then direction = down
# y_dot > 0, then direction =  right
    #start = time.time()
    # print "number of points : ",len(x_dot/2)
    # y[0] = bot3.state[0]/end_x
    # x[0] = bot3.state[1]/end_y
    # x_dot[len(x_dot)-1] = x_dot[len(x_dot)-2]
    # y_dot[len(x_dot)-1] = y_dot[len(x_dot)-2]
    # x_dot[0] = x_dot[1]
    # y_dot[0] = y_dot[1]

    num_points = 6 # number of velocity commands that will be sent to the robot. After using up num_points values, we again generate trajectory from the new path

    if len(x) <= 6 :
        num_points = len(x) - 1 #by trial and error

    for i in range(num_points):
        # print (abs(error_theta) < 20)
        '''
        Condition to check if goal location is reached
        '''
        if abs(path_y[-1]*end_y - bot3.state[1])<error_y_pixel and abs(path_x[-1]*end_x - bot3.state[0])<error_x_pixel and abs(error_theta) < 0.1 and call == 0:
            bot3.kinematic_model(0, 0) # stop robot
            # print "final grid point (x,y) : ",bot3.state[1]/end_y,bot3.state[0]/end_x
            call = 1
            plt.figure(1)
            plt.subplot(311)
            length = [t for t in range(1, len(actual_traj_x)+1)]
            # print length
            #length.append(44)
            #print(len(actual_traj_x))
            #print(len(actual_traj_y))
            #print len(gen_traj_x)
            #print len(gen_traj_y)
            length = np.array(length)
            actual_traj_x = np.array(actual_traj_x)
            actual_traj_y = np.array(actual_traj_y)
            #print gen_traj_x
            gen_traj_x = np.array(gen_traj_x)
            gen_traj_y = np.array(gen_traj_y)
            # print gen_traj_y
            plt.plot(length, actual_traj_y, 'x', length, gen_traj_y,'o')
            plt.xlabel('time')
            plt.ylabel('Y')
            plt.subplot(312)
            plt.plot(length,actual_traj_x, 'x', length, gen_traj_x,'o')
            plt.xlabel('time')
            plt.ylabel('X')
            plt.subplot(313)
            plt.plot(actual_traj_y,actual_traj_x,'x',gen_traj_y,gen_traj_x,'o')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()
            break

        elif call == 0 : # why call??? remove call var if unnecessary
            print "Y error",(path_y[-1]*end_y - bot3.state[1])
            print "X error",(path_x[-1]*end_x - bot3.state[0])
            error_theta = math.atan2(math.sin(math.pi*(bot3.state[2]-destination_theta)/180),math.cos(math.pi*(bot3.state[2]-destination_theta)/180))
            actual_traj_x.append(bot3.state[0])
            actual_traj_y.append(bot3.state[1])
            gen_traj_x.append(x[i]*end_x)
            gen_traj_y.append(y[i]*end_y)
            # print i
            x_actual[i] = bot3.state[0]
            y_actual[i] = bot3.state[1]
            error_x = float(x[i]*end_x - bot3.state[0])    #end_x and end_y are multiplied to find the coordinate of robot wrt the image (to get image pixel values of robot)
            error_y = float(y[i]*end_y - bot3.state[1])    #x,y give the grid coordinates
            print "destination_theta : ",destination_theta,"bot3.state[2] : ",bot3.state[2], "theta error ",error_theta

            if i==0 : # initial value was always erroneous, so default error was assigned. Remove if unnecessary
                error_x = 0
                error_y = 0
                error_theta = 0
            #print "x", y[i]*end_x, "bot_x:", bot3.state[0], "y:", x[i]*end_y, "bot_y:", bot3.state[1]
            # print "error_x:", error_x, "error_y:", error_y, "error_theta : ", error_theta
            # print "x_dot:", x_dot[i], "y_dot:", y_dot[i]
            x_dot[i] = x_dot[i] + kp_x*error_x + kd_x*(error_x-old_error_x) + ki_x*sum_x
            y_dot[i] = y_dot[i] + kp_y*error_y + kd_y*(error_y-old_error_y) + ki_y*sum_y
            theta_dot = kp_theta*error_theta + kd_theta*(error_theta-old_error_theta) + ki_theta*sum_theta
            #
            # x_dot[i] = 0
            # y_dot[i] = 20

            bot3.kinematic_model(x_dot[i], y_dot[i] , theta_dot)

            # if x_dot[i] >= 3:
            #     x_dot[i] = 3
            # if y_dot[i] >= 3:
            #     y_dot[i] = 3
            # if x_dot[i] <= -3:
            #     x_dot[i] = -3
            # if y_dot[i] <= -3:
            #     y_dot[i] = -3
            sum_x = sum_x + error_x
            sum_y = sum_y + error_y
            sum_theta = sum_theta + error_theta

            old_error_x = error_x
            old_error_y = error_y
            old_error_theta = error_theta

            print "x_dot_p:", x_dot[i], "y_dot_p:", y_dot[i] , "theta_dot_p", theta_dot

            rate.sleep()
        #end = time.time()
        #time_t = end - start
        # print "total points = {0}".format(len(x_dot))
        # print "total time = {0}".format(time_t)

        #if (bot_state[0] == y[len(x_dot)] and bot_state[1] == x[len(x_dot)]) :


def callback_points(msg):

    path_x = msg.x
    path_y = msg.y
    # print "at callback:", path_x, ":", path_y
    # s_x.push(path_x)
    # s_y.push(path_y)
    traj_gen(path_x,path_y)

def callback_bot(msg):
    if msg.num_circles == 3:
        #print "x:", msg.pose.x, "y:", msg.pose.y, "theta:", msg.pose.theta
        bot3.update_state((msg.pose.x,msg.pose.y,msg.pose.theta))



if __name__ =="__main__":
    flag = 0
    try:
        bot3=robot.robot()
        rospy.init_node('trajectory_track',anonymous=True)
        rospy.Subscriber('path',route,callback_points, queue_size=1)
        rospy.Subscriber('bot_states',bot_state,callback_bot, queue_size=1)

        print "Waiting for path..."
        while not rospy.is_shutdown():
            flag = 1

        #pathPlanning.curve_fit(np.asarray(route_path))
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
