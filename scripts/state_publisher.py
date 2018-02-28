#!/usr/bin/env python
import cv2
import ip_module
import numpy as np
import rospy
import pathPlanning
import robot
import time
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from image_processing.msg import ball
from image_processing.msg import ball_predict
from image_processing.msg import bot_state
from image_processing.msg import route
SOFT_LIMIT_POSITIVE = 700
SOFT_LIMIT_NEGATIVE = 400
mat_size_2 = 21
mat_size_1 = 14

if __name__=="__main__":
    try:


        ball_object = ip_module.Ball()
        Robot = robot.robot()
        flag = 0    #why?
        bot3_x = 1 #why?
        bot3_y = 1
        goal_y = int(raw_input("goal_y:"))
        goal_x = int(raw_input("goal_x:"))
        found_ball = 0
        state_publisher = rospy.Publisher('bot_states',bot_state,queue_size=1)
        ball_state_publisher = rospy.Publisher('ball_state',ball,queue_size=1)
        ball_prediction_publisher = rospy.Publisher('ball_predicts',ball_predict,queue_size=1)
        route_path_publisher = rospy.Publisher('path', route,queue_size=1)
        rospy.init_node('state_publisher',anonymous=True)
        rate = rospy.Rate(ball_object.fps)
        centroid = (0,0)
        centroid_small = (0,0)

        mat = np.zeros((mat_size_1, mat_size_2), dtype=np.uint64)
        previous_mat = np.zeros((mat_size_1, mat_size_2), dtype=np.uint64)
        MAT = np.zeros((mat_size_1, mat_size_2), dtype=np.uint64)


        while not rospy.is_shutdown():
            original_image = ball_object.get_image()
            #ball_object.display_image(original_image,"original_image")
            image = ball_object.perspective_transform(original_image,ball_object.pts_in_img,ball_object.pts_reqd)
            im2 = image
            X = im2.shape[1]
            Y = im2.shape[0]
            start_x = 0
            start_y = 0
            end_x = X/mat_size_2
            end_y = Y/mat_size_1
            if found_ball == 0 :
                x_grid_num_ball = goal_x
                y_grid_num_ball = goal_y
            #im2 = ball_object.draw_grid(im2)
            imageGray = ball_object.rgb2gray(image)
            ret,thresh = ball_object.threshold_image(imageGray,245,255)
            # if not ball_object.display_image(thresh,"thresh"):
            #     flag = 1
            #     break
            ball_object.display_image(thresh,"thresh")
            contours,hierarchy = ball_object.find_contours(thresh)

            for i in range(len(contours)):
                c = contours[i]
    	        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                area = ball_object.find_area(contours[i])
                #print i,area

                if area > 3300 and area < 5200:
                    centroid = ball_object.get_center(contours[i])

                    x_grid_num_bot, y_grid_num_bot = centroid[0]/end_x, centroid[1]/end_y

                    if centroid != -1:

                        _,thresh = ball_object.threshold_image(imageGray,245,255) #value 245 was 220
                        cropped_image = ball_object.crop_image(thresh, centroid, 40)
                        # ball_object.display_image(cropped_image,"cropped_image")
                        # while(1):
                        #     if cv2.waitKey(1) == ord('q'):
                        #         cv2.destroyAllWindows()
                        #     continue
                        if isinstance(cropped_image,int):
                            print "The bot is out of bounds."
                            continue

                        cropped_contours,cropped_hierarchy = ball_object.find_contours(cropped_image)
                        count = 0
                        for j in range(len(cropped_contours)):

                            area_small = ball_object.find_area(cropped_contours[j])
                            #if area_small > 100 and area < 600:
                            #print "area_small:", area_small
                            if cropped_hierarchy[0][j][2] == -1 and cropped_hierarchy[0][j][3] != -1:
                                count += 1

                                if area_small > 200 and area_small < 600:
                                    centroid_small = ball_object.get_center(cropped_contours[j])

                        print "count = ", count
                        if count == 3:
                            bot3_x, bot3_y = centroid[0]/end_x, centroid[1]/end_y
                            # print "BOT 3 grid point " , bot3_y , bot3_x
                            mat[int(bot3_y)][int(bot3_x)]=2
                            #print "start assigned"
                            ball_object.update_bot_state(centroid[0],centroid[1])
                            #print "bot velocity from frame = ", ball_object.get_velocity_bot()

                        else :
                            #print "b4 MAT" , count
                            im2,MAT = ball_object.draw_grid(im2,int(x_grid_num_bot),int(y_grid_num_bot))

                        yaw_angle = ball_object.get_yaw_angle(40,40,centroid_small[0],centroid_small[1])
                        #print "State: ", centroid[0],centroid[1],yaw_angle
                        # print "Count: ",count
                        bot_msg = bot_state()
                        bot_msg.num_circles = count
                        bot_msg.pose.x = centroid[0]
                        bot_msg.pose.y = centroid[1]
                        bot_msg.pose.theta = -1*yaw_angle
                        # bot_msg.pose.theta = 120
                        #rospy.loginfo(bot_msg)
                        state_publisher.publish(bot_msg)

 		#for i in range(12):
		#	mat.append(row)

                    for i in range(mat_size_1):
                        for j in range(mat_size_2):
                            if mat[i][j] != 2 and mat[i][j] != 3:
                                mat[i][j] = mat[i][j] | MAT[i][j]

                    # mat = mat | MAT




            #if not np.array_equal(previous_mat, mat):


    	    #traj_time, traj_x, traj_y, traj_x_dot, traj_y_dot = pathPlanning.curve_fit(np.asarray(route_path))
            #pathPlanning.curve_fit(np.asarray(route_path))
            #Robot.kinematic_model()
    	    hsv_image = ball_object.rgb2hsv(image)
            mask_ball = ball_object.get_hsv_mask(hsv_image,ball_object.lower_ball,ball_object.upper_ball)
            contours_ball,hierarchy = ball_object.find_contours(mask_ball)

            for i in range(len(contours_ball)):
                area = ball_object.find_area(contours_ball[i])
                print i,area
                if area > 5: # previously : area > 60
                    found_ball = 1
                    # print "found_ball = 1"
                    centroid_ball = ball_object.get_center(contours_ball[i])
                    x_grid_num_ball, y_grid_num_ball = int(centroid_ball[0]/end_x), int(centroid_ball[1]/end_y)

                else :
                    found_ball = 0

    		    #i,j=5,5
    		    #cnt = cv2.rectangle(im2,(start_x+(X/9)*i,start_y+(Y/6)*j),(end_x+(X/9)*i,end_y+(Y/6)*j),(0,0,255))
    		    #print point_polygon_test(cnt,centroid_ball,False)
                    if centroid_ball == -1:
                        break
                    ball_object.update_state(centroid_ball[0],centroid_ball[1])
                    #print (ball_object.cx,ball_object.cy),ball_object.get_velocity()
                    # if ball_object.abs_vel() > 10:
                    #     ball_object.draw_arrow(image)

                    # imaginary lines
                # ball_object.prediction_lines(image)

                ball_msg = ball()
                ball_msg.x = ball_object.cx
                ball_msg.y = ball_object.cy
                ball_msg.vx = ball_object.vx_pixel
                ball_msg.vy = ball_object.vy_pixel
                rospy.loginfo(ball_msg)
                ball_state_publisher.publish(ball_msg)

                msg = ball_predict()
                # print ball_object.get_prediction(image)
                destination = ball_object.get_prediction(image)
                #print destination
                if destination == -1:
                    msg.predicted_x = -1
                    msg.predicted_y = -1
                elif (((ball_msg.vx**(2) + ball_msg.vy**(2))**(0.5)) < 50):
                    if ((ball_msg.vx > 0 and ball_msg.x > SOFT_LIMIT_POSITIVE) or (ball_msg.vx < 0 and ball_msg.y < SOFT_LIMIT_NEGATIVE)) and (((ball_msg.vx**(2) + ball_msg.vy**(2))**(0.5)) > 15):
                        msg.predicted_x = destination[0]
                        msg.predicted_y = destination[1]
                    else:
                        msg.predicted_x = -1
                        msg.predicted_y = -1
                else:
                    msg.predicted_x = destination[0]
                    msg.predicted_y = destination[1]

                ball_prediction_publisher.publish(msg)
                rospy.loginfo(msg)

            #if mat[goal_x][goal_y] != 2:

            if mat[y_grid_num_ball][x_grid_num_ball] != 2 :
                mat[y_grid_num_ball][x_grid_num_ball] = 3 #goal position
                route_length, route_path = pathPlanning.play(mat)
                #    previous_mat=mat
                    #print "route length = ", route_length
                path = np.asarray(route_path)
                # print "route path   = ", route_path
                path_y = np.ndarray.tolist(path[:,0])
                path_x = np.ndarray.tolist(path[:,1])
                route_msg = route()
                route_msg.x = path_x
                route_msg.y = path_y
                route_path_publisher.publish(route_msg)

            print "Printing mat..."
            for i in range(mat_size_1):
                s = ""
                for j in range(mat_size_2):
                    s += str(mat[i][j]) + " "

                print s


    	    mat=np.zeros((mat_size_1, mat_size_2), dtype=np.uint64)

            # print y_grid_num_ball,x_grid_num_ball

            if not ball_object.display_image(image):
                flag = 1
                break

            rate.sleep()

            if flag == 1:
                break
    except rospy.ROSInterruptException:
        pass
