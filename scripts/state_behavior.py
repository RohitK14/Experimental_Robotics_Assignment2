#!/usr/bin/env python
"""
#   \file finite_state_machine.py
#   \brief This file contains the state machine for implementing three behaviors-play, normal and sleep
#   \author Rohit Kumar
#   \version 0.2
#   \date 2020-11-13
#
#   \param [in] width the width of the discretized world
#   \param [in] height the height of the discretized world
#   \param [in] home_x is the x coordinate of the position where the robot sleeps
#   \param [in] home_y is the y coordinate of the position where the robot sleeps
#   \param [in] tired_level decides when the robot is tired to doing the task.
#   \param [in] speed tells the amount at which the robot travels between two points
#
#   \details
#
#   Description :
#
#   It uses smach libraries to implement the behaviors.
#   Specifically, the state machine defines the transition from a state to
#   another using the interface provided by smach.
#
#   
#   The states are defined in the respective classes, and the transistions are automatically performed
#   by the state machine from the smach libraries.
#
"""

# To run this file go to the src folder and type
# $ chmod +x state_behavior.py

import rospy
import smach
import smach_ros
import time
import random
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg
import exp_assignment2.msg

# Python libs
import sys
import time
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64


home_fixed=Point()
home_fixed.x = rospy.get_param('home_x',0)
home_fixed.y = rospy.get_param('home_y',0)
tired_level = rospy.get_param('tireness_level',5)
prev_pos = Point()


class Normal(smach.State):
    ##
    #   \brief __init__ initialises the Normal state in the smach_state
    #   \param outcomes are the possible transitions which are either it can go to normal ot play state
    #   \param input_keys These are possible input of the state
    #   \param output_keys These are possible outputs of the state.
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['go_sleep'],
                            input_keys=['normal_tired_counter_in'],
                            output_keys=['normal_tired_counter_out'])
        self.client = actionlib.SimpleActionClient('/robot/reaching_goal', 
                                                exp_assignment2.msg.PlanningAction)
        self.client.wait_for_server()
        self.goal = exp_assignment2.msg.PlanningGoal()
        # self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
        #                                    CompressedImage, self.callback,  queue_size=1)
        # self,found_image = rospy.Publisher("/found_img",Bool)
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        if len(cnts) > 0:
            global found_image 
            found_image = 1
        else:
            global found_image
            found_image = 0

    def execute(self, userdata):
        ##
        #   \brief In this execute() function, the robot randomly walks in the environment until it reaches the tired_level
        #   The lograndintic used here is that if we receive a command play in the middle, it reaches the
        #   last coordinate and that is published in the rostopic /moveToPose and shifts to Play state.
        #   Otherwise the robot goes to "Sleep" state after it gets tired
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state Normal')
            # global found_image
            # if found_image==1:
            #     print('Found the ball. Going to Play mode')
            #     return 'start_play'
            # else:
            #     global found_image
            #     if found_image==1:
            #         print('Found the ball. Going to Play mode')
            #        return 'start_play'
                #Random positions for the robot to move
            self.goal.target_pose.header.frame_id = "link_chassis"
            self.goal.target_pose.header.stamp = rospy.get_rostime()

            self.goal.target_pose.pose.position.x = random.randint(-7,7)
            self.goal.target_pose.pose.position.y = random.randint(-7,7)
            # self.goal.target_pose.pose.position.z = 0.0
            # self.goal.target_pose.pose.orientation.w = 0.0
            print('Robot going to: ',self.goal.target_pose.pose.position.x,
                    ',',self.goal.target_pose.pose.position.y)
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            self.client.get_result()
            userdata.normal_tired_counter_out = userdata.normal_tired_counter_in+1
            if userdata.normal_tired_counter_in >= tired_level:
                print('Robot is tired. Going to sleep...')
                return 'go_sleep'
                
                    


# define state Sleep
class Sleep(smach.State):
    ##
    #   \brief __init__ initialises the Sleep state in the smach_state
    #   \param outcomes are the possible transition is it can to normal state by wake_up transition
    #   \param input_keys These are possible input of the state
    #   \param output_keys These are possible outputs of the state.
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['wake_up'])
        self.client = actionlib.SimpleActionClient('/robot/reaching_goal', 
                                                exp_assignment2.msg.PlanningAction)    
    def execute(self, userdata):
        ##
        #   \brief In this execute() function, the robot goes to predefined home_fixed position
        #   The position is published in the topic /moveToPose
        #   The logic used here is that if we receive a command play in the middle, it reaches the
        #   last coordinate and that is published in the rostopic /moveToPose and shifts to Play state.
        #   Otherwise the robot goes to "Sleep" state after it gets tired

        rospy.loginfo('Executing state Sleep')
        self.client.wait_for_server()
        goal = exp_assignment2.msg.PlanningGoal()
        goal.target_pose.pose.position.x = home_fixed.x
        goal.target_pose.pose.position.y = home_fixed.y
        print('Sleeping at location: ',home_fixed.x,',',home_fixed.y)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        print(self.client.get_result())
        rospy.sleep(10)
        print('I am awake now')
        return 'wake_up'


VERBOSE = False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.camera_pub = rospy.Publisher("joint1_position_controller/command", 
                                          Float64, queue_size=1)
        self.flag_arrive = False
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = 0.005*(center[0]-400)
                vel.linear.x = -0.01*(radius-200)
                self.camera_pub.publish(0)
                self.vel_pub.publish(vel)

                #Rotate head +45 and -45 degrees 
                ##
                if self.flag_arrive == False and abs(radius-200) < 2:
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    cam_angle = Float64()
                    cam_angle.data = -math.pi/4
                    self.camera_pub.publish(cam_angle)
                    cv2.imshow('window',image_np)
                    cv2.waitKey(1)
                    time.sleep(1)
                    cam_angle.data = math.pi/4
                    self.camera_pub.publish(cam_angle)
                    cv2.imshow('window',image_np)
                    cv2.waitKey(1)
                    time.sleep(1)
                    cam_angle.data = 0.0
                    self.camera_pub.publish(cam_angle)
                    cv2.imshow('window',image_np)
                    
                    time.sleep(1)
                    self.flag_arrive = True
                    
            else:
                vel = Twist()
                self.camera_pub.publish(0)
                vel.linear.x = 0.5
                self.vel_pub.publish(vel)
        else:
            vel = Twist()
            vel.angular.z = 1
            print('rotate the robot')
            self.vel_pub.publish(vel)
            self.flag_arrive = False



        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()

# define state Play
# class Play(smach.State):
#     ##
#     #   \brief __init__ initializes the Play state with the outcome go_to_normal.
#     #   \param  outcomes lists the possible transitions. From play we can go to normal state.
#     #   \param input_keys It is the possible input of the state
#     #   \pram output keys It is the possible output of the state
#     def __init__(self):
#         smach.State.__init__(self, 
#                             outcomes=['go_to_normal'])
#     def execute(self, userdata):
#         ##
#         #   In this execute(), we implement play behavior.
#         #   A random position is generated for a person. The robot goes to the person, waits for the gesture and 
#         #   and goes to gesture position.
#         #   the robot goes and comes back to the gesture position and waits for another gesture position until 
#         #   it gets tired.
#         #   At last the robot goes to Normal position.
        
#         rospy.loginfo('Executing state Play')
#         ic = image_feature()

#         return 'go_to_normal'
                  

# main
def main():
    rospy.init_node('state_behavior')


    random.seed()
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Behaviours interface for sleep, normal and play'])

    sm.userdata.tireness = 0
    sm.userdata.person = Point()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_sleep':'SLEEP'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        # smach.StateMachine.add('PLAY', Play(), 
        #                        transitions={'go_to_normal':'NORMAL'})
    sis = smach_ros.IntrospectionServer('robot_behavior', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    cv2.destroyAllWindows()
    sis.stop()
if __name__ == '__main__':
    main()
