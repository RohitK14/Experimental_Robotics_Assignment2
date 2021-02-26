#!/usr/bin/env python
# To run this file go to the src folder and type
# $ chmod +x state_behavior.py


"""
    \package exp_assignment3
    \file state_behavior.py
    \brief This file contains the behaviour of a of the finite state machine.
    \author Rohit Kumar
    \date 25/02/2021

    \param [in] home_x
    \param [in] home_y
    \param [in] tireness_level

    Returns:
        [Finite state diagram]: [Differnet states can be visualised with the help of smach_viewer]
    """
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

"""
Parameters:
    [home_x]: [The launch file describes the home position of the robot. ]
    [tireness_level]: [It tells us the number of times a task can be performed before the fatigue.]
"""
home_fixed=Point()
home_fixed.x = rospy.get_param('home_x',0)
home_fixed.y = rospy.get_param('home_y',0)
tired_level = rospy.get_param('tireness_level',2)

global flag_state
flag_state = 1

class Normal(smach.State):
     """
       \brief __init__ initialises the Normal state in the smach_state
       \param outcomes are the possible transitions which are either it can go to normal ot play state
       \param input_keys These are possible input of the state
       \param output_keys These are possible outputs of the state.

    Subscribers:
    	subscriber: It subscribes to /robot/camera1/image_raw/compressed
        This is used to see if the robot can see the ball in the frame.
    Actions:
    	client: Client for action /robot/reaching_goal
		The client calls the action sever to move the robot to the specified target on the plane.
		goal: geometry_msgs.PoseStamped
		result: geometry_msgs.Pose
    """
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['go_sleep','start_play'],
                            input_keys=['normal_tired_counter_in'],
                            output_keys=['normal_tired_counter_out'])
        self.client = actionlib.SimpleActionClient('/robot/reaching_goal', 
                                                exp_assignment2.msg.PlanningAction)
        self.client.wait_for_server()
        self.goal = exp_assignment2.msg.PlanningGoal()
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
        self.found_image = 0
        self.pubBall = rospy.Publisher('/lost_ball',Bool,queue_size=1)
        global flag_state
        flag_state = 1
        self.counter = 0
    def callback(self, ros_data):
        """
        Callback function of subscribed topic. 
        Here images get converted and features detected.

        The function turns the varibale found_image to true when it sees the ball.
       
        """        
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
        global flag_state
        if len(cnts) > 0 and flag_state == 1:
            print('found Ball')
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
            cv2.circle(image_np, center, 5, (0, 0, 255), -1)
            self.found_image = 1
            self.client.cancel_all_goals()
            self.pubBall.publish(False)
            # self.subscriber.unregister()
        else:
            self.found_image = 0
            self.pubBall.publish(True)

        cv2.imshow('window', image_np)
        cv2.waitKey(2)


    def execute(self, userdata):
        """
        \brief In this execute() function, the robot randomly walks in the environment until it reaches the tired_level
           The algorithm used here is that if we receive a found_image variable as true,
           we shift to Play state.
         Otherwise the robot goes to "Sleep" state after it gets tired with respect to tireness_level
         The goals are cancelled as soon as the found_image is true.
         Returns:
            Sleep state - This is returned after tiring
            Play state - This is returned after the dound_image is turned to true.
        """
        userdata.normal_tired_counter_out = 0
        self.counter = 0
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state Normal')
            global flag_state
            flag_state = 1
            if self.found_image == 1:
                print('Found the ball. Going to Play mode')
                self.client.cancel_all_goals()
                return 'start_play'
            # Random positions for the robot to move
            self.goal.target_pose.header.frame_id = "link_chassis"
            self.goal.target_pose.header.stamp = rospy.get_rostime()

            self.goal.target_pose.pose.position.x = random.randint(-6,6)
            self.goal.target_pose.pose.position.y = random.randint(-6,6)
            self.goal.target_pose.pose.position.z = 0.0
            self.goal.target_pose.pose.orientation.w = 0.0
            print("Robot going to: ",self.goal.target_pose.pose.position.x,
                    ",",self.goal.target_pose.pose.position.y)
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            self.client.get_result()
            self.pubBall.publish(True)
            userdata.normal_tired_counter_out = userdata.normal_tired_counter_in+1
            self.counter = self.counter + 1
            if self.counter >= tired_level:
                self.found_image = 0
                self.pubBall.publish(True)
                print("Robot is tired. Going to sleep...")
                global flag_state
                flag_state = 0 
                return 'go_sleep'
                    
                    


# define state Sleep
class Sleep(smach.State):
    """
    \brief __init__ initialises the Sleep state in the smach_state
       \param outcomes are the possible transition is it can to normal state by wake_up transition

    Args:
        smach ([state]): The state of the finite machine is taken

    Action server: 
            client: Action server to mobve the robot to specified coordinates in the world.
    """
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['wake_up'])
        self.client = actionlib.SimpleActionClient('/robot/reaching_goal', 
                                                exp_assignment2.msg.PlanningAction)    
    def execute(self, userdata):
        """\brief In this execute() function, the robot goes to predefined home_fixed position
           
           The robot goes to "Sleep" state after it gets tired. It stays for 10s and then shifts back to Normal state.

        Returns:
            [wake_up]: transition state 
        """
        global flag_state
        flag_state = 0
        rospy.loginfo('Executing state Sleep')
        self.client.wait_for_server()
        goal = exp_assignment2.msg.PlanningGoal()
        goal.target_pose.pose.position.x = home_fixed.x
        goal.target_pose.pose.position.y = home_fixed.y
        print("Sleeping at location: ",home_fixed.x,",",home_fixed.y)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        self.client.get_result()
        rospy.sleep(10)
        print('I am awake now')
        return 'wake_up'


global inPlay
inPlay = 0
# define state Play
class Play(smach.State):
    """   
        \brief __init__ initializes the Play state with the outcome go_to_normal.
       \param  outcomes lists the possible transitions. From play we can go to normal state.

    The human is considered to be stationary. The robot sees the ball and tracks in using the camera sensor.
    When the image of the ball is not reveived, it shifts back to Normal state.
    
    Args:
        smach ([state]): This state is responsible for the Play behavior of the robot
    Subscribers: 
        subBall: subscribes (sts_msgs.Bool) from /lost_ball
    Publishers:
        pubBall: writes (sts_msgs.Bool) to /lost_ball
    Returns:
        [go_to_normal]: when the seach is over returns back to normal state.
    """
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['go_to_normal'])
        self.subBall = rospy.Subscriber('/lost_ball',Bool , self.callBackLost, queue_size=1)
        self.pubBall = rospy.Publisher('/found_ball',Bool,queue_size=1)
        self.ballLost_ = False
    def callBackLost(self, data):
        """It is used to record the data in ballLost_ variable 

        Args:
            data (std_msgs.Bool):  to check if the ball is lost or not
        """
        self.ballLost_ = data.data
    def execute(self, userdata):
        """
           In this execute(), we implement play behavior.
          The robot tracks the ball when ballLost variable is false. When it is set to false, the transition
          is made to Normal state.

        Returns:
            [go_to_normal]: transition state to switch back to Normal state
                    """
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state Play')
            if self.ballLost_ == True:
                self.pubBall.publish(False)
                global inPlay
                inPlay = 0
                return 'go_to_normal'
            else:
                self.pubBall.publish(True)
                global inPlay
                inPlay = 1
                
            
# main
def main():
    rospy.init_node('state_behavior')


    random.seed()
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Behaviours interface for sleep, normal and play'])

    sm.userdata.tireness = 0
    sm.userdata.person = Point()
    # Open the container
    """Generating the finite state machine with NORMAL, SLEEP and PLAY state.
    """
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_sleep':'SLEEP',
                                             'start_play':'PLAY'},
                               remapping={'normal_tired_counter_in':'tireness',
                                          'normal_tired_counter_out':'tireness'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_to_normal':'NORMAL'})
    sis = smach_ros.IntrospectionServer('robot_behavior', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    cv2.destroyAllWindows()
    sis.stop()
if __name__ == '__main__':
    main()
