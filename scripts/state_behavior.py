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
import time
import random


global reached, command
command = None
width =0
height=0
tired_level=0
home_fixed=Point()

width = rospy.get_param('world_width',20)
height = rospy.get_param('world_height',20)
speed = rospy.get_param('fast',1)    
home_fixed.x = rospy.get_param('home_x',0)
home_fixed.y = rospy.get_param('home_y',0)
tired_level = rospy.get_param('tireness_level',5)
prev_pos = Point()

##
#   brief reachCallback is a callback for the rostopic /Reached
#   \param x [std_msgs/Bool] is the confirmation that the robot reached the position
def reachCallback(x):
    global reached 
    reached = x.data

##
#   brief commandCallback is a callback for the rostopic /command
#   \param x [std_msgs/Strings] is the command to call the robot for playing with the person
def commandCallback(str_play):
    global command 
    command = str_play.data
##
#   \class Normal
#   \brief This class defines the state of the state machine corresponding to the robot 
#   randomly moving in the 2D plane.
#
#   It is a inheritance from smach and the state is added to the smach_state.
#   In this state the robot moves around randomly by subscribing to 
#   the rostopic /moveToPose. 
#   It has a function execute() providing the intended behavior. 

# define state Normal
class Normal(smach.State):
    ##
    #   \brief __init__ initialises the Normal state in the smach_state
    #   \param outcomes are the possible transitions which are either it can go to normal ot play state
    #   \param input_keys These are possible input of the state
    #   \param output_keys These are possible outputs of the state.
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['go_sleep','start_play'])
        self.client = actionlib.SimpleActionClient('/robot/reaching_goal', exp_assignment2.msg.PlanningAction)
        

    def execute(self, userdata):
        ##
        #   \brief In this execute() function, the robot randomly walks in the environment until it reaches the tired_level
        #   The logic used here is that if we receive a command play in the middle, it reaches the
        #   last coordinate and that is published in the rostopic /moveToPose and shifts to Play state.
        #   Otherwise the robot goes to "Sleep" state after it gets tired
        while not rospy.is_shutdown():
            
            rospy.loginfo('Executing state Normal')
            global command
            if command=="play":
                print('Command of playing received. Going to Play mode.')
                command = None

                return 'start_play'
            else:
                if userdata.normal_tired_counter_in >= tired_level:
                    print('Robot is tired. Going to sleep...')
                    return 'go_sleep'
                else:
                    if command=="play":
                        print('Command received while reaching the position')
                        print('Going to play mode')
                        return 'start_play'
                    random_pos = Point()
                    random_pos.x = random.randint(0,width)
                    random_pos.y = random.randint(0,height)
                    self.pub.publish(random_pos)
                    a = np.array((random_pos.x,random_pos.y))
                    b = np.array((prev_pos.x,prev_pos.y))
                    t = np.linalg.norm(a-b)/speed
                    rospy.sleep(t)
                    prev_pos = random_pos
                    print('Location: ',random_pos,'reached in time',t)
                    userdata.normal_tired_counter_out = userdata.normal_tired_counter_in+1
                    print('Tireness level of the robot: ',userdata.normal_tired_counter_in)


# define state Sleep
class Sleep(smach.State):
    ##
    #   \brief __init__ initialises the Sleep state in the smach_state
    #   \param outcomes are the possible transition is it can to normal state by wake_up transition
    #   \param input_keys These are possible input of the state
    #   \param output_keys These are possible outputs of the state.
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['wake_up'],
                            input_keys=['sleep_timer_in'],
                            output_keys=['sleep_timer_out'])
        self.pub = rospy.Publisher('/moveToPose', Point, queue_size=10)
        self.isReached = rospy.Subscriber('/Reached',Bool,reachCallback)
    def execute(self, userdata):
        ##
        #   \brief In this execute() function, the robot goes to predefined home_fixed position
        #   The position is published in the topic /moveToPose
        #   The logic used here is that if we receive a command play in the middle, it reaches the
        #   last coordinate and that is published in the rostopic /moveToPose and shifts to Play state.
        #   Otherwise the robot goes to "Sleep" state after it gets tired

        rospy.loginfo('Executing state Sleep')
        self.pub.publish(home_fixed)
        print('Sleeping at location: ',home_fixed.x,',',home_fixed.y)
        rospy.sleep(10)
        print('I am awake now')
        userdata.sleep_timer_out = 0
        return 'wake_up'

# define state Play
class Play(smach.State):
    ##
    #   \brief __init__ initializes the Play state with the outcome go_to_normal.
    #   \param  outcomes lists the possible transitions. From play we can go to normal state.
    #   \param input_keys It is the possible input of the state
    #   \pram output keys It is the possible output of the state
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['go_to_normal'],
                            input_keys=['play_timer_in'],
                            output_keys=['play_timer_out'])
        self.pub = rospy.Publisher('/moveToPose', Point, queue_size=10)
    def execute(self, userdata):
        ##
        #   In this execute(), we implement play behavior.
        #   A random position is generated for a person. The robot goes to the person, waits for the gesture and 
        #   and goes to gesture position.
        #   the robot goes and comes back to the gesture position and waits for another gesture position until 
        #   it gets tired.
        #   At last the robot goes to Normal position.
        
        rospy.loginfo('Executing state Play')
        person = Point()
        person.x = random.randint(0,width)
        person.y = random.randint(0,height)
        print('Person called from position: ',person)
        print('Going to man position.')
        self.pub.publish(person)
        rospy.sleep(20)
        print('Reached man!!')
        print("We will play " + str(tired_level) + " times")
        while not rospy.is_shutdown():
            if userdata.play_timer_in >= tired_level:
                print('Robot tired. Going to Normal mode')
                return 'go_to_normal'
            else:
                gesture = Point()
                gesture.x = random.randint(0,width)
                gesture.y = random.randint(0,height)
                print('Gesture command: ',gesture)
                self.pub.publish(gesture)
                a = np.array((gesture.x,gesture.y))
                b = np.array((person.x,person.y))
                t = np.linalg.norm(a-b)/speed
                rospy.sleep(t)
                print('Reached the gesture position. Going back to person',person)
                self.pub.publish(person)
                rospy.sleep(t)  
                userdata.play_timer_out = userdata.play_timer_in+1
                print('Tireness level of the robot: ',userdata.play_timer_in) 
                print('Waiting for next gesture location.')
                rospy.sleep(5)       

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
                               transitions={'go_sleep':'SLEEP', 
                                            'start_play':'PLAY'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_to_normal':'NORMAL'})
    sis = smach_ros.IntrospectionServer('robot_behavior', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
