#!/usr/bin/env python


""" 
    \package exp_assignment2
    \file human_command.py
    \brief This file contains the behaviour of a human to interact with the robot.
    \author Rohit Kumar
    \date 25/02/2021
    """
# import ros stuff
import rospy
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


def command(flag=0):
    """This funciton is used to move the ball in the environment. It is done by using an actionlib librabry
    The z direction coordinates are made negative in order to move the ball invisible.
    Actionlib function reaching_goal is used to for sending random positions to the ball

    Args:
        flag (int, optional): If the value is 1, we make the ball invisible by moving under the ground. 
        Defaults to 0.

    Returns:
        client.get_result(): return when the ball has reached the position.
    """
    client = actionlib.SimpleActionClient('reaching_goal', exp_assignment2.msg.PlanningAction)

    client.wait_for_server()

    goal = exp_assignment2.msg.PlanningGoal()
    
    goal.target_pose.pose.position.x = random.randint(-6,6)
    goal.target_pose.pose.position.y = random.randint(-6,6)
    if flag==0:
        goal.target_pose.pose.position.z = 0.5
    else:
        goal.target_pose.pose.position.z = -2.0

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()



def time_counter(seconds):
	"""A time_counter function to wait for the specfied number of time.

    Args:
        seconds ([integer]): [This argument is used to calculate the time passed when comapared to current time]
    """
	start_time = time.time()
	my_time = 0
	while (my_time < seconds):
		my_time = time.time()-start_time

def main():
    """The main function tries to emulate the human client by moving the ball around in the environment
    This is implemented using the function command.
    """
    rospy.init_node('human_command')
    time_counter(5)
    command(1)
    
    while True:
        n_play = random.randint(2,3)
        time_bw_calls = random.randint(30,40)
        i = 1
        print('We will play %d times' %n_play)
        while i <= n_play:
            print('Play: %d' %i )
            command()
            time_counter(15)
            i = i+1
        command(1)
        time_counter(time_bw_calls)

if __name__ == "__main__":
    main()
