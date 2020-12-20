#!/usr/bin/env python

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

    client = actionlib.SimpleActionClient('reaching_goal', exp_assignment2.msg.PlanningAction)

    client.wait_for_server()

    goal = exp_assignment2.msg.PlanningGoal()
    
    goal.target_pose.pose.position.x = random.randint(-7,7)
    goal.target_pose.pose.position.y = random.randint(-7,7)
    if flag==0:
        goal.target_pose.pose.position.z = 0.5
    else:
        goal.target_pose.pose.position.z = -2.0

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()



def time_counter(seconds):
	""" Function to wait the specified seconds
	"""
	start_time = time.time()
	my_time = 0
	while (my_time < seconds):
		my_time = time.time()-start_time

def main():

    rospy.init_node('human_command')
    # command(1)
    # time_counter(10)
    while True:
        n_play = random.randint(2,5)
        time_bw_calls = random.randint(20,30)
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
