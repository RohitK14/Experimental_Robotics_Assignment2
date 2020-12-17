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

    client = actionlib.SimpleActionClient('/reaching_goal', exp_assignment2.msg.PlanningAction)

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



def main():

    rospy.init_node('human_command')
    
    while True:
        n_play = random.randint(2,5)
        time_bw_calls = random.randint(10,15)
        i = 1
        print('We will play %d times' %n_play)
        while i <= n_play:
            print('Play: %d' %i )
            command()
            time.sleep(5)
            i = i+1
        command(1)
        time.sleep(time_bw_calls)

if __name__ == "__main__":
    main()
