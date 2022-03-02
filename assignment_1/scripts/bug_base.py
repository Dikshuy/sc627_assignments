#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *

#import other helper files if any


rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file

lines = []
with open('../input.txt') as f:
    lines = f.readlines()

start_x, start_y = lines[0].split(",")
goal_x, goal_y = lines[1].split(",")
step_size = lines[2]
goal = [goal_x,goal_y]
# no_of_obs = 3
# obs = [[[0]*2]*no_of_obs]

# for i in range(no_of_obs):
#     obs[i][0][0],obs[i][0][1] = lines[4].split(",")
#     obs[i][1][0],obs[i][1][1] = lines[5].split(",")
#     obs[i][2][0],obs[i][2][1] = lines[6].split(",")

obstacle1=[[1,2],[1,0],[3,0]]
obstacle2=[[2,3],[4,1],[5,2]]
    
#setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0 #in radians (0 to 2pi)

while step_size > dist(goal,[result.pose_final.x,result.pose_final.y]):     # terminating condition
    wp = MoveXYGoal()
    wp.pose_dest.x = 1 + result.pose_final.x
    wp.pose_dest.y = 0
    wp.pose_dest.theta = 0 #theta is the orientation of robot in radians (0 to 2pi)

    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()

    #write to output file (replacing the part below)
    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)