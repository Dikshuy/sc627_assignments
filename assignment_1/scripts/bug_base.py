#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *
from path import *

def forward(curr_pos, goal, step_size):
    a, b = goal[0]-curr_pos[0], goal[1]-curr_pos[1]
    norm = math.sqrt(a**2+b**2)
    return [curr_pos[0]+step_size*a/norm, curr_pos[1]+step_size*b/norm]

def update(curr_pos, new_pos, client):
    wp = MoveXYGoal()
    wp.pose_dest.x = new_pos[0]
    wp.pose_dest.y = new_pos[1]
    wp.pose_dest.theta = math.atan2((new_pos[1]-curr_pos[1]),(new_pos[0]-curr_pos[0]))
    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()

    #write to output file (replacing the part below)
    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
    return [result.pose_final.x, result.pose_final.y]

def bug_base(start, goal, step_size, obstaclesList):
    rospy.init_node('test', anonymous= True)
    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
    client.wait_for_server()

    curr_pos = start 
    path = []
    while dist(curr_pos, goal) > step_size:
        min_dist = float("inf")
        for i in obstaclesList:
            d,_,_ = computeDistancePointToPolygon(curr_pos, i)
            min_dist = min(d, min_dist)
        if min_dist < step_size:
            print("Failure: There is an obstacle lying between the start and goal \n")
            write(path, "base")
            return path 

        last_pos = curr_pos
        curr_pos = forward(curr_pos, goal, step_size)
        curr_pos = update(last_pos, curr_pos, client)
        path.append(curr_pos)
    # path.append(goal)
    print("Success")
    write(path, "base")

    return path

if __name__ == '__main__':
    filename = "/home/dikshant/catkin_ws/src/sc627_assignments/assignment_1/input.txt"
    start, goal, step_size, obstaclesList = read(filename)
    bug_base(start, goal, step_size, obstaclesList)