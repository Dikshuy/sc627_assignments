#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *
import math


def read(filename):
    file = open(filename, "r")
    lines = file.readlines()
    start_x, start_y = lines[0].split(',')
    start = [float(start_x), float(start_y)]
    goal_x, goal_y = lines[1].split(',')
    goal = [float(goal_x), float(goal_y)]
    step_size = float(lines[2])
    line = lines[3:]
    obstaclesList = []
    tmp = []
    for i, obs in enumerate(line):
        if obs == "\n":
            tmp = []
            obstaclesList.append(tmp)
            continue
        obs = obs.split('\n')
        for j in obs:
            if not j == "":
                x,y = j.split(',')
                tmp.append((float(x), float(y)))

    return start, goal, step_size, obstaclesList 

def write(path):
    filename = "output_base.txt"
    file = open(filename, "w")
    for i in path:
        file.write(str[i][0]+", "+str[i][1]+"\n")
    file.close()

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
        min_dist = math.inf
        for i in obstaclesList:
            d,_,_ = computeDistancePointToPolygon(curr_pos, i)
            min_dist = min(d, min_dist)
        if min_dist < step_size:
            print("Failure: There is an obstacle lying between the start and goal \n")
            return None 

        last_pos = curr_pos
        curr_pos = forward(curr_pos, goal, step_size)
        curr_pos = update(last_pos, curr_pos, client)
        print(path)
        path.append(curr_pos)

    print("Success")
    write(path)

    return path

if __name__ == '__main__':
    filename = "/home/dikshant/catkin_ws/src/sc627_assignments/assignment_1/input.txt"
    start, goal, step_size, obstaclesList = read(filename)
    bug_base(start, goal, step_size, obstaclesList)