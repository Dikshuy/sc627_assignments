#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *
from path import *
import math
import time 
import os
import matplotlib.pyplot as plt 


def forward(curr_pos, goal, step_size):
    a, b = goal[0]-curr_pos[0], goal[1]-curr_pos[1]
    norm = math.sqrt(a**2+b**2)
    return [curr_pos[0]+step_size*a/norm, curr_pos[1]+step_size*b/norm]

def move(curr_pos, x, y, step_size):
    return [curr_pos[0]+x*step_size, curr_pos[1]+y*step_size]

def plotter(curr_pos, goal, start_time, time_elapsed, total_distance, step_size, distance_from_goal):
    time_elapsed.append(time.time()-start_time)
    total_distance += step_size
    distance_from_goal.append(dist(curr_pos, goal))
    return time_elapsed, total_distance, distance_from_goal

def circumnavigate(p_hit, obs, goal, step_size, client, path, start_time, time_elapsed, total_distance, distance_from_goal):
    curr_pos = p_hit
    p_leave = curr_pos
    min_dist = dist(curr_pos, goal)
    wp = 0 
    count = 0 
    while dist(curr_pos, p_hit) > 2*step_size or count < 3:
        d = dist(curr_pos, goal)
        count += 1
        if d < min_dist:
            min_dist = d 
            p_leave = curr_pos
        x, y = computeTangentVectorToPolygon(curr_pos, obs)
        dis, wt, index = computeDistancePointToPolygon(curr_pos, obs)
        wp = wt 
        last_pos = curr_pos
        curr_pos = move(curr_pos, x, y, step_size)
        curr_pos = update(last_pos, curr_pos, client)
        path.append(curr_pos)
        time_elapsed, total_distance, distance_from_goal = plotter(curr_pos, goal, start_time, time_elapsed, total_distance, step_size, distance_from_goal)
    return p_leave, curr_pos, time_elapsed, total_distance, distance_from_goal

def leave(p_leave, curr_pos, obs, goal, step_size, client, path, start_time, time_elapsed, total_distance, distance_from_goal):
    wp = 0 
    while computeDistancePointToSegment(curr_pos, p_leave, goal)[0] > step_size:
        x, y = computeTangentVectorToPolygon(curr_pos, obs)
        dist, wt, index = computeDistancePointToPolygon(curr_pos, obs)
        wp = wt 
        last_pos = curr_pos
        curr_pos = move(curr_pos, x, y, step_size)
        curr_pos = update(last_pos, curr_pos, client)
        path.append(curr_pos)
        time_elapsed, total_distance, distance_from_goal = plotter(curr_pos, goal, start_time, time_elapsed, total_distance, step_size, distance_from_goal)
    return curr_pos, time_elapsed, total_distance, distance_from_goal

def update(curr_pos, new_pos, client):
    wp = MoveXYGoal()
    wp.pose_dest.x = new_pos[0]
    wp.pose_dest.y = new_pos[1]
    wp.pose_dest.theta = math.atan2((new_pos[1]-curr_pos[1]),(new_pos[0]-curr_pos[0]))
    # send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()

    # getting updated robot location
    result = client.get_result()

    # write to output file (replacing the part below)
    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
    return [result.pose_final.x, result.pose_final.y]

def bug_1(start, goal, step_size, obstaclesList):
    rospy.init_node('test', anonymous= True)
    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
    client.wait_for_server()

    total_distance = 0 
    time_elapsed = []
    distance_from_goal = []
    start_time = time.time()
    curr_pos = start 
    path = [curr_pos]
    while dist(curr_pos, goal) > step_size:
        min_dist = float("inf")
        for obs in obstaclesList:
            d,_,_ = computeDistancePointToPolygon(curr_pos, obs)
            if d < min_dist:
                min_dist = d
                closest_obs = obs

        if min_dist < step_size:
            p_leave, curr_pos, time_elapsed, total_distance, distance_from_goal = circumnavigate(curr_pos, closest_obs, goal, step_size, client, path, start_time, time_elapsed, total_distance, distance_from_goal)
            curr_pos, time_elapsed, total_distance, distance_from_goal = leave(p_leave, curr_pos, closest_obs, goal, step_size, client, path, start_time, time_elapsed, total_distance, distance_from_goal)

        last_pos = curr_pos
        curr_pos = forward(curr_pos, goal, step_size)
        curr_pos = update(last_pos, curr_pos, client)
        path.append(curr_pos)
        time_elapsed, total_distance, distance_from_goal = plotter(curr_pos, goal, start_time, time_elapsed, total_distance, step_size, distance_from_goal)
        
    path.append(goal)
    print("Success")
    print("total time time_elapsed:", round(time_elapsed[-1], 5))
    print("total distance covered:", round(total_distance, 3))
    # plt.title("Distance from goal vs time for bug 1 algorithm")
    # plt.xlabel("Time")
    # plt.ylabel("Distance from goal")
    # plt.plot(time_elapsed, distance_from_goal, color = "green")
    # plt.show()
    write(path, "1")

    return path

if __name__ == '__main__':
    filename = os.getcwd()+"/src/sc627_assignments/assignment_1/input.txt"
    start, goal, step_size, obstaclesList = read(filename)
    bug_1(start, goal, step_size, obstaclesList)