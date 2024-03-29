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


def forward(curr_pos, grad, step_size, goal):
    return [curr_pos[0]+grad[0]*step_size, curr_pos[1]+grad[1]*step_size]

def plotter(curr_pos, goal, start_time, time_elapsed, total_distance, step_size, distance_from_goal):
    time_elapsed.append(time.time()-start_time)
    total_distance += step_size
    distance_from_goal.append(dist(curr_pos, goal))
    return time_elapsed, total_distance, distance_from_goal

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

def attractive_potential(curr_pos, goal, d_star):
    d = dist(curr_pos, goal)
    if d < d_star:
        return 0.5*0.8*(d**2)
    else:
        return d_star*0.8*d - 0.5*0.8*(d_star**2)

def attractive_potential_gradient(curr_pos, goal, d_star, chi):
    d = dist(curr_pos, goal)
    if d < d_star:
        return [chi*(goal[0]-curr_pos[0]), chi*(goal[1]-curr_pos[1])]
    else:
        return [d_star*chi*(goal[0]-curr_pos[0])/d, d_star*chi*(goal[1]-curr_pos[1])/d]

def repulsive_potential(curr_pos, obstaclesList, q_star):
    pot = 0 
    for i in range(len(obstaclesList)):
        d_i,_,_= computeDistancePointToPolygon(curr_pos, obstaclesList[i])
        if d_i < q_star:
            pot += 0.5*0.8*(1/d_i - 1/q_star)**2
        else:
            pot += 0 
    return pot 

def repulsive_potential_gradient(curr_pos, obstaclesList, q_star, eta):
    grad = [0, 0]
    n = len(obstaclesList)
    for i in range(n):
        d_i,_,_= computeDistancePointToPolygon(curr_pos, obstaclesList[i])
        if d_i < q_star:
            del_x, del_y= computeNormalVectorToPolygon(curr_pos, obstaclesList[i])
            grad[0] += (eta*(-1/q_star+1/d_i)*(del_x))*(1/(d_i)**2)
            grad[1] += (eta*(-1/q_star+1/d_i)*(del_y))*(1/(d_i)**2)
    return grad

def potential(start, goal, step_size, obstaclesList):
    rospy.init_node('test', anonymous= True)
    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
    client.wait_for_server()

    potential = 0 
    d_star, q_star = 2, 2
    total_distance = 0 
    time_elapsed = []
    distance_from_goal = []
    start_time = time.time()

    curr_pos = start
    path = [start]
    while dist(curr_pos, goal) > step_size:
        att_grad = attractive_potential_gradient(curr_pos, goal, d_star, 0.8)
        rep_grad = repulsive_potential_gradient(curr_pos, obstaclesList, q_star, 0.8)
        grad_x = att_grad[0]+rep_grad[0]
        grad_y = att_grad[1]+rep_grad[1]
        norm = math.sqrt(grad_x**2 + grad_y**2)
        grad = [grad_x, grad_y]

        last_pos = curr_pos
        curr_pos = forward(curr_pos, grad, step_size, goal)
        curr_pos = update(last_pos, curr_pos, client)

        ###
        # att = attractive_potential(curr_pos, goal, d_star)
        # rep = repulsive_potential(curr_pos, obstaclesList, q_star)
        # print("potential=", att+rep)
        ###

        path.append(curr_pos)
        time_elapsed, total_distance, distance_from_goal = plotter(curr_pos, goal, start_time, time_elapsed, total_distance, step_size, distance_from_goal)

    path.append(goal)
    write(path)
    print("Success")
    print("total time time_elapsed:", round(time_elapsed[-1], 5))
    print("total distance covered:", round(total_distance, 3))
    # plt.title("Distance from goal vs time for potential path planner algo")
    # plt.xlabel("Time")
    # plt.ylabel("Distance from goal")
    # plt.plot(time_elapsed, distance_from_goal, color = "green")
    # plt.show()
    return path

if __name__ == '__main__':
    filename = os.getcwd()+"/src/sc627_assignments/assignment_2/input.txt"
    start, goal, step_size, obstaclesList = read(filename)
    potential(start, goal, step_size, obstaclesList)