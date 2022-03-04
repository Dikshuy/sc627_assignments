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


def attractive_potential(curr_pos, goal, d_star):
    d = dist(curr_pos, goal)
    if d < d_star:
        return 0.5*0.8*(d**2)
    else:
        return d_star*0.8*d - 0.5*0.8*(d_star**2)

def attractive_potential_gradient(curr_pos, goal, d_star):
    d = dist(curr_pos, goal)
    if d < d_star:
        return [0.8*(curr_pos[0]-goal[0]), 0.8*(curr_pos[1]-goal[1])]
    else:
        return [d_star*0.8*(curr_pos[0]-goal[0])/d, d_star*0.8*(curr_pos[1]-goal[1])/d]

def repulsive_potential(curr_pos, obstaclesList, q_star):
    pot = 0 
    for i in range(len(obstaclesList)):
        d_i,_,_= computeDistancePointToPolygon(curr_pos, obstaclesList[i])
        if d_i < q_star:
            pot += 0.5*0.8*(1/d_i - 1/q_star)**2
        else:
            pot += 0 
    return pot 

def repulsive_potential_gradient(curr_pos, obstaclesList, q_star):
    grad = [0, 0]
    for i in range(len(obstaclesList)):
        d_i,_,_= computeDistancePointToPolygon(curr_pos, obstaclesList[i])
        dx, dy = computeTangentVectorToPolygon(curr_pos, obstaclesList[i])
        # print(d_i, q_star)
        if d_i < q_star:
            grad[0] += 0.8*(1/q_star-1/d_i)*(-dy)/(d_i)**2
            grad[1] += 0.8*(1/q_star-1/d_i)*(dx)/(d_i)**2
    return grad

def forward(curr_pos, goal, step_size):
    a, b = goal[0]-curr_pos[0], goal[1]-curr_pos[1]
    norm = math.sqrt(a**2+b**2)
    return [curr_pos[0]+step_size*a/norm, curr_pos[1]+step_size*b/norm]

def potential(start, goal, step_size, obstaclesList):
    potential = 0 
    d_star, q_star = 2, 2
    curr_pos = start
    # attg = attractive_potential_gradient(curr_pos, goal, q_star)
    path = [start]
    while dist(curr_pos, goal) > step_size:
        att_grad = attractive_potential_gradient(curr_pos, goal, d_star)
        rep_grad = repulsive_potential_gradient(curr_pos, obstaclesList, q_star)
        grad_x = att_grad[0]+rep_grad[0]
        grad_y = att_grad[1]+rep_grad[1]

        curr_pos[0] = curr_pos[0] - grad_x*step_size
        curr_pos[1] = curr_pos[1] - grad_y*step_size
        print(att_grad, rep_grad)
        print("grad:", grad_x, grad_y, "pos:", curr_pos)
        print()
        path.append(curr_pos)

    path.append(goal)
    write(path)
    # print(path)
    print("Success")
    return path

if __name__ == '__main__':
    filename = os.getcwd()+"/src/sc627_assignments/assignment_2/input.txt"
    start, goal, step_size, obstaclesList = read(filename)
    potential(start, goal, step_size, obstaclesList)