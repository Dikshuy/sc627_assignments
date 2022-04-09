#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time

ANG_MAX = math.pi/18
VEL_MAX = 0.15
OBS_DIA = 0.15

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 5 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def dist(p1,p2):
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)

def norm(x):
    l = math.sqrt(x[0]**2+x[1]**2)
    if l == 0:
        return [0,0]
    return [x[0]/l, x[1]/l]

def vector_angle(a,b):
    a = norm(a)
    b = norm(b)
    return math.acos(a[0]*b[0]+a[1]*b[1])

def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    # print(data)
    n = len(data.obstacles)
    global obs_apex, obs_vel
    i=0
    for obs in data.obstacles:
        obs_apex[i][0] = obs.pose_x
        obs_apex[i][1] = obs.pose_y
        obs_vel[i][0] = obs.vel_x
        obs_vel[i][1] = obs.vel_y
        i += 1
    return obs_apex, obs_vel

def callback_odom(data):
    '''
    Get robot data
    '''
    # print(data)
    robot_pos[0] = data.pose.pose.position.x 
    robot_pos[1] = data.pose.pose.position.y 
    vel = data.twist.twist.linear.x 
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    ang_vel = data.twist.twist.angular.z
    return robot_pos, vel, yaw, ang_vel

def feasible_points(vel):
    points = []
    point = [0,0]
    for vel in np.linspace(0, VEL_MAX, 15):
        for ang in np.linspace(yaw-ANG_MAX, yaw+ANG_MAX, 20):
            if vel!=0:
                point[0] = vel + vel*math.cos(ang)
                point[1] = vel + vel*math.sin(ang)
                points.append(point)
    return points

def collision_cone():
    apex_angles = []
    for i in range(len(obs_apex)):
        dist_from_obs = dist(obs_apex[i], robot_pos)
        if OBS_DIA > dist_from_obs:
            apex_angles.append(math.pi/2)
        else:
            apex_angles.append(math.asin(OBS_DIA/dist_from_obs))
    return apex_angles

def collision_check(pt):
    collide = False
    v_rel =[0,0]
    obs_vec = [0,0]
    apex_angles = collision_cone()
    for i in range(len(obs_apex)):
        v_rel = [pt[0]-obs_vel[i][0], pt[1]-obs_vel[i][1]]
        obs_vec = [obs_apex[i][0]-robot_pos[0], obs_apex[i][1]-robot_pos[1]]
        collide = vector_angle(v_rel, obs_vec) < apex_angles[i]
        # print(collide)
    return collide

def feasible_next_step(): 
    points = feasible_points(vel)   
    next_step_points = []
    for i in range(len(obs_apex)):
        for pt in points:
            # print("came here")
            if collision_check(pt):
                next_step_points.append(pt)
    print(next_step_points)
    return next_step_points

def optimum_step(goal):
    goal_dir = [goal[0]-robot_pos[0], goal[1]-robot_pos[1]]
    next_step_points = feasible_next_step()
    min_dev = 2*math.pi 
    min_dev_pt = [0,0]
    for pt in next_step_points:
        curr_ang = vector_angle(goal_dir, pt)
        if curr_ang < min_dev:
            min_dev = curr_ang
            min_dev_pt = pt 
    return min_dev_pt

def collision_avoidance():
    rospy.init_node('assign3_skeleton', anonymous = True)
    rospy.Subscriber('/obs_data', ObsData, callback_obs) #topic name fixed
    rospy.Subscriber('/bot_1/odom', Odometry, callback_odom) #topic name fixed

    pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
    r = rospy.Rate(30)
    
    while dist(robot_pos, goal) > 0.3: 
        next_pt = optimum_step(goal)
        v_lin, v_ang = velocity_convert(robot_pos[0], robot_pos[1], yaw, next_pt[0], next_pt[1])
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)
        
        #store robot path with time stamps (data available in odom topic)

        r.sleep()

if __name__ == '__main__':
    start = [0,0]
    goal = [5,0]

    #original robot data
    robot_pos = [0,0]    # x, y position of the bot
    vel = 0              # velocity of the bot
    yaw = 0              # yaw of the bot

    # obs data
    obs_apex= [[0]*2]*3
    obs_vel = [[0]*2]*3
    
    try:
        collision_avoidance()
    except rospy.ROSInterruptException:
        pass
