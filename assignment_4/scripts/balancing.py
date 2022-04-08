#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time

ANG_MAX = math.pi/18
VEL_MAX = 0.15

def dist(p1,p2):
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)

def callback_odom(data):
    '''
    Get robot data
    '''
    # print(data)
    robot_pos[0] = data.pose.pose.position.x 
    robot_pos[1] = data.pose.pose.position.y 
    vel = data.twist.twist.linear.x 
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    path[0].append(robot_pos[0])
    path[1].append(robot_pos[1])
    total_time.append(time.time()-t_start)
    return robot_pos, vel, yaw

def callback_left_odom(data):
    '''
    Get left robot data
    '''
    print('left robot')
    # print(data)
    left_robot_pos[0] = data.pose.pose.position.x 
    left_robot_pos[1] = data.pose.pose.position.y 
    left_vel = data.twist.twist.linear.x 
    roll, pitch, yaw_left = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    return left_robot_pos, left_vel, yaw_left

def callback_right_odom(data):
    '''
    Get right robot data
    '''
    print('right robot')
    # print(data)
    right_robot_pos[0] = data.pose.pose.position.x 
    right_robot_pos[1] = data.pose.pose.position.y 
    right_vel = data.twist.twist.linear.x 
    roll, pitch, yaw_right = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    return right_robot_pos, right_vel, yaw_right 

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 8 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)
    ang_err = 0  # b'coz we are only moving in x-direction without rotataion
    v_lin = min(max(vel_x, -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def balancing():
    rospy.init_node('balancing', anonymous = True)
    rospy.Subscriber('/odom', Odometry, callback_odom) #topic name fixed
    rospy.Subscriber('/left_odom', Odometry, callback_left_odom) #topic name fixed
    rospy.Subscriber('/right_odom', Odometry, callback_right_odom) #topic name fixed

    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    r = rospy.Rate(30)
    itr = 0
    while itr < 1000: 
        #calculate v_x, v_y as per the balancing strategy
        u = dist(robot_pos, right_robot_pos)-dist(robot_pos, left_robot_pos)
        v_x = 1.2*u        # P controller
        # print(v_x)
    
        #convert velocity vector to linear and angular velocties using velocity_convert function given above
        v_lin, v_ang = velocity_convert(robot_pos[0], robot_pos[1], yaw, v_x, 0.0)
        #publish the velocities below
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)
        #store robot path with time stamps (data available in odom topic)
        itr += 1
        r.sleep()

if __name__ == '__main__':

    #original robot data
    robot_pos = [0,0]    # x, y position of the bot
    vel = 0              # velcoity of the bot
    ang_vel = 0          # angular velocity of the bot

    #left robot data
    left_robot_pos = [0,0]
    left_vel = 0        

    #right robot data
    right_robot_pos = [0,0]
    right_vel = 0 

    # yaw data
    yaw = 0
    yaw_left = 0 
    yaw_right = 0

    # plotting
    path = [[],[]]
    total_time = []
    t_start = 0

    try:
        balancing()
        plt.plot(total_time, path[0])
        plt.title("Position of robot w.r.t. time")
        plt.xlabel("time")
        plt.ylabel("x-position")
        plt.show()
        plt.plot(path[0], path[1])
        plt.title("Path Traced")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()

    except rospy.ROSInterruptException:
        pass