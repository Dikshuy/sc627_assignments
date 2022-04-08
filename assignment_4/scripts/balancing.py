#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

ANG_MAX = math.pi/18
VEL_MAX = 0.15

#original robot data
robot_pos = [0,0]
vel = [0,0] 
ang_vel = 0 

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

def dist(p1,p2):
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def callback_odom(data):
    '''
    Get robot data
    '''
    # print(data)
    robot_pos[0] = data.pose.pose.position.x 
    robot_pos[1] = data.pose.pose.position.y 
    vel[0] = data.twist.twist.linear.x 
    vel[1] = data.twist.twist.linear.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.orientation.w])
    ang_vel = data.twist.twist.angular.z
    return robot_pos, vel, yaw, ang_vel

def callback_left_odom(data):
    '''
    Get left robot data
    '''
    print('left robot')
    # print(data)
    left_robot_pos[0] = data.pose.pose.position.x 
    left_robot_pos[1] = data.pose.pose.position.y 
    left_vel = data.twist.twist.linear.x 
    roll, pitch, yaw_left = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.orientation.w])
    return left_robot_pos, left_vel, yaw_left

def callback_right_odom(data):
    '''
    Get right robot data
    '''
    print('right robot')
    # print(data)

    right_robot_pos = [0,0]
    right_vel = 0 
    right_robot_pos[0] = data.pose.pose.position.x 
    right_robot_pos[1] = data.pose.pose.position.y 
    right_vel = data.twist.twist.linear.x 
    roll, pitch, yaw_right = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.orientation.w])
    return right_robot_pos, right_vel, yaw_right 

def balancing():
    rospy.init_node('balancing', anonymous = True)
    rospy.Subscriber('/odom', Odometry, callback_odom) #topic name fixed
    rospy.Subscriber('/left_odom', Odometry, callback_left_odom) #topic name fixed
    rospy.Subscriber('/right_odom', Odometry, callback_right_odom) #topic name fixed

    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    r = rospy.Rate(30)

    while not rospy.is_shutdown(): 
        u = dist(robot_pos, right_robot_pos)-dist(robot_pos, left_robot_pos)
        v_x = 0.8*u
        #calculate v_x, v_y as per the balancing strategy
        #Make sure your velocity vector is feasible (magnitude and direction)

        #convert velocity vector to linear and angular velocties using velocity_convert function given above
        v_lin, v_ang = velocity_convert(robot_pos[0], robot_pos[1], yaw, v_x, 0)
        #publish the velocities below
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)
        print(v_lin, v_ang)
        #store robot path with time stamps (data available in odom topic)

        r.sleep()

if __name__ == '__main__':
    try:
        balancing()
    except rospy.ROSInterruptException:
        pass