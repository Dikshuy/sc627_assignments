#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from math import pi, cos, sin, sqrt, atan2

ANG_MAX = math.pi/18
VEL_MAX = 0.15

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 7 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def dist(p1,p2):
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)

def norm(x):
    l = math.sqrt(x[0]**2+x[1]**2)
    return [x[0]/l, x[1]/l]

def cos_from_dot(a,b):
    a = norm(a)
    b = norm(b)
    return math.acos(a[0]*b[0]+a[1]*b[1])

def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    # print(data)
    n = len(data.obstacles)
    obs_cone_pos = [[0]*2]*n
    obs_cone_axis = [[0]*2]*n 
    obs_cone_ang = [0]*n 
    cnt=0
    for i in data.obstacles:
        obs_cone_ang[cnt]=math.asin(2*size/(dist(robot_pos,[i.pose_x,i.pose_y])))
        obs_cone_pos[cnt]=[i.vel_x,i.vel_y]
        obs_cone_axis[cnt]=norm([i.pose_x-robot_pos[0],i.pose_y-robot_pos[1]])
        cnt+=1
    return obs_cone_pos, obs_cone_axis, obs_cone_ang

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

def dist_to_edge(angle):
    if (angle>=-pi/4 and angle<=pi/4):
        return max_vel_dev/cos(angle)
    elif(angle>=pi/4 and angle<=3*pi/4):
        return max_vel_dev/sin(angle)
    elif (angle>=3*pi/4 and angle<=5*pi/4):
        return max_vel_dev/cos(pi-angle)
    else:
        return max_vel_dev/cos(3*pi/2-angle)

def collision(vel):
    cnt = 0
    for i in obs_cone_pos:
        if cos_from_dot(obs_cone_axis[cnt], [vel[0]-i[0], vel[1]-i[1]])<obs_cone_ang[cnt]:
            return False
        cnt += 1
    return True

def v_max_check(vel):
        return sqrt(vel[0]**2+vel[1]**2)<VEL_MAX

def check_in_max_theta_dev(vel):
        ang = atan2(vel[1], vel[0])
        return abs(ang-yaw)< max_angle_dev

def velocity_sample(goal):
    for i in range(35):
        moving_dir = math.atan2(goal[1]-robot_pos[1], goal[0]-robot_pos[0]) + alpha*i/34
        d = dist_to_edge(moving_dir)
        for j in range(6):
            vel_arr = [vel*math.cos(yaw)+cos(moving_dir)*(5-j)/5*d, vel*math.sin(yaw)+sin(moving_dir)*(5-j)/5*d] 
            collision_free = collision(vel_arr)
            v = v_max_check(vel_arr)
            ang = check_in_max_theta_dev(vel_arr)
            if collision_free and v and ang:
                return vel_arr 
        moving_dir = math.atan2(goal[1]-robot_pos[1], goal[0]-robot_pos[0]) - alpha*i/34
        d = dist_to_edge(moving_dir)
        for j in range(6):
            vel_arr = [vel*math.cos(yaw)+cos(moving_dir)*(5-j)/5*d, vel*math.sin(yaw)+sin(moving_dir)*(5-j)/5*d] 
            collision_free = collision(vel_arr)
            v = v_max_check(vel_arr)
            ang = check_in_max_theta_dev(vel_arr)
            if collision_free and v and ang:
                return vel_arr 
    return [0,0]

def collision_avoidance():
    rospy.init_node('assign3_skeleton', anonymous = True)
    rospy.Subscriber('/obs_data', ObsData, callback_obs) #topic name fixed
    rospy.Subscriber('/bot_1/odom', Odometry, callback_odom) #topic name fixed

    pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
    r = rospy.Rate(30)

    while dist(robot_pos, goal) > 0.3: 
        vel_arr = velocity_sample(goal)
        v_lin, v_ang = velocity_convert(robot_pos[0], robot_pos[1], yaw, vel_arr[0], vel_arr[1])
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)
        
        #store robot path with time stamps (data available in odom topic)

        r.sleep()

if __name__ == '__main__':
    # envt
    size = 0.15 
    alpha = 0.7
    max_angle_dev = 0.175
    max_vel_dev = 0.05
    start = [0,0]
    goal = [5,0]

    #original robot data
    robot_pos = [0,0]    # x, y position of the bot
    vel = 0              # velocity of the bot
    yaw = 0              # yaw of the bot

    obs_cone_pos = []
    obs_cone_axis = []
    obs_cone_ang = []

    try:
        collision_avoidance()
    except rospy.ROSInterruptException:
        pass



