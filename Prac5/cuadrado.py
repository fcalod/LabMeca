#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x = 2
y = 2
z = 0
theta = 0

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        dtheta = desired_angle_goal-theta        
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (dtheta < 0.03):
            break

def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        kv = 0.5 # Speed coefficient
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = kv * distance
    
        ka = 4.0 # Angular speed coefficient
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        dtheta = desired_angle_goal-theta        
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (distance < 0.05):
            break

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)     

        '''
        #orientate(2.0, 5.0)
        time.sleep(1.0)
        go_to_goal(2.0, 5.0)
    	time.sleep(1.0)
        '''
    
        time.sleep(2.0)
    
        orientate(5.0, 2.0)
        time.sleep(2.0)
        go_to_goal(5.0, 2.0)
        time.sleep(2.0)        
    
        orientate(2.0, 5.0)
        time.sleep(2.0)
        go_to_goal(2.0, 5.0)
        time.sleep(2.0)                
    
        orientate(5.0, 8.0)
        time.sleep(2.0)
        go_to_goal(5.0, 8.0)
        time.sleep(2.0)
    
        orientate(8.0, 2.0)
        time.sleep(2.0)
        go_to_goal(8.0, 2.0)
        time.sleep(2.0)        
    
        orientate(5.0, 2.0)
        time.sleep(2.0)
        go_to_goal(5.0, 2.0)
        time.sleep(2.0)    
    
    except rospy.ROSInterruptException:        
        pass
