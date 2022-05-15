#! /usr/bin/env python3
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import actionlib
from tt_213268_miniprj.msg import StartAction, StartGoal, StartFeedback, StartResult
wall_counter = 0
start_counter = False
forward_distance = 0
left_distance = 0
right_distance = 0
backward_distance = 0
from geometry_msgs.msg import Twist
from time import sleep
index =0
vel = Twist()
closest_point = 0

class LaserModel(object):

    def __init__(self, scan_data, angle_min, angle_max, range_min, range_max):

        # here the basic settings of your laser are defined
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.range_min = range_min
        self.range_max = range_max
        self.angle_inc = 0

        # some more member variables that we use during execution
        self.scan_data = scan_data
    '''
    Setter and getter methods for some member variables
    '''

    def set_angle_inc(self, angle_inc):
        self.angle_inc = angle_inc

    def get_angle_inc(self):
        return self.angle_inc

    def get_scan_length(self):
        return len(self.scan_data)

    '''
    TODO: calculate the angle increment
    '''

    def calc_angle_inc(self):
        angle_inc = (self.angle_max-self.angle_min)/ self.get_scan_length()
        return angle_inc

    '''
    TODO: port your code from previous exercise
    '''
    def calc_closest_point(self):
        mask_scan_data = np.ma.masked_equal(self.scan_data, 0.0, copy=False)
        minVal = mask_scan_data.min()
        return(minVal)

    def calc_index_of_closest_point(self):
        mask_scan_data = np.ma.masked_equal(self.scan_data, 0.0, copy=False)
        minVal = mask_scan_data.min()
        for i in range (0, self.get_scan_length()-1):
            if self.scan_data[i] == minVal:
                index1 = i
        return(index1)

class RobotMovement(object):
    def __init__(self):
        global vel
        global pub

    def robot_forward(self):
        vel.linear.x = 0.3
        pub.publish(vel)

    def robot_stop(self):
        vel.linear.x = 0
        vel.angular.z =0
        pub.publish(vel)

    def robot_turn_left(self):
        vel.angular.z = 0.3
        pub.publish(vel)

    def robot_turn_right(self):
        vel.angular.z = -0.3
        pub.publish(vel)

    def turn_90_degrees(self):
        print('do tommorow')



def actCallback(goal):
    global forward_distance
    global left_distance
    global right_distance
    global backward_distance
    global start_counter
    global closest_point
    global index
    left_wall =0
    right_wall =0
    close_flag = 0
    move = RobotMovement()
    while 1:
        print('left', left_distance)
        print('right', right_distance)
        print('forward', forward_distance)
        print('closest', closest_point)
        print('index', index)
        sleep(2)

    result.final_state = 'robot started'
    action_server.set_succeeded(result)




def subCallback(msg):
    global forward_distance
    global left_distance
    global right_distance
    global backward_distance
    global start_counter
    global closest_point
    global wall_counter
    global index
    forward_distance = msg.ranges[0]
    left_distance = msg.ranges[90]
    right_distance = msg.ranges[270]
    backward_distance = msg.ranges[180]
    scan_data = msg.ranges
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    range_min = msg.range_min
    range_max = msg.range_max
    app =LaserModel(scan_data,angle_min,angle_max,range_min,range_max)
    index = app.calc_index_of_closest_point()
    closest_point = app.calc_closest_point()





rospy.init_node('bot_controller')
pub = rospy.Publisher('cmd_vel', Twist)
action_server = actionlib.SimpleActionServer("mini_project_action_server", StartAction, actCallback, auto_start = False)
action_server_name = "Fibonacci Action Server"
action_server.start()
feedback = StartFeedback()
result = StartResult()
rospy.Subscriber('scan', LaserScan, subCallback)




rospy.spin()
