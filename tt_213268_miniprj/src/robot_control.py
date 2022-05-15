#! /usr/bin/env python3
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import actionlib
from tt_213268_miniprj.msg import StartAction, StartGoal, StartFeedback, StartResult
wall_counter = 0                #it is flag to record first instance of comming in contact with wall

forward_distance = 0            # These six variables will store the distances measured at various angles
left_distance = 0
right_distance = 0
backward_distance = 0
right_45 = 0
left_45 = 0

right_wall = 0                  #flags which indicate on which side wall is present
left_wall = 0

index = 0                       #this variable gives the index of the closest point


from geometry_msgs.msg import Twist  #messege for cmd_vel topic
from time import sleep
vel = Twist()
closest_point = 0
first_start = 0

''' this class handels all laser related functions'''
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
        index1 = 0
        mask_scan_data = np.ma.masked_equal(self.scan_data, 0.0, copy=False)
        minVal = mask_scan_data.min()
        for i in range (0, self.get_scan_length()-1):
            if self.scan_data[i] == minVal:
                index1 = i

        return(index1)

''' this class handles all robot movement related functions'''
class RobotMovement(object):
    def __init__(self):
        global vel
        global pub

    def robot_forward(self):
        vel.linear.x = 0.2
        vel.angular.z =0
        pub.publish(vel)

    def robot_stop(self):
        vel.linear.x = 0
        vel.angular.z =0
        pub.publish(vel)

    def robot_turn_left(self):
        vel.angular.z = 0.5
        vel.linear.x = 0
        pub.publish(vel)

    def robot_turn_right(self):
        vel.angular.z = -0.5
        vel.linear.x =0
        pub.publish(vel)

    def robot_90_left(self):
        global index
        global right_wall
        print('turning left 1')
        self.robot_turn_left()
        while 1:
            if 275 > index > 265:     # idealy the index of closest point should be 275 degrees
                break
            sleep(0.001)
        right_wall = 1

    def robot_90_right(self):
        global index
        global left_wall
        print('turning right 1')
        self.robot_turn_right()
        while 1:
            if 95 > index > 85:     # idealy the index of closest point should be 275 degreess
                break
            sleep(0.001)
        left_wall = 1


def deleteObstacle():
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)  # Create the service client
    kk = DeleteModelRequest()                                                       # Create an object of type DeleteModelRequest
    kk.model_name = "obstacle"                                                    # Fill the variable model_name of this object with the desired value
    result = delete_model_service(kk)    #delete obstacle when first encountering the wall

def actCallback(goal):
    global forward_distance
    global left_distance
    global right_distance
    global backward_distance
    global closest_point
    global index
    global left_wall
    global right_wall
    global right_45
    global left_45
    left_wall =0
    right_wall =0
    close_flag = 0
    move = RobotMovement()
    while 1:
        sleep(0.1)
        move.robot_forward()

        ''' the robot will encounter wall for first time, it will decide between right and left turn,
        the close_flag and left or right wall flag will be updated after this '''
        if 0.1 < closest_point < 1 and close_flag == 0:
            print('got close to wall')
            move.robot_stop()
            sleep(0.2)
            close_flag =1
            if left_distance != 'inf': # distance are 'inf' if not reachable
                move.robot_90_right()
            else:
                move.robot_90_left()

            move.robot_stop()
            sleep(1)
            move.robot_forward()
            print ('moving forward 1')

        ''' while following left wall the robot will check its front left laser data, and try to keep the distance within the range'''
        if left_wall == 1:
            if left_45 > 2.4:
                move.robot_turn_left()
                sleep(0.5)
                move.robot_forward()
                sleep(1.5)
            if left_45 <1.6:
                move.robot_turn_right()
                sleep(0.5)
                move.robot_forward()
                sleep(1.5)
            if left_45 > 3.4:               # extreme measures will be taken if the robot is too much away or near to the wall
                move.robot_turn_left()
                sleep(2)
                move.robot_forward()
                sleep(1.5)
            if left_45 < 0.5:
                move.robot_turn_right()
                sleep(2)
                move.robot_forward()
                sleep(1.5)

        if right_wall == 1:                     # same code for right wall
            if right_45 > 2.4:
                move.robot_turn_right()
                sleep(1)
                move.robot_forward()
                sleep(1.5)
            if right_45 <0.5:
                move.robot_turn_left()
                sleep(1)
                move.robot_forward()
                sleep(1.5)
            if right_45 > 3.4 :
                move.robot_turn_right()
                sleep(2)
                move.robot_forward()
                sleep(1.5)
            if right_45 <0.5:
                move.robot_turn_left()
                sleep(2)
                move.robot_forward()
                sleep(1.5)

    result.final_state = 'robot started'
    action_server.set_succeeded(result)




def subCallback(msg):
    global forward_distance
    global left_distance
    global right_distance
    global backward_distance
    global closest_point
    global wall_counter
    global right_45
    global left_45
    global index

    forward_distance = msg.ranges[0]                        # distances at six angles are measured
    left_distance = msg.ranges[90]
    right_distance = msg.ranges[270]
    backward_distance = msg.ranges[180]
    right_45 = msg.ranges[315]
    left_45 = msg.ranges[45]

    scan_data = msg.ranges
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    range_min = msg.range_min
    range_max = msg.range_max
    app =LaserModel(scan_data,angle_min,angle_max,range_min,range_max)          # object for class LaserModel
    index = app.calc_index_of_closest_point()
    closest_point = app.calc_closest_point()

    ''' to rove wall when first detecting the wall'''
    if closest_point > 0.1 and closest_point < 4 and wall_counter < 3:
        wall_counter += 1

    if wall_counter == 1:
        print('wall detected!!, removing obstacle')
        deleteObstacle()





rospy.init_node('bot_controller')
pub = rospy.Publisher('cmd_vel', Twist)
rospy.Subscriber('scan', LaserScan, subCallback)
action_server = actionlib.SimpleActionServer("mini_project_action_server", StartAction, actCallback, auto_start = False)
action_server_name = "Fibonacci Action Server"
action_server.start()
feedback = StartFeedback()
result = StartResult()





rospy.spin()
