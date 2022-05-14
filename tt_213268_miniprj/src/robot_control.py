#! /usr/bin/env python3
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
wall_counter = 0

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
        print(minVal)
        for i in range (0, self.get_scan_length()-1):
            if self.scan_data[i] == minVal:
                index1 = i
        return(index1)

def deleteObstacle():
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)  # Create the service client
    kk = DeleteModelRequest()                                                       # Create an object of type DeleteModelRequest
    kk.model_name = "obstacle"                                                    # Fill the variable model_name of this object with the desired value
    result = delete_model_service(kk)


def callback(msg):
    global wall_counter
    forward_distance = msg.ranges[0]
    scan_data = msg.ranges
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    range_min = msg.range_min
    range_max = msg.range_max
    app =LaserModel(scan_data,angle_min,angle_max,range_min,range_max)
    closest_point = app.calc_closest_point()
    if closest_point > 0.1 and closest_point < 4:
        wall_counter += 1
    if wall_counter == 1:
        print('wall detected!!, removing obstacle')
        deleteObstacle()

rospy.init_node('bot_controller')
rospy.Subscriber('scan', LaserScan, callback)
rospy.spin()
