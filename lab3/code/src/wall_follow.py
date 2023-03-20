#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#PID CONTROL PARAMS
kp = 0.7 #8
kd = 0.06 #0.005
ki = 0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 360 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 #meters
DESIRED_DISTANCE_LEFT = 1.4 #meters
VELOCITY = 4.5 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
a_angle = np.pi/3 # 60 degrees / follow left wall
b_angle = np.pi/2 # 90 degrees / follow left wall
theta = b_angle - a_angle
lookahead_distance = 1
#prev_time = time.time()

class WallFollow:
    """ Implement Wall Following on the car"""
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped,queue_size = 1000)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        indice = int((angle - data.angle_min) / data.angle_increment)
        range_value = data.ranges[indice]
        if not math.isnan(range_value) and not math.isinf(range_value): 
            return range_value
        return data.range_max

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global prev_time
        
        #compute the integral part
        """
        actual_time = time.time()
        dt = actual_time - prev_time
        prev_time = actual_time
        integral += prev_error
        steering_angle = kp * error + kd * (error - prev_error) / dt + ki * integral
        """
        
        steering_angle = kp * error + kd * (error - prev_error) + ki * integral
        
        #prev_error for the next implementation of steering angle 
        prev_error = error
        
        #control
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity * 2 / (2 + abs(steering_angle))
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist): 
        #compute the distance to leftDist : error
        a, b = self.getRange(data, a_angle), self.getRange(data, b_angle)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        distance = b * np.cos(alpha) + lookahead_distance * np.sin(alpha)
        error = distance - leftDist
        return error

    def lidar_callback(self, data):
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
