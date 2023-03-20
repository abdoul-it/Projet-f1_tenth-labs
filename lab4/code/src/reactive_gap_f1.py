#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

kp = 0.5
radius = 10 # radius of bubble around the nearest point
FOV = np.radians(75)
VELOCITY = 2
half_wind = 1 # half size of the window

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan,self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        min_idx = int((-FOV - data.angle_min) / data.angle_increment)
        max_idx = int((FOV - data.angle_min) / data.angle_increment)
        ranges = np.array(data.ranges)

        window = 2 * half_wind + 1
        padding = np.array([np.nan]*half_wind)
        padded = np.concatenate((padding, ranges, padding)) 
        ranges = np.convolve(padded, np.ones(window) / window, 'valid')

        proc_ranges = ranges[min_idx : max_idx+1]
        proc_ranges[proc_ranges > 3] = 3

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        The max gap should not include nan 
        """
        size = len(free_space_ranges)
        start, max_gap = 0, 0
        tmp_length, tmp_idx = 0, 0
        threshold = 2

        for i in range(size):
            if free_space_ranges[i] > threshold:
                tmp_length +=1

                if tmp_length == 1:
                    tmp_idx = i
            else:
                if tmp_length > max_gap:
                    start = tmp_idx
                    max_gap = tmp_length
                tmp_length = 0
        
        if tmp_length > max_gap:
            start = tmp_idx
            max_gap = tmp_length
            
        return start, start + max_gap- 1
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        return int((start_i + end_i)/2)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(data)

        #find closest point and setting a safety buble
        min_idx = np.argmin(proc_ranges)
        free_space_ranges = np.array(proc_ranges)
        free_space_ranges[max(min_idx-radius, 0): min(min_idx+radius, len(free_space_ranges)-1)+1] = 0

        #Find max gap
        start_i, end_i = self.find_max_gap(free_space_ranges)

        #Find the best point in the gap
        idx_best_point = self.find_best_point(start_i, end_i, free_space_ranges)

        #Publish Drive message
        angle_best_point = idx_best_point * data.angle_increment  - FOV
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = kp * angle_best_point
        drive_msg.drive.speed = VELOCITY * 4 / (4 + abs(angle_best_point))
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
