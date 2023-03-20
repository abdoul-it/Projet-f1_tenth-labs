#!/usr/bin/env python
import rospy
import csv
import numpy as np
import tf
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray 
from numpy import linalg as LA
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from os.path import expanduser


# TODO: import ROS msg types and libraries
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

home = expanduser('~')
look_head_distance = 1.5
kp = 1.2
file_waypoint = '/home/vboxuser/catkin_ws/src/f1tenth_labs/lab6/code/scripts/points.csv'
VELOCITY = 5

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
   
        rospy.Subscriber('/odom', Odometry, self.pose_callback)

        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size=1000)
        self.way_points_list = np.genfromtxt(file_waypoint, delimiter=',', usecols=(0, 1))
        self.index = -1

        # simulation
        self.marker_publisher = rospy.Publisher('waypoints', MarkerArray, queue_size=1000)
        self.marker_publisher_goal = rospy.Publisher('goal_waypoints', Marker, queue_size=1000)
        self.way_points_marker(self.way_points_list)


    def way_points_marker(self, way_points_list):
        markers = MarkerArray()
        for i in range(way_points_list.shape[0]):
            marker = Marker()
            marker.header.frame_id = 'map' 
            marker.id = i
            marker.type = Marker.SPHERE 
            marker.action = Marker.ADD 
            marker.pose.position.x = way_points_list[i][0] 
            marker.pose.position.y = way_points_list[i][1] 
            marker.pose.position.z = 0 
            marker.pose.orientation.w = 1.0 
            marker.scale.x = 0.2 
            marker.scale.y = 0.2 
            marker.scale.z = 0.2 
            marker.color.a = 1.0 
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.lifetime = rospy.Duration()
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def goal_marker(self, position):
        marker = Marker()
        marker.header.frame_id = 'map' 
        marker.id = 3100000
        marker.type = Marker.SPHERE 
        marker.action = Marker.ADD 
        marker.pose.position.x = position[0] 
        marker.pose.position.y = position[1] 
        marker.pose.position.z = 0 
        marker.pose.orientation.w = 1.0 
        marker.scale.x = 0.4 
        marker.scale.y = 0.4 
        marker.scale.z = 0.4 
        marker.color.a = 1.0 
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        self.marker_publisher_goal.publish(marker)

    def transform_to_robot_frame(self, pose_msg, x_goal_, y_goal_, current_position):
        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                           pose_msg.pose.pose.orientation.y, 
                           pose_msg.pose.pose.orientation.z, 
                           pose_msg.pose.pose.orientation.w])

        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        x = x_goal_ - current_position[0]
        y = y_goal_ - current_position[1]
        x_goal_robot_frame = x*np.cos(yaw) + y*np.sin(yaw)
        y_goal_robot_frame = y*np.cos(yaw) - x*np.sin(yaw) 
        return [x_goal_robot_frame, y_goal_robot_frame]

    def get_closest_way_points(self, way_points_list, current_position):
        while LA.norm(self.way_points_list[self.index]-current_position, 2) > look_head_distance:
            self.index += 1
            if self.index == len(self.way_points_list):
                self.index = 0


    def pose_callback(self, pose_msg):
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        current_position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])

        # avoid backloop
        if self.index < 0:
            self.get_closest_way_points(self.way_points_list, current_position)

        # find goal
        while LA.norm(self.way_points_list[self.index]-current_position, 2) < look_head_distance:
            self.index += 1
            if self.index == len(self.way_points_list):
                self.index = 0
        
        # TODO: transform goal point to vehicle frame of reference
        x_goal_, y_goal_ = self.way_points_list[self.index][0], self.way_points_list[self.index][1]
        markers_ = MarkerArray()
        self.goal_marker([x_goal_, y_goal_])

        position_goal = self.transform_to_robot_frame(pose_msg, x_goal_, y_goal_, current_position)
        distance_goal2 = LA.norm(position_goal, 2)

        # TODO: calculate curvature/steering angle
        steering_angle = 2 * position_goal[1] / (distance_goal2**2)

        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "odom"
        drive_msg.drive.steering_angle = kp * steering_angle 
        drive_msg.drive.speed = VELOCITY * 2 / (2 + abs(steering_angle))
        self.drive_pub.publish(drive_msg)

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()