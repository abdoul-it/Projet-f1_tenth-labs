#!/usr/bin/env python
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


# the car deceleration is at 9.26 m/s^2 and velocity 1,.8 m/s
ttc_threshold = 0.35
FOV = np.pi/2

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        
        self.speed = 0
        
        # TODO: create ROS subscribers and publishers.
        
        #rospy.init_node('odom_listener')
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #rospy.init_node('scan_listener')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        #rospy.init_node('publisher_brake')
        self.pub_brake = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=1)
        #rospy.init_node('publisher_brake_bool')
        self.pub_brake_bool = rospy.Publisher("/brake_bool", Bool, queue_size=1)
        
        
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        
        # stationary robot
        if self.speed == 0.0:
            print("the robot is at a standstill")

        # preprocessing
        idx_max = int((FOV - scan_msg.angle_min) / scan_msg.angle_increment)
        idx_min = int((-FOV - scan_msg.angle_min) / scan_msg.angle_increment)

        angles_scan_truncate = np.arange(-FOV, FOV, scan_msg.angle_increment)
        ranges_truncate = np.array(scan_msg.ranges[idx_min : idx_max])

        # avoid division by zero
        speeds = np.maximum(self.speed * np.cos(angles_scan_truncate), 0.01)
        
        # caculate ttc
        ttcs = ranges_truncate / speeds
        min_ttc = np.min(ttcs)
        print(min_ttc ," seconds before the impact")

        # TODO: publish brake message and publish controller bool
        if min_ttc < ttc_threshold:
            state = AckermannDriveStamped()
            state.drive.speed  = 0.0
            self.pub_brake_bool.publish(True)
            self.pub_brake.publish(state)

        else :
            self.pub_brake_bool.publish(False)
        	
def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
