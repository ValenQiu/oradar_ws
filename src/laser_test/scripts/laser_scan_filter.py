#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy import nan, inf

class LaserScanFilter:
    def __init__(self):
        # Initialize the node
        rospy.init_node('laser_scan_filter', anonymous=True)
        self.rate = rospy.Rate(5)

        self.RAD2DEG = 180 / math.pi
        self.DEG2RAD = math.pi / 180

        # Define the range
        # self.min_angle = -45
        # self.max_angle = 135
        self.min_angle = rospy.get_param('~detect_angle_min', -45)
        self.max_angle = rospy.get_param('~detect_angle_max', 135)
        self.ResponseDist_max = rospy.get_param('~ResponseDist_max', 0.8)
        self.ResponseDist_min = rospy.get_param('~ResponseDist_min', 0.15)
        self.coordinate_360_degree = rospy.get_param('~coordinate_360_degree', False)

        self.frame_id = rospy.get_param('~frame_id', 'laser1')

        self.topic_subscribed = rospy.get_param('~topic_subscribed', '/scan1')
        self.topic_published = rospy.get_param('~topic_published', '/filtered1')
        rospy.loginfo(self.topic_published)
        # Subscriber to the original laser scan topic
        self.subscriber = rospy.Subscriber(self.topic_subscribed, LaserScan, self.scan_callback)

        # Publisher for the filtered laser scan topic
        self.publisher = rospy.Publisher(self.topic_published, LaserScan, queue_size=1)

    def scan_callback(self, msg):
        # Create a new LaserScan message for the filtered data
        filtered_scan = LaserScan()
        filtered_ranges = self.scan_filter(msg.angle_min, msg.angle_increment, msg.ranges)

        # Copy header and other necessary fields
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        # Calculate the index range for angles 0 to 90 degrees
        start_index = int(0 / msg.angle_increment)
        end_index = int(90 / msg.angle_increment)

        # Filter the ranges and intensities
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = msg.intensities

        # Publish the filtered scan
        self.publisher.publish(filtered_scan)
        rospy.loginfo("sent")
        self.rate.sleep()
    
    def scan_filter(self, angle_min, angle_increment, ranges):
        filtered_ranges = np.array(ranges)

        for i in range(len(ranges)):
            angle = (angle_min + angle_increment * i) * self.RAD2DEG
            if self.coordinate_360_degree == False:
                # when ths coordinate_360_degree flag is False (by default)
                if angle > 180: angle -= 360
                # else, keep the 360 coordinate to filter out the range
            if not (angle >= self.min_angle) & (angle <= self.max_angle):
                filtered_ranges[i] = nan
            else:
                if (filtered_ranges[i] > self.ResponseDist_max) or (filtered_ranges[i] < self.ResponseDist_min):
                    filtered_ranges[i] = nan
        return filtered_ranges

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        laser_filter = LaserScanFilter()
        laser_filter.run()
    except rospy.ROSInterruptException:
        pass