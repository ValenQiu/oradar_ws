#!/usr/bin/env python3
# coding:utf-8

import rospy
import threading
import math
import time
import numpy as np
from numpy import nan, inf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class LaserDataProcessor:
    def __init__(self):
        self.RAD2DEG = 180 / math.pi
        self.DEG2RAD = math.pi / 180
        
        self.topic_subscribed = rospy.get_param('~topic_subscribed', '/scan1')
        self.subcriber = rospy.Subscriber(self.topic_subscribed, LaserScan, self.scan_callback)
        self.topic_published = rospy.get_param('~topic_published', '/object1')
        self.publisher = rospy.Publisher(self.topic_published, PoseStamped, queue_size=1)
        self.min_angle = rospy.get_param('~detect_angle_min', -45)
        self.max_angle = rospy.get_param('~detect_angle_max', 135)
        self.ResponseDist_max = rospy.get_param('~ResponseDist_max', 0.8)
        self.ResponseDist_min = rospy.get_param('~ResponseDist_min', 0.15)
        self.coordinate_360_degree = rospy.get_param('~coordinate_360_degree', False)
        self.frame_id = rospy.get_param('~frame_id', 'laser1')
        
        self.rate = rospy.Rate(5)
        self.lock = threading.Lock()
        
        self.scan = None
        
    def scan_callback(self, msg):
        with self.lock:
            self.scan = msg
        
    def run(self):
        while not rospy.is_shutdown():
            self.process_data()
            
            self.rate.sleep()
    
    def process_data(self):
        if self.scan == None:
            return
        start_time = time.time()
        rospy.loginfo('processing some data')
        angle_min = self.scan.angle_min
        angle_increment = self.scan.angle_increment
        ranges = self.scan.ranges
        filtered_ranges = self.scan_filter(angle_min, angle_increment, ranges)
        minimum_distance, minimum_distance_angle = self.get_distance_min(angle_min, angle_increment, filtered_ranges)
        self.pack_object_message(minimum_distance, minimum_distance_angle)
        
        end_time = time.time()
        elaspsed_time = end_time - start_time
        rospy.loginfo('processing time takes: %s', elaspsed_time)
    
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
    
    def get_distance_min(self, angle_min, angle_increment, ranges):
        minimum_distance = inf
        minimum_distance_angle = None

        for i in range(len(ranges)):
            angle = (angle_min + angle_increment * i) * self.RAD2DEG
            distance = ranges[i]
            if distance != nan:
                if distance < minimum_distance:
                    minimum_distance = distance
                    minimum_distance_angle = angle
        
        return minimum_distance, minimum_distance_angle

    def quaternion_from_z_ratation(self, radians):
        
        # calculate the quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(radians / 2)
        qw = math.cos(radians / 2)
    
        return (qx, qy, qz, qw)

    def pack_object_message(self, minimum_distance, minimum_distance_angle):
        rospy.loginfo('minimum_distance: %s', minimum_distance)
        if minimum_distance != inf:
            rospy.loginfo(minimum_distance)
            rospy.loginfo(minimum_distance_angle)
            # when the minimum distance exist
            minimum_distance_angle_radian = minimum_distance_angle * self.DEG2RAD
            rospy.loginfo(minimum_distance_angle_radian)
            x = minimum_distance * math.cos(minimum_distance_angle_radian)
            y = minimum_distance * math.sin(minimum_distance_angle_radian)
            orientation = self.quaternion_from_z_ratation(minimum_distance_angle_radian)
        else:
            x, y = inf, inf
            orientation = (0, 0, 0, 0)
            
        object_detected = PoseStamped()
        object_detected.header.stamp = rospy.Time.now()
        object_detected.header.frame_id = self.frame_id

        object_detected.pose.position.x = x
        object_detected.pose.position.y = y
        object_detected.pose.position.z = 0

        object_detected.pose.orientation.x = orientation[0]
        object_detected.pose.orientation.y = orientation[1]
        object_detected.pose.orientation.z = orientation[2]
        object_detected.pose.orientation.w = orientation[3]
        
        self.publisher.publish(object_detected)
        
if __name__ == '__main__':
    try:
        # start the ROS node
        rospy.init_node('laser_data_processor', anonymous=False)
        rospy.loginfo('node created')
        laser_data_processor = LaserDataProcessor()
        rospy.loginfo('start the processor')
        # start the thread
        thread = threading.Thread(target=laser_data_processor.run)
        thread.start()
        rospy.loginfo('thread started')
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass