#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math
from numpy import nan, inf
import tf

class ObjectDetection:
    def __init__(self):
        # Initialize the node
        rospy.init_node('object_detection', anonymous=True)
        self.rate = rospy.Rate(5)

        self.RAD2DEG = 180 / math.pi
        self.DEG2RAD = math.pi / 180

        self.topic_subscribed = rospy.get_param('~topic_subscribed', '/filtered1')
        self.topic_published = rospy.get_param('~topic_published', '/object1')
        self.frame_id = rospy.get_param('~frame_id', 'laser1')
        # Subscriber to the original laser scan topic
        self.subscriber = rospy.Subscriber(self.topic_subscribed, LaserScan, self.detect_callback)

        # Publisher for the filtered laser scan topic
        self.publisher = rospy.Publisher(self.topic_published, PoseStamped, queue_size=1)

    def quaternion_from_z_ratation(self, radians):
        
        # calculate the quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(radians / 2)
        qw = math.cos(radians / 2)
    
        return (qx, qy, qz, qw)

    def detect_callback(self, msg):
        # Create a new LaserScan message for the filtered data
        header = msg.header
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        time_increment = msg.time_increment
        range_min = msg.range_min
        range_max = msg.range_max
        ranges = msg.ranges
        intensities = msg.intensities

        minimum_distance = inf
        minimum_distance_angle = None

        for i in range(len(ranges)):
            angle = (angle_min + angle_increment * i) * self.RAD2DEG
            distance = ranges[i]
            if distance != nan:
                if distance < minimum_distance:
                    minimum_distance = distance
                    minimum_distance_angle = angle
            
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

        marker = Marker()
        # marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "laser"  # Replace with your frame ID
        marker.ns = "dynamic_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set the scale of the arrow based on distance
        marker.scale.x = 0.1  # Diameter of the arrow shaft
        marker.scale.y = 0.2  # Diameter of the arrow head
        marker.scale.z = distance  # Length of the arrow
        
        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)
        
        # Set the pose of the arrow
        marker.pose = object_detected.pose
        
        # Publish the filtered scan
        self.publisher.publish(object_detected)
        rospy.loginfo("sent")
        self.rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        object_detection = ObjectDetection()
        object_detection.run()
    except rospy.ROSInterruptException:
        pass
