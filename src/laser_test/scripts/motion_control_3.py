#!/usr/bin/env python3
# coding:utf-8

import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker
import math
from numpy import nan, inf
import time

class Controller:
    def __init__(self):
        # Initialize the node
        rospy.init_node('object_detection', anonymous=True)
        self.rate = rospy.Rate(5)

        self.RAD2DEG = 180 / math.pi
        self.DEG2RAD = math.pi / 180

        # range of distances for avoidance, in meters (m)
        self.R = 1.2
        self.r = 0.75

        # Store the poses
        self.poses = [None, None, None]     # [object1, object2, object3]
        self.frame_ids = ['laser1', 'laser2', 'laser3']
        self.trans_poses = [None, None, None]
        self.dist = np.array([inf, inf, inf])
        self.angle = [None, None ,None]
        
        self.stop_motion_flag = False
        self.stop_motion_count = 0
        self.backward_flag = False

        self.topic_subscribed_1 = rospy.get_param('~topic_subscribed_1', '/object1')
        self.topic_subscribed_2 = rospy.get_param('~topic_subscribed_2', '/object2')
        self.topic_subscribed_3 = rospy.get_param('~topic_subscribed_3', '/object3')
        self.topic_published = rospy.get_param('~topic_published', '/motion')
        self.tf_listener = tf.TransformListener()
        self.frame_id = rospy.get_param('~frame_id', 'base')
        # Subscriber to the original laser scan topic
        self.subscriber_1 = rospy.Subscriber(self.topic_subscribed_1, PoseStamped, self.pose_callback)
        self.subscriber_2 = rospy.Subscriber(self.topic_subscribed_2, PoseStamped, self.pose_callback)
        self.subscriber_3 = rospy.Subscriber(self.topic_subscribed_3, PoseStamped, self.pose_callback)

        # Publisher for the filtered laser scan topic
        self.publisher = rospy.Publisher(self.topic_published, Twist, queue_size=1)

    def cal_dist(self, x, y):
        dist = math.sqrt(x**2 + y**2)
        return dist
    
    def pose_callback(self, msg):
        frame_id = msg.header.frame_id
        pose = msg.pose.position

        if frame_id == 'laser1':
            self.poses[0] = pose
        elif frame_id == 'laser2':
            self.poses[1] = pose
        elif frame_id == 'laser3':
            self.poses[2] = pose


    def determine_angular_speed(self, theta):
        # input: theta in range of [0, 360]
        angular = 0
        if 0 <= theta <= 60 | 300 <= theta <= 360 | 135 <= theta <= 225:
            angular = 0
        elif 60 < theta < 135:
            angular = -0.3
        elif 225 < theta < 300:
            angular = 0.3
        
        return angular
    
    def calculate_min_distance(self):
        # Calculate the pairwise distances
        distances = []
        poses = []
        for i in range(len(self.poses)):
            trans = self.transform_pose('base', self.frame_ids[i])
            rospy.loginfo('---------------------------------------')
            rospy.loginfo('index: %s', i)
            rospy.loginfo('trans: %s',trans)
            rospy.loginfo('before transpose: %s', [self.poses[i].x, self.poses[i].y])
            x = self.poses[i].x + trans[0]
            y = self.poses[i].y + trans[1]
            rospy.loginfo('after transpose: %s', [x, y])
            rospy.loginfo('---------------------------------------')
            poses.append([x,y])
        
        for i in range(len(poses)):
            distance = self.cal_dist(poses[i][0] , poses[i][1])
            

        min_distance = min(distances)
        min_distance_index = distances.index(min_distance)
        min_distance_pose = poses[min_distance_index]
        # Return the minimum distance
        return min_distance, min_distance_index, min_distance_pose
    
    def transform_pose(self, current_frame, target_frame):
        # 转换 PoseStamped 到目标坐标系
        (trans, rot) = self.tf_listener.lookupTransform(current_frame, target_frame, rospy.Time(0))
        # rospy.loginfo('---------------------------------------')
        # rospy.loginfo('trans: %s',trans)
        # rospy.loginfo('---------------------------------------')
        return trans
    
    def process_data(self):
        for i in range(len(self.poses)):
            if self.poses[i] != None:
                # rospy.loginfo('---------------------------------------')
                # rospy.loginfo('index: %s', i)
                # rospy.loginfo('pose: %s', self.poses[i])
                trans = self.transform_pose('base', self.frame_ids[i])
                # rospy.loginfo('trans: %s',trans)
                # rospy.loginfo('before transpose: %s', [self.poses[i].x, self.poses[i].y])
                x = self.poses[i].x + trans[0]
                y = self.poses[i].y + trans[1]
                # rospy.loginfo('after transpose: %s', [x, y])
                self.trans_poses[i] = np.array([x, y])
                # rospy.loginfo('check the document: %s', self.trans_poses[i])
                # rospy.loginfo('---------------------------------------')
                
        for i in range(len(self.trans_poses)):
            if self.trans_poses[i] is None:
                return
            distance = self.cal_dist(self.trans_poses[i][0] , self.trans_poses[i][1])
            # rospy.loginfo('---------------------------------------')
            # rospy.loginfo('index: %s', i)
            # rospy.loginfo('distance: %s', distance)
            # rospy.loginfo('---------------------------------------')
            self.dist[i] = distance
        
        dist_min = np.min(self.dist)
        index = np.argmin(self.dist)
        x = self.trans_poses[index][0]
        y = self.trans_poses[index][1]
        angle = math.atan2(y, x) * self.RAD2DEG
        
        
        self.pack_message(dist_min, angle)
        
    def pack_message(self, dist, angle):
        linear_speed = 0
        angular_speed = 0
        
        if angle < 0:
            angle += 360
        
        rospy.loginfo('dist: %s', dist)
        rospy.loginfo('angle: %s', angle)
        
        motion = Twist()
        
        if not self.backward_flag:
            if dist > self.R:
                linear_speed = 1
                angular_speed = 0
                motion.linear.x = linear_speed
                motion.angular.z = angular_speed
                self.publisher.publish(motion)
                
            elif self.r < dist < self.R:
                if self.stop_motion_flag:
                    linear_speed = 0
                    angular_speed = 0
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                    rospy.sleep(2)
                    self.stop_motion_flag = False
                else:
                    linear_speed = 1
                    if 30 < angle < 150:
                        angular_speed = -1
                    elif 210 < angle < 330:
                        angular_speed = 1
                    else:
                        angular_speed = 0
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                
            elif dist < self.r:
                if self.stop_motion_flag is not True:
                    linear_speed = 0
                    angular_speed = 0
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                    self.stop_motion_flag = True
                    rospy.sleep(2)

                else:
                    linear_speed = -1
                    if 30 < angle < 150:
                        angular_speed = 1
                    elif 210 < angle < 330:
                        angular_speed = -1
                    elif 150 < angle < 210:
                        linear_speed = 1
                        angular_speed = 0
                    else:
                        angular_speed = -1
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                    rospy.loginfo(self.stop_motion_count)
                    rospy.sleep(2)
                    self.backward_flag = True
        # reverse the motion
        else:
            if dist > self.R:
                linear_speed = -1
                angular_speed = 0
                motion.linear.x = linear_speed
                motion.angular.z = angular_speed
                self.publisher.publish(motion)
                
            elif self.r < dist < self.R:
                if self.stop_motion_flag:
                    linear_speed = 0
                    angular_speed = 0
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                    rospy.sleep(2)
                    self.stop_motion_flag = False
                else:
                    linear_speed = -1
                    if 30 < angle < 150:
                        angular_speed = 1
                    elif 210 < angle < 330:
                        angular_speed = -1
                    else:
                        angular_speed = 0
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                
            elif dist < self.r:
                if self.stop_motion_flag is not True:
                    linear_speed = 0
                    angular_speed = 0
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                    self.stop_motion_flag = True
                    rospy.sleep(2)

                else:
                    linear_speed = 1
                    if 30 < angle < 150:
                        angular_speed = -1
                    elif 210 < angle < 330:
                        angular_speed = 1
                    elif 150 < angle < 210:
                        linear_speed = 1
                        angular_speed = 0
                    else:
                        angular_speed = -1
                    motion.linear.x = linear_speed
                    motion.angular.z = angular_speed
                    self.publisher.publish(motion)
                    rospy.loginfo(self.stop_motion_count)
                    rospy.sleep(2)
                    self.backward_flag = False      
            # self.stop_motion_flag = False
        rospy.loginfo('backward flag: %s', self.backward_flag)
        rospy.logwarn('published message: %s', motion)
    
    
    def run(self):
        while not rospy.is_shutdown():
            self.process_data()
            
            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        motion_control = Controller()
        motion_control.run()
    except rospy.ROSInterruptException:
        pass