#!/usr/bin/env python3
# coding:utf-8

import rospy
from rospy.node import Node
from sensor_msgs.msg import LaserScan
import time
import numpy as np
import ord_lidar_driver  # Replace with the actual lidar driver import

class LidarNode(Node):
    def __init__(self):
        super().__init__('oradar_ros')

        # Parameters
        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('angle_max', 180.0)
        self.declare_parameter('angle_min', -180.0)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('clockwise', False)
        self.declare_parameter('motor_speed', 10)
        self.declare_parameter('device_model', 'ms200')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_topic', 'scan')

        # Get parameters
        self.port = self.get_parameter('port_name').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.max_range = self.get_parameter('range_max').get_parameter_value().double_value
        self.min_range = self.get_parameter('range_min').get_parameter_value().double_value
        self.clockwise = self.get_parameter('clockwise').get_parameter_value().bool_value
        self.motor_speed = self.get_parameter('motor_speed').get_parameter_value().integer_value
        self.device_model = self.get_parameter('device_model').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # Publisher
        self.publisher = self.create_publisher(LaserScan, self.scan_topic, 10)

        # Initialize Lidar Driver
        self.device = ord_lidar_driver.OrdlidarDriver(ord_lidar_driver.ORADAR_TYPE_SERIAL, ord_lidar_driver.ORADAR_MS200)

        # Connect to Lidar
        self.connect_to_lidar()

    def connect_to_lidar(self):
        while True:
            if self.device.Connect(self.port, self.baudrate):
                self.get_logger().info('Lidar device connected successfully.')
                break
            else:
                self.get_logger().warning('Lidar device connecting...')
                time.sleep(1)

    def publish_scan(self):
        while rospy.ok():
            start_scan_time = self.get_clock().now()

            scan_data = self.device.GrabFullScanBlocking(1000)

            end_scan_time = self.get_clock().now()
            scan_duration = (end_scan_time - start_scan_time).nanoseconds / 1e9

            if scan_data:
                self.publish_msg(scan_data, start_scan_time, scan_duration)

            time.sleep(0.1)  # Adjust the sleep time as necessary

    def publish_msg(self, scan_frame, start, scan_time):
        scan_msg = LaserScan()
        point_nums = scan_frame.vailtidy_point_num

        scan_msg.header.stamp = start.to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = np.radians(scan_frame.data[0].angle)
        scan_msg.angle_max = np.radians(scan_frame.data[point_nums - 1].angle)
        diff = scan_frame.data[point_nums - 1].angle - scan_frame.data[0].angle
        scan_msg.angle_increment = np.radians(diff / point_nums)
        scan_msg.scan_time = scan_time
        scan_msg.time_increment = scan_time / point_nums
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range

        scan_msg.ranges = [float('nan')] * point_nums
        scan_msg.intensities = [float('nan')] * point_nums

        for i in range(point_nums):
            range_value = scan_frame.data[i].distance * 0.001
            intensity = scan_frame.data[i].intensity

            if range_value > self.max_range or range_value < self.min_range:
                range_value = float('nan')
                intensity = float('nan')

            dir_angle = (360.0 - scan_frame.data[i].angle) if not self.clockwise else scan_frame.data[i].angle

            if dir_angle < self.angle_min or dir_angle > self.angle_max:
                range_value = float('nan')

            angle = np.radians(dir_angle)
            index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)

            if 0 <= index < point_nums:
                if np.isnan(scan_msg.ranges[index]):
                    scan_msg.ranges[index] = range_value
                elif range_value < scan_msg.ranges[index]:
                    scan_msg.ranges[index] = range_value
                
                scan_msg.intensities[index] = intensity

        self.publisher.publish(scan_msg)

def main(args=None):
    rospy.init(args=args)
    lidar_node = LidarNode()
    try:
        lidar_node.publish_scan()
    except KeyboardInterrupt:
        pass
    finally:
        lidar_node.device.Disconnect()
        rospy.shutdown()

if __name__ == '__main__':
    main()