#!/usr/bin/env python3
# coding:utf-8

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control_node')  # 初始化 ROS 节点
        self.pub = rospy.Publisher('/motion', Twist, queue_size=1)  # 发布速度命令
        self.remote_control_pub = rospy.Publisher('/remote_control_enabled', Bool, queue_size=1)
        self.rate = rospy.Rate(5)  # 10 Hz
        self.velocity = Twist()  # 创建 Twist 消息对象
        self.remote_control_enabled = Bool()
        self.remote_control_flag = False

    def update_remote_flag(self):
        self.remote_control_flag = False if self.remote_control_flag else True
        self.update_remote_data()
        rospy.loginfo('Updated Remote Flag to %s', self.remote_control_flag)
        
    def update_remote_data(self):
        remote_control_enabled_data = self.remote_control_flag
        self.remote_control_enabled.data = True if remote_control_enabled_data else False

        
    def on_press(self, key):
        try:
            if key.char == 'p': # remote control on/off
                self.update_remote_flag()
                rospy.logwarn('Remote Control State is now: %s', self.remote_control_flag)
            
            if self.remote_control_flag:
                if key.char == 'w':  # 向前
                    self.velocity.linear.x = 1.0
                    self.velocity.angular.z = 0.0
                    rospy.loginfo('Moving Forward')
                elif key.char == 's':  # 向后
                    self.velocity.linear.x = -1.0
                    self.velocity.angular.z = 0.0
                    rospy.loginfo('Moving Backard')
                elif key.char == 'a':  # 向左
                    self.velocity.linear.x = 1.0
                    self.velocity.angular.z = 1.0
                    rospy.loginfo('Steering Forward Left')
                elif key.char == 'd':  # 向右
                    self.velocity.linear.x = 1.0
                    self.velocity.angular.z = -1.0
                    rospy.loginfo('Steering Forward Right')
            else:
                rospy.loginfo("Not in Remote Control State")
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:  # 按下 Esc 键停止
            return False
        # 停止运动
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

    def run(self):
        # 启动键盘监听
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        while not rospy.is_shutdown():
            self.remote_control_pub.publish(self.remote_control_enabled)
            if self.remote_control_flag:
                self.pub.publish(self.velocity)  # 发布速度命令
            self.rate.sleep()  # 控制循环频率

if __name__ == '__main__':
    control = KeyboardControl()
    control.run()