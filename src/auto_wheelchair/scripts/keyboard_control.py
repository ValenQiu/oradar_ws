#!/usr/bin/env python3
# coding:utf-8

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control_node')  # 初始化 ROS 节点
        self.pub = rospy.Publisher('/motion', Twist, queue_size=1)  # 发布速度命令
        self.rate = rospy.Rate(5)  # 10 Hz
        self.velocity = Twist()  # 创建 Twist 消息对象

    def on_press(self, key):
        try:
            if key.char == 'w':  # 向前
                self.velocity.linear.x = 1.0
                self.velocity.angular.z = 0.0
            elif key.char == 's':  # 向后
                self.velocity.linear.x = -1.0
                self.velocity.angular.z = 0.0
            elif key.char == 'a':  # 向左
                self.velocity.linear.x = 1.0
                self.velocity.angular.z = 1.0
            elif key.char == 'd':  # 向右
                self.velocity.linear.x = 1.0
                self.velocity.angular.z = -1.0
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
            self.pub.publish(self.velocity)  # 发布速度命令
            self.rate.sleep()  # 控制循环频率

if __name__ == '__main__':
    control = KeyboardControl()
    control.run()