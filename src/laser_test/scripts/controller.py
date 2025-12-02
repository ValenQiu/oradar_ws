#!/usr/bin/env python3
# coding:utf-8

import rospy
from geometry_msgs.msg import Twist
from move_base_control import MoveBase

class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.rate = rospy.Rate(5)
        
        self.controller = MoveBase()
        self.topic_subscribed = rospy.get_param('~topic_subscribed', '/motion')
        self.subscriber = rospy.Subscriber(self.topic_subscribed, Twist, self.controller_callback)
        
    def controller_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        
        if linear_speed > 0 and angular_speed == 0:
            self.controller.forward(1)
        elif linear_speed > 0 and angular_speed > 0:
            self.controller.stearing_forward_left(1, 1)
        elif linear_speed > 0 and angular_speed < 0:
            self.controller.stearing_forward_right(1, -1)
        elif linear_speed < 0 and angular_speed == 0:
            self.controller.backward(-1)
        elif linear_speed < 0 and angular_speed > 0:
            self.controller.stearing_backward_left(-1, 1)
        elif linear_speed < 0 and angular_speed < 0:
            self.controller.stearing_backward_right(-1, -1)
        else:
            self.controller.stop()
        
        self.rate.sleep()
    
    def run(self):
        rospy.spin()
        self.rate.sleep()
        
if __name__ == '__main__':  
    try:
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass