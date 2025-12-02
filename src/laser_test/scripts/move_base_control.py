#!/usr/bin/env python3
# coding:utf-8

import RPi.GPIO as GPIO
import rospy
import math
from geometry_msgs.msg import Twist
import time

class MoveBase:
    
    def __init__(self):
        # Initialize the node
        # rospy.init_node('move_base_control', anonymous=True)
        self.GPIO_setup()
        # self.topic_subscribed = rospy.get_param('~topic_subscribed', '/motion')
        # self.subscriber = rospy.Subscriber(self.topic_subscribed, Twist, self.move_base_callback)
    
    def GPIO_setup(self):
        GPIO.setmode(GPIO.BOARD)
        
        self.speed_max = 50
        
        # define the pin
        # Motor_1 -> Left
        self.M1_IN1 = 11
        self.M1_IN2 = 13
        self.M1_PWM = 15
        
        GPIO.setup(self.M1_IN1, GPIO.OUT)
        GPIO.setup(self.M1_IN2, GPIO.OUT)
        GPIO.setup(self.M1_PWM, GPIO.OUT)
        GPIO.output(self.M1_IN1, GPIO.LOW)
        GPIO.output(self.M1_IN2, GPIO.LOW)
        self.pwm1 = GPIO.PWM(self.M1_PWM, 200)   # 50Hz
        self.pwm1.start(0)
        # GPIO.output(self.M1_PWM, GPIO.LOW)
        
        # Motor_2 -> Right
        self.M2_IN1 = 33
        self.M2_IN2 = 35
        self.M2_PWM = 37
        
        GPIO.setup(self.M2_IN1, GPIO.OUT)
        GPIO.setup(self.M2_IN2, GPIO.OUT)
        GPIO.setup(self.M2_PWM, GPIO.OUT)
        GPIO.output(self.M2_IN1, GPIO.LOW)
        GPIO.output(self.M2_IN2, GPIO.LOW)
        self.pwm2 = GPIO.PWM(self.M2_PWM, 200)   # 50Hz
        self.pwm2.start(0)
        # GPIO.output(self.M2_PWM, GPIO.LOW)
        
        # Relay
        self.R1 = 38    # Left
        self.R2 = 40    # Right
        
        GPIO.setup(self.R1, GPIO.OUT)
        GPIO.setup(self.R2, GPIO.OUT)
        GPIO.output(self.R1, GPIO.HIGH)
        GPIO.output(self.R2, GPIO.HIGH)
        
        # GPIO.output(self.M1_IN1, GPIO.HIGH)   # going backward
        # self.pwm1.ChangeDutyCycle(3)
    
    def move(self, M1_speed, M2_speed):
        # inputs:
        #   M1_speed: the speed of the left motor, the speed is the ratio to the maximum speed by setting
        #   M2_speed: the speed of the right motor, the speed is the ratio to the maximum speed by setting
        if M1_speed > 0:
            GPIO.output(self.M1_IN1, GPIO.LOW)
            GPIO.output(self.M1_IN2, GPIO.HIGH)
        elif M1_speed < 0:
            GPIO.output(self.M1_IN1, GPIO.HIGH)
            GPIO.output(self.M1_IN2, GPIO.LOW)
        else:
            GPIO.output(self.M1_IN1, GPIO.LOW)
            GPIO.output(self.M1_IN2, GPIO.LOW)
            
        if M2_speed > 0:
            GPIO.output(self.M2_IN1, GPIO.LOW)
            GPIO.output(self.M2_IN2, GPIO.HIGH)
        elif M2_speed < 0:
            GPIO.output(self.M2_IN1, GPIO.HIGH)
            GPIO.output(self.M2_IN2, GPIO.LOW)
        else:
            GPIO.output(self.M2_IN1, GPIO.LOW)
            GPIO.output(self.M2_IN2, GPIO.LOW)
            
        self.pwm1.ChangeDutyCycle(self.speed_max * abs(M1_speed))
        self.pwm2.ChangeDutyCycle(self.speed_max * abs(M2_speed))
            
    def stop(self):
        GPIO.output(self.M1_IN1, GPIO.LOW)
        GPIO.output(self.M1_IN2, GPIO.LOW)
        GPIO.output(self.M2_IN1, GPIO.LOW)
        GPIO.output(self.M2_IN2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        rospy.loginfo('stop')
        
    def forward(self, speed):
        # the speed must be positive
        if speed > 0:
            self.move(speed, speed)
            rospy.loginfo('moving forward')
        else:
            self.stop()
            rospy.logwarn('moving forward failed, stop')
        
    def backward(self, speed):
        if speed < 0:
            self.move(speed, speed)
            rospy.loginfo('moving backward')
        else:
            self.stop()
            rospy.logwarn('moving backward failed, stop')
            
    def stearing_forward_left(self, speed_linear, speed_angular):
        # inputs:
        #   speed_linear: the linear speed of the motion, the speed is the ratio to the maximum speed by setting
        #   speed_angular: the angular speed of the motion, the speed is positive if turning anticlockwise
        
        if speed_linear > 0 and speed_angular > 0:
            # some processing of the speed if needed
            # ..
            M1_speed = 0.2
            M2_speed = 1.5
            self.move(M1_speed, M2_speed)
            rospy.loginfo('stearing forward left')
        else:
            self.stop()
            rospy.logwarn('stearing forward left failed, stop')
        
    def stearing_backward_left(self, speed_linear, speed_angular):
        # inputs:
        #   speed_linear: the linear speed of the motion, the speed is the ratio to the maximum speed by setting
        #   speed_angular: the angular speed of the motion, the speed is positive if turning anticlockwise
        
        if speed_linear < 0 and speed_angular > 0:
            # some processing of the speed if needed
            # ..
            M1_speed = -0.2
            M2_speed = -1.5
            self.move(M1_speed, M2_speed)
            rospy.loginfo('stearing backward left')
        else:
            self.stop()
            rospy.logwarn('stearing backward left failed, stop')
            
    def stearing_forward_right(self, speed_linear, speed_angular):
        # inputs:
        #   speed_linear: the linear speed of the motion, the speed is the ratio to the maximum speed by setting
        #   speed_angular: the angular speed of the motion, the speed is positive if turning anticlockwise
        
        if speed_linear > 0 and speed_angular < 0:
            # some processing of the speed if needed
            # ..
            M1_speed = 1.5
            M2_speed = 0.2
            self.move(M1_speed, M2_speed)
            rospy.loginfo('stearing forward right')
        else:
            self.stop()
            rospy.logwarn('stearing forward right failed, stop')
            
    def stearing_backward_right(self, speed_linear, speed_angular):
        # inputs:
        #   speed_linear: the linear speed of the motion, the speed is the ratio to the maximum speed by setting
        #   speed_angular: the angular speed of the motion, the speed is positive if turning anticlockwise
        
        if speed_linear < 0 and speed_angular < 0:
            # some processing of the speed if needed
            # ..
            M1_speed = -1.5
            M2_speed = -0.2
            self.move(M1_speed, M2_speed)
            rospy.loginfo('stearing backward right')
        else:
            self.stop()
            rospy.logwarn('stearing backward right failed, stop')
        
    def run(self):
        rospy.spin()
        # rospy.sleep(5)
        
    
        
if __name__ == '__main__':  
    try:
        move_base = MoveBase()
        move_base.forward(1)
        rospy.loginfo('forward')
        time.sleep(20)
        move_base.backward(-1)
        rospy.loginfo('backward')
        time.sleep(20)
        # move_base.stearing_forward_right(1, -1)
        move_base.run()
        # time.sleep(20)
        
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass