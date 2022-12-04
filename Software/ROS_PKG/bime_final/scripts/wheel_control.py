#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BOARD)

# Set GPIO Pins
PWM2 = 11
PWM4 = 13
PWM6 = 15
PWM8 = 16
PWM3 = 33
PWM5 = 36
PWM7 = 31
PWM9 = 29

class WheelSingle:
    """A simple class for single wheel"""
    def __init__(self, a, b, ratio):
        self.in_a = a
        self.in_b = b
        self.ratio = ratio # ratio between 0 ~ 1

    def __del__(self):
        self.pwm_a.stop()
        self.pwm_b.stop()

    def setup(self):
        GPIO.setup(self.in_a, GPIO.OUT)
        GPIO.setup(self.in_b, GPIO.OUT)
        self.pwm_a = GPIO.PWM(self.in_a, 100) # 100 Hz
        self.pwm_b = GPIO.PWM(self.in_b, 100)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def rotate(self, speed):
        if speed > 100: # PWM Duty Cycle 0~100, set speed maximum to 100
            speed = 100
        elif speed < -100:
            speed = -100
        if speed == 0:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)
        elif speed > 0:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(speed * self.ratio)
        elif speed < 0:
            self.pwm_a.ChangeDutyCycle(abs(speed * self.ratio))
            self.pwm_b.ChangeDutyCycle(0)

wheel_1 = WheelSingle(PWM2, PWM4, 1)
wheel_2 = WheelSingle(PWM6, PWM8, 1)
wheel_3 = WheelSingle(PWM3, PWM5, 1)
wheel_4 = WheelSingle(PWM7, PWM9, 1)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received Linear: x=%.2f, y=%.2f, z=%.2f | Angular: x=%.2f, y=%.2f, z=%.2f', data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z)
    if data.linear.x != 0:
        wheel_1.rotate(data.linear.x)
        wheel_2.rotate(data.linear.x)
        wheel_3.rotate(data.linear.x)
        wheel_4.rotate(data.linear.x)
    elif data.linear.y != 0:
        wheel_1.rotate(data.linear.y)
        wheel_2.rotate(-1 * data.linear.y)
        wheel_3.rotate(-1 * data.linear.y)
        wheel_4.rotate(data.linear.y)
    elif data.angular.z != 0:
        wheel_1.rotate(-1 * data.angular.z)
        wheel_2.rotate(-1 * data.angular.z)
        wheel_3.rotate(data.angular.z)
        wheel_4.rotate(data.angular.z)
 
def wheel_control():
    rospy.init_node('wheel_control')

    rospy.Subscriber('cmd_vel', Twist, callback)

    wheel_1.setup()
    wheel_2.setup()
    wheel_3.setup()
    wheel_4.setup()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    wheel_control()
