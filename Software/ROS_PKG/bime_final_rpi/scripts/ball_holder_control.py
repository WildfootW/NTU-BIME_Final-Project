#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

# GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BOARD)

# Set GPIO Pins
PWM0 = 32

class ServoMotor:
    """A simple class for ServoMotor"""
    def __init__(self, pin, minimum = 0, maximum = 100):
        self.pin = pin
        self.minimum = minimum
        self.maximum = maximum

    def __del__(self):
        self.pwm.stop()

    def setup(self):
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 100) # 100 Hz
        self.pwm.start(0)

    def rotate(self, angle):
        pwm_value = angle / 180

        # PWM Duty Cycle 0~100, set speed maximum to 100
        pwm_value = min(pwm_value, 100)
        pwm_value = max(pwm_value,   0)

        self.pwm.ChangeDutyCycle(pwm_value)
        rospy.loginfo(rospy.get_caller_id() + ' Angle: %d, Set PWM to %.2f', angle, pwm_value)

ball_holder = ServoMotor(PWM0)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received CMD')
    if data.data:
        ball_holder.rotate(10)
    else:
        ball_holder.rotate(170)
 
def ball_holder_control():
    rospy.init_node('ball_holder_control')

    rospy.Subscriber('cmd_ball_holder', Bool, callback)

    ball_holder.setup()

    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    ball_holder_control()
