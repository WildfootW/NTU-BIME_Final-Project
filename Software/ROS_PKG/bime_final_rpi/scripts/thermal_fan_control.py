#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

import rospy
from std_msgs.msg import UInt16
import RPi.GPIO as GPIO
import time

# GPIO Mode (Board / BCM)
GPIO.setmode(GPIO.BOARD)

# Set GPIO Pins
PWM1 = 37

class ThermalFan:
    """A simple class for ThermalFan"""
    def __init__(self, pin):
        self.pin = pin

    def __del__(self):
        self.pwm.stop()

    def setup(self):
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50) # 50 Hz
        self.pwm.start(70)

    def rotate(self, pwm_value):
        pwm_value = min(pwm_value, 100)
        self.pwm.ChangeDutyCycle(pwm_value)
        rospy.loginfo(rospy.get_caller_id() + ' Set PWM to %.1f', pwm_value)

thermal_fan = ThermalFan(PWM1)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Received CMD')
    thermal_fan.rotate(data.data)
 
def thermal_fan_control():
    rospy.init_node('thermal_fan')

    rospy.Subscriber('cmd_thermal_fan', UInt16, callback)

    thermal_fan.setup()

    rospy.loginfo(rospy.get_caller_id() + " Setup Complete")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " Spin END")
    GPIO.cleanup()

if __name__ == '__main__':
    thermal_fan_control()


#set GPIO direction (IN / OUT)
GPIO.setup(PWM1, GPIO.OUT)

